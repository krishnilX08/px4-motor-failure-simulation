# INTER IIT TECH MEET 14.0: PREPATHON PS
# Part 2: PX4 Motor Failure Simulation

## Overview
This project demonstrates a two-phase implementation involving **PX4 SITL** setup, takeoff, hover and **runtime modification of the control allocation matrix** to simulate motor failure in flight.

### The primary objectives:

1. Autonomous takeoff and stable hover at 20m altitude.

2. Runtime modification of the effectiveness matrix to disable one motor and remove it from mixing.

3. Achieving a Controlled Fall without engaging failure detection or recovery algorithms.

---

## Phase 1 — PX4 Setup and Takeoff
- Configured PX4 firmware using **Gazebo SITL** with the **standard quadrotor mixer** (gz_x500).
- Used QGroundControl to command the copter to ascend to 20m and hover autonomously as a mission.
- Verified takeoff behavior up to **20 meters altitude** with all motors operational.
- Ensured stable control allocation through the default mixer configuration.

**Result:** Stable ascent to 20 m with all motors functioning.

---

## Phase 2 — Motor Mixing Modification

### Objective
Modify PX4’s **ControlAllocator** at runtime to simulate the failure of one motor by altering the **effectiveness matrix**.

### Approach

- I first attempted it by directly outputting one of the matrix column as 0 in ControlAllocator.cpp, which fails the motor superficially, so this method was **discarded. I have taken this as Approach 1.**

- In Approach 2, I researched a lot more and was able to accomplish some level of understanding of the pipeline. **So this approach 2 was finalized and implemented.**

#### Approach 1: Direct Matrix Manipulation (Initial Attempt)
- Modified effectiveness matrix directly within ControlAllocator. This bypassed PX4's effectiveness framework architecture.

-  The manipulation of matrices was not done at the proper layer. This showed the ability to hack the system without following the architecture.

The implementation can be found in the commented out code at the end of ControlAllocator.cpp.

**Result:** Uncontrolled fall, mostly due to the system not knowing about the failure. Underactuation.

#### Approach 2: Pipeline understanding and manipulation (Final Attempt)
- Used proper PX4 APIs through ActuatorEffectivenessMultirotor::disableMotor() and found the ultimate Matrix building class of ActuatorEffectivenessRotors

- Tried to follow correct control allocation pipeline:
ControlAllocator → ActuatorEffectivenessMultirotor → ActuatorEffectivenessRotors

The implementation is explained ahead and can be found in attached files.

**Result:** Uncontrolled fall due to underactuation and oversaturated outputs to the remaining motors after failure and reallocation.


---
## Location of all files used-

- PX4-Autopilot/src/module/control_allocator/ControlAllocator.cpp
- PX4-Autopilot/src/module/control_allocator/ControlAllocator.hpp
- PX4-Autopilot/src/module/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessMultirotor.hpp
- PX4-Autopilot/src/module/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessRotors.cpp
- PX4-Autopilot/src/module/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessRotors.hpp
                                          
---

## Technical Explanation of Approach 2
This approach integrates motor failure(removal) and handling directly within the PX4 control allocation pipeline.

### Key Files Modified
1. ControlAllocator.cpp/.hpp
2. ActuatorEffectivenessMultirotor.hpp
3. ActuatorEffectivenessRotors.cpp/.hpp

## Why were these specific files chosen

### 1. ControlAllocator
- This class is the final or high level stage in PX4's control pipeline, responsible for converting torque and thrust setpoint into the actuator commands.
- It is like the manager of allocation, and can be the ideal point to introduce the runtime modification, since it calls the ActuatorEffectiveness layer to rebuild the allocation matrix.

**What did I implement here:** 

 This function was called in- void ControlAllocator::Run()

    void ControlAllocator::remove_motor()
    {
        //local structure to hold the position data
        vehicle_local_position_s pos{};

        //try to copy latest position data from subscription
        if (_vehicle_local_position_sub.copy(&pos)) {
            float altitude = -pos.z;  // NED frame: down is positive, so invert

            static bool motor_disabled = false; //flag
            static bool reached_20m = false; //flag
            static hrt_abstime reach_20m_time = 0;

            // Check if we just reached 20m for the first time
            if (altitude >= 20.0f && !reached_20m) {
                reached_20m = true;
                reach_20m_time = hrt_absolute_time();
                PX4_INFO("Reached 20m altitude, starting 5-second hover period");
            }

            //if 20m is reached and motor is working and 5 second hover is over, disable the motor
            if (reached_20m && !motor_disabled && hrt_elapsed_time(&reach_20m_time) > 5000000) {
                PX4_INFO("Altitude = 20m, hovering 5 sec and disabling motor 0");

                auto *multirotor = dynamic_cast<ActuatorEffectivenessMultirotor*>(_actuator_effectiveness);
                if (multirotor != nullptr) {
                    multirotor->disableMotor(0);
                    // Force an immediate effectiveness matrix update
                    update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE);
                }
                
                motor_disabled = true;
            }
        }
    }

- Defined a new function remove_motor() which subscribes to the muORB topic vehicle_local_position to fetch the altitude in NED frame. As it reaches 20m, motor_disabled flag is triggered and the quadcopter hovers there for 5 seconds before completely removing or skipping the motor from mixing in ActuatorEffectivenessRotors.

- The dynamic_cast checks if the vehicle is actually a multirotor by checking the motor configuration, as _actuator effectiveness can be of many vehicles. If a multirotor, then it proceeds

- It just calls the disableMotor(int) of the ActuatorEffectivenessMultiRotor.
 This keeps the controller logic untouched, so that higher level behaviour remains standard.

### 2. ActuatorEffectivenessMultirotor

- The effectiveness classes how torques and thrust translate to actuator outputs.
 The ActuatorEffectivenessMultirotor is a high level configuration class that represents a multirotor vehicle. It holds a collection of rotors and itself isn't responsible for matrix building. It calls lower level geometry builder (_mc_rotors), i.e. it is the interface between the ControlAllocator and the low level matrix builder.

**What did I implement here:** 

in ActuatorEffectivenessMultirotor.hpp

    void disableMotor(int index)
    {
        _mc_rotors.setMotorToDisable(index); 
        //call the motor setter function in ActuatorEffectivenessRotors

        // Force immediate matrix rebuild
        Configuration config{};
        getEffectivenessMatrix(config, EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE);
    }


- As the ControlAllocator cannot directly mark the motor as disabled internally in in _mc_rotors of    ActuatorEffectivenessRotors class.
 A new API is added to this class- disableMotor(int index).

- It exposes a high-level command from external modules (ControlAllocator) that internally marks a motor as disabled in _mc_rotors and rebuilds the matrix immediately.

- So the allocator doesn’t need to know the details of how the matrix is built — it just calls disableMotor(0) and continues working.

### 3. ActuatorEffectivenessRotors

- ActuatorEffectivenessRotors is a low-level class that defines the mechanical configuration of the rotors:
 Their positions relative to the center of mass
 their rotation direction, the thrust and torque coefficients and their contribution to roll, pitch, yaw, and thrust.

- It ultimately builds the Effectiveness Matrix (B-matrix) — a 6×4 matrix (4 control axes and 4 rotors). This matrix is passed upward to PX4’s control allocator to map.

**What did I implement here:** 

in ActuatorEffectivenessRotors.cpp
 
    int ActuatorEffectivenessRotors::computeEffectivenessMatrix()
   
	if (i == _motor_to_disable) {
		PX4_INFO("Skipping motor %d (disabled)", i);
		continue; //skip the motor i
	}

and

    bool
    ActuatorEffectivenessRotors::getEffectivenessMatrix(Configuration &configuration,
          EffectivenessUpdateReason external_update)
    {
	   //force to rebuild the matrix
	   if (_motor_to_disable >= 0) {
		return addActuators(configuration);
	   }
    }


in ActuatorEffectivenessRotors.hpp

    public:
        // 
	    void setMotorToDisable(int index) {_motor_to_disable = index;}

    private:
	    // which motor to disable
	    int _motor_to_disable{-1};
    


- I added the logic to dynamically skip one rotor during matrix generation.

- So whenever _motor_to_disable is set (0 or front right motor in our case), the corresponding rotor’s column is not added to the effectiveness matrix.
This dynamically removes that rotor’s contribution from PX4’s mixing logic, effectively telling the control allocator that this motor no longer exists.

- The controller now builds a 6x3 matrix instead of 6x4 matrix and the control is now distributed to the remaining motors.


### How it all connects:

 Rate Controller -> 
 ControlAllocator module (altitude monitor, calls Multirotor-> disableMotor(0), triggers matrix rebuild) -->
 
 ActuatorEffectivenessMultirotor (forwards disable command, rebuilds configuration, calls _mc_rotors.addActuators()) -->

 ActuatorEffectivenessRotors (builds effectiveness matrix, skips disabled motor)          

### Physics Behind the Result

**Why Uncontrolled Fall is Physically Correct:**
- **Underactuation**: 3 motors cannot control a quadcopter to hover
- **Lost Control**: Cannot generate required torques for stabilization
- **PS limitation**: No artificial recovery algorithms and failure detection

---

## 🧪 Deployment Steps
1. Build the firmware and launch simulation:

       make px4_sitl gz_x500
   
2. Open QGroundControl and command the quadcopter  to attain 20m altitude and hover.

3. Observe takeoff to 20 m, and then hover for 5 seconds.

4. Watch for PX4 log message:

       Altitude = 20m, hovering 5 sec and disabling motor 0

5. Verify motor disable behavior and observe the fall.

---

## 📄 References

- https://docs.px4.io/main/en/concept/control_allocation.html
- https://docs.px4.io/main/en/concept/architecture.html
- https://docs.px4.io/main/en/flight_stack/controller_diagrams.html
- https://github.com/PX4/PX4-Autopilot/issues/18556


## 👤 Author
- **Name:** Krishnil Rishianand Shindore
- **Institution:** IIT (BHU) Varanasi  
- **Project:** PX4 Motor Failure Simulation (Control Allocator)
- **Github:** https://github.com/krishnilX08
- **Email:** krishnilshindore@outlook.com, krishnil.shindore.chy24@itbhu.ac.in

---
