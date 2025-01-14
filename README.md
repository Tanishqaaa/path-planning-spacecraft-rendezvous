Here’s a GitHub README template to understand the project futher:  

---

# **Path planning for spacecraft rendezvous and collision avoidance**    

> **A service or chaser spacecraft from 5 km away follows an optimized and constraint path to achieve docking with a client or target spacecraft for the use case of in-orbit servicing while avoiding collision from dynamic debris.**  

---

## **Table of Contents**  
- [About The Project](#about-the-project)
- [Prerequisites](#prerequisites)  
- [Getting Started](#getting-started)    
- [Instruction for use](#instruction-for-use) 
- [License](#license)  
- [Contact](#contact)  
- [Acknowledgments](#acknowledgments)  

---

## **About The Project**  
### **Purpose**  
NewSpace involves activities relating to debris removal, avoidance and in-orbit repairs or service of satellites. This project preents a solution to the path planning problem during rendezvous. The problem of rendezvous in space is rather complicated with constraints on thrust, fuel, terminal position, terminal velocity, time and safety margins to prevent collision. Further, the solution needs to be optimized to achieve a time and cost-efficient solution.
To achieve this following criteria was kept in mind:

### **Key Challenges**  
- 🛠️ Maintaining a safe distance from debris  
- 🛠️ Minimizing fuel consumption via efficient thrust control  
- 🛠️ Ensuring precise positioning and velocity matching within terminal constraints
- 🛠️ Managing thrust and trajectory smoothness constraints.

### **Constraints**  
- 🔧 Thrust: Limited to ±0.01 km/s² per axis (max Thrust 10N)  
- 🌟 Debris Avoidance: Minimum safe distance of 50 meters (0.05 km)
- 🔧 Fuel Minimization: Minimize total delta V loss due to fuel consumption
- 🌟 Terminal State: Position within 1–5 meters and velocity within 0.22 m/s of the target
- 🔧 Smoothness: Limit abrupt changes in control inputs to avoid wear and tear

### **Built With**  
List of frameworks/libraries/tools used:  
- [MATLAB 2024](https://de.mathworks.com/)  
- [Model Predictive Control ToolBox](https://de.mathworks.com/products/model-predictive-control.html)

---
### **Prerequisites**  
- State-Space Dynamics
- Model Predictive Control
- Spacecraft fundamentals
- Coding with MATLAB
  
## **Getting Started**  

**Objective**: To perform a rendezvous maneuver between a chaser spacecraft and a target object in orbit while avoiding debris, minimizing fuel consumption, and adhering to dynamic constraints.   

### **Logic and Methodology**
#### 1. **State-Space Dynamics**
   The spacecraft's motion is modeled using the **Clohessy-Wiltshire-Hill (CWH)** equations, which describe relative motion between two orbiting objects. The continuous-time state-space representation is defined by:
   - **State Vector**: \([x, y, z, \dot{x}, \dot{y}, \dot{z}]\) (relative position and velocity).
   - **Control Vector**: \([F_x, F_y, F_z]\) (thrust forces in 3 axes).
   - **Dynamics**:
     \[
     \dot{x} = Ax + Bu
     \]
     where \(A\) and \(B\) are derived from the linearized CWH equations.

   The system is discretized with a sampling time of \(T_s = 3 \, \text{seconds}\) for compatibility with the MPC controller.

---

#### 2. **Model Predictive Control (MPC)**
   - **Prediction and Control Horizons**:
     - Prediction Horizon: 30 steps (\(90 \, \text{seconds}\)).
     - Control Horizon: 20 steps (\(60 \, \text{seconds}\)).
   - **Weights**:
     - Minimize control effort to reduce fuel consumption.
     - Prioritize position over velocity to ensure a safer trajectory.
   - **Constraints**:
     - **Thrust Limits**: Bound thrust forces between \(-\text{maxAcc}\) and \(\text{maxAcc}\).
     - **Terminal State**: Enforce final position and velocity constraints as soft constraints.

---

#### 3. **Collision Avoidance**
   - Real-time debris avoidance is implemented by checking the distance between the chaser and debris objects.
   - If the squared distance is below the safety threshold (\(50 \, \text{m}\)), a dynamic constraint is applied to the MPC problem to avoid collision.

---

#### 4. **Fuel Consumption**
   - Fuel usage is modeled using the **Tsiolkovsky Rocket Equation**, incorporating specific impulse and exhaust velocity.
   - Fuel consumption is tracked over time, and the total mass used is calculated at the end.

---

### **Implementation Details**
1. **MPC Design**:
   - The MPC controller is created using MATLAB's `mpc` function.
   - Constraints on thrust (\(u\)) and terminal states are enforced directly via the controller's `ManipulatedVariables` and `OutputVariables`.

2. **Debris Avoidance**:
   - Debris trajectories are propagated using the same Hill dynamics.
   - A collision detection mechanism dynamically adds constraints to the MPC problem when a potential collision is detected.

3. **Terminal Constraints**:
   - Soft constraints ensure final position and velocity requirements are met while allowing some flexibility for feasibility.

4. **3D Visualization**:
   - Relative position and velocity are converted to the **Earth-Centered Inertial (ECI)** frame for animation.
   - The `satelliteScenario` toolbox is utilized to simulate orbital motion and render the rendezvous in 3D.

---

### **Results**
1. **Trajectory Tracking**:
   - The chaser follows a smooth trajectory while avoiding debris.
   - Position and velocity converge to the target within the specified terminal constraints.

2. **Fuel Efficiency**:
   - Fuel consumption is minimized by penalizing control effort in the MPC optimization.

3. **Collision Avoidance**:
   - The chaser successfully avoids all debris objects while adhering to safety margins.

4. **Visualization**:
   - The animation provides an intuitive understanding of the rendezvous process, including debris avoidance and trajectory adjustments.

---

### **Improvements and Extensions**
1. **Safety Margins**:
   - Add constraints of atleast 1 m sphere around the target.
   - Add constraint so that the trajectory does not intersect the Earth
2. **Advanced Debris Modeling**:
   - Incorporate non-linear dynamics or stochastic modeling for debris motion such as a Kalman Filter
3. **3D Models**:
   - Upgrade visualization with detailed spacecraft models and camera animations.
4. **Optimization Tuning**:
   - Experiment with prediction and control horizons for better computational efficiency
5. **Orbital Pertubations**:
   - Increase the fidelity of model by adding pertubations such as SRP, J2 perturbation, third body perturbation, uneven gravitational field, drag and so on

---

This implementation demonstrates a robust framework for autonomous rendezvous and proximity operations in space, with a focus on safety, fuel efficiency, and precise terminal state control.


### **Instruction for use**  
Instructions for setting up the project.  
1. Download all the files in the same folder  
2. Add it to MATLAB path
3. Run main.m

---

Include screenshots or videos where possible.  

---

## **License**  
This project is licensed as Personal Work. The contents of this repository, including all code, designs, and documentation, are for personal use only and may not be copied, distributed, or shared without explicit written permission from the author. 

---

## **Contact**  
Connect – [tanishqa_jk](https://www.linkedin.com/in/tanishqa-jk/) – khantedtanishqa22@gmail.com  

Project Link: [https://github.com/Tanishqaaa/path-planning-spacecraft-rendezvous/](https://github.com/Tanishqaaa/path-planning-spacecraft-rendezvous/))  

---

## **Acknowledgments**  
Shoutouts to those who helped:  
- [Fundamentals of Astrodynamics and Applications by David Vallado]([https://example.com](https://books.google.com/books/about/Fundamentals_of_Astrodynamics_and_Applic.html?id=PJLlWzMBKjkC&printsec=frontcover&source=kp_read_button&hl=en&newbks=1&newbks_redir=1))   

---

### Feel free to get in touch!
