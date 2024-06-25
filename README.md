# AircraftFlightDynamics

Let's start by looking at **an aircraft and its controls.**

## Aircraft
What is **an aircraft**?

Ans: "Craft or vessel which navigates through the air."

What are **aircraft controls**?

Ans: Means by which a pilot or an autopilot controls the direction and attitude(roll, pitch, yaw) of an aircraft in flight.

What is looking at something?

Ans: Clear and pure observation through eyes(brain).


**Coordinate Frame: Body-axes coordinate system.**

What is a **coordinate frame**?

Ans: To describe any phenomenon of a system (single/multiple particle system or a field), one must choose a set of coordinates to uniquely determine its positions on a manifold or a frame. This frame is called a coordinate frame. 

What is a **body-axes coordinate system**?

Ans: A coordinate system fixed to the airplane. The origin is at the center of gravity; the x-axis is in the airplane plane of symmetry and points out the nose of the airplane; the z-axis is in the plane of symmetry and points downward, and the y-axis is perpendicular to the x and z axes and points out the right wing.
![Sketch](https://github.com/Praful22/AircraftFlightDynamics/assets/65821250/d958eda9-7a2e-489e-aa57-bd01f76c9d32)

The origin of the coordinate system is at the center of gravity represented by the black dot in the above figure. 

## Equations of Motions

**Assumptions:**
What are **assumptions** and why **assume**?

-> Assumptions 

1. An aircraft is treated as a rigid body whose dynamics are comprised of three translational and three rotational degrees of freedom, hence 6-DoF motion.

2. Body axes coordinate frame is fixed at the aircraft's center of gravity (CG).

Motion can be described by:
1. Translational Motion:

    a. Forward Velocity $\textit{u}$ (positive along the fuselage-body x-axis)

    b. Lateral Velocity $\textit{v}$ (positive along the right-wing-body y-axis)

    c. Vertical Velocity $\textit{w}$ (positive down and along the body z-axis)


2. Rotational degrees of freedom representing rotational motion:

    a. Body roll rate $\textit{p}$ (around the x-axis)

    b. Body pitch rate $\textit{q}$ (around the y-axis)

    c. Body yaw rate $\textit{r}$ (around the z-axis)

   Caveat: Positive angular rates(p,q,r) result in the counterclockwise rotations around their respective axis (x,y,z).

The 6-DoF aircraft equations of motion may be modeled as:

Translational Degree of Freedom: 
```math
\begin{aligned}
& m\left(\begin{array}{c}
\dot{u} \\
\dot{v} \\
\dot{w}
\end{array}\right)=-\left[\left(\begin{array}{c}
p \\
q \\
r
\end{array}\right) \times\left(\begin{array}{c}
u \\
v \\
w
\end{array}\right)\right]+\left(\begin{array}{c}
F_x \\
F_y \\
F_z
\end{array}\right)
+ m \underbrace{\underbrace{\|\vec{g}\|}_g\left(\begin{array}{c}
-\sin \theta \\
\cos \theta \sin \varphi \\
\cos \theta \cos \varphi
\end{array}\right)}_{\vec{g}} \\ 
\end{aligned}
```
Rotational Degree of Freedom:
```math
\begin{aligned}
 J\left(\begin{array}{c}
\dot{p} \\
\dot{q} \\
\dot{r}
\end{array}\right)=-\left[\left(\begin{array}{c}
p \\
q \\
r
\end{array}\right) \times J\left(\begin{array}{c}
p \\
q \\
r
\end{array}\right)\right]+\left(\begin{array}{c}
\bar{L} \\
M \\
N
\end{array}\right) \\
\end{aligned}
```
where, 

```math
\begin{align*}
m &\triangleq \text{aircraft mass} \\
J &\triangleq \text{Vehicle Inertia Matrix} \\
(F_x, F_y, F_z) &\triangleq \underbrace{\text{Body (x,y,z) components of forces}}_{\text{Due to Aerodynamics and Propulsion}} \\
(\bar{L}, M, N) &\triangleq \underbrace{\text{Body (x,y,z) components of Moments}}_{\text{Due to Aerodynamics and Propulsion}} \\
\vec{g} &\triangleq \text{gravity vector} \\
g &\triangleq \|\vec{g}\| = \text{Magnitude of the gravity vector}\\
\end{align*}
```
The gravity vector is expressed in the aircraft body axes coordinates in terms of the following:

```math
\begin{align*}
\phi &\triangleq \text{vehicle bank angle. Positive: Aircraft's right-wing down} \\
\theta &\triangleq \text{Pitch angle. Positive: Aircraft nose-up} \\
\psi &\triangleq \text{True heading angle. Positive: Clockwise rotation of the aircraft nose from the true north direction} \\
\end{align*}
```
The three Euler angles $(\phi, \theta, \psi)$ represent the inertial angular orientation of aircraft if treated as a 
rigid body moving in three-dimensional inertial space.
## TODO: ADD a picture representing the Euler angles.

The following kinematics relation describes the dynamics of the Euler angles versus the aircraft body angular rates ($\textit{p},\textit{q},\textit{r}$):

```math
\begin{align*}
\left(\begin{array}{c}
\dot{\varphi} \\
\dot{\theta} \\
\dot{\psi}
\end{array}\right)=\left(\begin{array}{ccc}
1 & \sin \varphi \tan \theta & \cos \varphi \tan \theta \\
0 & \cos \varphi & -\sin \varphi \\
0 & \frac{\sin \varphi}{\cos \theta} & \frac{\cos \varphi}{\cos \theta}
\end{array}\right)\left(\begin{array}{l}
p \\
q \\
r
\end{array}\right)
\end{align*}
```

Now,

**State Vector**: A mathematical representation used to describe the complete state of the dynamic system at a specific point in time.

***Why State Vector?*** 
1. A state vector captures all the essential information needed to describe the system's current state. For example, in the case of a moving object, it would include its position, velocity, and possibly acceleration in three-dimensional space.
2. State vectors are particularly important for dynamic systems, where the system's behavior changes over time. By updating the state vector as time progresses and abiding by the laws that govern the dynamics of a system, you can predict future states and behaviors.

There are many reasons why a state vector is essential. We will go into much detail later.

The aircraft 6-Degree of Freedom (6 DoF) state vector is given by:

```math
\begin{align*}
\vec{x} &\triangleq \left(\begin{array}{c}
u \\
v \\
w \\
p \\
q \\
r \\
\phi \\
\theta \\
\psi
\end{array}\right)
\end{align*}
```

With n = 9 states in the system dynamics.

## Primary Control Inputs

There are four primary control inputs to an aircraft. They are as follows:

1. **Ailerons** (Left and Right ailerons, $\delta_{left ail}, \delta_{right ail}$): The differential aileron $\delta_{a} = \delta_{left ail} - \delta_{right ail}$ is the primary roll control input.

<img src="https://github.com/Praful22/AircraftFlightDynamics/assets/65821250/b3749d8f-19c4-4d8f-9290-02f45f62dd96" width="500" height="500">

2. **Horizontal Stabilizers**: Slow-movable surface control used to trim/equalize the aircraft's longitudinal forces and moments.

3. **Elevator**: Controls the pitching motion of the aircraft. ($\delta_e$)

<img src="https://github.com/Praful22/AircraftFlightDynamics/assets/65821250/fcb90b26-641b-46f9-8a91-52eaa1d8f00a" width="500" height="500">


4. **Rudder**: Controls the yawing motion of the aircraft ($\delta_r$).

<img src="https://github.com/Praful22/AircraftFlightDynamics/assets/65821250/8cbff590-f9a5-473d-ba44-37aa40ac6f50" width="500" height="500">


If the aircraft is not a glider then additional primary control input will be 

5. **Thrust force** ($\delta_{th}$): Created by either propellers or jet engines which provide airspeed control.

Disregarding horizontal stabilizers, the aircraft's primary control vector inputs are shown below:


```math
\begin{align*}

\vec{u} &\triangleq \left(\begin{array}{c}
\delta_{th} \\
\delta_a \\
\delta_e \\
\delta_r
\end{array}\right)
\end{align*}
```

By changing the individual control input, the aircraft dynamics can be modified to fly and maneuver the aircraft.

## System Output

The system output signals can be defined based on the availability of physical measurement devices that are installed on the aircraft.

What are the physical measurement devices on an aircraft?
1. **Rate Gyroscopes**: Measures the body angular velocity or rate of rotation of an object about one or more axes. Hence, $\textit{(p,q,r)}$ is measured by a rate gyroscope which is attached close to the Center of Gravity and is configured accordingly to provide three Euler Angles.

```math
(\phi, \theta, \psi)
```
2. **Accelerometers**: Provides online measurements of longitudinal, lateral, and vertical loads denoted by ($A_{x}, A_{y}, A_{z}$) so at least three accelerometers are used. Each device measures an acceleration component ( in ft/s^2 or g-s) at the point of installation along its corresponding axis:

```math
\begin{align*}
A_{x} = \frac{F_{x}}{mg} \\
A_{y} = \frac{F_{y}}{mg} \\
A_{z} = \frac{F_{z}}{mg}
\end{align*}
```
3. **Angle of Attack Probe or Angle of Attack Indicator**: AoA is the angle($\alpha$) between the oncoming air and the chord line of the wing. It's a critical parameter for controlling the aircraft's lift and stall characteristics. AoA is typically measured using an instrument called an "angle of attack indicator" or "AoA probe." The most common types of **AoA probes include**:

        a. Vane Type AoA Indicator: This uses a vane exposed to the relative wind to measure the angle of attack.

        b. Pressure-Based AoA Probe: Some aircraft use pressure sensors on the wing's leading edge to measure the pressure difference between the upper and lower surfaces. This pressure difference is then used to calculate the AoA.

        c. Angle of Attack Vanes: These are physical vanes attached to the wing that directly measure the AoA. 
```math
\begin{align*}
\alpha = \arctan{\frac{w}{u}}
\end{align*}
```
4. **Side Slip Indicator or Slip/Skid Indicator**: AoSS is the angle($\beta$) between the aircraft's longitudinal axis (the direction the aircraft is pointing) and the relative wind. It indicates how the aircraft is oriented with respect to the oncoming air. AoSS is often measured using a device called a "sideslip indicator" or "slip/skid indicator." This instrument usually includes a ball suspended in a curved tube, similar to a spirit level. When the ball is centered, the aircraft is in coordinated flight (no sideslip). When it's off-center, it indicates a sideslip condition. 
```math
\begin{align*}
\beta = \arcsin{\frac{v}{V_T}}
\end{align*}
```
5. **Pitot-Static System**: True airspeed is the actual speed of the aircraft through the air, corrected for altitude and temperature ($V_{T}$). TAS is typically measured using a **pitot-static system, which includes:**

        a. Pitot Tube: This is a forward-facing tube on the aircraft that measures the pressure of the oncoming air. The dynamic pressure (ram air pressure) is used to determine the indicated airspeed (IAS).

        b. Static Ports: These are static pressure ports typically located on the sides of the aircraft. They measure the atmospheric pressure at the aircraft's current altitude.

        c. Air Data Computer (ADC): The pitot and static pressure measurements are fed into an ADC, which computes the TAS by correcting the IAS for altitude (pressure altitude) and temperature.

        d. Outside Air Temperature (OAT) Probe: Temperature is a factor in the TAS calculation, so an OAT probe is used to measure the temperature of the ambient air.


```math
\begin{align*}
V_{T} = \sqrt{u^2+v^2+w^2}
\end{align*}
```
###### Combining all these measurements, we may formulate a system output represented by the following equation:
```math
\begin{align*}
\vec{y} = (A_x, A_y, A_z, V_T, \beta, \alpha, p, q, r, \phi, \theta, \psi)
\end{align*}
```
with $n_y$ = 12 components.

## Forces:
1. Lift: The aerodynamic lift force is perpendicular to the vehicle's true airspeed $V_T$.

2. Drag: Drag force resists the vehicle's motion along the airspeed direction. 
```math
\begin{align*}
F_{x} = X_{a} + X_{T} \\
F_{y} = Y_{a} + Y_{T} \\
F_{z} = Z_{a} + Z_{T} \\
\end{align*}
```
## TODO: Add a picture representing the lift, and drag forces on an aircraft with the angle of attack, air density, 
The aerodynamic forces $(X_{a}, Y_{a}, Z_{a})$ can easily be written in terms of lift and drag as follows:
```math
\begin{aligned}
& X_{\mathrm{a}}=L \sin \alpha-D \cos \beta \cos \alpha \\
& Y_{\mathrm{a}}=D \sin \beta \\
& Z_{\mathrm{a}}=-L \cos \alpha-D \cos \beta \sin \alpha
\end{aligned}
```
## TODO: 

## General Equations of Motions and Challenges:
In General, the aircraft equations of motion represent a continuous dynamical multi-input multi-output system in the form:

```math
\begin{aligned}
& \dot{\vec{x}}=f(\vec{x}, \vec{u}) \\
& \vec{y}=h(\vec{x}, \vec{u})
\end{aligned}
```

with the state $\vec{x} \in R^9$, with control input $\vec{u} \in R^4$, and with the output $\vec{y} \in R^{12}$.

## Additional Equation of motion describing aircraft dynamics (Body-fixed velocities to Earth-fixed inertial velocities):

These are the three relations that connect the aircraft body-fixed
velocities (u, v, w) with the northeast-altitude inertial velocities $(\dot{x},\dot{y},\dot{h})$
```math
\begin{align*}
\left(\begin{array}{c}
\dot{x} \\
\dot{y} \\
-\dot{h}
\end{array}\right)=\left(\begin{array}{ccc}
1 & 0 & 0 \\
0 & \cos \varphi & \sin \varphi \\
0 & -\sin \varphi & \cos \varphi
\end{array}\right)\left(\begin{array}{ccc}
\cos \theta & 0 & -\sin \theta \\
0 & 1 & 0 \\
\sin \theta & 0 & \cos \theta
\end{array}\right)\left(\begin{array}{ccc}
\cos \psi & \sin \psi & 0 \\
-\sin \psi & \cos \psi & 0 \\
0 & 0 & 1
\end{array}\right)\left(\begin{array}{c}
u \\
v \\
w
\end{array}\right)
\end{align*}
```
The inertial speeds and positions are needed to design guidance algorithms for steering the vehicle along the prescribed trajectories. Also, these quantities become important during the landing and takeoff phases of flight. The three inertial velocities and positions can be added to the system output y. In that case, the aircraft dynamics would become 12-dimensional, with the extended state vector.

```math
\begin{align*}
\vec{X}=(\underbrace{u, v, w, p, q, r, \varphi, \theta, \psi}_{\vec{x}}, x, y, h)^T
\end{align*}
```
and with the redefined 18-dimensional system output
```math
\begin{align*}
\vec{Y}=(\underbrace{A_x, A_y, A_z, V_T, \beta, \alpha, p, q, r, \varphi, \theta, \psi}_{\vec{y}}, \dot{x}, \dot{y}, \dot{h}, x, y, h)
\end{align*}
```

Equations of motions described by https://github.com/Praful22/AircraftFlightDynamics#equations-of-motions would most likely result in an impractical control solution of unnecessary complexity and with an undesirable high sensitivity due to model parameters.

What does that mean?

Imagine you're trying to build a remote control for a model airplane. You could use a very detailed model of the airplane to make the remote control, but that might make things overly complicated and sensitive to small differences. So, here's the challenge: How much detail should you include in your model to make sure the remote control is easy to use, works well, and does what it's supposed to do? Well, the answer depends on what you want to achieve with your model airplane and remote control. What do we do then? 


## Simplified Flight Dynamics for Control Design:

The aircraft 6-DoF motion equations may be formulated into a steady state around an operating point called a trim point and dynamics of perturbations around the trim conditions. This formulation will help to reduce the overall nonlinearity of fully coupled 6-DoF aircraft dynamics into a tractable form allowing practical control design and analysis. Trimming an aircraft involves achieving an equilibrium between aerodynamic, propulsive, and gravitational forces and moments acting on the aircraft. An aircraft is trimmed by setting its primary control values that would result in the desired steady-state flight conditions.

Mathematically, a system equilibrium pair 
```math 
(\vec{x}_{eq},\vec{y}_{eq})
```
would zero out the translational and rotational acceleration in translational and rotational equations of motion such that:

```math
\text{Translational DoF}:
0=-\left[\left(\begin{array}{c}p \\ q \\ r\end{array}\right) \times\left(\begin{array}{c}u \\ v \\ w\end{array}\right)\right]+\left(\begin{array}{l}F_x \\ F_y \\ F_z\end{array}\right)+m \vec{g}
```

```math
\text{Rotational DoF}: 0 =-\left[\left(\begin{array}{c}p \\ q \\ r\end{array}\right) \times J\left(\begin{array}{c}p \\ q \\ r\end{array}\right)\right]+\left(\begin{array}{c}\bar{L} \\ M \\ N\end{array}\right)
```

Equivalently:
```math
0 = f(\vec{x}_{eq}, \vec{u}_{eq})
```
The trim points rely on altitude and airspeed and involve creating control-oriented models and designing flight control.

This process includes:

1. Establishing a dense set of trim points across the flight envelope.
2. Developing simplified linear models for each trim point.
3. Designing fixed-point flight controllers based on these models.
4. Combining linear controllers through interpolation for different flight conditions.
The outcome is a gain-scheduled flight control system applicable to the entire operational range.

In the Google Collab files that are included in this directory, we'll focus on Step 2, where we derive linear models for a specific trim point when a conventional aircraft is trimmed wings-level at certain flight conditions. When a typical aircraft is adjusted to maintain level flight with its wings even, and under specific flight conditions, the vehicle's dynamics naturally separate into two distinct modes: longitudinal and lateral-directional. We will derive each of these modes independently.

Link to Lateral Directional Dynamics: https://github.com/Praful22/AircraftFlightDynamics/blob/main/Lateral_Directional_Dynamics.ipynb 

Link to Longitudinal Dynamics: https://github.com/Praful22/AircraftFlightDynamics/blob/main/Longitudinal_Dynamics.ipynb 

## Create a SIMULINK Block for Aircraft Physical Model/Plant:

![image](https://github.com/Praful22/AircraftFlightDynamics/assets/65821250/73e903d6-094a-4b75-8cb3-1b6b8a8361bd)

![image](https://github.com/Praful22/AircraftFlightDynamics/assets/65821250/042f58f1-ed9e-4ca4-bac1-44fb3d326a48)




## Aerospace Toolbox and uses: Control Systems Toolbox, Optimization Toolbox, 
