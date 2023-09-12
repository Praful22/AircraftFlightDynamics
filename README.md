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
The three Euler angles $(\phi, \theta, \psi)$ represent the inertial angular orientation of aircraft, if treated as a 
rigid body moving in three-dimensional inertial space.

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

1. Ailerons :


2. Horizontal Stabilizers : 
3. Elevator : 
4. Rudder : 
![image](https://github.com/Praful22/AircraftFlightDynamics/assets/65821250/8cbff590-f9a5-473d-ba44-37aa40ac6f50)

If the aircraft is not a glider then another primary control input will be 

5. Thrust force $(\delta_th)$ created by either propellers or jet engines.



