# AircraftFlightDynamics

Let's start by looking at **an aircraft and its controls.**

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
The three Euler angles $(\phi, \theta, \psi)$ give the inertial angular orientation of aircraft, if treated as a rigid body moving in three-dimensional
inertial space.





