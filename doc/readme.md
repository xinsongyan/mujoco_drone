# mujoco_drone


## Kinematics
Rotors are arraged as shown:

![alt text](drone_model.drawio.svg)

![alt text](drone_model_3d_air.drawio.svg)

$\omega_i$: angular velocity of rotor $i$

$f_i$: force created by rotor around its axis,  $f_i=k_f \omega_i^2$

$\tau_i$: torque created by the robot around its axis, $\tau_{i}=k_{\tau} \omega_i^2$




## Dynamics
### Force and Moments
The resultant force applied to the body w.r.t. its own frame are:

$`
\mathbf{T}^B=\left[\begin{array}{l}
0 \\
0 \\
T
\end{array}\right]=\left[\begin{array}{c}
0 \\
0 \\
\sum_{i=1}^4 f_i
\end{array}\right]=\left[\begin{array}{c}
0 \\
0 \\
k_f \sum_{i=1}^4 \omega_i^2
\end{array}\right]
`$

$`
M^B=\left[\begin{array}{c}
\tau_x \\
\tau_y \\
\tau_z
\end{array}\right]=\left[\begin{array}{c}
-f_1 \cdot d-f_2 \cdot d+f_3 \cdot d+f_4 \cdot d \\
f_1 \cdot d-f_2 \cdot d+f_3 \cdot d-f_4 \cdot d \\
\tau_1-\tau_2-\tau_3+\tau_4
\end{array}\right]=\left[\begin{array}{c}
d k_f\left(-\omega_1^2-\omega_2^2+\omega_3^2+\omega_4^2\right) \\
d k_f\left(\omega_1^2-\omega_2^2+\omega_3^2-\omega_4^2\right) \\
k_\tau\left(\omega_1^2-\omega_2^2-\omega_3^2+\omega_4^2\right)
\end{array}\right]
`$

where $d$ is the distance from the rotor center to x-axis or y-aixs, here assuming they are the same.

In simulation, we directly control the force of the rotor instead of the rotational velocity, the formulation is slightly different: 
$`
\begin{aligned}
T & =f_1+f_2+f_3+f_4 \\
\tau_x & =d_x\left(-f_1-f_2+f_3+f_4\right) \\
\tau_y & =d_y\left(f_1-f_2+f_3-f_4\right) \\
\tau_z & =k\left(f_1-f_2-f_3+f_4\right)
\end{aligned}
`$

the inverse mapping of it is: 

$`
\begin{aligned}
& f_1=\frac{1}{4}\left(T-\frac{\tau_x}{d_x}+\frac{\tau_y}{d_y}+\frac{\tau_z}{k}\right) \\
& f_2=\frac{1}{4}\left(T-\frac{\tau_x}{d_x}-\frac{\tau_y}{d_y}-\frac{\tau_z}{k}\right) \\
& f_3=\frac{1}{4}\left(T+\frac{\tau_x}{d_x}+\frac{\tau_y}{d_y}-\frac{\tau_z}{k}\right) \\
& f_4=\frac{1}{4}\left(T+\frac{\tau_x}{d_x}-\frac{\tau_y}{d_y}+\frac{\tau_z}{k}\right)
\end{aligned}
`$

This is what we used in simulation to map the thrust force and body moment back to motor forces and torques.

### Equation of Motion

In the inertial frame, the acceleration of the quadrotor is due to thrust, gravity. We can obtain the thrust force in the inertial frame by using the rotation matrix $\mathbf{R}^I_{B}$ to map it from the body frame to the inertial frame. Thus, the linear part of the equation of motion is:

$`
m\ddot{\mathbf{p}} = \mathbf{T} + \mathbf{G}
`$

$`
m \ddot{\mathbf{p}}=
\mathbf{R}_B^I \mathbf{T}^B + 
m\left[\begin{array}{c}
0 \\
0 \\
-g
\end{array}\right]
`$

where $\mathbf{p}$ is the position of the quadrotor in the inertial frame, $g$ is the gravity acceleration, and $\mathbf{f}$ is the thrust force in inertia frame and $\mathbf{f}^B$ is the thrust force in the body frame.

While it is convenient to have the linear equations of motion in the inertial frame, the rotational equations of motion is simpler to derive in the body frame: 

$`
\mathbf{I}^B \dot{\omega}^B+\omega^B \times\left(\mathbf{I}^B \omega^B\right)=\tau^B
`$

where $\omega^B$ is the angular velocity vector in the body frame, $I^B$ is the inertia matrix in the body frame, and $\tau^B$ is resultant torque from the rotors in the body frame. For drone with small angular velocity, it can be further simplifed since $\boldsymbol{\omega} \times(I \boldsymbol{\omega}) \approx 0$:

$`
\mathbf{I}^B \dot{\boldsymbol{\omega}}^B=\tau^B
`$

## PD control
The quadrotor has six states, three positions and three angles, but only four control inputs, the angular velocities of the four rotors. It is an underactuated system. 
We can choose to control the total thrust and torques in its body frame. They can be easily mapped back to the four rotor speeds. 
With the torques and total thrust in the body frame, we can design a simple controller to make the robot stabilise in the air: 

$`
\begin{aligned}
T & =\left(g+K_{z, D}\left(\dot{z}_d-\dot{z}\right)+K_{z, P}\left(z_d-z\right)\right) \frac{m}{C_\phi C_\theta}, \\
\tau_\phi & =\left(K_{\phi, D}\left(\dot{\phi}_d-\dot{\phi}\right)+K_{\phi, P}\left(\phi_d-\phi\right)\right) I_{x x}, \\
\tau_\theta & =\left(K_{\theta, D}\left(\dot{\theta}_d-\dot{\theta}\right)+K_{\theta, P}\left(\theta_d-\theta\right)\right) I_{y y}, \\
\tau_\psi & =\left(K_{\psi, D}\left(\dot{\psi}_d-\dot{\psi}\right)+K_{\psi, P}\left(\psi_d-\psi\right)\right) I_{z z},
\end{aligned}
`$

The mapping between the control variable and rotor angular speeds can be derived as:

$`
\begin{gathered}
T=k\left(\omega_1^2+\omega_2^2+\omega_3^2+\omega_4^2\right) \\
\tau_x=k d\left(-\omega_1^2-\omega_2^2+\omega_3^2+\omega_4^2\right) \\
\tau_y=k d\left(\omega_1^2-\omega_2^2+\omega_3^2-\omega_4^2\right) \\
\tau_z=b\left(\omega_1^2-\omega_2^2-\omega_3^2+\omega_4^2\right)
\end{gathered}
`$

solving the system, we have:

$`
\begin{gathered}
 \omega_1^2=\frac{1}{4}\left(\frac{T}{k}-\frac{\tau_x}{k d}+\frac{\tau_y}{k d}+\frac{\tau_z}{b}\right) \\
 \omega_2^2=\frac{1}{4}\left(\frac{T}{k}-\frac{\tau_x}{k d}-\frac{\tau_y}{k d}-\frac{\tau_z}{b}\right) \\
 \omega_3^2=\frac{1}{4}\left(\frac{T}{k}+\frac{\tau_x}{k d}+\frac{\tau_y}{k d}-\frac{\tau_z}{b}\right) \\
 \omega_4^2=\frac{1}{4}\left(\frac{T}{k}+\frac{\tau_x}{k d}-\frac{\tau_y}{k d}+\frac{\tau_z}{b}\right)
 \end{gathered}
`$

## SE(3) control

[1] Lee, Taeyoung, Melvin Leok, and N. Harris McClamroch. "Geometric tracking control of a quadrotor UAV on SE (3)." 49th IEEE conference on decision and control (CDC). IEEE, 2010.

[2] Yun Yu, Shuo Yang, Mingxi Wang, Cheng Li, and Zexiang Li, ‘High performance full attitude control of a quadrotor on SO(3)’, in 2015 IEEE International Conference on Robotics and Automation (ICRA), Seattle, WA, USA: IEEE, May 2015, pp. 1698–1703. doi: 10.1109/ICRA.2015.7139416.



* Position Controller on SO(3)

Given desired position, we can calculate required acceleration to reach the target:
$$
\mathbf{a}_{cmd} = k_p(\mathbf{p}^*-\mathbf{p})+k_d(\mathbf{v}^*-\mathbf{v})+\mathbf{a}^*
$$
based on the Newton's Equation:
$$
\mathbf{T}_{cmd} = m\mathbf{a}_{cmd} - m\mathbf{g}
$$

then 
$$
\mathbf{T}_{cmd}^B = \mathbf{R}_I^B \mathbf{T}_{cmd}
$$
The thrust would be: 
$$
\mathbf{T}_{z,cmd}^B = \mathbf{T}_{cmd}^B \cdot [0, 0, 1]^T
$$

or can be simply the projection of $\mathbf{T}_{cmd}$ on to the $z$ axis of the body frame: 
$
\mathbf{T}_{z,cmd}^B = \mathbf{T}_{cmd} \cdot \mathbf{z}_B^I
$

* Attitude Controller on SO(3)

The desired orientation is determined by the direction of the command force $\mathbf{f}_{cmd}$ and the desired heading direction $\mathbf{h}_d$, 

The desired $z$ axis is simply the normalized thrust force: 
$$
\mathbf{z}_d = \hat{\mathbf{f}}_{cmd}
$$

The $y$ axis can be defined as: 
$$
\mathbf{y}_d = \mathbf{z}_d \times \hat{\mathbf{h}}_d 
$$
The $x$ axis is then defiend as: 
$$
\mathbf{x}_d = \mathbf{y}_d \times\mathbf{z}_d
$$

The final desired orientation matrix can be assembed as: 

$`
\mathbf{R}_d = \left[\begin{array}{ccc}
\mid & \mid & \mid \\
\mathbf{x}_d & \mathbf{y}_d & \mathbf{z}_d \\
\mid & \mid & \mid
\end{array}\right]
`$



Then the moment command is:

$`
\tau=K_P\log (R^TR_d) + K_D (^b\omega_d-^b\omega) + ^b\omega \times ^bJ ^b\omega
`$

And log is calculated as:

$`
\log (R)=\frac{\phi}{2 \sin \phi}\left(R-R^T\right)^{\vee} \quad \in \mathfrak{s o}(3)
`$

in which $\phi$ is $\cos ^{-1}\left(\frac{\operatorname{tr}(R)-1}{2}\right)$ and $|\phi|<\pi$. 


3. Total Control Inputs
- Thrust: $\mathbf{T}_{z}^B$
- Moment: $\mathbf{M}^B$


# Ground mode
When the drone rolls on the ground, an additional ground reaction force is exerted on it through the contact between the cage and the ground. 
<!-- ![alt text](drone_model_3d_ground.drawio.svg) -->
<img src="drone_model_3d_ground.drawio.svg" width="50%"/>

The dynamics is updated: 

$`
m \ddot{\mathbf{p}}=\mathbf{T}+\mathbf{G}+\mathbf{F}_C
`$

The rotational part is also upated: 

$`
\mathbf{I} \dot{\omega}+\omega \times\left(\mathbf{I}\omega\right)=\mathbf{M} + \mathbf{r}_C \times \mathbf{F}_C
`$

However, the small angular velocity assumption no longer holds in gorund mode, as the robot relies on rotation for movement.
We need to take the byroscopic term into consideration.

In terms of control, the robot has 5 DoFs (2 translations and 3 rotations), however, the two translational DoFs are coupled with two orientational DoFs due to nonholonomic rolling constraints. 

The nonholonomic rolling constraints could be expressed as zero contact point velocity in inertial frame: 

$`
\mathbf{v}_{\text {contact }}=\dot{\mathbf{p}}+\boldsymbol{\omega} \times(-R \hat{z})=\dot{\mathbf{p}}-R \boldsymbol{\omega} \times \hat{z} = 0
`$

where $\mathbf{p}$ is the location of the body frame, $R$ is the radius of the sphere cage. 
The equation indicates that no slip in x, y direction and no bump in z direction. Another implecit assumption made here is that the ground is flat so that the contact point in right below the center of mass of the robot. 

The relation can be simplified as: 

$`
{\mathbf{v}}=R \omega \times \hat{{z}}
`$

This can be solved as: 

$`
\begin{aligned}
& v_x=R \omega_y \\
& v_y=-R \omega_x
\end{aligned}
`$

Our primary goal is navigation, so we choose to control the two translational DoFs instead of the rotational DoFs. 

Given desired location $x_d$, $y_d$, the desired position can be $\mathbf{p}_d =[x_d, y_d, R] $.

Based on the desired location, we can calculate the required command acceleration to reach the target:
$$
\mathbf{a}_{cmd} = k_p(\mathbf{p}_{des}-\mathbf{p})+k_d(\mathbf{v}_{des}-\mathbf{v})+\mathbf{a}_{des}
$$

Based on the Newton's Equation,

$`
\mathbf{T}_{cmd} + \mathbf{F}_{C,cmd} = m \mathbf{a}_{cmd} - G
`$

as we know the desired location and current location, we can have: 

$`
{\mathbf{v}}_{cmd} = k_p(\mathbf{p}_{des} - \mathbf{p}) + k_d(\mathbf{v}_{des}-\mathbf{v})
`$

then from the nonholonomic constraints $`
{\mathbf{v}}=R \omega \times \hat{{z}}
`$, the commanded angular velocity can be calculated: 

$`
\begin{aligned}
& \omega_{x,cmd} = - v_{y,cmd}/R \\ 
& \omega_{y,cmd}=v_{x,cmd}/R  
\end{aligned}
`$

The command moment can be:

$`
M=-k_{\omega} {\omega}_{cmd}+\omega \times \mathbf{I} \omega
`$