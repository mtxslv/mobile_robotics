# Creating the Position Controller

## Assumptions
In order to create the position controller, let's assume we want to go from an actual point to a given one. The "actual point" is the one the robot is currently sitting on. This way of testing is useful for creating the class itself without having to worry how to find the nearest curve point.  

I think I might have come up with an way of creating the position controller, considering the idea of going from a point to another one (no curve's nearest point calculation involved). To do so, consider the robot is found in a current configuration $[x_{current}, y_{current}, \theta_{current}]^T$ and we want to send it to another config: $[x_{goal}, y_{goal}, \theta_{goal}]^T$.

## First Computations
First of all, it is necessary to calculate the position and angle difference:
* $\Delta x = x_{goal} - x_{current}$
* $\Delta y = y_{goal} - y_{current}$
* $\Delta L^* = \sqrt(\Delta x² + \Delta y²)$
* $\Delta \theta = \theta_{goal} - \theta_{current}$
* $\Delta L = cos(\Delta \theta) \Delta L^* $

## Virtual variables Control
Through PID, we can control u_omega and u_v, such that:
* $\Delta \theta$ -> [PID] -> $u_{\omega}$ -> [ROBOT] -> $\theta$
* $\Delta L$ -> [PID] -> $u_v$ -> [ROBOT] -> x,y

## Coupling/Decoupling Matrix
Naturally, we cannot input $u_{\omega}$ and $u_v$ directly to the robot. How do we do, then? We map them using the decoupling matrix:

$$
\begin{bmatrix} v \\ \omega \end{bmatrix} = 
\begin{bmatrix} r_r/2 & r_l/2 \\ r_r/b & -r_l/b \end{bmatrix} 
\begin{bmatrix} \omega_d \\ \omega_e \end{bmatrix}
$$

## Control Loop Explained
In general, we have:
* x*,y* -> [POSITION CONTROLLER] -> $\omega_r , \omega_l$ ->[ROBOT] -> $x,y, \theta$

Where:
* x*,y* -> [POSITION CONTROLLER] -> $\omega_r , \omega_l$  = x*,y* -> [ -> [variable reference computation] -> $\Delta L, \Delta \theta$ -> [decoupled controller] -> ] -> $\omega_r , \omega_l$  

The **decoupled controller** has a PID control for each input variable (as shown above on _Virtual Variables Control_). The variables resulting from the PID control are re-coupled using the matrix shown.

## A tip on fine-tuning
Another detail regards the PID controls. There is a PID control for each variable ( $u_{\omega}$ and $u_v$). They need to be fine-tuned. It is recommended to fine-tune the $u_{\omega}$ PID before $u_v$ PID, so the resulting curve will be smoother.

## A tip on P3DX decoupling matrix

According to (what looks like) [P3DX's official documentation](https://www.generationrobots.com/media/Pioneer3DX-P3DX-RevA.pdf), the wheel's radius equals 97.5 millimeters, while the distance between the wheels is 381 millimeters. Thus, numerically, the matrix equals:

$$
\begin{bmatrix} v \\ \omega \end{bmatrix} = 
\begin{bmatrix} 48.75 & 48.75 \\ 0.255905512 & -0.255905512 \end{bmatrix} 
\begin{bmatrix} \omega_d \\ \omega_e \end{bmatrix}
$$

What leads to...

$$
\begin{bmatrix} \omega_d \\ \omega_e \end{bmatrix} = 
\begin{bmatrix} 0.010 & 1.954 \\ 0.010 & -1.954 \end{bmatrix} 
\begin{bmatrix} v \\ \omega \end{bmatrix} 
$$