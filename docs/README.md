# Autonomous Parking System Documentation

## Project Overview

This project generates and visualizes a parking trajectory and its control
inputs while considering vehicle dynamics, state and input bounds, and
obstacles. The planner uses RRT* to initialize a collision-free geometric
path, then refines that path with an SQP-like sequence of quadratic programs
(QPs) solved by OSQP.

## Development Environment and Technologies

- **Language**: C++
- **Libraries**: Eigen, OSQP, OsqpEigen
- **Tools**: VS Code, Docker, CMake
- **Environment**: Ubuntu 22.04

## Algorithm

The planning pipeline has two stages:

1. **RRT*** generates a geometric path in $(x, y, \theta)$ space. Collision
   checking models the vehicle as a circle whose radius is half of the
   vehicle diagonal. The path is then resampled to the prediction horizon.
2. **Iterative QP refinement** starts from that path and repeatedly solves a
   QP built from a local linearization of the vehicle model and obstacle
   constraints. A merit-function line search determines how much of the QP
   update is applied.

RRT* is a geometric planner in this implementation; it does not enforce the
vehicle's full nonlinear dynamics or its velocity and steering limits.

## System Model

The vehicle state and input are

$$
\mathbf{x}(t) =
\begin{bmatrix}
x(t) & y(t) & \theta(t) & v(t) & \delta(t)
\end{bmatrix}^{T},
\qquad
\mathbf{u}(t) =
\begin{bmatrix}
a(t) & \dot{\delta}(t)\end{bmatrix}^{T}.
$$

The five-state kinematic bicycle model used by `SystemModel` is

$$
\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u}) =
\begin{bmatrix}
v\cos\theta \\
v\sin\theta \\
\dfrac{v\tan\delta}{L} \\
a \\
\dot{\delta}
\end{bmatrix},
$$

where $L$ is the configured vehicle length. The model is discretized with a
second-order approximation. Its Jacobians and affine term are returned as
$\mathbf{A}_k$, $\mathbf{B}_k$, and $\mathbf{g}_k$:

$$
\mathbf{x}_{k+1} \approx
\mathbf{A}_k\mathbf{x}_k +
\mathbf{B}_k\mathbf{u}_k +
\mathbf{g}_k.
$$

## Optimization Problem

Let $N = \lceil T / \Delta t \rceil$. The QP decision vector contains all
state and input increments, followed by goal and obstacle slack variables:

$$
\mathbf{z} =
\begin{bmatrix}
\Delta\mathbf{x}_0 \\ \vdots \\ \Delta\mathbf{x}_N \\
\Delta\mathbf{u}_0 \\ \vdots \\ \Delta\mathbf{u}_{N-1} \\
\mathbf{s}_{\mathrm{goal}} \\
\mathbf{s}_{\mathrm{obs}}
\end{bmatrix}.
$$

There are $5(N+1)$ state variables, $2N$ input variables, five goal-slack
variables, and one obstacle-slack variable for every obstacle at every state
sample. The current nominal trajectory is denoted by $\bar{\mathbf{x}}_k$
and $\bar{\mathbf{u}}_k$.

### Objective

The implementation uses a quadratic objective of the form

$$
\min_{\mathbf{z}}
\quad
\frac{1}{2}\mathbf{z}^{T}\mathbf{H}\mathbf{z}
 + \mathbf{q}^{T}\mathbf{z},
$$

where the configured diagonal state and input weights penalize the nominal
state and input values plus their QP increments. The goal and obstacle slack
variables are also penalized quadratically:

$$
\mathbf{H}_{\mathrm{goal}} =
\rho_{\mathrm{goal}}\mathbf{I}_5,
\qquad
\mathbf{H}_{\mathrm{obs}} =
\rho_{\mathrm{obs}}\mathbf{I}.
$$

Thus, the implemented cost is not a continuous-time integral of linear slack
penalties. The configuration values `rho_goal` and `rho_obs` control the
quadratic slack penalties.

### Equality Constraints

The initial-state increment is fixed so that the updated initial state stays
at the configured initial state. For each $k = 0, \ldots, N-1$, the
linearized dynamics constraint is

$$
\Delta\mathbf{x}_{k+1}
- \mathbf{A}_k\Delta\mathbf{x}_k
- \mathbf{B}_k\Delta\mathbf{u}_k
= \mathbf{A}_k\bar{\mathbf{x}}_k
 + \mathbf{B}_k\bar{\mathbf{u}}_k
 + \mathbf{g}_k
 - \bar{\mathbf{x}}_{k+1}.
$$

The final state is softly tied to the configured goal by the five-dimensional
goal slack:

$$
\Delta\mathbf{x}_N + \mathbf{s}_{\mathrm{goal}}
= \mathbf{x}_{\mathrm{goal}} - \bar{\mathbf{x}}_N.
$$

The goal slack is quadratically penalized but is not constrained to be
non-negative in the current QP.

There is no final-input equality constraint in the current implementation.
Inputs are defined only for $k = 0, \ldots, N-1$ and are constrained by their
bounds.

### State and Input Bounds

State and input bounds are imposed on the updated values:

$$
\mathbf{x}_{\min} - \bar{\mathbf{x}}_k
\leq \Delta\mathbf{x}_k
\leq \mathbf{x}_{\max} - \bar{\mathbf{x}}_k,
$$

$$
\mathbf{u}_{\min} - \bar{\mathbf{u}}_k
\leq \Delta\mathbf{u}_k
\leq \mathbf{u}_{\max} - \bar{\mathbf{u}}_k.
$$

Only velocity and steering angle are bounded in $\mathbf{x}_{\min}, \mathbf{x}_{\max}$;
position and heading are left unconstrained. The input bounds cover
acceleration and steering-rate limits.

### Obstacle Constraints

For each obstacle and each state sample, the code finds the closest point on
the obstacle rectangle to the current $(x,y)$ position. Let $d_k$ be that
distance and $\mathbf{n}_k$ its planar unit direction. The local constraint is

$$
\mathbf{n}_k^T
\begin{bmatrix}
\Delta x_k \\ \Delta y_k
\end{bmatrix}
 + s_{\mathrm{obs},k}
\geq r_{\mathrm{vehicle}} + m - d_k,
$$

with

$$
s_{\mathrm{obs},k} \geq 0,
\qquad
r_{\mathrm{vehicle}} = \frac{1}{2}\sqrt{L^2 + W^2}.
$$

Here $m$ is the configured safety margin. The obstacle constraint is a local
linearization of point-to-rectangle distance; it is not an exact constraint on
the full oriented vehicle footprint. One obstacle slack variable is created
for each obstacle and state sample.

## SQP-like Update

At every outer iteration, the optimizer builds the QP around the current
nominal trajectory and solves it with OSQP. A merit function combines the QP
objective with constraint violation. The accepted update is

$$
\mathbf{w}_{\mathrm{new}}
= \mathbf{w}_{\mathrm{current}}
+ \alpha\,\Delta\mathbf{w},
\qquad 0 < \alpha \leq 1,
$$

where $\alpha$ is reduced by backtracking when the merit function does not
improve. Iteration stops when the maximum or mean norm of the decision
increment is below the configured implementation thresholds, or when the
configured number of SQP iterations is reached. The increment thresholds are
currently fixed in the implementation; the maximum iteration count is read
from the YAML configuration.

## Results

![demo3](assets/demo/demo3.gif)

The generated trajectory can be simulated with the returned state and input
sequences. Scenarios and planner parameters, including vehicle dimensions,
state/input bounds, penalties, safety margin, and obstacle geometry, are
configured through YAML files.
