# Linear System Simulator

This target provides various simulation methods for the linear time-invariant (LTI) system defined by

$$
\begin{align}
    \dot{x} &= Ax + Bu + Ev \\
    y &= Cx + Du + Fw
\end{align}
$$

where

- $x\in\mathbb{R}^n$ is the state,
- $u\in\mathbb{R}^m$ is the control input,
- $y\in\mathbb{R}^p$ is the output,
- $v\in\mathbb{R}^q$ is the disturbance, and
- $w\in\mathbb{R}^r$ is the measurement noise.

## Simulation 1

The **simulation1** solves the LTI system (1) and (2) on a finite time interval when $D = 0$, $E=0$, and $F=0$:

$$
\begin{align}
    \dot{x} &= Ax + Bu \\
    y &= Cx
\end{align}
$$

The reference signal $r:\mathbb{R}\to\mathbb{R}^p$ for the output is given as a step function defined by

$$
\begin{align}
    r(t) &= \begin{cases}0, & t < s_0 \\ r_0, & t\ge s_0\end{cases} \\
    \dot{r}(t) &= 0,\quad t\in\mathbb{R}
\end{align}
$$

Then, we apply the PID controller for the control input $u$

$$
\begin{equation}
    u(t) = K_pe(t) + K_d\dot{e}(t) + K_i\int_{t_0}^te(s)ds
\end{equation}
$$

where $e(t) = r(t) - y(t)$.

For the numerical simulation, we update the variables by using the **Euler method**. Let $[t_0,t_1]\subset\mathbb{R}$ be the simulation time interval, $t_s>0$ be the fixed sampling time to update the simulation, and $x_0\in\mathbb{R}^n$ be the initial value of the state $x$ at $t=t_0$. Then, the simulation update rule is given as:

$$
\begin{align}
    x(t_0) &= x_0 \\
    y(t_0) &= Cx_0 \\
    \dot{y}(t_0) &= 0 \\
    r(t_0) &= \begin{cases}0, & t_0 < s_0 \\ r_0, & t_0\ge s_0\end{cases} \\
    e(t_0) &= r(t_0) - y(t_0) \\
    \dot{e}(t_0) &= \int_{t_0}^{t_0}e(s)ds = 0 \\
    u(t_0) &= K_pe(t_0)
\end{align}
$$

for $i=0$ and

$$
\begin{align}
    t_i &= t_{i-1} + t_s = t_0 + it_s \\
    x(t_i) &= x(t_{i-1}) + t_s\big(Ax(t_{i-1}) + Bu(t_{i-1})\big) \\
    y(t_i) &= Cx(t_i) \\
    \dot{y}(t_i) &= \frac{y(t_i) - y(t_{i-1})}{t_s} \\
    r(t_i) &= \begin{cases}0, & t_i < s_0 \\ r_0, & t_i\ge s_0\end{cases} \\
    e(t_i) &= r(t_i) - y(t_i) \\
    \dot{e}(t_i) &= -\dot{y}(t_i) \\
    \int_{t_0}^{t_i}e(s)ds &= \int_{t_0}^{t_{i-1}}e(s)ds + t_se(t_{i-1}) \\
    u(t_i) &= K_pe(t_i) + K_d\dot{e}(t_i) + K_i\int_{t_0}^{t_i}e(s)ds
\end{align}
$$

for $i=1,2,\dots,N$ where $N = \lfloor(t_1-t_0)/t_s\rfloor$.