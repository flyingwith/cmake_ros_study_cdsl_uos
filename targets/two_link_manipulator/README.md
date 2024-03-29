# Two-Link Manipulator

This target provides various simulation methods for the two-link manipulator whose dynamics equation is defined by

$$
\begin{equation}
    M(q)\ddot{q} + c(q,\dot{q}) + g(q) + \tau_d(t,q,\dot{q}) = \tau
\end{equation}
$$

where

- $q = (q_1,q_2)\in\mathbb{R}^2$ is the joint position,
- $\tau = (\tau_1,\tau_2)\in\mathbb{R}^2$ is the joint torque,
- $M(q)\in\mathbb{R}^{2\times 2}$ is the inertia matrix,
- $c(q,\dot{q})\in\mathbb{R}^2$ is the coriolis and centrifugal term,
- $g(q)\in\mathbb{R}^2$ is the gravitational term, and
- $\tau_d(t,q,\dot{q})$ is the disturbance term generated by, e.g., the friction.

## Simulation 1

### Two-Link Manipulator

The **simulation1** assumes that the mass of each link is concentrated at the distal end of each link (see Section 6.8 of [Introduction to Robotics: Mechanics and Control](https://www.amazon.com/Introduction-Robotics-Mechanics-Control-3rd/dp/0201543613) by Craig). Then, we have

$$
\begin{align}
    M(q) &= \begin{bmatrix} l_2^2m_2 + 2l_1l_2m_2\cos(q_2) + l_1^2(m_1+m_2) & l_2^2m_2+l_1l_2m_2\cos(q_2) \\ l_2^2m_2+l_1l_2m_2\cos(q_2) & l_2^2m_2 \end{bmatrix} \\
    c(q,\dot{q}) &= \begin{bmatrix} -m_2l_1l_2\sin(q_2)\dot{q}_2^2 - 2m_2l_1l_2\sin(q_2)\dot{q}_1\dot{q}_2 \\ m_2l_1l_2\sin(q_2)\dot{q}_1^2 \end{bmatrix} \\
    g(q) &= \begin{bmatrix} m_2l_2{\sf g}\cos(q_1+q_2) + (m_1+m_2)l_1{\sf g}\cos(q_1) \\ m_2l_2{\sf g}\cos(q_1+q_2) \end{bmatrix}
\end{align}
$$

where $m_i$ and $l_i$ are the mass and the length, respectively, of the $i$-th link for $i=1,2$ and ${\sf g}$ is the gravitational constant. We also assume that there is **no disturbance**, i.e.,

$$
\begin{equation}
    \tau_d(t,q,\dot{q}) \equiv 0
\end{equation}
$$

### Dynamical System

Let $x = (x_1,x_2) = (q,\dot{q})\in\mathbb{R}^4$, $u = \tau\in\mathbb{R}^2$, and $y = x_1 = q\in\mathbb{R}^2$. Then, we can rewrite the manipulator dynamics as

$$
\begin{equation}
    \dot{x}_2 = \underbrace{-M^{-1}(x_1)\big(c(x_1,x_2) + g(x_1)\big)}_{\kappa(x)} + \underbrace{M^{-1}(x_1)}_{J(x)}u
\end{equation}
$$

and define the dynamical system for the manipulator as

$$
\begin{align}
    \dot{x} &= \underbrace{\begin{bmatrix} x_2 \\ \kappa(x) \end{bmatrix}}_{f(x)} + \underbrace{\begin{bmatrix} 0 \\ J(x) \end{bmatrix}}_{G(x)}u \\
    y &= \underbrace{x_1}_{h(x)}
\end{align}
$$

### Input-Output Feedback Linearization

Let's define the control input as

$$
\begin{equation}
    u = J^{-1}(x)(v - \kappa(x)) = M(x_1)v + c(x_1,x_2) + g(x_1)
\end{equation}
$$

where $v\in\mathbb{R}^2$ is the external reference control input that can be chosen arbitrarily. By applying this control input to the dynamical system, we can linearize the input-output relation of the dynamical system as

$$
\begin{align}
    \dot{x} &= Ax + Bv \\
    y &= Cx
\end{align}
$$

where

$$
\begin{align}
    A &= \begin{bmatrix} 0 & I_2 \\ 0 & 0 \end{bmatrix} \in \mathbb{R}^{4\times 4} \\
    B &= \begin{bmatrix} 0 \\ I_2 \end{bmatrix} \in \mathbb{R}^{4\times 2} \\
    C &= \begin{bmatrix} I_2 & 0 \end{bmatrix} \in \mathbb{R}^{2\times 4}
\end{align}
$$

This technique is called the **input-output feedback linearization**.

### Output Tracking Control

The reference signal $r:\mathbb{R}\to\mathbb{R}^2$ for the output is given as a step function defined by

$$
\begin{align}
    r(t) &= \begin{cases}0, & t < s_0 \\ r_0, & t\ge s_0\end{cases} \\
    \dot{r}(t) &= 0,\quad t\in\mathbb{R} \\
    \ddot{r}(t) &= 0,\quad t\in\mathbb{R}
\end{align}
$$

Then, we design the external reference control input $v$ of the linearized system as

$$
\begin{equation}
    v(t) = \ddot{r}(t) + K_d\dot{e}(t) + K_pe(t)
\end{equation}
$$

where $e(t) = r(t) - q(t)$. By applying this $v$ to the input-output linearized system, we can formulate the error dynamics as

$$
\begin{equation}
    \ddot{e}(t) + K_d\dot{e}(t) + K_pe(t) = 0
\end{equation}
$$

This technique is called the **computed torque method** (see Chapter 10 of [Introduction to Robotics: Mechanics and Control](https://www.amazon.com/Introduction-Robotics-Mechanics-Control-3rd/dp/0201543613) by Craig).

### Modeling Uncertainty

In practice, we do not know the manipulator dynamics accurately. For example, the manipulator parameters $l_1$, $l_2$, $m_1$, and $m_2$ could be inaccurate. Let $\tilde{l}_1$, $\tilde{l}_2$, $\tilde{m}_1$, and $\tilde{m}_2$ be the estimated parameters we know. Then, the control input has to be calculated by using these estimated values as

$$
\begin{equation}
    u = \tilde{J}^{-1}(x)(v - \tilde{\kappa}(x)) = \tilde{M}(x_1)v + \tilde{c}(x_1,x_2) + \tilde{g}(x_1)
\end{equation}
$$

where $\tilde{M}(x)$, $\tilde{c}(x_1,x_2)$, and $\tilde{g}(x_1)$ are defined similarly to $M(x)$, $c(x_1,x_2)$, and $g(x_1)$, respectively, except that the estimated parameters are used.

### Numerical Simulation

For the numerical simulation, we update the variables by using the **Euler method**. Let $[t_0,t_1]\subset\mathbb{R}$ be the simulation time interval, $t_s>0$ be the fixed sampling time to update the simulation, and $x_0 = (q_0, \dot{q}_0)\in\mathbb{R}^4$ be the initial value of the state $x$ at $t=t_0$. Let's denote $M(t) := M(q(t))$, $c(t) := c(q(t),\dot{q}(t))$, and $g(t) := g(q(t))$. Then, the simulation update rule is given as:

$$
\begin{align}
    q(t_0) &= q_0 \\
    \dot{q}(t_0) &= \dot{q}_0 \\
    r(t_0) &= \begin{cases}0, & t_0 < s_0 \\ r_0, & t_0\ge s_0\end{cases} \\
    e(t_0) &= r(t_0) - q(t_0) \\
    \dot{e}(t_0) &= -\dot{q}(t_0) \\
    v(t_0) &= K_d\dot{e}(t_0) + K_pe(t_0) \\
    \tau(t_0) &= \tilde{M}(t_0)v(t_0) + \tilde{c}(t_0) + \tilde{g}(t_0)
\end{align}
$$

for $i=0$ and

$$
\begin{align}
    t_i &= t_{i-1} + t_s = t_0 + it_s \\
    q(t_i) &= q(t_{i-1}) + t_s\dot{q}(t_{i-1}) \\
    \dot{q}(t_i) &= \dot{q}(t_{i-1}) + t_sM^{-1}(t_{i-1})\big(\tau(t_{i-1}) - c(t_{i-1}) - g(t_{i-1})\big) \\
    r(t_i) &= \begin{cases}0, & t_i < s_0 \\ r_0, & t_i\ge s_0\end{cases} \\
    e(t_i) &= r(t_i) - q(t_i) \\
    \dot{e}(t_i) &= -\dot{q}(t_i) \\
    v(t_i) &= K_d\dot{e}(t_i) + K_pe(t_i) \\
    \tau(t_i) &= \tilde{M}(t_i)v(t_i) + \tilde{c}(t_i) + \tilde{g}(t_i)
\end{align}
$$

for $i=1,2,\dots,N$ where $N = \lfloor(t_1-t_0)/t_s\rfloor$.