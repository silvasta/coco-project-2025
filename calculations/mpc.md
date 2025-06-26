# MPC

## Reference Tracking

$$
\begin{aligned}
& \operatorname{min} u_s^TR_su_s
\\
x_s & = Ax_s + Bu_s
\\
Hx_s & = r
\end{aligned}
$$

if no solution exists, compute closest point to r:

$$
\begin{aligned}
& \operatorname{min} (Hx_s-r)^TQ_s(Hx_s-r)
\\
x_s & = Ax_s + Bu_s
\\
Hx_s & = r
\end{aligned}
$$

## Delta Formulation

$\Delta x(k)=x(k)-x_s$

$$
\begin{aligned}
\operatorname{min} \sum_{i=0}^{N-1}&
\Delta x_i^TQ_s\Delta x_i +\Delta u_i^TR_s\Delta u_i + V_f(\Delta x_N)
\\
\text{s.t. }
\Delta x_0 &=\Delta x(k)
\\
\Delta x_{i+1} & = A\Delta x_i + B\Delta u_i
\\
G_x\Delta x_i &\le h_x - G_xx_s
\\
G_u\Delta u_i &\le h_u - G_uu_s
\\
\Delta x_N &\in X_f
\end{aligned}
$$

## Constant disturbances

$$
\begin{aligned}
x(k+1)&=Ax(k)+Bu(k)+B_dd
\\
y(k)&=Cx(k)+C_dd
\end{aligned}
$$

## Offset-free control

$$
x_s=Ax_s+Bu_s+B_dd
\\
Cx_s+C_dd = r
$$
