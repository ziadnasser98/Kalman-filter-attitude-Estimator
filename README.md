# Kalman-filter-attitude-Estimator
An Implementation of three algorithms for Estimating the Attitude of a rigid body from IMU sensor data depending on Kalman filter.

## the first method
This method uses unit quaternions as a representation of the attitude of the rigid body. 

The state vector $\(\mathbf{x}\)$ is defined as the vector (imaginary) part of the error quaternion $\( q_{\text{error}} \)$. The error quaternion $\( q_{\text{error}} \)$ is obtained by multiplying the real quaternion $\( q_{\text{real}} \)$ by the conjugate of the estimated quaternion $\( q_{\text{estimated}} \)$:

$$q_{\text{error}} = q_{\text{real}} \cdot q_{\text{estimated}}^{-1}$$

Thus, the state vector $\( \mathbf{x} \)$ is given by:

$$\mathbf{x} = \text{Im}(q_{\text{error}}) = \text{Im}(q_{\text{real}} \cdot q_{\text{estimated}}^{-1})$$

The state equation of the system is defined as follows:

$$x_k = F_k \cdot x_{k-1} + G_{k} \cdot w_k$$

Where:

$$
F_k = \exp(\Delta t \cdot \begin{pmatrix}
0 & -\omega_3 & \omega_2 \\
\omega_3 & 0 & -\omega_1 \\
-\omega_2 & \omega_1 & 0
\end{pmatrix})
$$

$$ G_k = \frac{-\Delta t}{2} \cdot I_{3\times 3}
$$

$` \Delta t`$ is the time difference between each two consecutive readings and $` \omega_k =(\omega_1,\omega_2,\omega_3)`$ is the gyroscope reading at time step number $k$.

The measurement equation of the system is defined as follows:

$$
z_k = H_k \cdot x_k + v_k
$$

Where:

$$
H_k = \begin{bmatrix}
2\cdot \begin{bmatrix} C_n^{\text{ }b-} \cdot f_k^{\text{ }n} \times \end{bmatrix} \\
2\cdot \begin{bmatrix} C_n^{\text{ }b-} \cdot M_k^{\text{ }n} \times \end{bmatrix}
\end{bmatrix}
$$

$`C_n^{\text{ }b-}`$ is the Rotation matrix from the NED frame to the body frame calculated from our gyroscope reading. $`f_k^{\text{ }n}`$ the gravity vector in the NED frame. $`M_k^{\text{ }n}`$ the magnetic field vector in the NED frame. Finally, the notation $` \begin{bmatrix} x \times \end{bmatrix}`$ is the skew-symmetric matrix of the vector $`x`$.

### Algorithm steps:

- Estimating the quaternion -prior estimation- using Gyroscope reading (Prediction stage):


$`
\bar{q}_k^{-} = \exp(\frac{\Delta t}{2} \cdot \Omega(\omega_{k-1})) \cdot \bar{q}_{k-1}^+
`$

- Estimating covariance matrix of the state vector:

  $` P_k^{-} = F_{k}\cdot P_{k-1}^+ \cdot F_{k}^T + G_k \cdot Q_k \cdot G_k^T `$

- Calculating Kalman Gain:

  $` K_k = P_k^-\cdot H_k^T(H_k \cdot P_k^-\cdot H_k^T + R_k )^{-1} `$

