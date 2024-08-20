# Kalman-filter-attitude-Estimator
An Implementation of three algorithms for Estimating the Attitude of a rigid body from IMU sensor data depending on Kalman filter.

## the first method
This method uses unit quaternions as a representation of the attitude of the rigid body. 

The state vector $\(\mathbf{x}\)$ is defined as the vector (imaginary) part of the error quaternion $\( q_{\text{error}} \)$. The error quaternion $\( q_{\text{error}} \)$ is obtained by multiplying the real quaternion $\( q_{\text{real}} \)$ by the conjugate of the estimated quaternion $\( q_{\text{estimated}} \)$:


$q_{\text{error}} = q_{\text{real}} \cdot q_{\text{estimated}}^{-1}$


Thus, the state vector $\( \mathbf{x} \)$ is given by:


$\mathbf{x} = \text{Im}(q_{\text{error}}) = \text{Im}(q_{\text{real}} \cdot q_{\text{estimated}}^{-1})$


### Algorithm steps:

$`
\bar{q}_k^- = \exp\left(\frac{\Delta t}{2} \cdot \Omega(\omega_{k-1})\right) \cdot \bar{q}_{k-1}^+
`$





