Under Construction

## Modeling

summary about modeling

### Pneumatic Spring

summary of spring

#### Definitions

|                  | Description                                                   | Units     |
|------------------|---------------------------------------------------------------|-----------|
| \(x\)            | Piston position                                               | m         |
| \(v\)            | Piston velocity                                               | m/s       |
| \(m\)            | Mass of gas in chamber                                        | kg        |
| \(T\)            | Temperature of gas                                            | K         |
| \(P\)            | Gas pressure                                                  | Pa        |
| \(V\)            | Chamber volume (dependent on piston position)                 | m\(^{3}\) |
| \(R_{specific}\) | Specific Gas Constant N\(_2\)                                 | J/kg/K    |
| \(V_{dead}\)     | Chamber dead volume (fully compressed)                        | m\(^{3}\) |
| \(c_p\)          | Specific Heat Capacity N\(_2\) (constant pressure)            | J/kg/K    |
| \(A_{piston}\)   | Surface area of piston head                                   | m\(^{2}\) |
| \(n\)            | Polytropic index (Adiabatic if \(n=\lambda=1.4\) for N\(_2\)) | N/A       |
| \(r\)            | Coefficient of heat transfer (Newton's Law of Cooling)        | 1/s       |


#### Model

Under compression and expansion, the pressure, volume and temperature of the Nitrogen in each
chamber evolves according to Ideal Gas Law:

$$ P V = m R_{specific} T $$

and a polytropic process:

$$ P = P_0 \left({V_{0} \over V}\right)^{n} $$

with hysteresis, there are two values for the polytropic index, \(n\), to capture behavior when
the gas is compressing or expanding. Using this quasi-static solution and
discrete time steps, and also incorporating hysteresis, the process becomes:

$$ V_k = x_k A_{piston} + V_{dead} $$
$$
P_k = P_{k-1} \left({V_{k-1} \over V_k}\right)^n ,\,\,\, n = \begin{cases}
    n1 & \text{if } v \ge 0 \\
    n2 & \text{otherwise.}
\end{cases}
$$
$$ T_k = {P_k V_k \over m R_{specific}} $$

where \(k\) is the current time step.

Whenever the piston velocity is slow enough, the process is dominated by heat loss and
modeled with Newton's Law of Cooling (using forward difference):

$$ T_k = r\, \Delta t\, (T_{env} - T_{k-1}) + T_{k-1} $$

The mass of the Nitrogen in each chamber is determined from inputs in the SDF:

$$ m = {P_0 (x_0 A_{piston} + V_{dead}) \over R_{specific} T_0} $$

and is used for mass flow between chambers in simulating the pump/valve.


#### Determining Parameter Values

Linear regression was used to determine the polytropic indices for each chamber using empirical
data from the physical system. Using pressure vs volume curves, \(n_1\) is determined from
increasing volume, and \(n_2\) is determined from decreasing volume. The data is then preconditioned
by taking the logarithm to linearize and perform regression to find the parameters. For a
polytropic process:

$$ P V^{n} = C $$

so,

$$ \log{P} + n \log{V} = \log{C} $$
$$
\left[ { \begin{array}{cc}
    \log{\bf V} & 1 \\
\end{array} } \right] \left[ { \begin{array}{c}
    -n \\
    \log{C} \\
\end{array} } \right] = \log{\bf P}
$$

in block matrix notation where \({\bf V}\) and \({\bf P}\) are the arrays of volume and
pressure data, respectively.

The other parameters in the system are taken from CAD or empirically determined by comparing logged
data from prescribed motion between simulation and physical test bench.
