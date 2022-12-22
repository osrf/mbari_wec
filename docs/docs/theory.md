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
| \(V_{dead}\)     | Dead volume (fully compressed)                                | m\(^{3}\) |
| \(c_p\)          | Specific Heat Capacity N\(_2\) (constant pressure)            | J/kg/K    |
| \(A_{piston}\)   | Surface area of piston head                                   | m\(^{2}\) |
| \(n\)            | Polytropic index (Adiabatic if \(n=\lambda=1.4\) for N\(_2\)) | N/A       |
| \(r\)            | Heat transfer coefficient                                     | N/A       |


#### Model

Under compression and expansion, the pressure, volume and temperature of the Nitrogen in each
chamber evolves according to Ideal Gas Law:

$$ P V = m R_{specific} T $$

and a polytropic process:

$$ P = P_0 \left({V_{0} \over V}\right)^{n}$$

with hysteresis, there are two values for the polytropic index, \(n\), to capture behavior when
the gas is compressing or expanding. With hysteresis in mind, using this quasi-static solution and
discrete time steps, the process becomes:

$$ V[nT] = x[nT] A_{piston} + V_{dead} $$
$$ P[nT] = P[(n-1)T] \left({V[(n-1)T] \over V[nT]}\right)^{n_{polytropic}} $$
$$ T_{gas}[nT] $$

where \(n_{polytropic}\) is the polytropic index defined above, \(T_{gas}\) is the gas temperature
and \(nT\) is the current time step.

To gain more fidelity and match experimental data, whenever the piston velocity is in a certain
slower range, the following heat-loss dominated mode is used in place of the process above:

$$  $$

The mass of the Nitrogen in each chamber is determined from inputs in the SDF:

$$ m = { P_0 (x_0 A_{piston} + V_{dead}) \over R_{specific} T_0} $$

and is used for mass flow between chambers in simulating the pump/valve.
