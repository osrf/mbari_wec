## Modeling Technique Overview
The numerical modeling used in this simulator relies upon the Gazebo simulators ability to solve for the 
motion of a collection of rigid bodies connected by various types of joints.  Gazebo can use several 
different physics solvers to perform this solution, and the wave-energy buoy simulator in this project
uses the <a href="https://dartsim.github.io/") target="_blank">DART</a> physics engine.  The physics engine
solves the multi-body six degree-of-freedom problem for the motion of the connected buoy, power-take-off device, and heave cone,
subject to initial conditions and forces that act on the various components as the simulation progresses.  In the Gazebo simulator,
these forces are provided by plugin code that applies forces to each body based on the state (position and velocity) of each
component in the system at each timestep.  

The Gazebo simulator already includes several plugins that provide relevant forces such as buoyancy and hydrodynamic drag. 
Additionally, several additional plugins have been created for this simulator that provide the forcings on the system
due to the electro-hydraulic power-take-off system, the pneumatic spring system, the tether connecting the PTO to the
heave-cone, the mooring system that anchors the system, and the forcing on the buoy from the ocean surface waves. 
The sections below outline some details about how each of these forcings are modelled and computed.  First however, 
the specific physical characteristics of the MBARI WEC are tabulated for reference.

--------------------------------------------------------------------------------------------------------
## Physical Characteristics

Each rigid body in the simulation has a "Link Frame" coordinate system in which all other characteristics of the
body are defined in for computational purposes.  This link-frame coordinate system is often selected to be at the
location of a joint that connects the various bodies (which are also called links in the vernacular of Gazebo).
In the following sections, all quantities are defined in the body's link frame unless otherwise noted.  Also, the
description below includes alternative notation for added mass, e.g.  \(\\ \mu_{xx}\), in the style of Newman.


### Surface Buoy
|                  | Description                                               |                  |   Units   |
|:----------------:|:----------------------------------------------------------|:----------------:|:---------:|
| \(m\)            | Buoy Mass                                                 |  1400            | kg        |
| \(V\)            | Displacement (undisturbed buoy)                           |  1.0             | m\(^3\)   |
| \(\bf{x}_{COB}\) | Center of Gravity in Link Frame (x,y,z)                   | (0.0, 0.0, 2.03) | m         |
| \(\bf{x}_{COG}\) | Center of Buoyancy in Link Frame (x,y,z)                  | (0.0, 0.0, 2.05) | m         |
| \(\bf{x}_{COW}\) | Center of Waterplane in Link Frame, including PTO and cone| (0.0, 0.0, 2.27) | m         |
| \(I_{xx}\)       | Roll Moment of Inertia (MOI), origin at pivot             |  7000            | kg m\(^2\)|
| \(I_{yy}\)       | Pitch Moment of Inertia origin at pivot                   |  7040            | kg m\(^2\)|
| \(I_{zz}\)       | Yaw Moment of Inertia, origin at pivot                    |  670             | kg m\(^2\)|
| \(A_{wp}\)       | Waterplane Area (undisturbed buoy)                        |  5.0             | m\(^2\)   |
| \(X_{\dot{u}}\)  | Surge Added Mass (\(\mu_{xx}\))                           |  260             | kg        |
| \(Y_{\dot{v}}\)  | Sway Added Mass ( \(\mu_{yy}\))                           |  260             | kg        |
| \(Z_{\dot{w}}\)  | Heave Added Mass (\(\mu_{zz}\))                           |  3080            | kg        |
| \(K_{\dot{p}}\)  | Roll Added Mass MOI, origin at pivot (\(\mu_{pp}\))       |  780             | kg m\(^2\)|
| \(M_{\dot{q}}\)  | Pitch Added Mass MOI, origin at pivot (\(\mu_{qq}\))      |  780             | kg m\(^2\)|
| \(X_{\dot{q}}\)  | Surge-Pitch Added Mass, origin at pivot (\(\mu_{xq}\))    |  370             | kg        |
| \(Y_{\dot{p}}\)  | Sway-Roll Added Mass, origin at pivot (\(\mu_{yp}\))      | -370             | kg        |
| \(X_{u\mid u \mid}\)   | Surge Quadratic Drag                                | -430             | kg/m      |
| \(Y_{v\mid v \mid}\)   | Sway Quadratic Drag                                 | -430             | kg/m      |
| \(Z_{w\mid w \mid}\)   | Heave Quadratic Drag                                | -3280            | kg/m      |
| \(K_{p\mid p \mid}\)   | Roll Quadratic Drag:                                |  -880            | kg m\(^2\)|
| \(M_{q\mid q \mid}\)   | Pitch Quadratic Drag:                               |  -880            | kg m\(^2\)|
| \(N_{r\mid r \mid}\)   | Yaw Quadratic Drag:                                 |  -50             | kg m\(^2\)|


- Buoy Link Frame is located at base of the buoy bridle at the pivot.
- Unspecified stability derivative values (\(M_{\dot{r}}\), \(X_{uv}\), \(Y_{vu}\), etc) are zero. 
- Inertias, Added mass inertias, and stability derivatives are all specified about the link frame origin, i.e. the pivot.
- Added mass values are infinite frequency.
- Free-Surface Hydrodynamic Coefficients and Impulse Response Functions can be found <a href="https://www.google.com/") target="_blank">here</a>:

### Power Take-Off Device
|                   | Description                                |                  | Units     |
|:-----------------:|:-------------------------------------------|:----------------:|:---------:|
| \(m\)             | PTO Mass                                   |  605             | kg        |
| \(V\)             | PTO Displacement                           |  1.0             | m\(^3\)   |
| \(\bf{x}_{COG}\)  | Center of Gravity in Link Frame (x,y,z)    | (0.0, 0.0, -4.0) | m         |
| \(\bf{x}_{COB}\)  | Center of Buoyancy in Link Frame (x,y,z)   | (0.0, 0.0, -3.0) | m         |
| \(I_{xx}\)        | Roll Moment of Inertia                     |  32600           | kg m\(^2\)|
| \(I_{yy}\)        | Pitch Moment of Inertia                    |  32600           | kg m\(^2\)|
| \(I_{zz}\)        | Yaw Moment of Inertia                      |  7               | kg m\(^2\)|
| \(I_{xy}\)        | Roll-Pitch Product of Inertia              |  0               | kg m\(^2\)|
| \(I_{xz}\)        | Roll-Yaw Product of Inertia                |  -2              | kg m\(^2\)|
| \(I_{yz}\)        | Pitch-Yaw Product of Inertia               |  -3              | kg m\(^2\)|
| \(X_{\dot{u}}\)   | Surge Added Mass (\(\mu_{xx}\))            |          310     | kg        |
| \(X_{\dot{q}}\)   | Surge-Pitch Added Mass MOI (\(\mu_{xq}\))  |         1250     | kg m      |
| \(Y_{\dot{v}}\)   | Sway Added Mass (\(\mu_{yy}\))             |          310     | kg        |
| \(Y_{\dot{p}}\)   | Sway-Roll Added Mass MOI (\(\mu_{yp}\))    |        -1250     | kg m      |
| \(Z_{\dot{w}}\)   | Heave Added Mass (\(\mu_{yy}\))            |           10     | kg        |
| \(K_{\dot{p}}\)   | Roll Added Mass MOI  (\(\mu_{pp}\))        |         7040     | kg m\(^2\)|
| \(M_{\dot{q}}\)   | Pitch Added Mass MOI (\(\mu_{qq}\))        |         7040     | kg m\(^2\)|
| \(X_{u\mid u \mid}\)    | Surge Quadratic Drag                 |  -1140           | kg/m      |
| \(Y_{v\mid v \mid}\)    | Sway Quadratic Drag                  |  -1140           | kg/m      |
| \(Z_{w\mid w \mid}\)    | Heave Quadratic Drag                 |  -50             | kg/m      |
| \(K_{p\mid p \mid}\)    | Roll Quadratic Drag                  |  -195400         | kg m\(^2\)|
| \(M_{p\mid p \mid}\)    | Pitch Quadratic Drag                 |  -195400         | kg m\(^2\)|
| \(N_{r\mid r \mid}\)    | Yaw Quadratic Drag                   |  -50             | kg m\(^2\)|

- PTO Link Frame is located at top attachment of the PTO (where connects to the buoy).
- Unspecified stability derivative values (\(M_{\dot{r}}\), \(X_{uv}\), \(Y_{vu}\), etc) are zero. 
- Inertias, Added mass inertias, and stability derivatives are all specified about the link frame origin.
- Added mass values are infinite frequency.

### Piston 
|                   | Description                                |                   | Units     |
|:-----------------:|:-------------------------------------------|:-----------------:|:---------:|
| \(m\)             | Piston Mass                                |  48.0             | kg        |
| \(\bf{x}_{COG}\)  | Center of Gravity in Link Frame (x,y,z)    | (0.0, 0.0, -2.58) | m         |
| \(I_{xx}\)        | Roll Moment of Inertia                     |  100.0            | kg m\(^2\)|
| \(I_{yy}\)        | Pitch Moment of Inertia                    |  100.0            | kg m\(^2\)|
| \(I_{zz}\)        | Yaw Moment of Inertia                      |  5.0              | kg m\(^2\)|

- The piston is contained with the PTO housing, so it has mass and moments of inertial, but contributes 
no buoyancy, added mass, or quadratic fluid drag.

### Heave Cone  
|                   | Description                                |                  | Units     |
|:-----------------:|:-------------------------------------------|:----------------:|:---------:|
| \(m\)             | Heave Cone Mass                            |  820             | kg        |
| \(V\)             | Heave Cone Displacement                    |  1.0             | m\(^3\)   |
| \(\bf{x}_{COG}\)  | Center of Gravity in Link Frame (x,y,z)    | (0.0, 0.0, -1.25)| m         |
| \(\bf{x}_{COB}\)  | Center of Buoyancy in Link Frame (x,y,z)   | (0.0, 0.0, -1.21)| m         |
| \(I_{xx}\)        | Roll Moment of Inertia                     |  4200            | kg m\(^2\)|
| \(I_{yy}\)        | Pitch Moment of Inertia                    |  4200            | kg m\(^2\)|
| \(I_{zz}\)        | Yaw Moment of Inertia                      |  610             | kg m\(^2\)|
| \(I_{xy}\)        | Roll-Pitch Product of Inertia              |  0               | kg m\(^2\)|
| \(I_{xz}\)        | Roll-Yaw Product of Inertia                |  -1              | kg m\(^2\)|
| \(I_{yz}\)        | Pitch-Yaw Product of Inertia               |  0               | kg m\(^2\)|
| \(X_{\dot{u}}\)   | Surge Added Mass (\(\mu_{xx}\))            |       720        | kg        |
| \(X_{\dot{q}}\)   | Surge-Pitch Added Mass MOI (\(\mu_{xq}\))  |       900        | kg m      |
| \(Y_{\dot{v}}\)   | Sway Added Mass (\(\mu_{yy}\))             |       720        | kg        |
| \(Y_{\dot{p}}\)   | Sway-Roll Added Mass MOI (\(\mu_{yp}\))    |      -900        | kg m      |
| \(Z_{\dot{w}}\)   | Heave Added Mass: Doors Closed (\(\mu_{yy}\))|    9330        | kg        |
| \(Z_{\dot{w}}\)   | Heave Added Mass: Doors Open               |  3000            | kg        |
| \(K_{\dot{p}}\)   | Roll Added Mass MOI (\(\mu_{pp}\))         |      3990        | kg m\(^2\)|
| \(M_{\dot{q}}\)   | Pitch Added Mass MOI(\(\mu_{qq}\))         |      3990        | kg m\(^2\)|
| \(N_{\dot{r}}\)   | Yaw Added Mass MOI (\(\mu_{rr}\))          |        10        | kg m\(^2\)|
| \(X_{u\mid u \mid}\)    | Surge Quadratic Drag                 |  -1580           | kg/m      |
| \(Y_{v\mid v \mid}\)    | Sway Quadratic Drag                  |  -1580           | kg/m      |
| \(Z_{w\mid w \mid}\)    | Vertical Quadratic Drag: Doors Open  |  -3200           | kg/m      |
| \(Z_{w\mid w \mid}\)    | Vertical Quadratic Drag: Doors Closed|  -3900           | kg/m      |
| \(K_{p\mid p \mid}\)    | Roll Quadratic Drag:                 |  -4620           | kg m\(^2\)|
| \(M_{q\mid q \mid}\)    | Pitch Quadratic Drag:                |  -4620           | kg m\(^2\)|
| \(N_{r\mid r \mid}\)    | Yaw Quadratic Drag:                  |  -50             | kg m\(^2\)|

- Heave-Cone Link Frame is located at top attachment of the Heave Cone (where it connects to the tether).
- Unspecified stability derivative values (\(M_{\dot{r}}\), \(X_{uv}\), \(Y_{vu}\), etc) are zero. 
- Inertias, Added mass inertias, and stability derivatives are all specified about the link frame origin.
- Added mass values are infinite frequency.


--------------------------------------------------------------------------------------------------------
## Electro-Hydraulic PTO Forces



--------------------------------------------------------------------------------------------------------
## Pneumatic Spring Forces

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

with hysteresis, there are two values for the polytropic index, \(n_1\) and \(n_2\), to capture behavior when
the gas is compressing or expanding. Using this quasi-static solution and
discrete time steps, and also incorporating hysteresis, the process becomes:

$$ V_k = x_k A_{piston} + V_{dead} $$
$$
P_k = P_{k-1} \left({V_{k-1} \over V_k}\right)^n ,\,\,\, n = \begin{cases}
    n_1 & \text{if } v \ge 0 \\
    n_2 & \text{otherwise.}
\end{cases}
$$
$$ T_k = {P_k V_k \over m R_{specific}} $$

where \(k\) is the current time step.

Whenever the piston velocity is slow enough, the process is dominated by heat loss and
modeled with Newton's Law of Cooling (using forward difference) followed by an update of pressure using
Ideal Gas Law:

$$ T_k = r\, \Delta t\, (T_{env} - T_{k-1}) + T_{k-1} $$
$$ V_k = x_k A_{piston} + V_{dead} $$
$$ P_k = {m R_{specific} T_k \over V_k} $$

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

in block matrix notation where \({\bf V}\) and \({\bf P}\) are the arrays of volume and pressure data, respectively.
The other parameters in the system are taken from CAD or empirically determined by comparing logged
data from prescribed motion between simulation and the physical test bench.


--------------------------------------------------------------------------------------------------------
## Tether Forces


--------------------------------------------------------------------------------------------------------
## Mooring Forces


--------------------------------------------------------------------------------------------------------
## Ocean Wave Forces
