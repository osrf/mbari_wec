The MBARI Wave-Energy-Converter is a small point absorber design that includes a surface expression, an electro-hydraulic PTO, and a submerged heave-cone device.  The system is moored to the seafloor (typically in 80m of water) through a chain-catenary mooring connected to an anchor.  As waves excite the system, a differential motion results between the buoy at the surface and the submerged heave cone.  Resisting this motion results in energy being absorbed by the system, and this energy is converted to electrical form and stored in a battery bank on the buoy. The rest of this section provides details about the various components of the system

![some description](images/MBARI_WEC_Assembly_12-5-2019-1.png)

## Buoy, Heave Cone, and Mooring
The buoy in the MBARI-WEC has a diameter of 2.6m, a water-plane area of 5 m^2, and a mass of 1400kg.  This buoy houses the system battery and compute infrastructure, described below.

The heave-cone component sits at about 30m depth and provides inertia and drag for the surface-buoy to pull against.  The heave-cone has operable doors that can be opened to reduce the drag and inertial of this component in high sea-states.  When the doors are open, the heave-cone has added-mass of about 10,000kg, in addition to it's own 600kg mass.  When opened, the added-mass reduces to about 3,000kg, which reduces the inertial forcing and increases the natural frequency of the buoy -- heave-cone pair.

A chain-catenary mooring and anchor connects to heave-cone to the ocean floor, keeping the buoy on-station.  The system loading due to the mooring increases in higher winds and currents, but remains relatively low compared with the inertia forces the heave-cone creates.

## PTO System
The power take-off device is located below the buoy and converts the differential motion and forces between the buoy and heave-cone from mechanical to electrical energy.  This device is an electro-hydraulic device in which a piston pumps oil through a hydraulic motor, causing an electrical motor/generator to spin and generate electrical energy.  In parallel to the hydraulic ram, a pneumatic piston charged with an inert gas provides a sprint returning force for the system. 

 The combination of the hydraulic and pneumatic pistons creates a spring-damper system for which the spring constant is set by the amount of gas in the system, and the damping behavior is adjustable electronically by varying the torque on the hydraulic motor in response to conditions.  The electrical-drive is a four-quadrant device in which the electric motor can operate as a generator in which energy flows into the battery, or as a motor in which energy is drawn from the battery.  The winding-currents (and resulting torque) can be set arbitrarily, but by default the power take-off device acts as a generator, i.e. a damper resisting motion.

## Electrical System
The electrical system of this buoy consists of a 325V battery system for energy storage, and a 24V system for powering ancillary instrumentation.  The 325V battery is connected directly to the power take-off device motor drive electronics.  In normal use, the motor-drive device is generating electrical energy at 325V which charges the battery system.  In the case the battery is full (or dis-connected), the motor-drive system directs excess energy to an electrical load-dump device.  This submerged heater plays a critical role in maintaining a load on the power take-off device at all times.  

The electrical system also includes 300V-24V power supplies that provides 24 volts to the compute and instrumentation infrastructure in the system.  In the case of low battery voltage due to an extended period of calm seas, the 

## Compute and Control Systems
The compute and control architecture of the system is such that critical functions are performed by micro-controllers throughout the system that implement default behaviors and stream sensor data continuously.  A Linux computer on the buoy performs data-logging and provides a command interface to the underlying micro-controllers. See figure. The system is designed such that the Linux computer is not necessary for safe behavior, if this computer re-boots or goes offline, the system will default to safe behaviors.  Additionally, the micro-controllers will ignore damaging commands from the Linux computer.  This architecture allows control algorithms running on the Linux computer to be started, stopped, and changed while the device is at sea through the cell-modem connection.  

![some description](images/SoftwareArchitectureDiagramV2aDeployed.png)

The fundamental system behaviors are performed by a network of micro-controller based compute nodes, that communicate with the buoy Linux computer and with one-another through a Controller Area Network (CAN) bus.  There are four of these controllers as follows:

- Battery Controller (BC_):  This micro-controller monitors battery voltage, currents, state-of-charge, cell-balance, and environmental conditions inside the battery enclosure.  This controller is largely a data-telemetry gathering item, but also includes an important low-voltage disconnect features which shuts down the 24V battery bus during periods of low-battery state-of-charge.  During these periods the system continues to convert wave-energy and charge the batteries, but all sources of significant battery drain are disconnected which allows the battery to re-charge to a serviceable level, even in calm conditions.

- Spring Controller (SC_): This micro-controller primarily monitors the piston position and a load-cell located between the buoy and power take-off component.  Additionally, this controller responds to commands to change the gas pressure in each chamber of the pneumatic spring, a pump to move gas from the lower pressure chamber to the higher pressure chamber, and a valve to do the opposite.  Additionally, this controller can turn power on and off to the heave-cone.

- Power Converter (PC_):  This controller implements the field-oriented control of the winding current in the generator.  It's core function is to control the winding current to a set target, but it performs a number of other specialized functions as well.  In particular, the converter monitors and reports motor RPM, and by default sets the motor winding current target as a pre-defined function of RPM.  This default behavior can be modified (scale and offset), or over-ridden entirely.  In this later case the power converter only over-rides the default winding-current for a short period of time (2 seconds). If a new winding-current command does not arrive in that time, the system reverts to the built-in default behavior.  The power converter also can switch power on and off to the batteries, divert power to an internal load-dump if the batteries are full or disconnected, and monitors the bus voltage and other health functions.  The power converter also obeys current draw/charge limits specified depending on the size and capability of the attached battery pack.

- Heave-cone Controller (TC_):  This controller is responsible for opening and closing the heave-cone doors when commanded.  For storm-safety, the controller will open the doors one hour after power from the surface is lost, or a watchdog timer expires.  Battery back-up of the system enables this, and the buoy system must issue a watchdog reset at least every hour to keep the doors closed.  In addition, this controller has an IMU and pressure sensor that provides attitude and depth information about the heave-cone.  Note:  This is denoted "TC_", the T stands for trefoil, which is French for clover and refers to the heave-cone doors themselves.


The on-board linux computer is accessible from shore over a radio link (cell-modem, satellite, or line-of-site radio).  This link enables  enabling data-telemetry, real-time control, and software-updates to be applied to the Linux computer.


## Sensors and Measurements
** Load Cell: ** This is a load cell between the buoy and the power take-off device, and has a range of up to 20,000lbs.

** Piston Position: ** Inside the pneumatic spring there is a laser range-finder that continuously monitors the position of the power take-off piston.  

** Pneumatic Pressures: **  The spring controller monitors the pressures of the two gas-chambers that make up the pneumatic spring.

** Hydraulic Compensator Pressure **  The hydraulic system has a small pressurized accumulator attached to the low-pressure side of the hydraulic circuit. This applies a small pressure (3psi) to ensure seawater does not enter the hydraulic system.  An abrupt drop in this pressure indicates a hydraulic leak and is therefore monitored and reported.

** Buoy Inertial Measurement Unit: ** On-board the buoy there is a GPS disciplined six DOF attitude-heading and reference system.  This unit monitors and reports the buoys attitude and GPS location.

** Heave Cone Inertial Measurement Unit: **  On-board the heave-cone there is an attitude-heading and reference system with a magnetometer and pressure sensor that reports the heave-cones orientation and depth.

** Electrical System Sensors: **  Voltages and currents are monitored throughout the electrical system and reported by various controllers.  In particular the motor winding currents, the load-dump current, and the battery current are all independently monitored and reported.

