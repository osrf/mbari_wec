##### Start-up parameters
There are a number of parameters that change the behavior of the simulator, and must be specified at start-up:

- Sea-State: This is specified as a Wave Height and Period.  If a positive value of wave-period is specified, a Pierson-Moskowitch spectrum with the specfied significant wave-height and peak-period is used.  If a negative value of wave-period is specified, a mono-chromatic incoming wave at the specified period and height is used.
- Real-time factor:  This specifies how fast the simulator will run.  A real-time factor of 1.0 cooresponds to the simulation time proceeding at the same speed as the wall-clock.  A larger value runs the simulator faster than real-time, practical upper limits on normal hardware are a real-time factor of about 30, and if this is not possible the simulator will run as fast as possible.
- Heave-cone door position:  The simulation can be run with the heave-cone doors either open, or closed. This can not be changed while the simulation is running.
- Simulation Rendering: The simulator can be run with our without a visual rendering of the system.  The simulator may run faster without the rendering graphics.

##### Run-time control
