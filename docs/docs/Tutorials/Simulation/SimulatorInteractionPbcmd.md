## Run-Time Control using pbcmd

#### Introduction
There is a small command interpreter running on the physical buoy that provides some control over the behavior of the buoy while it is deployed.  This is a Linux executable that implements a number of commands, accepts appropriate arguments, and issues commands over the CANbus or ROS2 on the buoy to effect a change of buoy behavior.  These commands can also be issued programmatically as described in subsequent tutorials, but pbcmd is the human interface.  

This same command and interface is implemented in the simulation environment, so it is possible to change the behavior of the simulated buoy from the command line in the same way as can be done on the physical buoy.  However many of the possible commands are not sensible in the simulated environment, so are not implemented, but instead return a message indicating they aren't relevant in simulation.  A key example is the command to open and close the heave-cone doors, because the simulator can not change this behavior while running, the command to do so is inactive.  Similarly, it makes no sense to turn on and off the electrical ground fault detector that exists on the buoy, but does not exist in simulation.

#### pbcmd Usage
Issuing 'pbcmd' at the command prompt provides guidance on the possible commands.  In simulation it also indicates which commands are supported in simulation, on the at-sea system, all commands have an effect.  

```
$ ./pbcmd 

pbcmd: Multi-call command Power Buoy dispatcher
Supported commands (Simulation):
*               pump  - Spring Controller pump  off or on for a time in minutes <= 10
*              valve  - Spring Controller valve off or on for a time in seconds <= 12
*       sc_pack_rate  - Set the CANBUS packet rate from the spring controller
*           pc_Scale  - Set the scale factor
*         pc_Retract  - Set the retract factor
*        pc_WindCurr  - Set the winding current target
*        pc_BiasCurr  - Set the winding current bias
*        pc_PackRate  - Set the CANBUS packet rate

Currently unsupported commands (Simulation):
*             bender  - Sets the state of the bender module
*      reset_battery  - Reset battery controller (caution - no args)

*             tether  - Spring Controller tether power on or off
*       reset_spring  - Reset Spring Controller (caution - no args)

*        pc_VTargMax  - Set the max target voltage
*   pc_ChargeCurrLim  - Set the maximum battery charge current
*     pc_DrawCurrLim  - Set the maximum battery current draw
*      pc_BattSwitch  - Set the battery switch state
*            pc_Gain  - Set the gain scheduler gain
*      pc_StdDevTarg  - Set the target RPM standard deviation

*          tf_SetPos  - Open/close the doors in the heave-cone
*    tf_SetActualPos  - Open/close the doors in the heave-cone
*         tf_SetMode  - Set controller mode
*   tf_SetChargeMode  - Set Battery Charge mode
* tf_SetStateMachine  - Set Battery Charge mode
*      tf_SetCurrLim  - Set controller current limit
*        tf_WatchDog  - Toggle controller watchdog (caution - no args)
*           tf_Reset  - Reset Controller (caution - no args)


For help on a command use the command name, i.e. "bender";

Except the reset commands which take no arguments.

DO NOT enter reset_battery and expect to get help. The command will execute!
```

Note that at the end of this usage message, there is a hint that typing most commands without an argument will supply some further help.


####  Command descriptions

- **pump** - This command turns on and off the gas pump that pumps Nitrogen from the upper pneumatic spring chamber to the lower spring chamber.  The result of this is the mean position of the position will slowly rise.  The rate is about 1" per minute of pump action, a required argument for this command specifies how long the pump will run for, in minutes, and must be between 0 and 10.  After this timeout the pump will stop even if no further commands are issued.  A "pump 0" command stops the pump immediately.

- **valve** - The valve command releases Nitrogen gas from the lower chamber to the upper chamber, resulting in the mean position of the piston lowering.  This process is much faster so the required argument for this command is in seconds, and must be between 0 and 10.  After this timeout the valve closes even if no further commands are issued.  A "valve 0" command closes the pump immediately.

- **sc_pack_rate** - This command sets the data packet rate for data coming from the spring controller, the required argument between 10 and 50 indicates the desired data rate in Hz.  This controls the rate of the ROS2 messages from the spring controller on the buoy, and in the simulator.


#### Example Usage
