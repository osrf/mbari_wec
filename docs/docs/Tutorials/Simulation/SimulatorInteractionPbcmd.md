## Run-Time Control using pbcmd

## Introduction
There is a command interpreter running on the physical buoy that provides some control over the behavior of the buoy while it is deployed.  This is a Linux executable that implements a number of commands, accepts appropriate arguments, and issues commands over the CANbus or ROS 2 on the buoy to effect a change of buoy behavior.  These commands can also be issued programmatically as described in subsequent tutorials, but pbcmd is the human interface.  

This same command and interface is implemented in the simulation environment, so it is possible to change the behavior of the simulated buoy from the command line in the same way as can be done on the physical buoy.  However many of the possible commands are not sensible in the simulated environment, so are not implemented, but instead return a message indicating they aren't relevant in simulation.  A key example is the command to open and close the heave-cone doors, because the simulator can not change this behavior while running, the command to do so is inactive.  Similarly, it makes no sense to turn on and off the electrical ground fault detector that exists on the buoy, but does not exist in simulation.

## pbcmd Usage
Issuing 'pbcmd' at the command prompt provides guidance on the possible commands.  In simulation it also indicates which commands are supported in simulation, on the at-sea system, all commands have an effect.  

```
$ ./pbcmd 

pbcmd: Multi-call command Power Buoy dispatcher
Commands currently supported in Simulation:
*               pump  - Spring Controller pump  off or on for a time in minutes <= 10
*              valve  - Spring Controller valve off or on for a time in seconds <= 12
*       sc_pack_rate  - Set the CANBUS packet rate from the spring controller
*           pc_Scale  - Set the scale factor
*         pc_Retract  - Set the retract factor
*        pc_WindCurr  - Set the winding current target
*        pc_BiasCurr  - Set the winding current bias
*        pc_PackRate  - Set the CANBUS packet rate

Commands currently not supported in Simulation:
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


##  Command descriptions

- **pump** - This command turns on and off the gas pump that pumps Nitrogen from the upper pneumatic spring chamber to the lower spring chamber.  The result of this is the mean position of the position will slowly rise.  The rate is about 1" per minute of pump action, a required argument for this command specifies how long the pump will run for, in minutes, and must be between 0 and 10.  After this timeout the pump will stop even if no further commands are issued.  A "pump 0" command stops the pump immediately.

- **valve** - The valve command releases Nitrogen gas from the lower chamber to the upper chamber, resulting in the mean position of the piston lowering.  This process is much faster so the required argument for this command is in seconds, and must be between 0 and 10.  After this timeout the valve closes even if no further commands are issued.  A "valve 0" command closes the pump immediately.

- **sc_pack_rate** - This command sets the data packet rate for data coming from the spring controller, the required argument between 10 and 50 indicates the desired data rate in Hz.  This controls the rate of the ROS 2 messages from the spring controller on the buoy, and in the simulator.

- **pc_scale** - This command adjusts the multiplier that is applied to the default motor-current/RPM relationship that is programmed into the power converter.  This value can be from 0.4 to 1.5, allowing a range of damping to be applied.

- **pc_Retract** - This command adjusts an additional multiplier that is applied to the default motor-current/RPM relationship during piston retraction.  This value can be from 0.4 to 1.0, and allows the damping behavior of the system to be asymmetrical, promoting retraction since the pneumatic spring can not pull as hard as the waves can.

- **pc_WindCurr** - This command directly sets the winding current in the electric motor and accepts a value between -35 Amps and +35 Amps.  This value is the quadrature current in the permanent magnet electric motor, and therefore corresponds directly with applied torque. A positive winding current produces a torque that applies a force that extends the piston. The controller applies this specified torque for two seconds.  After that time, if no new pc_WindCurr command is executed, the system returns to following the default motor-current/RPM relationship (adjusted by the Scale and Retract factor as described).  This allows safety in the case of a communication failure, and makes it a bit impractical to manipulate the winding current manually from the keyboard. As described in subsequent tutorials, programmatically adjusting this value in response to the behavior of the wave-energy converter is the primary automated external control mechanism.

- **pc_BiasCurr** - This command applies an offset to the default motor-current/RPM relationship.  This value can be between -15 Amps and +15 Amps and is applied for 15 seconds before the system reverts to the default motor-current/RPM relationship.  A positive current corresponds to a torque that tends to extend the piston.  This command is useful for temporarily changing the equilibrium point of the piston at sea.


- **pc_pack_rate** - This command sets the data packet rate for data coming from the power converter, the required argument between 10 and 50 indicates the desired data rate in Hz.  This controls the rate of the ROS 2 messages from the power converter on the buoy, and in the simulator.

## Example Usage
As an example, issue the following commands in a terminal where the workspace has been sourced:

1. Launch the simulation without incident waves by issuing:
```
$ ros2 launch buoy_gazebo mbari_wec.launch.py 
```
2. Start the simulation in the GUI by pressing the play button.

3. Start PlotJuggler
```
$ ros2 run plotjuggler plotjuggler &
```

4. Select topics /power_data and /spring_data, and then create plots to display winding current, piston position, and motor RPM.

5. Issue the following command to introduce a -15A winding current offset in the motor-current/RPM relationship.

```
$ PC_BiasCurr -15
```
The plotjuggler window should look approximately as below.  One can see that the command resulted in an additional retraction force to be applied.  This causes the motor to spin and the piston to retract.  After the timeout (12 seconds), this offset is removed and the heave-cone pulls the piston out again, oscillating until it comes to rest at the original position.

![](images/PlotJuggler.png)
