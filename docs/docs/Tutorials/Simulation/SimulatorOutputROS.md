# View ROS 2 Messages

While running, the simulator generates exactly the same ROS 2 messages that the buoy hardware does
during operation.  These are grouped into ROS 2 topics that corresponds to data being produced by
each micro-controller or instrument on the buoy.  To see all ROS 2 topics being published to on the
system, issue the following command (after sourcing the workspace if needed in a new terminal
` $ . ~/mbari_wec_ws/install/setup.bash`):

``` 
$ ros2 topic list 
/ahrs_data
/battery_data
/bc_record
/clock
/events/write_split
/joint_states
/latent_data
/parameter_events
/pc_record
/power_data
/rosout
/sc_record
/spring_data
/tf
/tf_record
/tf_static
/trefoil_data
/trefoil_imu
/trefoil_mag
/xb_gps
/xb_imu
/xb_record
```

Several of the topics listed with `ros2 topic list` are only available in simulation. Only the
following topics will be present on the real buoy:

- /ahrs_data
- /battery_data
- /spring_data
- /power_data
- /trefoil_data

corresponding to:
- buoy-based instrumentation (AHRS)
- battery controller
- spring controller
- power-converter controller
- heave-cone controller


To see the data being published in these topics, issue the following command and the data will
scroll by, for example:

```
$ ros2 topic echo power_data
---
header:
  stamp:
    sec: 712
    nanosec: 710000000
  frame_id: ''
seq_num: 6703
rpm: 369.927978515625
sd_rpm: 0.0
voltage: 313.98431396484375
draw_curr_limit: 0.0
bcurrent: -0.14509780704975128
wcurrent: -0.2554447054862976
torque: 0.0
diff_press: 2.9100000858306885
bias_current: 0.0
loaddc: 0.0
scale: 1.0
retract: 0.6000000238418579
target_v: 0.0
target_a: -0.2554447054862976
charge_curr_limit: 0.0
status: 0
---
```


Several ROS 2 services are used by the command-line interface `pbcmd`
([Tutorial: Control Simulator Output with pbcmd](SimulatorInteractionPbcmd.md)) to issue commands to the
controllers on the buoy.  To see a list issue the following command (result below only shows
important services):

```
$ ros2 service list
/bc_reset_command
/bender_command
/bus_command
/gain_command
/inc_wave_height
/pc_batt_switch_command
/pc_bias_curr_command
/pc_charge_curr_lim_command
/pc_draw_curr_lim_command
/pc_pack_rate_command
/pc_retract_command
/pc_scale_command
/pc_std_dev_targ_command
/pc_v_targ_max_command
/pc_wind_curr_command
/pump_command
/sc_pack_rate_command
/sc_reset_command
/tether_command
/tf_reset_command
/tf_set_actual_pos_command
/tf_set_charge_mode_command
/tf_set_curr_lim_command
/tf_set_mode_command
/tf_set_pos_command
/tf_set_state_machine_command
/tf_watchdog_command
/valve_command
```

A detailed list of ROS 2 messages and services on these topics can be found in

The data in each topic and service correspond to the message descriptions which can be seen in
[ROS 2 Interface->Interfaces (ROS 2 Messages)](../../ROS2/messages.md) along with a description of
each field.

## Logging During Simulation

By default, data is logged to .csv files matching the format and file layout of the log files on
the physical buoy system. When running the simulator in single or batch modes,
[rosbag2](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)
logging to file may also be enabled (using [mcap](https://mcap.dev/guides/getting-started/ros-2) storage mode).
A detailed tutorial of logging with .csv and rosbag2 can be found in:
[Tutorial: Simulator Output Data Logs](SimulatorOutputLogs.md)

## Plotting Live Data

The next tutorial, [Tutorial: View Messages with Plotjuggler](SimulatorOutputPlotjuggler.md), shows how to
conveniently plot these ROS 2 data items while the simulator is running.
