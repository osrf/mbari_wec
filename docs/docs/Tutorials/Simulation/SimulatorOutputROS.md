[Back to [Tutorials](../../tutorials.md) list]

# View ROS 2 Messages

While running, the simulator generates exactly the same ROS 2 messages that the buoy hardware does during operation.  These are grouped into ROS 2 topics that corresponds to data being produced by each micro-controller or instrument on the buoy.  To see all ROS 2 topics being published to on the system, issue the following command (after sourcing the workspace if needed in a new terminal ``` $ . ~/mbari_wec_ws/install/setup.sh```

``` 
$ ros2 topic list 
/ahrs_data
/clock
/joint_states
/parameter_events
/power_data
/rosout
/spring_data
/tf
/tf_static
/xb_imu
```

The topics /ahrs_data, /battery_data, /spring_data, /power_data, and /heavecone_data corresponds to the buoy-based instrumentation (AHRS), battery controller, spring controller, power-converter controller, and heave-cone controller.  Several of these topics are only available in simulation, and only /ahrs_data, /battery_data, /spring_data, /power_data, and /heavecone_data will be present on the real buoy.


To see the data being published in these topics, issue the following command and the data will scroll by, for example:

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

The data in each topic corresponds to the message descriptions which can be seen here along with a description of each field.

The next tutorial "[View Messages with Plotjuggler](SimulatorOutputPlotjuggler.md)" shows how to conveniently plot these data items while the simulator is running.


[Back to [Tutorials](../../tutorials.md) list]
