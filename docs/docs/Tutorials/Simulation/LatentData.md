# View Latent Data

In simulation, there are data items available that are not measurable on the real system.  Much of this "Latent Data" is encoded in ROS 2 messages and published on a topic called latent_data.  To view these messages, the ros2 command line interface can be used.  To do this, issue the following command (after sourcing the workspace if needed in a new terminal ``` $ . ~/mbari_wec_ws/install/setup.bash```

``` 
$ ros2 topic echo latent_data 
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

A detailed description of each of these data fields in "[View Messages with Plotjuggler](SimulatorOutputPlotjuggler.md)".  In addition to seeing the data from the command line, these data can also be plotted using plotjuggler by selecting the latent_data topic ("[View Messages with Plotjuggler](SimulatorOutputPlotjuggler.md)"), or be interpreted programatically from a c++ or python program, as described here:  

A particular type of latent data is the wave-elevation at the buoy location, or any specified location in the wave-field nearby the buoy.  By default, the latent_data topic includes the wave-elevation at the buoy location, but the wave-elevation at an arbitrary location and time can be accessed using a ROS service.  A ROS 2 Service is a mechanism that provides data in response to a request, while the simulation is running, this can be done from the command line as shown in the examples below.  

``` 
$ ros2 topic echo latent_data 
---

``` 

