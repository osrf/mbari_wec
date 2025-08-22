# Buoy Interfaces (ROS 2 Messages)

Several ROS 2 messages/services have been defined in the
[buoy_interfaces](https://github.com/osrf/mbari_wec_utils/tree/v2.0.0-rc1/buoy_interfaces) package to
access telemetry and send commands.

For more information about ROS 2 interfaces, see
[docs.ros.org](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html).


## Messages (.msg) and Topics

Collection of telemetry (feedback) from buoy sensors and microcontrollers.


### Telemetry

Telemetry data from sensors available per microcontroller

- type: `BCRecord`  
  topic: `/battery_data`  
  Description: Battery Controller Telemetry

    ```
    std_msgs/Header header  # contains time: .stamp
    int16 seq_num           # incremental number of messages sent
    float32 voltage         # total battery voltage in Volts
    float32 ips             # current in Amps
    float32 vbalance        # cell balancing voltage
    float32 vstopcharge     # voltage threshold at which charging should stop
    float32 gfault          # ground fault level
    float32 hydrogen        # hydrogen gas level
    uint16 status           # microcontroller status bitfield
    ```

- type: `PCRecord`  
  topic: `/power_data`  
  Description: Power Controller Telemetry

    ```
    std_msgs/Header header     # contains time: .stamp
    int16 seq_num              # incremental number of messages sent
    float32 rpm                # motor speed
    float32 sd_rpm             # stddev rpm
    float32 voltage            # motor voltage
    float32 draw_curr_limit    # output current limit
    float32 bcurrent           # battery current
    float32 wcurrent           # torque controller winding current
    float32 torque             # motor torque (Nm)
    float32 diff_press         # hydraulics differential pressure
    float32 bias_current       # torque controller bias current (on windings)
    float32 loaddc             # load dump current
    float32 scale              # torque controller gain
    float32 retract            # additional torque controller gain applied in retraction
    float32 target_v           # target voltage
    float32 target_a           # target current
    float32 charge_curr_limit  # input charge current limit

    # Note: Not Currently Implemented in Sim Controller
    uint16 MODE=1                     # controller mode. specified by defines in config.h (GENERATOR_MODE, TORQUE_MODE)
    uint16 TORQUE_CMD_LIMITED=4       # defines what is limited the Torque Command
                                      # (0 = Not limited, 1 = Rate, 2 = Min, 3 = Max).
    uint16 BATT_SWITCH_REQUEST=8      # Indicates state of battery switch that the user desires.
                                      # 0 = off, 1 = on
    uint16 BATT_SWITCH_SETTING=16     # Indicates instantaneous setting of Battery Switch
    uint16 SLOW_SWITCH_SETTING=32     # Indicates instantaneous setting of Bettery Slow Switch
    uint16 EXT_VOLTAGE_DETECT=64      # Indicates if >170V is available on outside connector
    uint16 GAIN_SCHEDULE_MODE=128     # 0 = off. 1 = on.
    uint16 OVER_CURRENT_SHUTDOWN=256  # 1 indicates drive is in overcurrent shutdown.
    uint16 OVER_VOLTAGE_SHUTDOWN=512  # 1 indicates drive is in overvoltage shutdown.
    uint16 SPRING_RANGE_VALID=1024    # 1 indicates the current SC_Range value is valid (received within
                                      # approximately SC_RANGE_VALID_TIMEOUT milliseconds)
    uint16 PERMISSIVE_MODE=2048       # 1 indicates user has selected "permissive mode" which allows
                                      # WindingCurrent commands to be set even if SpringController Range
                                      # packets aren't arriving
    uint16 status                     # status bitfield
    ```

- type: `SCRecord`  
  topic: `/spring_data`  
  Description: Spring Controller Telemetry

    ```
    std_msgs/Header header  # contains time: .stamp
    int16 seq_num           # incremental number of messages sent
    int16 load_cell         # load cell reading (N)
    float32 range_finder    # piston position (in meters from full retraction)
    float32 upper_psi       # upper (top half of air spring) Nitrogen chamber pressure (PSI)
    float32 lower_psi       # lower (bottom half of air spring) Nitrogen chamber pressure (PSI)
    int64 epoch             # Timestamp in UNIX epoch time (seconds)
    float32 salinity        # (unused) salinity measurement
    float32 temperature     # (unused) temperature measurement

    # Status and control bitfields  
    uint16 RELIEF_VALVE_REQUEST=1     # Request to open/close valve
    uint16 RELIEF_VALVE_STATUS=128    # Status of Relief valve open/close
    uint16 PUMP_REQUEST=256           # Request to turn pump on or off
    uint16 PUMP_STATUS=512            # Status of pump switch
    uint16 PUMP_OVER_TEMP=1024        # Status of pump OverTemp signal
    uint16 PUMP_TOGGLE=2048           # Status of pump Toggle.
    uint16 TETHER_POWER_REQUEST=4096  # Request to turn tether power on or off
    uint16 TETHER_POWER_STATUS=8192   # Status of tether power relay
    uint16 LR_FAULT=16384             # Status of LRF fault input
    uint16 AUX_FAULT=32768            # Status of AUX fault input
    uint16 status                     # status bitfield
    ```

- type: `TFRecord`  
  topic: `/trefoil_data`  
  Description: Trefoil Controller Telemetry

    ```
    std_msgs/Header header         # contains time: .stamp
    int32 seq_num                  # incremental number of messages sent
    int32 power_timeouts           
    float32 tether_voltage         
    float32 battery_voltage        
    float32 pressure               

    # IMU containing orientation, angular velocity and linear acceleration
    sensor_msgs/Imu imu            

    # Magnetic field in Tesla
    sensor_msgs/MagneticField mag

    int16 status                   # status bitfield: contains trefoil door state
    int16 vpe_status               

    int32 comms_timeouts           
    int32 motor_status             
    int32 motor_current            
    int32 encoder                  
    ```

- type: `XBRecord`  
  topic: `/ahrs_data`  
  Description: Crossbow AHRS Telemetry

    ```
    std_msgs/Header header  # contains time: .stamp

    # IMU data:
    # - orientation: quaternion [x, y, z, w] (radians)
    # - angular_velocity: 3D vector in rad/s
    # - linear_acceleration: 3D vector in m/s² (not in g)
    sensor_msgs/Imu imu

    # GPS fix:
    # - latitude, longitude in degrees
    # - altitude in meters above the WGS84 ellipsoid
    sensor_msgs/NavSatFix gps

    # Velocity in the North-East-Down (NED) frame (m/s)
    # - x = North
    # - y = East
    # - z = Down (positive downwards)
    geometry_msgs/Vector3 ned_velocity

    # Temperature of the X-rate gyro (Celsius)
    sensor_msgs/Temperature x_rate_temp
    ```

### Consolidated (Time-Synchronized) Telemetry
Snapshot in time of all data from the buoy.

- type: `PBRecord`  
  topic: `/powerbuoy_data`  
  Description: All Telemetry in one time-synchronized message

    ```
    PCRecord pc  # Power Controller telemetry
    BCRecord bc  # Battery Controller telemetry
    SCRecord sc  # Spring Controller telemetry
    TFRecord tf  # Trefoil Controller telemetry
    XBRecord xb  # Crossbow AHRS telemetry
    ```

## Services (.srv)

- type: `PumpCommand.srv`  
  topic: `/pump_command`  
  Description: Turn pump on for a duration in minutes to raise mean piston position.

    ```
    float32 OFF=0
    float32 duration_mins  # OFF, 1..10
    ```

- type: `ValveCommand.srv`  
  topic: `/valve_command`  
  Description: Turn valve on for a duration in seconds to lower mean piston position.

    ```
    uint16 OFF=0
    uint16 duration_sec  # OFF, 1..127
    ```

- type: `TFSetPosCommand.srv`  
  topic: `/tf_set_pos_command`  
  Description: Open or close Trefoil doors

    ```
    uint16 OPEN=0
    uint16 CLOSED=1
    uint16 position
    ```

- type: `PCScaleCommand.srv`  
  topic: `/pc_scale_command`  
  Description: Set damping gain

    ```
    float32 scale  # 0.5..1.4
    ```

- type: `PCRetractCommand.srv`  
  topic: `/pc_retract_command`  
  Description: Set additional damping gain in retraction

    ```
    float32 retract  # 0.4..1.0
    ```

- type: `PCWindCurrCommand.srv`  
  topic: `/pc_wind_curr_command`  
  Description: Set winding current setpoint to control piston damping

    ```
    float32 wind_curr  # -35.0..35.0
    ```

- type: `PCBiasCurrCommand.srv`  
  topic: `/pc_bias_curr_command`  
  Description: Set winding bias current offset to control piston damping. This will move the piston
  back and forth.

    ```
    float32 bias_curr  # -15.0..15.0
    ```

- type: `PCWindCurrCommand.srv`  
  topic: `/pc_wind_curr_command`  
  Description: Set winding current setpoint to control piston damping

    ```
    float32 wind_curr  # -35.0..35.0
    ```


## Simulation-Only Data


#### Latent Data

- type: `LatentData.msg`  
  topic: `/latent_data`  
  Description: Latent Data provides values from the simulation that cannot be directly measured in
  the physical system and are only available as byproducts of models in simulation including forces
  and losses. The data are broken up into a few different data types contained within LatentData.

    ```
    std_msgs/Header header  # contains time: .stamp

    IncWaveHeight[] inc_wave_heights

    AirSpring upper_spring
    AirSpring lower_spring

    ElectroHydraulic electro_hydraulic  

    WaveBodyInteractions wave_body

    float64 piston_friction_force  # Newtons
    ```

- type: `IncWaveHeight.msg`  
  topic: N/A (see `LatentData.msg`)  
  Description:  Wave heights at specific locations within the simulation. These positions are set
  within the `model.sdf`.

    ```
    # relative_time is the time height was computed in decimal seconds
    # relative to simulation time in pose header.stamp
    # time of wave height = header.stamp + relative_time
    # Note: absolute_time from IncWaveHeight.srv converted to relative for response
    # Note: all fixed-points in SDF are computed with relative_time = 0.0
    float64 relative_time

    # For now, position is always in world coords (use_buoy_origin always False)
    bool use_buoy_origin

    # header.stamp = simulation time of computation
    #   time of wave height = header.stamp + relative_time
    # position = x, y, z(height above waterplane)
    # orientation = normal vector (slope of wave) at position
    geometry_msgs/PoseStamped pose
    ```

- type: `AirSpring.msg`  
  topic: N/A (see `LatentData.msg`)  
  Description: Air spring chamber modeling data including forces and losses

    ```
    float64 force  # Newtons
    float64 temperature  # K
    float64 heat_loss  # dQ/dt in Watts
    float64 piston_position  # meters
    float64 piston_velocity  # m/s
    float64 mass  # kg
    ```

- type: `ElectroHydraulic.msg`  
  topic: N/A (see `LatentData.msg`)  
  Description:  Electro-Hydraulic motor modeling data including forces and losses

    ```
    float64 rpm
    float64 upper_hydraulic_pressure  # Pa 
    float64 lower_hydraulic_pressure  # Pa 
    float64 force  # Newtons

    float64 supplied_hydraulic_power  # Watts

    float64 hydraulic_motor_loss
    float64 relief_valve_loss
    float64 motor_emf_power  # Watts

    float64 motor_drive_i2r_loss
    float64 motor_drive_switching_loss
    float64 motor_drive_friction_loss
    float64 load_dump_power
    float64 battery_i2r_loss
    float64 battery_storage_power
    ```

- type: `WaveBodyInteractions.msg`  
  topic: N/A (see `LatentData.msg`)  
  Description:  Hydrodynamical data including forces/torques and losses

    ```
    # Motion
    geometry_msgs/Pose pose  # position/orientation
    geometry_msgs/Twist twist  # linear/angular rates

    # Forces/Torques
    geometry_msgs/Wrench buoyancy
    geometry_msgs/Wrench radiation
    geometry_msgs/Wrench excitation

    # Total power = dot(force, velocity) + dot(torque, omega)
    float64 buoyancy_total_power  # Watts
    float64 radiation_total_power  # Watts
    float64 excitation_total_power  # Watts
    ```


#### Incident Wave Height Service

- type: `IncWaveHeight.srv`  
  topic: `/inc_wave_height`  
  Description: Retrieve wave heights in the simulation at various locations and times. See
  `IncWaveHeight.msg` above.

    IncWaveHeight Request

    ```
    # relative_time in decimal seconds to evaluate height at time relative to now (0)
    # (Note: may be in future for wave prediction)
    float64[] relative_time

    # absolute_time in epoch decimal seconds (from 01/01/1970 or sim start) to evaluate height
    # (Note: may be in future for wave prediction)
    float64[] absolute_time

    bool use_relative_time

    # x, y is relative to buoy origin; otherwise world origin
    bool use_buoy_origin

    # x, y to evaluate height above waterplane
    geometry_msgs/Point[] points
    ```

    IncWaveHeight Response

    ```
    IncWaveHeight[] heights
    bool valid
    ```
