<a id="buoy_api.interface"></a>

# buoy\_api.interface

<a id="buoy_api.interface.Interface"></a>

## Interface Objects

```python
class Interface(Node)
```

ROS2 Interface node for commanding and subscribing to buoy controllers and sensors.

Provides service clients and functions to send commands to and recieve telemetry from the
MBARI WEC controllers:

- AHRS
- Power
- Spring
- Battery
- Trefoil

If user has overridden one of these callbacks in their user-derived class, this will
subscribe to the correct topic and use their callback implementation. If user did not define
one, a subscriber for that topic will not be set up:

- self.ahrs_callback
- self.battery_callback
- self.spring_callback
- self.power_callback
- self.trefoil_callback
- self.powerbuoy_callback

<a id="buoy_api.interface.Interface.__init__"></a>

#### \_\_init\_\_

```python
def __init__(node_name,
             wait_for_services=False,
             check_for_services=True,
             **kwargs)
```

Initialize the Interface node.

**Arguments**:

- `node_name` (`str`): name of the ROS2 node
- `check_for_services` (`bool`): if True, attempt to verify service availability before use
- `wait_for_services` (`bool`): if True and if check_for_services, block until all services are available
- `kwargs`: additional keyword arguments forwarded to ROS 2 Node

<a id="buoy_api.interface.Interface.use_sim_time"></a>

#### use\_sim\_time

```python
def use_sim_time(enable=True)
```

Enable/Disable using sim time in Node clock from /clock.

**Arguments**:

- `enable` (`bool`): True to use /clock, False to use system time

<a id="buoy_api.interface.Interface.set_pc_pack_rate_param"></a>

#### set\_pc\_pack\_rate\_param

```python
def set_pc_pack_rate_param(rate_hz=50.0, blocking=True)
```

Set publish rate of PC Microcontroller telemetry.

**Arguments**:

- `rate_hz` (`float`): desired publish rate in Hz
- `blocking` (`bool`): if True, wait for the service call to complete

<a id="buoy_api.interface.Interface.set_sc_pack_rate_param"></a>

#### set\_sc\_pack\_rate\_param

```python
def set_sc_pack_rate_param(rate_hz=50.0, blocking=True)
```

Set publish rate of SC Microcontroller telemetry.

**Arguments**:

- `rate_hz` (`float`): desired publish rate in Hz
- `blocking` (`bool`): if True, wait for the service call to complete

<a id="buoy_api.interface.Interface.set_pc_pack_rate"></a>

#### set\_pc\_pack\_rate

```python
def set_pc_pack_rate(rate_hz=50, blocking=True)
```

Set publish rate of PC Microcontroller telemetry.

**Arguments**:

- `rate_hz` (`float`): desired publish rate in Hz
- `blocking` (`bool`): if True, wait for the service call to complete

<a id="buoy_api.interface.Interface.set_sc_pack_rate"></a>

#### set\_sc\_pack\_rate

```python
def set_sc_pack_rate(rate_hz=50, blocking=True)
```

Set publish rate of SC Microcontroller telemetry.

**Arguments**:

- `rate_hz` (`float`): desired publish rate in Hz
- `blocking` (`bool`): if True, wait for the service call to complete

<a id="buoy_api.interface.Interface.send_pump_command"></a>

#### send\_pump\_command

```python
def send_pump_command(duration_mins, blocking=True)
```

Turn pump on for a duration in minutes to raise mean piston position.

**Arguments**:

- `duration_mins` (`float`): pump on duration in minutes
- `blocking` (`bool`): if True, wait for the service call to complete

<a id="buoy_api.interface.Interface.send_valve_command"></a>

#### send\_valve\_command

```python
def send_valve_command(duration_sec, blocking=True)
```

Turn valve on for a duration in seconds to lower mean piston position.

**Arguments**:

- `duration_sec` (`float`): valve on duration in seconds
- `blocking` (`bool`): if True, wait for the service call to complete

<a id="buoy_api.interface.Interface.send_pc_wind_curr_command"></a>

#### send\_pc\_wind\_curr\_command

```python
def send_pc_wind_curr_command(wind_curr, blocking=True)
```

Set winding current setpoint to control piston damping.

**Arguments**:

- `wind_curr` (`float`): wind current setpoint in Amps
- `blocking` (`bool`): if True, wait for the service call to complete

<a id="buoy_api.interface.Interface.send_pc_bias_curr_command"></a>

#### send\_pc\_bias\_curr\_command

```python
def send_pc_bias_curr_command(bias_curr, blocking=True)
```

Set bias current setpoint to control piston damping offset.

A High bias in either direction will move the piston back and forth

**Arguments**:

- `bias_curr` (`float`): bias current setpoint in Amps
- `blocking` (`bool`): if True, wait for the service call to complete

<a id="buoy_api.interface.Interface.send_pc_scale_command"></a>

#### send\_pc\_scale\_command

```python
def send_pc_scale_command(scale, blocking=True)
```

Set damping gain.

**Arguments**:

- `scale` (`float`): damping gain
- `blocking` (`bool`): if True, wait for the service call to complete

<a id="buoy_api.interface.Interface.send_pc_retract_command"></a>

#### send\_pc\_retract\_command

```python
def send_pc_retract_command(retract, blocking=True)
```

Set additional damping gain in the piston retract direction.

**Arguments**:

- `retract` (`float`): additional damping gain for retraction
- `blocking` (`bool`): if True, wait for the service call to complete

<a id="buoy_api.interface.Interface.set_params"></a>

#### set\_params

```python
def set_params()
```

Set user-defined Node params (e.g. custom controller gains).

<a id="buoy_api.interface.Interface.ahrs_callback"></a>

#### ahrs\_callback

```python
def ahrs_callback(data)
```

Override this function to subscribe to /ahrs_data to receive XBRecord telemetry.

**Arguments**:

- `data`: incoming XBRecord

<a id="buoy_api.interface.Interface.battery_callback"></a>

#### battery\_callback

```python
def battery_callback(data)
```

Override this function to subscribe to /battery_data to receive BCRecord telemetry.

**Arguments**:

- `data`: incoming BCRecord

<a id="buoy_api.interface.Interface.spring_callback"></a>

#### spring\_callback

```python
def spring_callback(data)
```

Override this function to subscribe to /spring_data to receive SCRecord telemetry.

**Arguments**:

- `data`: incoming SCRecord

<a id="buoy_api.interface.Interface.power_callback"></a>

#### power\_callback

```python
def power_callback(data)
```

Override this function to subscribe to /power_data to receive PCRecord telemetry.

**Arguments**:

- `data`: incoming PCRecord

<a id="buoy_api.interface.Interface.trefoil_callback"></a>

#### trefoil\_callback

```python
def trefoil_callback(data)
```

Override this function to subscribe to /trefoil_data to receive TFRecord telemetry.

**Arguments**:

- `data`: incoming TFRecord

<a id="buoy_api.interface.Interface.powerbuoy_callback"></a>

#### powerbuoy\_callback

```python
def powerbuoy_callback(data)
```

Override this function to subscribe to /powerbuoy_data to receive PBRecord telemetry.

PBRecord contains a slice of all microcontroller's telemetry data

**Arguments**:

- `data`: incoming PBRecord

