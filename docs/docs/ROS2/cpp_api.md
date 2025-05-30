# class Interface

*Defined at buoy_api_cpp/include/buoy_api/interface.hpp#161*



**brief** ROS 2 Interface node for commanding and subscribing to buoy controllers and sensors.

 This template class uses the Curiously Recurring Template Pattern (CRTP) to provide a compile-time
 polymorphic interface for creating node-based controllers. By using CRTP, derived classes can
 override callback methods and parameter-setting behavior without incurring the overhead of virtual
 dispatch. The derived class must pass itself as the template parameter to `Interface`, enabling the
 base class to call into user-defined implementations.

 Provides service clients and functions to send commands to and receive telemetry from the MBARI WEC
 controllers:

 - AHRS
 - Power
 - Spring
 - Battery
 - Trefoil

 If the user has overridden one of these callbacks in their derived class, the corresponding topic
 subscriber will be set up and routed to their implementation. If not, that topic will not be
 subscribed to. The relevant callbacks include:

 - ahrs_callback
 - battery_callback
 - spring_callback
 - power_callback
 - trefoil_callback
 - powerbuoy_callback

Template argument: **ControllerImplCRTP** The concrete controller class that inherits from this
interface. It must implement any callbacks or parameter-setting routines it needs.

**How to Use:**

 1. Include the header for `Interface`:

    ```cpp
    #include <buoy_api/interface.hpp>
    ```

 2. Forward-declare any policies or helper classes you will use:

    ```cpp
    struct PBTorqueControlPolicy;  // defined by user in torque_control_policy.hpp
    ```

 3. Define your controller class by inheriting from `buoy_api::Interface<YourClass>`
    making sure to add `friend CRTP`:

    ```cpp
    class PBTorqueController final : public buoy_api::Interface<PBTorqueController>

    {

    public:

      explicit PBTorqueController(const std::string & node_name);

      ~PBTorqueController() = default;



    private:

      friend CRTP;  // Enables base to access overrides.

      void set_params() final;

      void power_callback(const buoy_interfaces::msg::PCRecord & data);

      std::unique_ptr<PBTorqueControlPolicy> policy_;

    };
    ```

 4. Implement `set_params()` to declare or update ROS2 parameters to update your policy class.

 5. Override any telemetry callbacks to process incoming data.

 6. Construct your controller in your `main()` function, passing in the desired node name.    The base `Interface` handles common setup: parameters, services, publishers, and subscribers.

Benefits of CRTP in This Context:

  - **Zero-cost abstraction**: No virtual table; callbacks are resolved at compile time.
  - **Flexible overrides**: Only override what you use.
  - **Simplified boilerplate**: Base class manages ROS2 setup.

Inherits from ROS 2 Node


## Functions


### Interface<ControllerImplCRTP\>

```cpp
public void Interface<ControllerImplCRTP>(const std::string & node_name)
```

*Defined at buoy_api_cpp/include/buoy_api/interface.hpp#166*


### Interface<ControllerImplCRTP\>

```cpp
public void Interface<ControllerImplCRTP>(const std::string & node_name, const bool wait_for_services, const bool check_for_services)
```

*Defined at buoy_api_cpp/include/buoy_api/interface.hpp#178*

**brief** Initialize the Interface node.

**node_name** Name of the ROS2 node.  
**wait_for_services** If true and check_for_services, block until services are available.  
**check_for_services** If true, attempt to verify service availability before use.  


### use_sim_time

```cpp
public void use_sim_time(bool enable)
```

*Defined at buoy_api_cpp/include/buoy_api/interface.hpp#409*

**brief** Enable/Disable using sim time in Node clock from /clock.

**enable** True to use /clock, False to use system time.


### set_pc_pack_rate

```cpp
public void set_pc_pack_rate(const uint8_t & rate_hz)
```

*Defined at buoy_api_cpp/include/buoy_api/interface.hpp#422*

**brief** Set publish rate of PC Microcontroller telemetry.

**rate_hz** Desired publish rate in Hz.


### set_sc_pack_rate

```cpp
public void set_sc_pack_rate(const uint8_t & rate_hz)
```

*Defined at buoy_api_cpp/include/buoy_api/interface.hpp#439*

**brief** Set publish rate of SC Microcontroller telemetry.

**rate_hz** Desired publish rate in Hz.


### set_pc_pack_rate_param

```cpp
public void set_pc_pack_rate_param(const double & rate_hz)
```

*Defined at buoy_api_cpp/include/buoy_api/interface.hpp#456*

**brief** Set publish rate of PC Microcontroller telemetry via parameter server.

**rate_hz** Desired publish rate in Hz.


### set_sc_pack_rate_param

```cpp
public void set_sc_pack_rate_param(const double & rate_hz)
```

*Defined at buoy_api_cpp/include/buoy_api/interface.hpp#477*

**brief** Set publish rate of SC Microcontroller telemetry via parameter server.

**rate_hz** Desired publish rate in Hz.


### send_valve_command

```cpp
public shared_future send_valve_command(const uint16_t & duration_sec)
```

*Defined at buoy_api_cpp/include/buoy_api/interface.hpp#553*

**brief** Turn valve on for a duration to lower mean piston position.

**duration_sec** Valve on duration in seconds.

**return** A future containing the service response.


### send_pump_command

```cpp
public shared_future send_pump_command(const float & duration_mins)
```

*Defined at buoy_api_cpp/include/buoy_api/interface.hpp#570*

**brief** Turn pump on for a duration to raise mean piston position.

**duration_mins** Pump on duration in minutes.

**return** A future containing the service response.


### send_pc_wind_curr_command

```cpp
public shared_future send_pc_wind_curr_command(const float & wind_curr)
```

*Defined at buoy_api_cpp/include/buoy_api/interface.hpp#587*

**brief** Set winding current setpoint to control piston damping.

**wind_curr** Wind current setpoint in Amps.

**return** A future containing the service response.


### send_pc_bias_curr_command

```cpp
public shared_future send_pc_bias_curr_command(const float & bias_curr)
```

*Defined at buoy_api_cpp/include/buoy_api/interface.hpp#606*

**brief** Set bias current setpoint to control piston damping offset.
A high bias in either direction will move the piston back and forth.

**bias_curr** Bias current setpoint in Amps.

**return** A future containing the service response.


### send_pc_scale_command

```cpp
public shared_future send_pc_scale_command(const float & scale)
```

*Defined at buoy_api_cpp/include/buoy_api/interface.hpp#623*

**brief** Set damping gain.

**scale** Damping gain.

**return** A future containing the service response.


### send_pc_retract_command

```cpp
public shared_future send_pc_retract_command(const float & retract)
```

*Defined at buoy_api_cpp/include/buoy_api/interface.hpp#640*

**brief** Set additional damping gain in the piston retract direction.

**retract** Additional damping gain for retraction.

**return** A future containing the service response.


### set_params

```cpp
protected void set_params()
```

*Defined at buoy_api_cpp/include/buoy_api/interface.hpp#659*

**brief** Set user-defined Node parameters (e.g., custom controller gains).


### ahrs_callback

```cpp
protected void ahrs_callback(const buoy_interfaces::msg::XBRecord & )
```

*Defined at buoy_api_cpp/include/buoy_api/interface.hpp#666*

**brief** Override this function to subscribe to /ahrs_data to receive XBRecord telemetry.

**data** Incoming XBRecord.


### battery_callback

```cpp
protected void battery_callback(const buoy_interfaces::msg::BCRecord & )
```

*Defined at buoy_api_cpp/include/buoy_api/interface.hpp#673*

**brief** Override this function to subscribe to /battery_data to receive BCRecord telemetry.

**data** Incoming BCRecord.


### spring_callback

```cpp
protected void spring_callback(const buoy_interfaces::msg::SCRecord & )
```

*Defined at buoy_api_cpp/include/buoy_api/interface.hpp#680*

**brief** Override this function to subscribe to /spring_data to receive SCRecord telemetry.

**data** Incoming SCRecord.


### power_callback

```cpp
protected void power_callback(const buoy_interfaces::msg::PCRecord & )
```

*Defined at buoy_api_cpp/include/buoy_api/interface.hpp#687*

**brief** Override this function to subscribe to /power_data to receive PCRecord telemetry.

**data** Incoming PCRecord.


### trefoil_callback

```cpp
protected void trefoil_callback(const buoy_interfaces::msg::TFRecord & )
```

*Defined at buoy_api_cpp/include/buoy_api/interface.hpp#694*

**brief** Override this function to subscribe to /trefoil_data to receive TFRecord telemetry.

**data** Incoming TFRecord.


### powerbuoy_callback

```cpp
protected void powerbuoy_callback(const buoy_interfaces::msg::PBRecord & )
```

*Defined at buoy_api_cpp/include/buoy_api/interface.hpp#701*

**brief** Override this function to subscribe to /powerbuoy_data to receive PBRecord telemetry.

**data** Incoming PBRecord containing a slice of all microcontroller telemetry data.
