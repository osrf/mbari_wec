# Quick Start -- Simple Linear Damper Controller (C++)

---

In this tutorial you will implement a simple linear damper controller for the piston in the WEC
Power-Take-Off (PTO). Given motor RPM, it outputs desired motor winding current (interpolated from
RPM->Torque lookup table) to generate a torque to resist piston velocity with a damping force.
Configurable gains (scale/retract factor) are applied before output. In the end, you will have a
working linear damper controller that is very close to the controller running on both the physical
and simulated buoy.

---

## Prerequisite

This tutorial assumes you are familiar the steps from the previous [tutorial](CPPTemplate.md)
and have built your own custom C++ ROS 2 controller package from the
[mbari_wec_template_cpp](https://github.com/mbari-org/mbari_wec_template_cpp) template
repository which we will use to implement a simple linear damper controller.

To begin, you should have a C++ ROS 2 controller package that looks similar to:

```
mbari_wec_linear_damper_cpp
    ├── CMakeLists.txt
    ├── config
    │   └── controller.yaml
    ├── include
    │   └── mbari_wec_linear_damper_cpp
    │       ├── controller.hpp
    │       └── control_policy.hpp
    ├── launch
    │   └── controller.launch.py
    ├── LICENSE
    ├── package.xml
    ├── README.md
    └── src
        └── controller.cpp
```

with the files modified from the previous tutorial. If you haven't already, follow the steps in
the above mentioned link to create a package for this tutorial named `mbari_wec_linear_damper_cpp`.

---

## Linear Damper ControlPolicy

A complete example starting from the template may be found
[here](https://github.com/mbari-org/mbari_wec_template_cpp/tree/linear_damper_example). Line numbers
in this tutorial correspond to the lines in relevant files in the full example.

### Parameters

Parameters for the controller are:

- `torque_constant`: Motor Torque Constant (N-m/Amp)  
  Constant to convert desired torque to applied motor winding current
<br/>
<br/>
- `n_spec`: Input Motor Speed (RPM) Breakpoints  
  \(N\) (RPM) is the input to the controller and `n_spec` are the x-components of the breakpoints
  \(\left(n\_spec, \frac{torque\_spec}{torque_constant}\right)\) for the interpolant,  
<br/>
  \(\hat{f}_{I}(n\_spec) = \frac{torque\_spec}{torque\_constant} \approx f_{I}(N) = I\)
<br/>
<br/>
- `torque_spec`: Desired Output Motor Torque (N-m) Breakpoints  
  Torque (N-m) is the eventual desired output of the controller given an input `N` (motor RPM) and
  `torque_spec` / `torque_constant` (Amps) are the y-components of the breakpoints for the
  interpolant. The controller actually outputs motor winding current (Amps) to generate a torque
  in the opposite direction of piston velocity to generate a damping force.

These can be configured using the `config/controller.yaml` file.

``` yaml linenums="1" title="config/controller.yaml"
/linear_damper:
  ros__parameters:
    torque_constant: 0.438
    n_spec: [0.0, 300.0, 600.0, 1000.0, 1700.0, 4400.0, 6790.0]
    torque_spec: [0.0, 0.0, 0.8, 2.9, 5.6, 9.8, 16.6]
```

As you can see, as motor speed increases, so does the damping torque. For low RPM (up to 300),
there is no damping.

Initialize these variables and create the interpolator,
`winding_current`, in `ControlPolicy` in
`include/mbari_wec_linear_damper_cpp/control_policy.hpp`. This
example makes use of `<simple_interp/interp1d.hpp>` from `mbari_wec_utils`, so don't forget to
include that as well as `<algorithm>` and `<vector>`.

``` cpp linenums="22" title="include/mbari_wec_linear_damper_cpp/control_policy.hpp"
#include <algorithm>
#include <vector>

#include <mbari_wec_linear_damper_cpp/controller.hpp>

// interp1d for rpm->winding current
#include <simple_interp/interp1d.hpp>


/* Simple Linear Damper Control Policy.
     Implements a simple linear damper controller for the piston in the WEC
     Power-Take-Off (PTO). Given motor RPM, outputs desired motor winding current (interpolated
     from RPM->Torque lookup table) to resist piston velocity. Configurable gains
     (scale/retract factor) are applied before output.
*/
struct ControlPolicy
{
  // declare/init any parameter variables here
  double Torque_constant;  // N-m/Amps
  std::vector<double> N_Spec;  // RPM
  std::vector<double> Torque_Spec;  // N-m
  std::vector<double> I_Spec;  // Amps

  // interpolator for rpm -> winding current
  simple_interp::Interp1d winding_current;

  ControlPolicy()
  : Torque_constant(0.438F),
    N_Spec{0.0F, 300.0F, 600.0F, 1000.0F, 1700.0F, 4400.0F, 6790.0F},
    Torque_Spec{0.0F, 0.0F, 0.8F, 2.9F, 5.6F, 9.8F, 16.6F},
    I_Spec(Torque_Spec.size(), 0.0F),
    winding_current(N_Spec, I_Spec)
  {
    update_params();
  }
```

Update the dependent variable, `I_Spec`, as well as the interpolator, `winding_current`.

``` cpp linenums="58" title="include/mbari_wec_linear_damper_cpp/control_policy.hpp"
  // Update dependent variables after reading in params
  void update_params()
  {
    std::transform(
      Torque_Spec.cbegin(), Torque_Spec.cend(),
      I_Spec.begin(),
      [tc = Torque_constant](const double & ts) {return ts / tc;});

    winding_current.update(N_Spec, I_Spec);
  }
```

Finally, define the `set_params` function of the `Controller` class and declare/get/set/update
these parameters from ROS 2 (as set in `config/controller.yaml`).

``` cpp linenums="115" title="include/mbari_wec_linear_damper_cpp/control_policy.hpp"
// Use ROS2 declare_parameter and get_parameter to set policy params
void Controller::set_params()
{
  this->declare_parameter("torque_constant", policy_->Torque_constant);
  policy_->Torque_constant = this->get_parameter("torque_constant").as_double();

  this->declare_parameter(
    "n_spec", std::vector<double>(
      policy_->N_Spec.begin(),
      policy_->N_Spec.end()));
  std::vector<double> temp_double_arr = this->get_parameter("n_spec").as_double_array();
  policy_->N_Spec.assign(temp_double_arr.begin(), temp_double_arr.end());

  this->declare_parameter(
    "torque_spec", std::vector<double>(
      policy_->Torque_Spec.begin(),
      policy_->Torque_Spec.end()));
  temp_double_arr = this->get_parameter("torque_spec").as_double_array();
  policy_->Torque_Spec.assign(temp_double_arr.begin(), temp_double_arr.end());

  // recompute any dependent variables
  policy_->update_params();
  RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), *policy_);
}
```

Add a helper function, for the `ControlPolicy` class for this example to report the
parameters used.

``` cpp linenums="90" title="include/mbari_wec_linear_damper_cpp/control_policy.hpp"
// Helper function to print policy parameters
std::ostream & operator<<(std::ostream & os, const ControlPolicy & policy)
{
  os << "ControlPolicy:" << std::endl;

  os << "\tTorque_constant: " << policy.Torque_constant << std::endl;

  os << "\tN_Spec: " << std::flush;
  std::copy(policy.N_Spec.cbegin(), policy.N_Spec.cend(), std::ostream_iterator<double>(os, ","));
  os << "\b \b" << std::endl;

  os << "\tTorque_Spec: " << std::flush;
  std::copy(
    policy.Torque_Spec.cbegin(),
    policy.Torque_Spec.cend(),
    std::ostream_iterator<double>(os, ","));
  os << "\b \b" << std::endl;

  os << "\tI_Spec: " << std::flush;
  std::copy(policy.I_Spec.cbegin(), policy.I_Spec.cend(), std::ostream_iterator<double>(os, ","));
  os << "\b \b" << std::endl;

  return os;
}
```

### Control Policy Target

To implement the torque control control policy, we use the `target` function in `ControlPolicy`.
This is where we accept feedback data and return a command value. In this case, we need the motor
`rpm`, and the gains applied to the winding current damping, `scale_factor` and `retract_factor`.
Typical values for these gains are

- scale_factor = 1
- retract_factor = 0.6

``` cpp linenums="69" title="include/mbari_wec_linear_damper_cpp/control_policy.hpp"
  // Calculate target value from feedback inputs
  double target(
    const double & rpm,
    const double & scale_factor,
    const double & retract_factor)
  {
    double N = fabs(rpm);
    double I = winding_current.eval(N);

    // apply damping gain
    I *= scale_factor;

    // Hysteresis due to gravity / wave assist
    if (rpm > 0.0F) {
      I *= -retract_factor;
    }

    return I;
  }
```

So, as you can see we apply a positive damping torque when RPM is negative (piston extending), and
a positive damping torque when RPM is positive (piston retracting). The damping torque required is
reduced when retracting.

### Controller

All that is left is to connect the necessary feedback data to the `ControlPolicy`. In this case,
`rpm`, `scale`, and `retract` are present in `buoy_interfaces.msg.PCRecord` on the `/power_data`
topic published by the Power Controller running on the buoy.

To access the data, all that is required is to define the callback
`void Controller::power_callback(const buoy_interfaces::msg::PCRecord & data)`
in the `Controller` class, and pass the data to `this->policy_->target` to get the desired winding
current command. Various commands are available, and this time we will be using  
`this->send_pc_wind_curr_command(wind_curr_amps);`

``` cpp linenums="54" title="src/controller.cpp"
// Callback for '/power_data' topic from Power Controller
void Controller::power_callback(const buoy_interfaces::msg::PCRecord & data)
{
  // Update class variables, get control policy target, send commands, etc.
  // get target value from control policy
  double wind_curr = this->policy_->target(data.rpm, data.scale, data.retract);

  RCLCPP_INFO_STREAM(
    rclcpp::get_logger(
      this->get_name()),
    "WindingCurrent: f(" << data.rpm << ", " << data.scale << ", " << data.retract << ") = " <<
      wind_curr);

  auto future = this->send_pc_wind_curr_command(wind_curr);
}
```

Finally, let's set the Power Controller's publish rate to the maximum of 50Hz. Uncomment the line
to set the PC Pack Rate in `Controller` constructor:

``` cpp linenums="22" title="src/controller.cpp"
Controller::Controller(const std::string & node_name)
: buoy_api::Interface<Controller>(node_name),
  policy_(std::make_unique<ControlPolicy>())
{
  this->set_params();

  // set packet rates from controllers here
  // controller defaults to publishing @ 10Hz
  // call these to set rate to 50Hz or provide argument for specific rate
  // this->set_sc_pack_rate_param();  // set SC publish rate to 50Hz
  this->set_pc_pack_rate_param();  // set PC publish rate to 50Hz
}
```

In this tutorial, we've named this controller `linear_damper`. Don't forget to update controller
names along with other changes according to the previous tutorial.

## Try It Out

Make sure to build and source your workspace. This tutorial assumes you cloned your package to
`~/controller_ws/src` and you have sourced `mbari_wec_gz` and `mbari_wec_utils`
```
$ cd ~/controller_ws
$ colcon build
$ source install/local_setup.bash
```

We will be using `ros2 launch` and `launch/controller.launch.py` to run our new controller.

To run the controller along with the simulation, launch your
controller:  
`$ ros2 launch mbari_wec_linear_damper_cpp controller.launch.py`

Then, in another terminal (with `mbari_wec_gz` sourced), launch the sim:  
`$ ros2 launch buoy_gazebo mbari_wec.launch.py`  
and click the play button.


You should see output similar to:

```
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215528.623306255] [mbari_wec_linear_damper_cpp]: Found all required services.
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215528.623437968] [mbari_wec_linear_damper_cpp]: ControlPolicy:
[mbari_wec_linear_damper_cpp-1]         Torque_constant: 0.438
[mbari_wec_linear_damper_cpp-1]         N_Spec: 0,300,600,1000,1700,4400,6790
[mbari_wec_linear_damper_cpp-1]         Torque_Spec: 0,0,0.8,2.9,5.6,9.8,16.6
[mbari_wec_linear_damper_cpp-1]         I_Spec: 0,0,1.82648,6.621,12.7854,22.3744,37.8995
[mbari_wec_linear_damper_cpp-1]
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215528.623585767] [mbari_wec_linear_damper_cpp]: WindingCurrent: f(2911.62, 1, 0.6) = -10.2531
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215528.623769091] [mbari_wec_linear_damper_cpp]: Successfully set publish_rate for power_controller
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215528.723139301] [mbari_wec_linear_damper_cpp]: WindingCurrent: f(2881.34, 1, 0.6) = -10.1886
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215528.723199020] [mbari_wec_linear_damper_cpp]: WindingCurrent: f(2881.34, 1, 0.6) = -10.1886
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215528.743295542] [mbari_wec_linear_damper_cpp]: WindingCurrent: f(2864.91, 1, 0.6) = -10.1535
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215528.763406662] [mbari_wec_linear_damper_cpp]: WindingCurrent: f(2845.41, 1, 0.6) = -10.112
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215528.783518884] [mbari_wec_linear_damper_cpp]: WindingCurrent: f(2822.43, 1, 0.6) = -10.063
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215528.803625212] [mbari_wec_linear_damper_cpp]: WindingCurrent: f(2796.29, 1, 0.6) = -10.0073
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215528.823736947] [mbari_wec_linear_damper_cpp]: WindingCurrent: f(2767.06, 1, 0.6) = -9.94502
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215528.843817290] [mbari_wec_linear_damper_cpp]: WindingCurrent: f(2734.81, 1, 0.6) = -9.87631
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215528.863931284] [mbari_wec_linear_damper_cpp]: WindingCurrent: f(2699.67, 1, 0.6) = -9.80143
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215528.884041064] [mbari_wec_linear_damper_cpp]: WindingCurrent: f(2661.74, 1, 0.6) = -9.7206
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215528.904159386] [mbari_wec_linear_damper_cpp]: WindingCurrent: f(2621.09, 1, 0.6) = -9.63398
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215528.924232170] [mbari_wec_linear_damper_cpp]: WindingCurrent: f(2577.85, 1, 0.6) = -9.54184
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215528.944361837] [mbari_wec_linear_damper_cpp]: WindingCurrent: f(2532.15, 1, 0.6) = -9.44445
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215528.964467851] [mbari_wec_linear_damper_cpp]: WindingCurrent: f(2484.08, 1, 0.6) = -9.34203
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215528.984588134] [mbari_wec_linear_damper_cpp]: WindingCurrent: f(2433.76, 1, 0.6) = -9.23479
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215529.004697490] [mbari_wec_linear_damper_cpp]: WindingCurrent: f(2381.3, 1, 0.6) = -9.12301
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215529.024807195] [mbari_wec_linear_damper_cpp]: WindingCurrent: f(2326.81, 1, 0.6) = -9.00691
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215529.044928940] [mbari_wec_linear_damper_cpp]: WindingCurrent: f(2270.4, 1, 0.6) = -8.88669
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215529.065039660] [mbari_wec_linear_damper_cpp]: WindingCurrent: f(2212.17, 1, 0.6) = -8.76262
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215529.085145551] [mbari_wec_linear_damper_cpp]: WindingCurrent: f(2152.24, 1, 0.6) = -8.63491
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215529.105256007] [mbari_wec_linear_damper_cpp]: WindingCurrent: f(2090.71, 1, 0.6) = -8.50379
[mbari_wec_linear_damper_cpp-1] [INFO] [1680215529.125363653] [mbari_wec_linear_damper_cpp]: WindingCurrent: f(2027.67, 1, 0.6) = -8.36947
```
