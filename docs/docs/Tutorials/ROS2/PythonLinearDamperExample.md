# Quick Start -- Simple Linear Damper Controller (Python)

---

In this tutorial you will implement a simple linear damper controller for the piston in the WEC
Power-Take-Off (PTO). Given motor RPM, it outputs desired motor winding current (interpolated from
RPM->Torque lookup table) to generate a torque to resist piston velocity with a damping force.
Configurable gains (scale/retract factor) are applied before output. In the end, you will have a
working linear damper controller that is very close to the controller running on both the physical
and simulated buoy.

---

## Prerequisite

This tutorial assumes you have followed the steps from the previous
[tutorial](PythonTemplate.md) on creating and customizing your own Python ROS 2 controller package
from the [mbari_wec_template_py](https://github.com/mbari-org/mbari_wec_template_py) template
repository.

To begin, you should have a Python ROS 2 controller package that looks similar to:

```
mbari_wec_linear_damper_py
├── config
│   └── controller.yaml
├── CONTRIBUTING.md
├── launch
│   └── controller.launch.py
├── LICENSE
├── mbari_wec_linear_damper_py
│   ├── controller.py
│   ├── __init__.py
├── package.xml
├── README.md
├── resource
│   └── mbari_wec_linear_damper_py
├── setup.cfg
├── setup.py
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

with the files modified from the previous tutorial. If you haven't already, follow the steps in
the above mentioned link to create a package for this tutorial named `mbari_wec_linear_damper_py`.

---

## Linear Damper ControlPolicy

A complete example starting from the template may be found
[here](https://github.com/mbari-org/mbari_wec_template_py/tree/linear_damper_example). Line numbers
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

Initialize these variables in `ControlPolicy` in `mbari_wec_linear_damper_py/controller.py`. This
example makes use of `numpy.array`.

``` py linenums="24" title="mbari_wec_linear_damper_py/controller.py"
class ControlPolicy(object):
    """
    Simple Linear Damper Control Policy.
    Implements a simple linear damper controller for the piston in the WEC
    Power-Take-Off (PTO). Given motor RPM, outputs desired motor winding current (interpolated
    from RPM->Torque lookup table) to resist piston velocity. Configurable gains
    (scale/retract factor) are applied before output.
    """

    def __init__(self):
        # Define any parameter variables here
        self.Torque_constant = 0.438  # N-m/Amps
        # Desired damping Torque vs RPM relationship
        self.N_Spec = np.array([0.0, 300.0, 600.0, 1000.0, 1700.0, 4400.0, 6790.0])  # RPM
        self.Torque_Spec = np.array([0.0, 0.0, 0.8, 2.9, 5.6, 9.8, 16.6])  # N-m
```

Update the dependent variable, `I_Spec`, and create the interpolator, `windcurr_interp1d`, which
uses `interp1d` from `scipy.interpolate`.

``` py linenums="43" title="mbari_wec_linear_damper_py/controller.py"
    def update_params(self):
        """Update dependent variables after reading in params."""
        # Convert to Motor Winding Current vs RPM and generate interpolator for f(RPM) = I
        self.I_Spec = self.Torque_Spec / self.Torque_constant  # Amps
        self.windcurr_interp1d = interpolate.interp1d(self.N_Spec, self.I_Spec,
                                                      fill_value=self.I_Spec[-1],
                                                      bounds_error=False)
```

Finally, in the `Controller` class, declare/get/set/update these parameters from ROS 2 (as set in
`config/controller.yaml`).

``` py linenums="118" title="mbari_wec_linear_damper_py/controller.py"
    def set_params(self):
        """Use ROS2 declare_parameter and get_parameter to set policy params."""
        self.declare_parameter('torque_constant', self.policy.Torque_constant)
        self.policy.Torque_constant = \
            self.get_parameter('torque_constant').get_parameter_value().double_value

        self.declare_parameter('n_spec', self.policy.N_Spec.tolist())
        self.policy.N_Spec = \
            np.array(self.get_parameter('n_spec').get_parameter_value().double_array_value)

        self.declare_parameter('torque_spec', self.policy.Torque_Spec.tolist())
        self.policy.Torque_Spec = \
            np.array(self.get_parameter('torque_spec').get_parameter_value().double_array_value)

        # recompute any dependent variables
        self.policy.update_params()
        self.get_logger().info(str(self.policy))
```

Add a helper function, `__str__`, in the `ControlPolicy` class for this example to report the
parameters used.

``` py linenums="66" title="mbari_wec_linear_damper_py/controller.py"
    def __str__(self):
        return """ControlPolicy:
\tTorque_constant: {tc}
\tN_Spec: {nspec}
\tTorque_Spec: {tspec}
\tI_Spec: {ispec}""".format(tc=self.Torque_constant,
                            nspec=self.N_Spec,
                            tspec=self.Torque_Spec,
                            ispec=self.I_Spec)
```

### Control Policy Target

To implement the torque control control policy, we use the `target` function in `ControlPolicy`.
This is where we accept feedback data and return a command value. In this case, we need the motor
`rpm`, and the gains applied to the winding current damping, `scale_factor` and `retract_factor`.
Typical values for these gains are

- scale_factor = 1
- retract_factor = 0.6

``` py linenums="52" title="mbari_wec_linear_damper_py/controller.py"
    def target(self, rpm, scale_factor, retract_factor):
        """Calculate target value from feedback inputs."""
        N = abs(rpm)
        I = self.windcurr_interp1d(N)

        # Apply damping gain
        I *= scale_factor

        # Hysteresis due to gravity assist
        if rpm > 0.0:
            I *= -retract_factor

        return float(I)
```

So, as you can see we apply a positive damping torque when `N` is negative (piston extending), and
a positive damping torque when `N` is positive (piston retracting). The damping torque required is
reduced when retracting.

### Controller

All that is left is to connect the necessary feedback data to the `ControlPolicy`. In this case,
`rpm`, `scale`, and `retract` are present in `buoy_interfaces.msg.PCRecord` on the `/power_data`
topic published by the Power Controller running on the buoy.

To access the data, all that is required is to define the callback `def power_callback(self, data)`
in the `Controller` class, and pass the data to `self.policy.target` to get the desired winding
current command. Various commands are available, and this time we will be using  
`self.send_pc_wind_curr_command(wind_curr, blocking=False)`

``` py linenums="107" title="mbari_wec_linear_damper_py/controller.py"
    def power_callback(self, data):
        """Provide feedback of '/power_data' topic from Power Controller."""
        # Update class variables, get control policy target, send commands, etc.
        wind_curr = self.policy.target(data.rpm, data.scale, data.retract)

        self.get_logger().info('WindingCurrent:' +
                               f' f({data.rpm:.02f}, {data.scale:.02f}, {data.retract:.02f})' +
                               f' = {wind_curr:.02f}')

        self.send_pc_wind_curr_command(wind_curr, blocking=False)
```

Finally, let's set the Power Controller's publish rate to the maximum of 50Hz. Uncomment the line
to set the PC Pack Rate in `Controller.__init__`:

``` py linenums="79" title="mbari_wec_linear_damper_py/controller.py"
    def __init__(self):
        super().__init__('linear_damper')

        self.policy = ControlPolicy()
        self.set_params()

        # set packet rates from controllers here
        # controller defaults to publishing feedback @ 10Hz
        # call these to set rate to 50Hz or provide argument for specific rate
        self.set_pc_pack_rate_param()  # set PC feedback publish rate to 50Hz
```

## Try It Out

We will be using `ros2 launch` and `launch/controller.launch.py` to run our new controller.

To run the controller along with the simulation, first source your workspace. Then, launch your controller:  
`$ ros2 launch mbari_wec_linear_damper_py controller.launch.py`

Then, launch the sim:  
`$ ros2 launch buoy_gazebo mbari_wec.launch.py`  
and click the play button.


You should see output similar to:

```
[linear_damper-1] [INFO] [1677864397.617058507] [linear_damper]: Found all required services.
[linear_damper-1] [INFO] [1677864397.618426488] [linear_damper]: ControlPolicy:
[linear_damper-1]       Torque_constant: 0.438
[linear_damper-1]       N_Spec: [   0.  300.  600. 1000. 1700. 4400. 6790.]
[linear_damper-1]       Torque_Spec: [ 0.   0.   0.8  2.9  5.6  9.8 16.6]
[linear_damper-1]       I_Spec: [ 0.          0.          1.82648402  6.62100457 12.78538813 22.37442922
[linear_damper-1]  37.89954338]
[linear_damper-1] [INFO] [1677864197.432679525] [linear_damper]: WindingCurrent: f(4962.91, 1.00, 0.60) = -15.62
[linear_damper-1] [INFO] [1677864197.532727531] [linear_damper]: WindingCurrent: f(7764.73, 1.00, 0.60) = -22.74
[linear_damper-1] [INFO] [1677864197.632748699] [linear_damper]: WindingCurrent: f(10504.88, 1.00, 0.60) = -22.74
[linear_damper-1] [INFO] [1677864197.732851121] [linear_damper]: WindingCurrent: f(11491.33, 1.00, 0.60) = -22.74
[linear_damper-1] [INFO] [1677864197.833078440] [linear_damper]: WindingCurrent: f(11075.84, 1.00, 0.60) = -22.74
[linear_damper-1] [INFO] [1677864197.933050356] [linear_damper]: WindingCurrent: f(9546.51, 1.00, 0.60) = -22.74
[linear_damper-1] [INFO] [1677864198.033185882] [linear_damper]: WindingCurrent: f(7499.68, 1.00, 0.60) = -22.74
[linear_damper-1] [INFO] [1677864198.133197926] [linear_damper]: WindingCurrent: f(5190.35, 1.00, 0.60) = -16.51
[linear_damper-1] [INFO] [1677864198.233322713] [linear_damper]: WindingCurrent: f(2353.02, 1.00, 0.60) = -9.06
[linear_damper-1] [INFO] [1677864198.333507127] [linear_damper]: WindingCurrent: f(-257.59, 1.00, 0.60) = 0.00
[linear_damper-1] [INFO] [1677864198.433489830] [linear_damper]: WindingCurrent: f(-2185.58, 1.00, 0.60) = 14.51
[linear_damper-1] [INFO] [1677864198.533538450] [linear_damper]: WindingCurrent: f(-2987.98, 1.00, 0.60) = 17.36
[linear_damper-1] [INFO] [1677864198.633671249] [linear_damper]: WindingCurrent: f(-3513.15, 1.00, 0.60) = 19.22
[linear_damper-1] [INFO] [1677864198.733703803] [linear_damper]: WindingCurrent: f(-3738.12, 1.00, 0.60) = 20.02
[linear_damper-1] [INFO] [1677864198.833889518] [linear_damper]: WindingCurrent: f(-3751.64, 1.00, 0.60) = 20.07
[linear_damper-1] [INFO] [1677864198.933993414] [linear_damper]: WindingCurrent: f(-3595.71, 1.00, 0.60) = 19.52
[linear_damper-1] [INFO] [1677864199.034078009] [linear_damper]: WindingCurrent: f(-3306.87, 1.00, 0.60) = 18.49
[linear_damper-1] [INFO] [1677864199.134273438] [linear_damper]: WindingCurrent: f(-3012.52, 1.00, 0.60) = 17.45
[linear_damper-1] [INFO] [1677864199.234371669] [linear_damper]: WindingCurrent: f(-2617.97, 1.00, 0.60) = 16.05
[linear_damper-1] [INFO] [1677864199.334275962] [linear_damper]: WindingCurrent: f(-2269.58, 1.00, 0.60) = 14.81
[linear_damper-1] [INFO] [1677864199.434369620] [linear_damper]: WindingCurrent: f(-1893.56, 1.00, 0.60) = 13.47
[linear_damper-1] [INFO] [1677864199.534461914] [linear_damper]: WindingCurrent: f(-1513.34, 1.00, 0.60) = 11.14
[linear_damper-1] [INFO] [1677864199.634556815] [linear_damper]: WindingCurrent: f(-1128.46, 1.00, 0.60) = 7.75
[linear_damper-1] [INFO] [1677864199.734798736] [linear_damper]: WindingCurrent: f(-825.91, 1.00, 0.60) = 4.53
[linear_damper-1] [INFO] [1677864199.834753871] [linear_damper]: WindingCurrent: f(-586.78, 1.00, 0.60) = 1.75
[linear_damper-1] [INFO] [1677864199.934809041] [linear_damper]: WindingCurrent: f(-393.25, 1.00, 0.60) = 0.57
[linear_damper-1] [INFO] [1677864200.035109715] [linear_damper]: WindingCurrent: f(-132.04, 1.00, 0.60) = 0.00
[linear_damper-1] [INFO] [1677864200.134981992] [linear_damper]: WindingCurrent: f(92.19, 1.00, 0.60) = -0.00
[linear_damper-1] [INFO] [1677864200.235094219] [linear_damper]: WindingCurrent: f(338.10, 1.00, 0.60) = -0.14
[linear_damper-1] [INFO] [1677864200.335164181] [linear_damper]: WindingCurrent: f(636.96, 1.00, 0.60) = -1.36
[linear_damper-1] [INFO] [1677864200.435227880] [linear_damper]: WindingCurrent: f(863.33, 1.00, 0.60) = -2.99
```
