# Quick Start -- Writing External Controller With Github Template Repository
---

There are two github
[template repositories](https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-repository-from-a-template)
set up (cpp/python) for a quick start on writing a
custom controller utilizing
[buoy_api_cpp](https://github.com/osrf/buoy_msgs/tree/main/buoy_api_cpp) and
[buoy_api_py](https://github.com/osrf/buoy_msgs/tree/main/buoy_api_py). Please see
[cpp examples](https://github.com/osrf/buoy_msgs/tree/main/buoy_api_cpp/examples) and
[python examples](https://github.com/osrf/buoy_msgs/tree/main/buoy_api_py/examples) for example
controller implementations.

* [mbari_wec_template_cpp](https://github.com/mbari-org/mbari_wec_template_cpp)
* [mbari_wec_template_py](https://github.com/mbari-org/mbari_wec_template_py)

---

## Using Python Template

### Creating your own repo from the template

You may also refer to Github's
[template documentation](https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-repository-from-a-template)

To start using the python Github template

1. Navigate to
[mbari_wec_template_py](https://github.com/mbari-org/mbari_wec_template_py) and click the green
button with the text `Use this template` and select `Create a new repository`

    ![use_this_template](resources/use_this_template_py.png)

2. Next, set up the repository like you would any new Github repository choosing the owner,
repository name, public/private, etc.

3. Now that your new repository is set up, clone it to your local machine, make a branch, etc.

`$ git clone https://github.com/<owner>/<repo_name>.git`

You should now have a python ROS2 package with the following structure:

```
<repo_name>
    ├── config
    │   └── controller.yaml
    ├── launch
    │   └── controller.launch.py
    ├── LICENSE
    ├── mbari_wec_template_py
    │   ├── controller.py
    │   └── __init__.py
    ├── package.xml
    ├── README.md
    ├── resource
    │   └── mbari_wec_template_py
    ├── setup.cfg
    ├── setup.py
    └── test
        ├── test_copyright.py
        ├── test_flake8.py
        └── test_pep257.py
```

### Customizing the controller

You may also refer to the `README.md` in your newly cloned repository.

#### Modify template for your package
Replace `mbari_wec_template_py` with your package name and modify other fields as necessary in:

- package.xml (lines 4-8)

``` xml linenums="1" title="package.xml"
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>repo_name</name>
  <version>3.14</version>
  <description>Your Controller Description</description>
  <maintainer email="your@email">Your Name</maintainer>
  <license>Your License</license>
```


- setup.py (lines 7, 11, 22-25, 29)

``` py linenums="7" hl_lines="1 5 16 17 18 19 23"
package_name = 'your_package_name'

setup(
    name=package_name,
    version='3.14',
    packages=[f'{package_name}'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email',
    description='Your package description',
    license='Your License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'your_controller_name = {package_name}.controller:main',
```

- setup.cfg (lines 2, 4)

```
[develop]
script_dir=$base/lib/your_package_name
[install]
install_scripts=$base/lib/your_package_name
```

- launch/controller.launch.py (lines 22, 34-35)

``` py linenums="22" hl_lines="1 13 14"
package_name = 'your_package_name'

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'controller.yaml'
        )

    node = Node(
        package=package_name,
        name='your_controller_name',
        executable='your_controller_name',
```

- config/controller.yaml (line 1)

``` yaml linenums="1" hl_lines="1"
/your_controller_name:
  ros__parameters:
    foo: 1.0
```

and rename two files/folders

- the empty file `resource/mbari_wec_template_py`
- the python package `mbari_wec_template_py` containing `controller.py`

resulting in the following folder structure:

```
repo_name
    ├── config
    │   └── controller.yaml
    ├── launch
    │   └── controller.launch.py
    ├── LICENSE
    ├── your_package_name
    │   ├── controller.py
    │   └── __init__.py
    ├── package.xml
    ├── README.md
    ├── resource
    │   └── your_package_name
    ├── setup.cfg
    ├── setup.py
    └── test
        ├── test_copyright.py
        ├── test_flake8.py
        └── test_pep257.py
```

Modify `setup.py` as desired and add any dependencies in `package.xml` following standard ROS2
documentation.

---

## Implement Controller
Assuming you have followed the above and renamed the python package `mbari_wec_template_py` to your package name,
`<your_package_name>/controller.py` is stubbed out to implement your custom external controller.
You may also use `config/controller.yaml` for any policy parameters.

### ControlPolicy

You may use the class `ControlPolicy` in `<your_package_name>/controller.py` to implement your controller.

``` py linenums="21"
class ControlPolicy(object):

    def __init__(self):
        # Define any parameter variables here
        self.foo = 1.0

        self.update_params()

    def update_params(self):
        '''Update dependent variables after reading in params'''
        self.bar = 10.0 * self.foo

        pass  # remove if there's anything to set above

    # Modify function inputs as desired
    def target(self, *args, **kwargs):
        '''Calculate target value from feedback inputs'''

        # secret sauce

        return 0.0  # obviously, modify to return proper target value
```

- Set any configurable parameters in `__init__` on line 23

``` py linenums="23"
    def __init__(self):
        # Define any parameter variables here
        self.foo = 1.0

        self.update_params()
```

- Set any dependent variables in `update_params` on line 29

``` py linenums="29"
    def update_params(self):
        '''Update dependent variables after reading in params'''
        self.bar = 10.0 * self.foo

        pass  # remove if there's anything to set above
```

- Declare/get/update params in the `set_params` function of the `Controller` class on line 111

``` py linenums="111"
    def set_params(self):
        '''Use ROS2 declare_parameter and get_parameter to set policy params'''
        self.declare_parameter('foo', self.policy.foo)
        self.policy.foo = \
            self.get_parameter('foo').get_parameter_value().double_value

        # recompute any dependent variables
        self.policy.update_params()
```

- Then, your control logic will go in the `target` function on line 36.
    Modify the input args as well as the return value as necessary

``` py linenums="36"
    # Modify function inputs as desired
    def target(self, *args, **kwargs):
        '''Calculate target value from feedback inputs'''

        # secret sauce

        return 0.0  # obviously, modify to return proper target value
```

### Controller

The `policy.target` function may be called from within the `Controller` class. You may call it inside any of
the data callbacks to enable feedback control (for example):

``` py
    # To subscribe to any topic, simply define the specific callback, e.g. power_callback
    def power_callback(self, data):
        '''Callback for '/power_data' topic from Power Controller'''
        # get target value from control policy
        target_value = self.policy.target(data.rpm, data.scale, data.retract)

        # send a command, e.g. winding current
        self.send_pc_wind_curr_command(target_value, blocking=False)
```

Or, set up a loop in `main()` and run open-loop:

``` py linenums="121"
def main():
    rclpy.init()
    controller = Controller()
    rate = controller.create_rate(50.0)  # Hz
    while rclpy.ok():

        # Open-loop control logic

        rclpy.spin_once(controller)
        rate.sleep()
    rclpy.shutdown()
```

You may get feedback data from any of the buoy topics by simply creating a specific callback
listed below. For feedback data you'd like to use in another area of the class, feel free to
assign them to class variables.

(Delete any callbacks you don't need in the `Controller` class)

Available callback functions:

`/ahrs_data` &rarr; `def ahrs_callback(self, data):`  
`/battery_data` &rarr; `def battery_callback(self, data):`  
`/spring_data` &rarr; `def spring_callback(self, data):`  
`/power_data` &rarr; `def power_callback(self, data):`  
`/trefoil_data` &rarr; `def trefoil_callback(self, data):`  
`/powerbuoy_data` &rarr; `def powerbuoy_callback(self, data):`  

You may also send commands from within the `Controller` class:

`self.send_pump_command(duration_mins, blocking=False)`  
`self.send_valve_command(duration_sec, blocking=False)`  
`self.send_pc_wind_curr_command(wind_curr_amps, blocking=False)`  
`self.send_pc_bias_curr_command(bias_curr_amps, blocking=False)`  
`self.send_pc_scale_command(scale_factor, blocking=False)`  
`self.send_pc_retract_command(retract_factor, blocking=False)`  

---

## Example

An example using this interface will follow in the next tutorial: [Linear Damper Example (Python)](/Tutorials/ROS2/PythonLinearDamperExample.md)