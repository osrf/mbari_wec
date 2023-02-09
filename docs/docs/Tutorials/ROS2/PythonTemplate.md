# Quick Start -- Writing External Controller With Github Template Repository
=========================================================================

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

## Using Python Template

### Creating your own repo from the template

You may also refer to Github's
[template documentation](https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-repository-from-a-template)

To start using the python Github template

1. Navigate to
[mbari_wec_template_py](https://github.com/mbari-org/mbari_wec_template_py) and click the green
button with the text `Use this template` and select `Create a new repository`

    ![use_this_template](/Tutorials/ROS2/resources/use_this_template_py.png)

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

#### Implement Controller
Assuming you have followed the above and renamed the python package `mbari_wec_template_py` to your package name,
`<your_package_name>/controller.py` is stubbed out to implement your custom external controller.



You may also use `config/controller.yaml` for any policy parameters.
