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
1. navigate to
[mbari_wec_template_py](https://github.com/mbari-org/mbari_wec_template_py) and click the green
button with the text `Use this template` and select `Create a new repository`
![use_this_template](Tutorials/ROS2/resources/use_this_template.png)

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
Replace `mbari_wec_template_py` with your package name in

- package.xml

<pre>
<code>
&lt;?xml version="1.0"?&gt;
&lt;?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?&gt;
&lt;package format="3"&gt;
  &lt;name&gt;<b>repo_name</b>&lt;/name&gt;
  &lt;version&gt;<b>3.14</b>&lt;/version&gt;
  &lt;description&gt;<b>Your Controller Description</b>&lt;/description&gt;
  &lt;maintainer email="<b>your@email</b>"&gt;<b>Your Name</b>&lt;/maintainer&gt;
  &lt;license&gt;<b>Your License</b>&lt;/license&gt;
</code>
</pre>


- setup.py

<pre>
<code>
package_name = <b>'your_package_name'</b>

setup(
    name=package_name,
    version=<b>'3.14'</b>,
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
    maintainer=<b>'Your Name'</b>,
    maintainer_email=<b>'your@email'</b>,
    description=<b>'Your package description'</b>,
    license=<b>'Your License'</b>,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'<b>your_controller_name</b> = {package_name}.controller:main',
</code>
</pre>

- setup.cfg
<pre>
<code>
[develop]
script_dir=$base/lib/<b>your_package_name</b>
[install]
install_scripts=$base/lib/<b>your_package_name</b>
</code>
</pre>

- launch/controller.launch.py

<pre>
<code>
package_name = <b>'your_package_name'</b>

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'controller.yaml'
        )

    node = Node(
        package=package_name,
        name=<b>'your_controller_name'</b>,
        executable=<b>'your_controller_name'</b>,
</code>
</pre>

and rename two files/folders

- the empty file resource/mbari_wec_template_py
- the python package `mbari_wec_template_py` containing `controller.py`

resulting in the following folder structure:

<pre>
<code>
<b>repo_name</b>
    ├── config
    │   └── controller.yaml
    ├── launch
    │   └── controller.launch.py
    ├── LICENSE
    ├── <b>your_package_name</b>
    │   ├── controller.py
    │   └── __init__.py
    ├── package.xml
    ├── README.md
    ├── resource
    │   └── <b>your_package_name</b>
    ├── setup.cfg
    ├── setup.py
    └── test
        ├── test_copyright.py
        ├── test_flake8.py
        └── test_pep257.py
</code>
</pre>

Modify `setup.py` as desired and add any dependencies in `package.xml` following standard ROS2
documentation.

#### Implement Controller
Assuming you have followed the above and renamed the python package `mbari_wec_template_py` to your package name,
`<your_package_name>/controller.py` is stubbed out to implement your custom external controller.

You may also use `config/controller.yaml` for any policy parameters.
If you modify the controller node name, i.e. `super().__init__('controller')` and the `name` field of `Node` in the launch file,
please be sure to update the first line of the `config/controller.yaml` file and to use the same node name.

Also, if you choose a more specific name for your controller,
consider renaming:

- `launch/controller.launch.py`
- `config/controller.yaml`
- `<your_package_name>/controller.py`

and update:

- `console_scripts` in `setup.py`
- `executable` field of `Node` in `launch/controller.launch.py`

accordingly.
