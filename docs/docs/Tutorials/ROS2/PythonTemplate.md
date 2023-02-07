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
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name><b>repo_name</b></name>
  <version><b>3.14</b></version>
  <description><b>Your Controller Description</b></description>
  <maintainer email="<b>your@email</b>"><b>Your Name</b></maintainer>
  <license><b>Your License</b></license>
</code>
</pre>


- setup.py
- setup.cfg
- launch/controller.launch.py

and rename two files/folders

- the empty file resource/mbari_wec_template_py
- the python package `mbari_wec_template_py` containing `controller.py`

Modify `setup.py` as desired and add any dependencies in `package.xml`.

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
