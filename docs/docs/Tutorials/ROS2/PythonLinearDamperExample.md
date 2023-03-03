# Quick Start -- Simple Linear Damper Controller (Python)

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

with the files modified according to the previous tutorial

---

## Linear Damper ControlPolicy

In this tutorial you will implement a simple linear damper controller for the piston in the WEC
Power-Take-Off (PTO). Given motor RPM, it outputs desired motor winding current (interpolated from
RPM->Torque lookup table) to generate a torque to resist piston velocity with a damping force.
Configurable gains (scale/retract factor) are applied before output.

### Parameters

Parameters for the controller are:

- `torque_constant`: Motor Torque Constant (N-m/Amp)
  Find motor winding current to apply for desired torque
- `n_spec`: Motor RPM Breakpoints
  `N` (RPM) is the input to the controller and `n_spec` are the x-components of the breakpoints
  (`n_spec`, `torque_spec` / `torque_constant`) for the interpolant,
  `f(n_spec) = torque_spec / torque_constant`
- `torque_spec`: Desired Motor Torque (N-m) Breakpoints
  Torque (N-m) is the eventual desired output of the controller given an input `N` (motor RPM) and
  `torque_spec` / `torque_constant` (Amps) are the y-components of the breakpoints for the
  interpolant. The controller actually outputs motor winding current (Amps) to generate a torque
  in the opposite direction of piston velocity to generate a damping force.
