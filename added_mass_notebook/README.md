Jupyter notebook to compare motions created with the fluid added mass [example world](https://github.com/gazebosim/gz-sim/blob/55ab535a41588b359df803d3efb4f9af68e62ff5/test/worlds/fluid_added_mass.sdf) with the ones created by integrating the equations of motion (in the notebook itself).

`pose.jsons` is a JSON log file with motions generated from the example world.

The equations of motion implemented in the notebook take into account a user-specified added mass matrix, with the only requirement that the full spatial inertia matrix (spatial inertia + added mass matrix) is invertible. Ideally, for comparison with the example world, the spatial inertia matrix, added mass matrix, initial conditions, and external forces/torques would match one of the _boxes_ in the example world.


## Requirements

You'll need [python](https://wiki.python.org/moin/BeginnersGuide/Download), [jupyter](https://jupyter.org/install), and the python packages [`numpy`](https://numpy.org/install/), [`numpy-quaternion`](https://github.com/moble/quaternion#quickstart), and [`scipy`](https://scipy.org/install/). To install the packages, a simple `pip install --user numpy numpy-quaternion scipy` should do.


## Motion Parameters

For the generated motion:

 - `full_spatial_inertia_body` contains the full spatial inertia matrix in body frame.

 - `force` and `torque` contain the constant forces and torques applied throughout the motion, in the absolute frame.

 - `p0`, `q0`, `v0`, and `w0`, contain the initial position, orientation (as a quaternion), linear velocity, and angular velocity, respectively.

`box_name` controls which motion from the example world will be used for comparison. See `box_names` for a list of available names.
