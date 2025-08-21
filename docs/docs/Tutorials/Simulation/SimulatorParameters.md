# Parameters and Batch Runs

## Introduction
When running a simple instance of the simulator as described in the
[Run the Simulator](RunSimulator.md) Tutorial. i.e. using:

```
$ ros2 launch buoy_gazebo mbari_wec.launch.py
```

the simulation uses a number of defaults for a range of parameters. In most cases, one may want to
run the simulator with different values for these parameters, and/or run a number of simulations
that iterate across a range of values for particular parameters. For instance, one may want to run
the simulator in a batch mode that runs the same buoy and controller set-up in a range of sea-states.

To facilitate this, a batch tool is provided that allows one to specify ranges for a number of
parameters, and then run a number of simulations for all combinations of parameters, saving the
results separately.

This tutorial describes these capabilities, demonstrates with some examples, and discusses how this
tool can be used.

## Parameters

There are a number of parameters that impact the behavior of the simulator, and must be specified at
start-up:

- **Simulation duration**:  How long the simulation will run for (simulation time) in seconds.
- **Time-step size**:  The simulation proceeds at a fixed time-step size, and this can be specified for each run.
- **Heave cone door-state**:  Simulations can be done with the heave-cone doors either opened or closed, the door position can not be changed during a simulation, so this is a parameter that must be specified at start-up.
- **Sea-State**:  Incoming waves can be specified as a monochromatic wave with a specified amplitude and period, a Bretschneider spectrum of waves described by significant wave-height and the peak period, or by a custom wave-spectrum defined by a curve of wave-energy versus frequency.  
- **Wave phase random seed**:  The phases of each wave-component are randomly assigned, if a seed is specified, the phases will be the same for each run of a simulation.  This is useful for comparing simulations for which one wants the same wave-excitation.  If the seed value is set to zero, or omitted, a different random phase assignment will be made for each simulation run.
- **Physics Real-Time Factor (RTF)**:  This parameter sets the maximum speed the simulation will be allowed to run, in terms of a multiple of real-time.  i.e. a Real-Time factor of 2.0 will limit the simulation to only running twice as fast as real-time.  As described below, for single runs for which one may want to monitor the motion as it occurs, a Real-Time Factor of 1.0 is appropriate.  For batches of runs one wants to complete as fast as possible, a large RTF (i.e. 100) will let the simulation run as fast as the processing resources allow.
- **Battery State-of-Charge**:  The starting battery state of charge can be specified between 0 (empty) and 1 (full).  A full battery can't absorb energy so more of the generated power will be diverted to the load-dump in the simulation.  An empty battery will allow most or all of the generated energy to flow to the battery. This does not effect the physical behavior of the simulated buoy, but does effect the battery voltages and currents as the simulation progresses.
- **Battery EMF**:  As an alternative to specifying the battery state of charge as a percentage, the zero-load battery voltage can be specified, between 270V (empty battery), and 320V (full battery).
- **Scale Factor**:  The buoy (and simulator) implements a default control algorithm in which the current in the motor windings is set as a function of motor RPM, with the resulting torque opposing the motors motion.  This approximately implements a linear damping behavior and the specified Scale Factor is multiplied by the default Winding-Current/RPM relationship before being applied.  Allowable values are from 0.5 to 1.4, and the result is a simple way to change the damping the power-takeoff device is applying to the system.  As discussed in a later tutorial, this relationship can be over-ridden with external code, so this Scale Factor only applies to the default behavior of the system.
- ** GUI Output**:  The graphical output of the simulator can be turned on or off as needed, for large batches of runs that are meant to run un-attended this is probably not appropriate, but for single runs or debugging the graphical output can be useful.

## Example Batch File
The above parameters are specified in a .yaml file that the batch-run tool reads in before execution
begins.  A commented example is below and illustrates the use of the above parameters.  Lines that
begin with # are comments and have no impact.

```
#
# Batch-Specific Scalar Parameters
#
duration: 300
seed: 42
physics_rtf: 11
#
# Run-Specific Parameters (Test Matrix)
#
physics_step: [0.001, 0.01, 0.1]
door_state: ['closed', 'open']
scale_factor: [0.5, 0.75, 1.0, 1.3, 1.4]
enable_gui: False
# May specify vector/scalar battery_soc (0.0 to 1.0) or battery_emf (270V to 320V)
battery_soc: [0.25, 0.5, 0.75, 1.0]
# battery_emf: [282.5, 295.0, 307.5, 320.0]
IncidentWaveSpectrumType:
 - MonoChromatic:
     # A & T may be vector/scalar in pairs (A & T same length)
     A: [1.0, 2.0, 3.0]
     T: [12.0, 14.0, 15.0]
 - Bretschneider:
     # Hs & Tp may be vector/scalar in pairs (Hs & Tp same length)
     Hs: 3.0
     Tp: 14.0
 # Multiple Custom Spectra must be listed individually (f & Szz are already vectors of same size)
 - Custom:
     f: [0.0, 0.2, 0.4, 0.6, 2.0]
     Szz: [0.0, 0.4, 1.0, 1.0, 0.0]
```

As seen in this example, some parameters are enforced to be scalar values and apply to the entire
batch of specified runs.  These are the specification of simulation duration (300), random seed (42),
and the physics real-time factor (11). 

The remaining run-specific parameters can be specified as arrays, and the batch-run tool then
executes simulations for all possible combinations of these values.  Note that some values are
specified in pairs.  For instance, three mono-chromatic waves are specified by this file with a
specification of (A=1.0m, T=12s), (A=2.0m, T=14s), and (A=3.0m, T=15s), not nine runs that include
all possible combinations of the specified amplitude and periods.

Obviously it would be very easy to write a batch file specification that includes thousands of runs,
more practical usage will most likely iterate over a small number of parameters at at a time. 

## Running an example
For a simpler example, a batch file that iterates across a range of sea-states is used.  As a
concise example, the following file illustrates this, comments have been removed for brevity.

```
duration: 3
seed: 42
physics_rtf: 11
enable_gui: False
physics_step: 0.01
door_state: ['closed']
scale_factor: [0.6, 1.0]
battery_soc:  0.5
IncidentWaveSpectrumType:
 - Bretschneider:
     Hs: [2.0, 4.0]
     Tp: [14.0, 16.0]
```
 
To run this example, create the above file in a new directory and name it "IrregularWaves.yaml",
source the simulator installation directory, and start the simulation using the batch tool.
(Note that the run duration is very short for this example to allow it to complete quickly)

```
$ mkdir FOO
$ cd FOO
$ Create file using editor of your choice, name it IrregularWaves.yaml
$ . ~/mbari_wec_ws/install/setup.bash
$ ros2 launch buoy_gazebo mbari_wec_batch.launch.py sim_params_yaml:=IrregularWaves.yaml
```

Running these commands will run the simulation, all output is stored in a directory named similar to
`batch_results_20230228210735', the trailing numbers indicate a timestamp.  Inside this directory
the yaml file is repeated, along with a log file 'batch_results.log' that lists all of the
simulation runs that were performed.  Alongside that are specific directories that hold output from
each run.  For convenience, a symbolic link is formed that points at the most recent batch output
directory.

Within the output directory, there is a file named 'batch_runs.log' that shows each individual run
that was performed, and the associated parameter.  In this case it has the following contents:

```
# Generated 4 simulation runs
RunIndex, SimReturnCode, StartTime, rosbag2FileName, PhysicsStep, PhysicsRTF, Seed, Duration, DoorState, ScaleFactor, BatteryState, IncWaveSpectrumType;In
cWaveSpectrumParams
0, 0, 20230228212457, rosbag2_batch_sim_0_20230228212457, 0.01, 11.0, 42, 3.0, closed, 0.6, 0.5, Bretschneider;Hs:2.0;Tp:14.0
1, 0, 20230228212502, rosbag2_batch_sim_1_20230228212502, 0.01, 11.0, 42, 3.0, closed, 0.6, 0.5, Bretschneider;Hs:4.0;Tp:16.0
2, 0, 20230228212506, rosbag2_batch_sim_2_20230228212506, 0.01, 11.0, 42, 3.0, closed, 1.0, 0.5, Bretschneider;Hs:2.0;Tp:14.0
3, 0, 20230228212510, rosbag2_batch_sim_3_20230228212510, 0.01, 11.0, 42, 3.0, closed, 1.0, 0.5, Bretschneider;Hs:4.0;Tp:16.0
```

For simulations that take longer to run, it can be convenient to tail this log file from the
terminal to keep track of progress. i.e.  From the directory the batch was started from:

```
$ tail -f latest_batch_results/batch_runs.log
```

## Finding the output
Within the batch process output directory, (e.g. `batch_results_20230301200627'), the output of each
simulation run is stored within a single sub-directory.  The resulting directory tree from the above
example is as follows:

```
$ tree batch_results_20230301200627/
batch_results_20230301200627/
├── batch_runs.log
├── IrregularWaves_20230301200627.yaml
├── latest_csv_dir -> results_run_3_20230301200640/pblog
├── latest_rosbag -> results_run_3_20230301200640/rosbag2
├── results_run_0_20230301200627
│   ├── pblog
│   │   ├── 2023.03.01T20.06.27.csv
│   │   └── latest -> 2023.03.01T20.06.27.csv
│   └── rosbag2
│       ├── metadata.yaml
│       └── rosbag2_0.db3
├── results_run_1_20230301200632
│   ├── pblog
│   │   ├── 2023.03.01T20.06.32.csv
│   │   └── latest -> 2023.03.01T20.06.32.csv
│   └── rosbag2
│       ├── metadata.yaml
│       └── rosbag2_0.db3
├── results_run_2_20230301200636
│   ├── pblog
│   │   ├── 2023.03.01T20.06.36.csv
│   │   └── latest -> 2023.03.01T20.06.36.csv
│   └── rosbag2
│       ├── metadata.yaml
│       └── rosbag2_0.db3
└── results_run_3_20230301200640
    ├── pblog
    │   ├── 2023.03.01T20.06.40.csv
    │   └── latest -> 2023.03.01T20.06.40.csv
    └── rosbag2
        ├── metadata.yaml
        └── rosbag2_0.db3
```

This output includes rosbag files and .csv files that are in the same format as the files generated
on the physical buoy.  In general the information in these two files are the same, the .csv files
are in clear text and are easy to inspect, and can be processed by the same tools used for the
actual buoy data.  The rosbag files are binary files that encode all of the ROS2 messages on the
computer during the simulation.  These files can be processed by a number of tools for post-
processing and inspection of results.  It is also possible to load the rosbag files into plotjuggler
for plotting and inspection, as described in the
[View Messages with Plotjuggler](SimulatorOutputPlotjuggler.md) Tutorial.

## Passing params directly to launch file

Many of these parameters may be passed directly as arguments to `mbari_wec.launch.py`. To see a list,
enter the following command:
```
$ ros2 launch buoy_gazebo mbari_wec.launch.py -s
Arguments (pass arguments as '<name>:=<value>'):

    'door_state':
        open or closed
        (default: 'None')

    'physics_step':
        step size in seconds
        (default: 'None')

    'physics_rtf':
        sim real-time factor
        (default: 'None')

    'scale_factor':
        target winding current scale factor
        (default: 'None')

    'inc_wave_seed':
        random seed for incident wave computation
        (default: 'None')

    'battery_soc':
        initial battery state of charge as pct (0-1)
        (default: 'None')

    'battery_emf':
        initial battery emf in Volts
        (default: 'None')

    'x_mean_pos':
        desired mean piston position in meters
        (default: 'None')

    'inc_wave_spectrum':
        incident wave spectrum defined as inc_wave_spectrum_type:type;p1:v1:v2;p2:v1:v2
        (default: 'None')

    'world_file':
        Gazebo world filename.sdf
        (default: 'mbari_wec.sdf')

    'world_name':
        Gazebo <world name>
        (default: 'mbari_wec_world')

    'pbloghome':
        root pblog directory
        (default: '/home/anderson/.pblogs')

    'pblogdir':
        specific pblog directory in pbloghome
        (default: '')

    'rosbag2':
        save rosbags to <pblogdir>/rosbag2/rosbag2_<datetime>
        (default: 'false')

    'debugger':
        run gazebo in gdb
        (default: 'false')

    'extra_gz_args':
        extra_gz_args
        (default: '')

    'regenerate_models':
        regenerate template models using defaults
        (default: 'true')
```

