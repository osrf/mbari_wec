# Simulator Output Data Logs

When running, the simulator logs data in two formats, a .csv file that matches the format of the log files the buoy system produces while in operation, and rosbag files that logs the ros message traffic in a format that can be examined using ROS 2 tools.  This tutorial describes the locations and contents of these files.

## .csv data files
While running, both the simulator and the buoy itself log telemetry data in text comma separated value files that can be examined while the simulator is running or processed subsequently. These data files contain all of the telemetry data available on the buoy while operational.  Because not all parameters are simulated, the .csv files from the simulator have all the fields present and match teh format of the at-sea buoy, but some values that don't make sense to simulate simply report unchanging default values.

### .csv file locations

To match the behavior of the buoy system, the simulator stores these files in a directory called ".pblogs" in the users home directory. The directory format of .pblogs is as follows:

```
.pblogs
├── 2025-06-21.000
│   ├── 2025.06.21T12.52.43.csv
│   └── latest -> 2025.06.21T12.52.43.csv
├── 2025-08-06.000
│   ├── 2025.08.06T08.03.20.csv
│   └── latest -> 2025.08.06T08.03.20.csv
├── 2025-08-06.001
│   ├── 2025.08.06T09.08.00.csv.gz
│   ├── 2025.08.06T10.08.00.csv
│   └── latest -> 2025.08.06T10.08.00.csv
└── latest_csv_dir -> 2025-08-06.001
```

As seen in this example, every time the simulator is run a new folder is created with the current date and an index of how many times the simulator has been started on that day, i.e. 2025-08-06.000 is the first time the simulator ran on August 6th, and 2025-08-06.001 is the second time the simulator ran on August 6th.  This scheme matches how the buoy maintains it's file structure as well, with the index referring to re-starts of the logging executable instead of simulator runs.  For convenience, a symbolic link is maintained that points to the most recent simulation run (latest_csv_dir).

Within each directory, .csv files are created that represent up to an hour of data.  At the end of each hour those files are zipped up and a new file is created for the next hour of data. Therefore, the timestamp shown in the directory listing for each .csv file is the time of the first data in the file.  In cases where the simulator is running faster than real time, the filenames represent the amount of simulated time that has passed since the beginning of the simulation.

Note:  The simulation environment creates this directory structure at ~/.pblogs when the simulator is run from the command line.  If simulations are run in a batch mode using the facility provided to support many simulation runs, then these files are located in a different location.  See this tutorial for details: [Parameters and Batch Runs](SimulatorParameters.md)

### .csv data format
The data in the .csv data files are organized by data source within the system. During operation on the buoy, each subsystem is generating telemetry asynchronously to one other, and the logging facility place the data from each on a new line of the .csv files.  

A sample of the first few lines of a typical .csv data files is as follows:

```
Source ID, Timestamp (epoch seconds), PC RPM, PC Bus Voltage (V), PC Winding Curr (A), PC Battery Curr (A), PC
 Status, PC Load Dump Current (A), PC Target Voltage (V), PC Target Curr (A), PC Diff PSI, PC RPM Std Dev, PC 
Scale, PC Retract, PC Aux Torque (mV), PC Bias Curr (A), PC Charge Curr (A), PC Draw Curr (A),  BC Voltage, BC
 Ips, BC V Balance, BC V Stopcharge, BC Ground Fault, BC_Hydrogen, BC Status,  XB Roll XB Angle (deg), XB Pitc
h Angle (deg), XB Yaw Angle (deg), XB X Rate, XB Y Rate, XB Z Rate, XB X Accel, XB Y Accel, XB Z Accel, XB Nor
th Vel, XB East Vel, XB Down Vel, XB Lat, XB Long, XB Alt, XB Temp,  SC Load Cell (lbs), SC Range Finder (in),
 SC Upper PSI, SC Lower PSI, SC Status, CTD Time, CTD Salinity, CTD Temp,  TF Power-Loss Timeouts, TF Tether V
olt, TF Batt Volt, TF Pressure psi, TF Qtn 1, TF Qtn 2, TF Qtn 3, TF Qtn 4, TF Mag 1 gauss, TF Mag 2, TF Mag 3
, TF Status, TF Ang Rate 1 rad/sec, TF Ang Rate 2,  TF Ang Rate 3, TF VPE status, TF Accel 1 m/sec^2, TF Accel
 2, TF Accel 3, TF Comms-Loss Timeouts, TF Maxon status, TF Motor curren mA, TF Encoder counts, 
4, 1754492880.215, ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,60, 48.000, 48.000, -44.221, -0.000, 0.707, 
0.000, 0.707, 0.226, 0.054, 0.421, 0, -0.000, 0.000, 0.000, 0, -8.956, -0.000, -0.009, 60, 0, 0, 0, 
3, 1754492880.215, ,,,,,,,,,,,,,,,,,,,,,,,0.000, 0.001, 0.000, 0.000, 0.000, 0.000, 0.076, -0.000, 6.935, 0.36
0, 0.009, 0.360, 36.74385, -121.87623, -2.821, 0.000, ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
0, 1754492880.215, ,,,,,,,,,,,,,,,,325.0, 0.00, 2.76, 2.79, 0.06, 3.26, 11264, ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
,,,,,,,,,,,,,,,,
1, 1754492880.215, ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,446.92, 26.92, 76.22, 188.73, 8192, 0, 0.000000, 0.0
00, ,,,,,,,,,,,,,,,,,,,,,,,
2, 1754492880.215, 3093.0, 325.0, -10.64, 3.57, 0, 0.83, 0.0, -10.64, 2.910, 0.0, 1.00, 0.60, 0.00, 0.00, 0.00
, 0.00, ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
1, 1754492880.315, ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,-48.78, 25.46, 77.90, 186.95, 8192, 0, 0.000000, 0.0
00, ,,,,,,,,,,,,,,,,,,,,,,,


```

The first line of each file provides a short text description of each field, and after that the data from each subsystem is logged as it becomes available.  To differentiate between data sources, the first data item in each file is an identifier, as follows:

- 0 = Battery System

- 1 = Spring Controller

- 2 = Power Converter

- 3 = Buoy AHRS/GPS

- 4 = Heave Cone Controller 


After the "Source ID" identifier comes Unix timestamp that records when the data on that line was captured.
As can be seen, to facilitate data ingestion from these files into other tools, the appropriate number of commas are included on each line so that the data presented lines up with the data-label on the first line.  Because the first character is a single digit identifier, unix tools can be used to look at this data controller by controller.  For example, the following command uses grep to pick out just the data from the power converter by selecting lines that have a 2 in the first column:

```
$ cat ~/.pblogs/2025-08-06.001/2025.08.06T10.08.00.csv | grep ^2

2, 1754492904.515, 1656.4, 311.3, -7.44, 1.62, 0, -0.00, 0.0, -7.44, 2.910, 0.0, 1.00, 0.60, 0.00, 0.00, 0.00, 0.00, ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

2, 1754492904.615, 1655.1, 311.3, -7.43, 1.61, 0, -0.00, 0.0, -7.43, 2.910, 0.0, 1.00, 0.60, 0.00, 0.00, 0.00, 0.00, ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

2, 1754492904.715, 1633.7, 311.0, -7.32, 1.57, 0, -0.00, 0.0, -7.32, 2.910, 0.0, 1.00, 0.60, 0.00, 0.00, 0.00, 0.00, ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

2, 1754492904.815, 1593.0, 310.3, -7.11, 1.48, 0, -0.00, 0.0, -7.11, 2.910, 0.0, 1.00, 0.60, 0.00, 0.00, 0.00, 0.00, ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

```

### .csv data contents
For each data source, particular data is logged as described below.  

#### Battery Controller Data (Source ID = 0)
```
BC Voltage, BC Ips, BC V Balance, BC V Stopcharge, BC Ground Fault, BC_Hydrogen, BC Status
```

- **BC_Voltage**:  Measured instantaneous Battery Voltage in Volts.

- **BC_Ips**: Measured Current Draw for the 24V supply that powers on-board instrumentation (Not Simulated) 

- **BC V Balance**:  Number of batteries currently balancing, Encoded value.  (Not Simulated)

- **BC V_StopCharge**: Number of batteries that are above their peak voltage, Encoded Value (Not Simulated)

- **BC Ground Fault**:  Encoded representation of 300V Bus Isolation Monitor result. (Not Simulated)

- **BC_Hydrogen**:  Measure of the hydrogen in the lead-acid battery pack.  %LEL. (Not Simulated)

- **BC Status**: 16 bit status word of Battery system.

#### Spring Controller Data (Source ID = 1)

```
SC Load Cell (lbs), SC Range Finder (in), SC Upper PSI, SC Lower PSI, SC Status, CTD Time, CTD Salinity, CTD Temp
```

- **SC Load Cell**:  Measured tension between the buoy and the power take-off device (lbs).

- **SC Range Finder (in)**:  Measured piston position in inches.  0.0 = fully retracted, 80.0 = fully extended

- **SC Upper PSI**: Measured upper spring chamber pressure (psia) 

- **SC Lower PSI**: Measured lower spring chamber pressure (psia) 

- **SC Status**: 16 bit status word of Spring Controller system.

- **CTD Time**:  Deprecated

- **CTD Salinity**:  Deprecated

- **CTD Temp**:  Deprecated


#### Power Converter Data (Source ID = 2)
```
PC RPM, PC Bus Voltage (V), PC Winding Curr (A), PC Battery Curr (A), PC Status, PC Load Dump Current (A), PC Target Voltage (V), PC Target Curr (A), PC Diff PSI, PC RPM Std Dev, PC Scale, PC Retract, PC Aux Torque (mV), PC Bias Curr (A), PC Charge Curr (A), PC Draw Curr (A)
```

- **PC RPM**: Instantaneous measured generator rotational velocity (RPM).

- **PC Bus Voltage**:  Measured bus voltage of power converter (Volts).

- **PC Winding Curr**:  Instantaneous measured motor quadrature current (Amps). Is proportional to torque.

- **PC Battery Curr**:  Instantaneous measured current flowing to Battery System (Amps).

- **PC Status**:  16 bit status word of Power Converter system.

- **PC Load Dump Current**:  Instantaneous measured current flowing to the load dump (Amps).

- **PC Target Voltage**:  Voltage limit setting, actual voltage may be less if the system is current limiting.

- **PC Target Current**: Instantaneous measured winding quadrature current (Amps).

- **PC Diff PSI**:  Measured pressure of hydraulic system compensator (psi).  Not Simulated.

- **PC RPM Std Dev**:  Standard deviation of RPM over the past 30 minutes (RPM).

- **PC Scale**:  Current damping scale factor setting.

- **PC Retract**: Current retract factor setting.

- **PC Aux Torque**:  Torque sensor input.  Not used at sea.  Not Simulated.

- **PC Bias Curr**:  Bias Current applied to motor current versus RPM relationship (Amps).

- **PC Charge Curr**:  Battery Current limit (Battery Charge Direction) Amps.

- **PC Draw Curr**:  Battery Draw Limit (Battery Draw Direction).  Amps.


#### GPS/AHRS Data (Source ID = 3)
```
XB Roll Angle (deg), XB Pitch Angle (deg), XB Yaw Angle (deg), XB X Rate, XB Y Rate, XB Z Rate, XB X Accel, XB Y Accel, XB Z Accel, XB North Vel, XB East Vel, XB Down Vel, XB Lat, XB Long, XB Alt, XB Temp
```
- **XB Roll Angle**: Roll Angle reported from Crossbow Nav440 on the buoy (degrees).

- **XB Pitch Angle**: Pitch Angle reported from Crossbow Nav440 on the buoy (degrees).

- **XB Yaw Angle**: Pitch Angle reported from Crossbow Nav440 on the buoy (degrees).


#### Heave Cone Controller Data (Source ID = 4)
```
TF Power-Loss Timeouts, TF Tether Volt, TF Batt Volt, TF Pressure psi, TF Qtn 1, TF Qtn 2, TF Qtn 3, TF Qtn 4, TF Mag 1 gauss, TF Mag 2, TF Mag 3, TF Status, TF Ang Rate 1 rad/sec, TF Ang Rate 2,  TF Ang Rate 3, TF VPE status, TF Accel 1 m/sec^2, TF Accel 2, TF Accel 3, TF Comms-Loss Timeouts, TF Maxon status, TF Motor curren mA, TF Encoder counts
 ```