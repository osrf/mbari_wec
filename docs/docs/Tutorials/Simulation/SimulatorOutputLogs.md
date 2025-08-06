# Simulator Output Data Logs

When running, the simulator logs data in two formats, a .csv file that matches the format of the log files the buoy system produces while in operation, and rosbag files that logs the ros message traffic in a format that can be examined using ROS 2 tools.  This tutorial describes the locations and contents of these files.

## .csv data files
While running, both the simulator and the buoy itself log telemetry data in text comma seperated value files that can be examined while the simulator is running or processed subsequently. These data files contain all of the telemetry data available on the buoy while operational.  Because not all parameters are simulated, the .csv files from the simulator have all the fields present and match teh format of the at-sea buoy, but some values that don't make sense to simulate simply report unchanging default values.

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

As seen in this example, every time the simulator is run a new folder is created with the current date and an index of how many times the simulator has been started on that day, i.e. 2025-08-06.000 is the first time the simulator ran on August 6th, and 2025-08-06.001 is the second time the simulator ran on August 6th.  This scheme matches how the buoy maintains it's file structure as well, with the index refering to re-starts of the logging executable instead of simulator runs.  For convenience, a symbolic link is maintained that points to the most recent simulation run (latest_csv_dir).

Within each direcotry, .csv files are created that represent up to an hour of data.  At the end of each hour those files are zipped up and a new file is created for the next hour of data. Therefore, the timestamp shown in the directory listing for each .csv file is the time of the first data in the file.  In cases where the simulator is running faster than real time, the filenames represent the amount of simulated time that has passed since the beginning of the simulation.

### .csv data format