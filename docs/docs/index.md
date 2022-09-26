# Background  

The MBARI Wave-Energy conveter is a point-absorber type wave-energy conveter that has been operating in Monterey Bay, CA since 2014.
This system was developed  as part of  [Monterey Bay Aquarium Research Institutes](http://www.mbari.org) goals of advancing and demonstrating an autonomous and persistent presence of oceanographic instrumentation in the worlds oceans.
This project is complemented by developments in autonomous underwater vehicles, underwater vehicle docking, oceanographic instrumentation, autonomy, and science use. 

The MBARI-WEC is currently maintained by MBARI and operates for six-month periods near the MBARI facility in Moss Landing, California, and averages about 250 Watts of power capture, averaged through the weather cycles and seasons.

THe MBARI WEC is a complete system with a four-quadrant electro-hydraulic power-take-off device, board battery storage, control-computers, sensors and instrumentation, and an always-on cell-modem connection to the internet.
The architecture of the system is such that critical functions are performed by micro-controllers throughout the system that implement default behaviors and stream sensor data continously.
A Linux computer on the buoy performs data-logging and provides a command interface to the underlying micro-controllers. The system is designed such that the Linux computer is not necessary for safe behavior, if this computer re-boots or goes offline, the system will default to safe behaviors.  Additionally, the microcontrollers will ignore damaging commands from the Linux computer. 
This architecture allows control algorithms running on the Linux computer to be started, stopped, and changed while the device is at sea through the cell-modem connection.  

This project provides a software interface to the system to allow such algorithms to be efficiently developed, tested, and executed.
Using this interfarce, MBARI intends to make the system available to external researchers.
By providing access to the hardware during the ongoing MBARI deployments of this system, the intention is to provide access to hardware that is often otherwise unavailable.  
To facilitate this, the project has developed a simulator that provides the same interface as the real hardware, allowing projects the ability to develop and test their work independetly, before deployment on the real system which will occur under MBARI supervision.

The following sections of this documentation outlines the physical system, describes the software-interfaces available, describes the simulation enviornoment, and provides all information needed to interact with this project.
The software interface is built upon the [ROS2](www.ros.org) framework, and the simulation environment uses [Ignition Gazebo](www.gazebo.org).
In addition to descriptions of these systems, the documentation provides a number of tutorials intended to lead a new user through the installation of the neccessary tools, basic operation of the system, and provide guidance on implement new alorithms to run on the simulator and ultimately the buoy.  This is an open-source project with all neccessray code and resources freely available.  

