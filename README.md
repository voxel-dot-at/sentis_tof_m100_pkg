sentis_tof_m100_ros_pkg
===================

---
This ROS package uses an old api for the Bluetechnix Sentis ToF m100 sensor.

There is a new package (https://github.com/voxel-dot-at/bta_tof_driver) that uses the new [BltToFApi](https://support.bluetechnix.at/wiki/Bluetechnix_'Time_of_Flight'_API) developed by Bluetechnix for interacting with their sensors.

We recommend to use the new bta_tof_driver package.

---

### ROS package for Bluetechnix Sentis ToF M100 camera. ###

# Summary #

This package explains how to configure your system and ROS to use the Sentis ToF M100 camera.
It includes an example allowing you to visualize point clouds using the rviz viewer included in ROS.
It shows you how to use the camera together with ROS and how you can modify different parameters of the Sentis ToF M100.

## First step: Get Ros ##

The sentis_tof_m100_ros_pkg works with ROS versions groovy and hydro. You can use catkin workspaces or the previous rosbuild to configure, compile and get ready ROS.

The following ROS tutorial links describe how to get ros_hydro and catkin workspace.

In Ubuntu:
Follow the ROS installation tutorial: 
>http://wiki.ros.org/hydro/Installation/Ubuntu.

Use catkin workspaces:
>http://wiki.ros.org/catkin 
>
>http://wiki.ros.org/catkin_or_rosbuild
>
>http://wiki.ros.org/catkin/Tutorials/create_a_workspace

To configure a catkin workspace in your ROS installation, follow this; 
>ROS tutorial: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

## Known Problems ##

Be sure your libboost library version is >= 1.49.
Previous versions as 1.46 generate error while compiling sentis_tof_m100_ros_pkg.

# 1. Installing the sentis_m100 API #

With your M100 camera you will get the developer API for Linux systems in both versions, amd64 and i386. If you are using Ubuntu or a Debian-derived distribution simply install the deb package corresponding to your system. 

You can also manually install the libraries and headers of the API. Please read the API instructions to get help for installing them.

# 2. Compiling the package #

#### 2.1 Install dependencies ####

Make sure you have the following ROS dependencies already installed:
<pre><code> apt-get install ros-hydro-pcl-ros ros-hydro-pcl-conversions ros-hydro-perception-pcl 
</code></pre>

#### 2.2 Install the package ####

Clone from repository: https://github.com/voxel-dot-at/sentis_tof_m100_pkg.git
to the src/ folder in your catkin workspace.
Now compile it with:
<pre><code>cd catkin_ws
source devel/setup.bash ## initialize search path to include local workspace
cd src/
git clone https://github.com/voxel-dot-at/sentis_tof_m100_ros_pkg.git
cd ..
catkin_make
</code></pre>

# 3. Usage #

We have included a .launch file to help you to get the camera working in a very simple way. We coded this ROS package to use the ROS parameter server. We will explain you in the following lines how you can write your own configuration for the Sentis ToF Camera. You can also run the package node standalone and set the camera configuration by line commands.

### * Watch our demo video:  ###

> http://youtu.be/3xegxf5VFWc

### 3.1 Use roslaunch ###
To easily start using the sentis_tof_m100 ROS package you can use the roslaunch config_file we have included. It will launch the ROS core, start the camera node loading the parameter configuration, start the run-time reconfiguration gui and the ROS viewer rviz already configured to show the depth information readed by the ToF camera.

In order to execute it you have just to type the following:

<pre><code>roslaunch sentis_tof_m100 start.launch
</code></pre>

#### 3.1.1 Write you own configuration to the parameter server ####

As we pointed out before, you can load the camera configuration to the parameter server. The file "launch/start.launch" includes, inside the tag "node", the tag "rosparam" in which you can indicate a file containing the server parameters. We have include the configuration file "/launch/sentis_tof_m100_1.yaml" that defines default configuration values for the camera. You may modify this file or add yours to load your own configuration.

To get more information about the Sentis_ToF_M100 camera parameter, please refer to the section 4. 

### 3.2 running the package without roslaunch ###

You can start up the camera without the help of roslaunch with the following steps.

#### 3.2.1 Start the ROS core ####

<pre><code>roscore &
</code></pre>

#### 3.2.2 Start capturing ####

<pre><code>rosrun sentis_tof_m100 sentis_tof_m100_node #[options]
</code></pre>

*Use --help parameter to display parameter initialization usage*

<pre><code> Using help for sentis_tof_m100_ros_pkg
 You can set the configuration values for the camera. If any option is missing the value of the parameter server or the default value will be used: 

 Usage:
 rosrun sentis_tof_m100 sentis_tof_m100_node <options> 
-tcp_ip *TCP IP Addresss* 
	Ip address for the control connection 
	(string, i.e: 192.168.0.10) 
-tcp_port *Port for tcp* 
	Defines the port used for the control connection 
	(unsigned short, i.e: 10001) 
-udp_ip *UDP IP Addresss* 
	Multicast ip address for the data connection 
	(string, i.e: 224.0.0.1) 
-udp_port *Port for udp* 
	Defines the port used for the data connection 
	(unsigned short, i.e: 10001) 
-it *Integration_Time* 
	Integration time(in usec) for the sensor 
	(min: 50 | max: 7000 | default: 1500) 
-mf  *Modulation_Frequency* 
	Sets the modulation frequency(Hz) of the sensor 
	(min: 5000 | max: 30000 | default: 20000) 
-fr *Frame_Rate* 
	Sets the frame rate of the camera 
	(min: 1 | max: 45 | default: 40)
-mef *MedianFilter* 
	Sets on or off the Median Filter. 
	(OFF: 0 | ON: any other integer value |  ON if not set ) 
-avf *AverageFilter* 
	Sets on or off the Average Filter. 
	(OFF: 0 | ON: any other integer value |  ON if not set ) 
-gaf *GaussFilter* 
	Sets on or off the Gauss Filter. 
	(OFF: 0 | ON: any other integer value |  ON if not set ) 
-sla *SlidingAverage* 
	Sets on or off the Sliding Average. 
	(OFF: 0 | ON: any other integer value |  ON if not set ) 
-wic *WigglingCompensation* 
	Sets on or off the Wiggling Compensation. 
	(OFF: 0 | ON: any other integer value |  ON if not set ) 
-fppnc *FPPNCompensation* 
	Sets on or off the FPPN Compensation. 
	(OFF: 0 | ON: any other integer value |  ON if not set ) 
-mfs *ModFreqScaling* 
	Sets on or off the ModFreq Scaling. 
	(OFF: 0 | ON: any other integer value |  ON if not set ) 
-smm *Scalingmm* 
	Sets on or off the Scaling to [mm]. 
	(OFF: 0 | ON: any other integer value |  ON if not set ) 
-aos *AdditiveOffset* 
	Sets on or off the Additive Offset. 
	(OFF: 0 | ON: any other integer value |  ON if not set ) 
-tmc *TemperatureCompensation* 
	Sets on or off the Temperature Compensation. 
	(OFF: 0 | ON: any other integer value |  ON if not set ) 
-sdcg *ScalingDistCalibGradient* 
	Sets on or off the Scaling via register DistCalibGradient. 
	(OFF: 0 | ON: any other integer value |  ON if not set ) 
-sdco *ScalingDistCalibOffset* 
	Sets on or off the Scaling via register DistCalibOffset. 
	(OFF: 0 | ON: any other integer value |  ON if not set ) 
-mefite *FilterMedian_Config* 
	Sets the nº of iteration for the Media filter. 
	(min: 1 | max: 255 | default: 1) 
-avfpix *FilterAverage_Config_Pixels* 
	Sets pixel matrix for the Average filter. 
	(3x3: 0 | 5x5: 1 | Default: 3x3 ) 
-avfite *FilterAverage_Config_Iters* 
	Sets the nº of iteration for the Average filter. 
	(min: 1 | max: 255 | default: 1) 
-gafpix *FilterGauss_Config_Pixels* 
	Sets pixel matrix for the Gauss filter. 
	(3x3: 0 | 5x5: 1 | Default: 3x3 ) 
-gafite *FilterGauss_Config_Iters* 
	Sets the nº of iteration for the Gauss filter. 
	(min: 1 | max: 255 | default: 1) 
-slacw *FilterSLAF_config* 
	Sets the SLAF filter windows size. 
	(min: 1 | max: 255 | default: 1) 
-af *Amplitude_Filter_On* 
	Whether to apply amplitude filter or not. Image pixels with amplitude values less than the threshold will be filtered out 
	(ON: if set | OFF: default) 
-at *Amplitude_Threshold* 
	What should be the amplitude filter threshold. Image pixels with smaller amplitude values will be filtered out. Amplitude Filter Status should be true to use this filter 
	(min: 0 | max: 2500 | default: 0) 

 Example:
rosrun sentis_tof_m100 sentis_tof_m100_node -tcp_ip 192.168.0.10 -tcp_port 10001 -it 1500 -mf 20000 -fr 20 
</code></pre>

#### 3.2.3 Visualization in rviz ####

<pre><code>rosrun rviz rviz
</code></pre>

*After the rviz window comes up, set the following options:*

1. In the "Display" sidebar on the left, set in the "Global Option" section the fixed_frame to **/tf_sentis_tof**
2. At the bottom in the "Display" sidebar click on the **add** Button
3. In the "Create visualization" dialog that opens, select the "By topic" tab and select the **/depth_non_filtered** topic.

Add a Pointcloud2 topic to visualize the clouds. Two different point sets are published with following topic names:
> - **/depth_non_filtered :** raw data from the tof camera
> - **/depth_filtered : after** applying statistical outlier detection from pcl

#### 3.2.4 Using filters and parameters configuration ####

To use the filters and change camera parameters at runtime, use rqt_reconfigure from ROS. To use it just run (after launching sentis_tof_m100_ros_pkg):

<pre><code>rosrun rqt_reconfigure rqt_reconfigure 
</code></pre>

*Select /sentis_tof_m100 to view the available options for modifications.*

#### 4 Camera parameters description ####

Here you will find a quick description of the camera parameter that can be set in this package:

* Network options

> **Note1:** Make sure your network is correctly configured. We recommend you to follow the API instructions to check whether the camera and your network are working without any problem.

> **Note2:** Network parameters cannot be changed at runtime.

* * **TCP_IP_Address :** The control connection with your sentis_tof_m100 camera.
* * **TCP_IP_Port :** The port used by the control connection.
* * **UDP_IP_Address :** The multicast data connection.
* * **UDP_IP_Port :** The port used by the data connection.
 
> **Note3 :** Following camera parameters and filtering methods can be accessed in runtime using the rqt_reconfigure. 
 
* Basic options
* * **Integration_Time :** Modifies the integration time of the sensor (in usec).
* * **Modulation_Frequency :** Modifies the modulation frequency of the sensor (in Hz).
* * **Frame_Rate :** Changes the camera capturing frame rate (in frames/s).

* Camera filters
* * **Median_Filter :** Activates the median filter.
* * **Average_Filter :** Activates the average filter.
* * **Gauss_Filter :** Activates the gauss filter.
* * **Wiggling_Compensation :** Activates the wiggling compensation.
* * **FPPN_Compensation :** Activates the FPPN compensation.
* * **ModFreq_Scaling :** Activates the ModFreq scaling.
* * **Scaling_to_MM :** Activates scaling to millimeters.
* * **Additive_Offset :** Activates the additive offset.

* * **Temperature_Compensation :** Activates the temperature compensation.
* * **Scaling_DistCalibGradient :** Activates scaling via the register DistCalibGradient.
* * **Scaling_DistCalibOffset :** Activates scaling via the register DistCalibOffset.

* Camera filters options
* * **Median_Filter_Iterations :** Defines the number of iterations for the median filter. A high value will decrease the frame time.
* * **Average_filter_Pixels :** Defines the pixel matrix for the average filter. 3x3 or 5x5 pixels.
* * **Average_Filter_Iterations :** Defines the number of iterations for the average filter. A high value will decrease the frame time..
* * **Gauss_filter_Pixels :** Defines the pixel matrix for the gauss filter. 3x3 or 5x5 pixels.
* * **Gauss_Filter_Iterations :** Defines the number of iterations for the gauss filter.
* * **SLAF_Filter_Windows_Size :** Defines the windows size for the SLAF filter.

* PCL filter
* * **Amplitude_Filter_On :** Indicates if the amplitude filter should be used or not
* * **Amplitude_Threshold :** Image pixels with smaller amplitude values will be filtered out. Amplitude_Filter_On status should be true to apply this filter.


