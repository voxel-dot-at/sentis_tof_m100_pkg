/******************************************************************************
 * Copyright (c) 2013
 * VoXel Interaction Design GmbH
 *
 * @author Angel Merino Sastre
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 ******************************************************************************/

/** @mainpage Sentis ToF M100 ROS package
 *
 * @section intro_sec Introduction
 *
 * This software defines a interface for working the ToF camera Sentis-ToF-M100 from Bluetechnix GmbH.
 *
 * @section install_sec Installation
 *
 * We encorage you to follow the instruction we prepared in:
 *
 * ROS wiki: http://wiki.ros.org/sentis_tof_m100
 * Github repository: https://github.com/voxel-dot-at/sentis_tof_m100_ros_pkg
 *
 */

#include <m100api.h>

#include <ros/console.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ros/publisher.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <stdio.h>
#include <time.h>
#include <sstream>

#include <sentis_tof_m100/sentis_tof_m100Config.h>
#include <dynamic_reconfigure/server.h>

/* Registers for Filter Configuration */
#define ImgProcConfig 				0x01E0
#define FilterMedianConfig 			0x01E1 
#define FilterAverageConfig 			0x01E2 
#define FilterGaussConfig 			0x01E3
#define FilterSLAFconfig 			0x01E5

/*Bitmasks for ImgProcConfig*/
#define Median_Filter 			1	// Bit[0]
#define Average_Filter			2	// Bit[1]
#define Gauss_Filter			4	// Bit[2]
#define Sliding_Average			16	// Bit[4]
#define Wiggling_Compensation		64	// Bit[6]
#define FPPN_Compensation		128	// Bit[7]
#define ModFreq_Scaling			256	// Bit[8]
#define Scaling_MM			512	// Bit[9]
#define Additive_Offset			1024	// Bit[10]
#define Temperature_Compensation	2048	// Bit[11]
#define Scaling_DistCalibGradient	4096	// Bit[12]
#define Scaling_DistCalibOffset		8192	// Bit[13]

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

/**
 * Global Parameter Declarations
 */

/**
 * Camera Configuration Parameters
 */
int integrationTime;
int modulationFrequency;
int frameRate;

bool amplitudeFilterOn;
float amplitudeThreshold;

unsigned short filterConfig, averageConfig, gaussConfig;

bool medianFilter, averageFilter, gaussFilter, slidingAverage, wigglingCompensation, FPPNCompensation, modFreqScaling, scalingMM,
	additiveOffset, temperatureCompensation, scalingDistCalibGradient, scalingDistCalibOffset;

bool filterAverageConfigPixels, filterGaussConfigPixels;
int medianConfigIters, averageConfigIters, gaussConfigIters, SLAConfigWindows;

bool first;
/**
 * Camera Driver Parameters
 */
T_SENTIS_HANDLE handle;
T_ERROR_CODE error;
T_SENTIS_DATA_HEADER header;

/**
 * ROS Parameters
 */
ros::Publisher pub_non_filtered;
ros::Publisher pub_filtered;

int help() {
		std::cout << "\n Using help for sentis_tof_m100_ros_pkg\n"
		" You can set the configuration values for the camera. If any option is missing the server parameter value or in last case, the default value will be used: \n" << std::endl;
	std::cout << " Usage:\n rosrun sentis_tof_m100 sentis_tof_m100_node <options> "<< std::endl
		<< "-tcp_ip *TCP IP Addresss* \n\tIp address for the control connection \n\t(string, i.e: 192.168.0.10) "<< std::endl
		<< "-tcp_port *Port for tcp* \n\tDefines the port used for the control connection \n\t(unsigned short, i.e: 10001) "<< std::endl
		<< "-udp_ip *UDP IP Addresss* \n\tMulticast ip address for the data connection \n\t(string, i.e: 224.0.0.1) "<< std::endl
		<< "-udp_port *Port for udp* \n\tDefines the port used for the data connection \n\t(unsigned short, i.e: 10001) "<< std::endl
		<< "-it *Integration_Time* \n\tIntegration time(in usec) for the sensor \n\t(min: 50 | max: 7000 | default: 1500) "<< std::endl
		<< "-mf  *Modulation_Frequency* \n\tSet the modulation frequency(Hz) of the sensor \n\t(min: 5000 | max: 30000 | default: 20000) "<< std::endl
		<< "-fr *Frame_Rate* \n\tSet the frame rate of the camera by setting the Phase Time \n\t(min: 1 | max: 45 | default: 40)" << std::endl

		<< "-mef *MedianFilter* \n\tSet on or off the Median Filter. \n\t(OFF: 0 | ON: any other integer value |  ON if not set ) " << std::endl 
		<< "-avf *AverageFilter* \n\tSet on or off the Average Filter. \n\t(OFF: 0 | ON: any other integer value |  ON if not set ) " << std::endl 
		<< "-gaf *GaussFilter* \n\tSet on or off the Gauss Filter. \n\t(OFF: 0 | ON: any other integer value |  ON if not set ) " << std::endl 
		<< "-sla *SlidingAverage* \n\tSet on or off the Sliding Average. \n\t(OFF: 0 | ON: any other integer value |  ON if not set ) " << std::endl 
		<< "-wic *WigglingCompensation* \n\tSet on or off the Wiggling Compensation. \n\t(OFF: 0 | ON: any other integer value |  ON if not set ) " << std::endl 
		<< "-fppnc *FPPNCompensation* \n\tSet on or off the FPPN Compensation. \n\t(OFF: 0 | ON: any other integer value |  ON if not set ) " << std::endl 
		<< "-mfs *ModFreqScaling* \n\tSet on or off the ModFreq Scaling. \n\t(OFF: 0 | ON: any other integer value |  ON if not set ) " << std::endl 
		<< "-smm *Scalingmm* \n\tSet on or off the Scaling to [mm]. \n\t(OFF: 0 | ON: any other integer value |  ON if not set ) " << std::endl 
		<< "-aos *AdditiveOffset* \n\tSet on or off the Additive Offset. \n\t(OFF: 0 | ON: any other integer value |  ON if not set ) " << std::endl 
		<< "-tmc *TemperatureCompensation* \n\tSet on or off the Temperature Compensation. \n\t(OFF: 0 | ON: any other integer value |  ON if not set ) " << std::endl 
		<< "-sdcg *ScalingDistCalibGradient* \n\tSet on or off the Scaling via register DistCalibGradient. \n\t(OFF: 0 | ON: any other integer value |  ON if not set ) " << std::endl 
		<< "-sdco *ScalingDistCalibOffset* \n\tSet on or off the Scaling via register DistCalibOffset. \n\t(OFF: 0 | ON: any other integer value |  ON if not set ) " << std::endl 

		<< "-mefite *FilterMedian_Config* \n\tSet the nº of iteration for the Media filter. \n\t(min: 1 | max: 255 | default: 1) "<< std::endl
		<< "-avfpix *FilterAverage_Config_Pixels* \n\tSet pixel matrix for the Average filter. \n\t(3x3: 0 | 5x5: 1 | Default: 3x3 ) " << std::endl
		<< "-avfite *FilterAverage_Config_Iters* \n\tSet the nº of iteration for the Average filter. \n\t(min: 1 | max: 255 | default: 1) "<< std::endl
		<< "-gafpix *FilterGauss_Config_Pixels* \n\tSet pixel matrix for the Gauss filter. \n\t(3x3: 0 | 5x5: 1 | Default: 3x3 ) " << std::endl
		<< "-gafite *FilterGauss_Config_Iters* \n\tSet the nº of iteration for the Gauss filter. \n\t(min: 1 | max: 255 | default: 1) "<< std::endl
		<< "-slacw *FilterSLAF_config* \n\tSet the SLAF filter windows size. \n\t(min: 1 | max: 255 | default: 1) "<< std::endl

		<< "-af *Amplitude_Filter_On* \n\tWhether to apply amplitude filter or not. Image pixels with amplitude values less than the threshold will be filtered out \n\t(ON: if set | OFF: default) " << std::endl
		<< "-at *Amplitude_Threshold* \n\tWhat should be the amplitude filter threshold. Image pixels with smaller aplitude values will be filtered out. Amplitude Filter Status should be true to use this filter \n\t(min: 0 | max: 2500 | default: 0) "<< std::endl
		<< "\n Example:" << std::endl
		<< "rosrun sentis_tof_m100 sentis_tof_m100 -tcp_ip 192.168.0.10 -tcp_port 10001 -it 1500 -mf 20000 -fr 20 \n" << std::endl;
	exit(0);
} //print_help

void callback(sentis_tof_m100::sentis_tof_m100Config &config, uint32_t level) {
	// Check the configuration parameters with those given in the initialization
	if(first) {
		config.Integration_Time = integrationTime;
		config.Modulation_Frequency = modulationFrequency;
		config.Frame_Rate = frameRate;
		
		config.Amplitude_Filter_On = amplitudeFilterOn;
		config.Amplitude_Threshold = amplitudeThreshold;
		
		frameRate = integrationTime = modulationFrequency = -1;
				
		config.MedianFilter = medianFilter;
		config.AverageFilter = averageFilter;
		config.GaussFilter = gaussFilter;
		config.SlidingAverage = slidingAverage;
		config.WigglingCompensation = wigglingCompensation; 
		config.FPPNCompensation = FPPNCompensation;
		config.ModFreqScaling = modFreqScaling;
		config.Scalingmm = scalingMM;
		config.AdditiveOffset = additiveOffset;
		config.TemperatureCompensation = temperatureCompensation;
		config.ScalingDistCalibGradient = scalingDistCalibGradient;
		config.ScalingDistCalibOffset = scalingDistCalibOffset;
		
		config.FilterMedian_Config = medianConfigIters;
		config.FilterAverage_Config_Pixels = filterAverageConfigPixels;
		config.FilterAverage_Config_Iters = averageConfigIters;
		config.FilterGauss_Config_Pixels = filterGaussConfigPixels;
		config.FilterGauss_Config_Iters = gaussConfigIters;
		config.FilterSLAF_config = SLAConfigWindows;
		medianConfigIters = averageConfigIters = gaussConfigIters = SLAConfigWindows = filterConfig = -1;
	}

	if(integrationTime != config.Integration_Time) {
		integrationTime = config.Integration_Time;
		error = STSwriteRegister(handle, IntegrationTime, integrationTime );
		if(error != 0) {
		    ROS_WARN_STREAM("Error writing register: " << error << "---------------");
		    STSclose(handle);
		    exit (1);
		}
	}

	if(modulationFrequency != config.Modulation_Frequency) {
		modulationFrequency = config.Modulation_Frequency;
		error = STSwriteRegister(handle, ModulationFrecuency, modulationFrequency );
		
		if(error != 0) {
		    ROS_WARN_STREAM("Error writing register: " << error << "---------------");
		    STSclose(handle);
		    exit (1);
		}
	}

	if(frameRate != config.Frame_Rate) {
		frameRate = config.Frame_Rate;
		error = STSwriteRegister(handle, FrameRate, frameRate );
		if(error != 0) {
		    ROS_WARN_STREAM("Error writing register: " << error << "---------------");
		    STSclose(handle);
		    exit (1);
		}
	}
	
	unsigned short filterConfNew = 0;
	if (config.MedianFilter)
		filterConfNew |= Median_Filter;
	if (config.AverageFilter)
		filterConfNew |= Average_Filter; 
	if (config.GaussFilter)
		filterConfNew |= Gauss_Filter; 
	if (config.SlidingAverage)
		filterConfNew |= Sliding_Average; 
	if (config.WigglingCompensation)
		filterConfNew |= Wiggling_Compensation;  
	if (config.FPPNCompensation)
		filterConfNew |= FPPN_Compensation; 
	if (config.ModFreqScaling)
		filterConfNew |= ModFreq_Scaling; 
	if (config.Scalingmm)
		filterConfNew |= Scaling_MM; 
	if (config.AdditiveOffset)
		filterConfNew |= Additive_Offset; 
	if (config.TemperatureCompensation)
		filterConfNew |= Temperature_Compensation;
	if (config.ScalingDistCalibGradient)
		filterConfNew |= Scaling_DistCalibGradient;
	if (config.ScalingDistCalibOffset)
		filterConfNew |= Scaling_DistCalibOffset;

	// TODO remove DEBUG
	//ROS_INFO_STREAM(filterConfNew);	
	//unsigned short res;
	//STSreadRegister(handle, ImgProcConfig, &res, 0, 0);
	//printf("ImgProcConfig : %#x	%d\n", res, res);	

	if (filterConfNew != filterConfig) {
		filterConfig = filterConfNew;
		error = STSwriteRegister(handle, ImgProcConfig, filterConfig );
		if(error != 0) {
		    // TODO remove DEBUG
		    //ROS_WARN_STREAM("errno: " << errno);

		    ROS_WARN_STREAM("Error writing register: " << error << "---------------");
		    STSclose(handle);
		    exit (1);
		}
	}
	

	if (medianConfigIters != config.FilterMedian_Config) {
		medianConfigIters = config.FilterMedian_Config;
		error = STSwriteRegister(handle, FilterMedianConfig, (unsigned short)medianConfigIters );
		if(error != 0) {
		    ROS_WARN_STREAM("Error writing register: " << error << "---------------");
		    STSclose(handle);
		    exit (1);
		}
	}
	
	filterConfNew = config.FilterAverage_Config_Iters << 8;
	if (config.FilterAverage_Config_Pixels)
		filterConfNew++;
	if (filterConfNew != averageConfig) {
		averageConfig = filterConfNew;
		error = STSwriteRegister(handle, FilterAverageConfig, averageConfig );
		if(error != 0) {
		    ROS_WARN_STREAM("Error writing register: " << error << "---------------");
		    STSclose(handle);
		    exit (1);
		}
	}
	
	filterConfNew = config.FilterGauss_Config_Iters << 8;
	if (config.FilterGauss_Config_Pixels)
		filterConfNew++;
	if (filterConfNew != gaussConfig) {
		gaussConfig = filterConfNew;
		error = STSwriteRegister(handle, FilterGaussConfig, gaussConfig);
		if(error != 0) {
		    ROS_WARN_STREAM("Error writing register: " << error << "---------------");
		    STSclose(handle);
		    exit (1);
		}
	}
	
	if (SLAConfigWindows != config.FilterSLAF_config) {
		SLAConfigWindows = config.FilterSLAF_config;
		error = STSwriteRegister(handle, FilterSLAFconfig, SLAConfigWindows);
		if(error != 0) {
		    ROS_WARN_STREAM("Error writing register: " << error << "---------------");
		    STSclose(handle);
		    exit (1);
		}
	}

	amplitudeFilterOn = config.Amplitude_Filter_On;
	amplitudeThreshold = config.Amplitude_Threshold;
	
	// TODO remove DEBUG
	//STSreadRegister(handle, ImgProcConfig, &res, 0, 0);
	//printf("ImgProcConfig : %#x	%d\n", res, res);
	
}

/**
 * Initialize the camera and initial parameter values. Returns 1 if properly initialized.
 */
int initialize(int argc, char *argv[],ros::NodeHandle nh){
	/*
	 * Inital Setup for parameters
	 */
	integrationTime = 1500;
	modulationFrequency = 20000;
	frameRate = 40;
	
	/*Default ImgProcConfig values: "2FC1"*/
	medianFilter = true;
	averageFilter = gaussFilter = slidingAverage = false;
	wigglingCompensation = FPPNCompensation = modFreqScaling = scalingMM = additiveOffset = temperatureCompensation =  true; 
	scalingDistCalibGradient = false;
	scalingDistCalibOffset = true;
	
	medianConfigIters = averageConfigIters = gaussConfigIters = 1;
	 
	filterAverageConfigPixels = filterGaussConfigPixels = false;
	
	filterConfig = 0x2FC1;
	averageConfig = gaussConfig = 0x0100;
	SLAConfigWindows = 5;
	
	amplitudeFilterOn = false;
	amplitudeThreshold = 0;
	
	/*
	 * Camera Initialization
	 */
	T_SENTIS_CONFIG sentisConfig;
	// Default configuration
        sentisConfig.tcp_ip = "192.168.0.10";
        sentisConfig.udp_ip = "224.0.0.1";
        sentisConfig.udp_port = 10002;
        sentisConfig.tcp_port = 10001;
        sentisConfig.flags = HOLD_CONTROL_ALIVE;
        
        std::string nodeName = ros::this_node::getName();
        
        // Overiding default configuration with parameter server.
        std::string aux_tcpIp,aux_udpIp;
        if (nh.getParam(nodeName+"/network/tcp_ip",aux_tcpIp))
        	sentisConfig.tcp_ip = aux_tcpIp.c_str();
        if (nh.getParam(nodeName+"/network/udp_ip",aux_udpIp))
       		sentisConfig.udp_ip = aux_udpIp.c_str();
       	
        int aux_port;
        if (nh.getParam(nodeName+"/network/tcp_port",aux_port))
        	sentisConfig.tcp_port = (unsigned short)aux_port;
        if (nh.getParam(nodeName+"/network/udp_port",aux_port))
        	sentisConfig.udp_port = (unsigned short)aux_port;  
        
        nh.getParam(nodeName+"/integrationTime",integrationTime);
        nh.getParam(nodeName+"/modulationFrecuency",modulationFrequency);
        nh.getParam(nodeName+"/frameRate",frameRate);

	nh.getParam(nodeName+"/amplitudeFilterOn",amplitudeFilterOn);
	double aux_double;
	if (nh.getParam(nodeName+"/amplitudeThreshold",aux_double))
      		amplitudeThreshold = (float)aux_double;
      	
      	// Filters flags	
      	nh.getParam(nodeName+"/filters/mediaFilter",medianFilter);
      	nh.getParam(nodeName+"/filters/averageFilter",averageFilter);
      	nh.getParam(nodeName+"/filters/gaussFilter",gaussFilter);
      	nh.getParam(nodeName+"/filters/slidingAverage",slidingAverage);
      	nh.getParam(nodeName+"/filters/wigglingCompensation",wigglingCompensation);
      	nh.getParam(nodeName+"/filters/FPPNCompensation",FPPNCompensation);
      	nh.getParam(nodeName+"/filters/modFreqScaling",modFreqScaling);
      	nh.getParam(nodeName+"/filters/scalingMM",scalingMM);
      	nh.getParam(nodeName+"/filters/additiveOffset",additiveOffset);
      	nh.getParam(nodeName+"/filters/temperatureCompensation",temperatureCompensation);
      	nh.getParam(nodeName+"/filters/scalingDistCalibGradient",scalingDistCalibGradient);
      	nh.getParam(nodeName+"/filters/scalingDistCalibOffset",scalingDistCalibOffset);
	
	// Filter configurations
	nh.getParam(nodeName+"/filters/FilterMedianConfig",medianConfigIters);
	nh.getParam(nodeName+"/filters/averageConfig_Iters",averageConfigIters);
	nh.getParam(nodeName+"/filters/filterAverageConfig_Pixels",filterAverageConfigPixels);
	nh.getParam(nodeName+"/filters/gaussConfig_Iters",gaussConfigIters);
	nh.getParam(nodeName+"/filters/filterGaussConfig_Pixels",filterGaussConfigPixels);
	nh.getParam(nodeName+"/filters/SLAConfigWindows",SLAConfigWindows);
	
	char aux_char[20]; 
	for( int i = 1; i < argc; i++) {
		if( std::string(argv[i]) == "-tcp_ip" ) {
			aux_tcpIp = argv[++i];
			sentisConfig.tcp_ip = aux_tcpIp.c_str();
		}
		else if( std::string(argv[i]) == "-udp_ip" ) {
			aux_udpIp = argv[++i];
			sentisConfig.udp_ip = aux_udpIp.c_str();
		}
		else if( std::string(argv[i]) == "-tcp_port" ) {
			if( sscanf(argv[++i], "%d", &aux_port) != 1 
				|| aux_port < 0 || aux_port > 65535 ) {
				ROS_WARN("*invalid tcp port");
				return help();
			}
			sentisConfig.tcp_port = (unsigned short)aux_port;
		}
		else if( std::string(argv[i]) == "-udp_port" ) {
			if( sscanf(argv[++i], "%d", &aux_port) != 1 
				|| aux_port < 0 || aux_port > 65535 ) {
				ROS_WARN("*invalid udp port");
				return help();
			}
			sentisConfig.udp_port = (unsigned short)aux_port;
		}
		else if( std::string(argv[i]) == "-it" ) {
			if( sscanf(argv[++i], "%d", &integrationTime) != 1 
				|| integrationTime < 50 || integrationTime > 7000 ) {
				ROS_WARN("*invalid integration time");
				return help();
			}
		}
		else if( std::string(argv[i]) == "-mf" ) {
			if( sscanf(argv[++i], "%d", &modulationFrequency) != 1 
				|| modulationFrequency < 5000 || integrationTime > 30000 ) {
				ROS_WARN("*invalid modulation frequency");
				return help();
			}
		}
		else if( std::string(argv[i]) == "-fr" ) {
			if( sscanf(argv[++i], "%d", &frameRate) != 1 
				|| frameRate < 1 || frameRate > 45 ) {
				ROS_WARN("*invalid frame rate");
				return help();
			}
		}
		else if( std::string(argv[i]) == "-mef" ) {
			if( sscanf(argv[++i], "%d", &aux_port) != 1 ) {
				if (aux_port == 0)
					medianFilter = false;
				else
					medianFilter = true;
				continue;			
			}
			ROS_WARN("*invalid value for Median Filter");
			return help();
		}
		else if( std::string(argv[i]) == "-avf" ) {
			if( sscanf(argv[++i], "%d", &aux_port) != 1 ) {
				if (aux_port == 0)
					averageFilter = false;
				else
					averageFilter = true;
				continue;			
			}
			ROS_WARN("*invalid value for Average Filter");
			return help();
		}
		else if( std::string(argv[i]) == "-gaf" ) {
			if( sscanf(argv[++i], "%d", &aux_port) != 1 ) {
				if (aux_port == 0)
					gaussFilter = false;
				else
					gaussFilter = true;
				continue;			
			}
			ROS_WARN("*invalid value for Gauss Filter");
			return help();
		}
		else if( std::string(argv[i]) == "-sla" ) {
			if( sscanf(argv[++i], "%d", &aux_port) != 1 ) {
				if (aux_port == 0)
					slidingAverage = false;
				else
					slidingAverage = true;
				continue;			
			}
			ROS_WARN("*invalid value for Sliding Average Filter");
			return help();
		}
		else if( std::string(argv[i]) == "-wic" ) {
			if( sscanf(argv[++i], "%d", &aux_port) != 1 ) {
				if (aux_port == 0)
					wigglingCompensation = false;
				else
					wigglingCompensation = true;
				continue;			
			}
			ROS_WARN("*invalid value for Wiggling Compensation");
			return help();
		}
		else if( std::string(argv[i]) == "-fppnc" ) {
			if( sscanf(argv[++i], "%d", &aux_port) != 1 ) {
				if (aux_port == 0)
					FPPNCompensation = false;
				else
					FPPNCompensation = true;
				continue;			
			}
			ROS_WARN("*invalid value for FPPN Compensation");
			return help();
		}
		else if( std::string(argv[i]) == "-mfs" ) {
			if( sscanf(argv[++i], "%d", &aux_port) != 1 ) {
				if (aux_port == 0)
					modFreqScaling = false;
				else
					modFreqScaling = true;
				continue;			
			}
			ROS_WARN("*invalid value for ModFreq Scaling");
			return help();
		}
		else if( std::string(argv[i]) == "-smm" ) {
			if( sscanf(argv[++i], "%d", &aux_port) != 1 ) {
				if (aux_port == 0)
					scalingMM = false;
				else
					scalingMM = true;
				continue;			
			}
			ROS_WARN("*invalid value for Scaling to [mm]");
			return help();
		}
		else if( std::string(argv[i]) == "-aos" ) {
			if( sscanf(argv[++i], "%d", &aux_port) != 1 ) {
				if (aux_port == 0)
					additiveOffset = false;
				else
					additiveOffset = true;
				continue;			
			}
			ROS_WARN("*invalid value for Additive Offset");
			return help();
		}
		else if( std::string(argv[i]) == "-tmc" ) {
			if( sscanf(argv[++i], "%d", &aux_port) != 1 ) {
				if (aux_port == 0)
					temperatureCompensation = false;
				else
					temperatureCompensation = true;
				continue;			
			}
			ROS_WARN("*invalid value for Temperature Compensation");
			return help();
		}
		else if( std::string(argv[i]) == "-sdcg" ) {
			if( sscanf(argv[++i], "%d", &aux_port) != 1 ) {
				if (aux_port == 0)
					scalingDistCalibGradient = false;
				else
					scalingDistCalibGradient = true;
				continue;			
			}
			ROS_WARN("*invalid value for Scaling via register DistCalibGradient");
			return help();
		}
		else if( std::string(argv[i]) == "-sdco" ) {
			if( sscanf(argv[++i], "%d", &aux_port) != 1 ) {
				if (aux_port == 0)
					scalingDistCalibOffset = false;
				else
					scalingDistCalibOffset = true;
				continue;			
			}
			ROS_WARN("*invalid value for Scaling via register DistCalibOffset");
			return help();
		}

		else if( std::string(argv[i]) == "-mefite" ) {
			if( sscanf(argv[++i], "%d", &medianConfigIters) != 1 
				|| medianConfigIters < 1 || medianConfigIters > 255 ) {
				ROS_WARN("*invalid value for Median filter Iterations value");
				return help();
			}
		}
		else if( std::string(argv[i]) == "-avfpix" ) {
			if( sscanf(argv[++i], "%d", &aux_port) != 1 ) {
				if (aux_port == 0) {
					filterAverageConfigPixels = false; //3x3
					continue;
				} else if (aux_port == 1) {
					filterAverageConfigPixels = true; //5x5
					continue;			
				}
			}
			ROS_WARN("*invalid value for Average Filter Pixels Configuration");
			return help();
		}
		else if( std::string(argv[i]) == "-avfite" ) {
		   if( sscanf(argv[++i], "%d", &averageConfigIters) != 1 
				|| averageConfigIters < 1 || averageConfigIters > 255 ) {
				ROS_WARN("*invalid value for Average filter Iterations value");
				return help();
			}
		}
		else if( std::string(argv[i]) == "-gafpix" ) {
			if( sscanf(argv[++i], "%d", &aux_port) != 1 ) {
				if (aux_port == 0) {
					gaussConfigIters = false; //3x3
					continue;
				} else if (aux_port == 1) {
					gaussConfigIters = true; //5x5
					continue;			
				}
			}
			ROS_WARN("*invalid value for Gauss Filter Pixels Configuration");
			return help();
		}
		else if( std::string(argv[i]) == "-gafite" ) {
		   if( sscanf(argv[++i], "%d", &gaussConfigIters) != 1 
				|| gaussConfigIters < 1 || gaussConfigIters > 255 ) {
				ROS_WARN("*invalid value for Gauss filter Iterations value");
				return help();
			}
		}
		else if( std::string(argv[i]) == "-slacw" ) {
		   if( sscanf(argv[++i], "%d", &SLAConfigWindows) != 1 
				|| SLAConfigWindows < 1 || SLAConfigWindows > 255 ) {
				ROS_WARN("*invalid value for SLAF Filter Windows size value");
				return help();
			}
		}
		// additional parameters
		else if( std::string(argv[i]) == "-af" ) {
			amplitudeFilterOn = true;
		}
		else if( std::string(argv[i]) == "-at" ) {
			if( sscanf(argv[++i], "%f", &amplitudeThreshold) != 1 
				|| amplitudeThreshold < 0 || amplitudeThreshold > 2500 ) {
				ROS_WARN("*invalid amplitude threshold");
				return help();
			}	
		}
		// print help
		else if( std::string(argv[i]) == "--help" ) {
			ROS_INFO_STREAM("argument: " << argc << " which: " << argv[i]);		
			return help();
		}
		else if( argv[i][0] == '-' ) {
			ROS_WARN_STREAM("invalid option " << argv[i]);
			return help();
		}
	} 	

	ROS_INFO_STREAM("Connection data: tcp_ip: " << sentisConfig.tcp_ip << " tcp_port: " << sentisConfig.tcp_port << " udp_ip: " << sentisConfig.udp_ip << " udp_port: " << sentisConfig.udp_port );

        //connect with the device
        printf("CLIENT: open connection\n");
        handle = STSopen(&sentisConfig,&error);
        if(error != 0) {
	    ROS_WARN_STREAM("Could not connect: " << error << "---------------");
            exit (1);
        }

        error = STSwriteRegister(handle, ImageDataFormat, XYZ_AMP_DATA );
        if(error != 0) {
	    ROS_WARN_STREAM("Error writing register: " << error << "---------------");
	    STSclose(handle);
            exit (1);
        }

	
	/*
	 * ROS Node Initialization
	 */
	pub_non_filtered = nh.advertise<PointCloud> ("depth_non_filtered", 1);
	pub_filtered = nh.advertise<PointCloud> ("depth_filtered", 1);
	return 1;
}

/**
 * Publish the data based on set up parameters.
 */
int publishData() {

	 int size = 153600; // 120*4*160*2bytes
   	 short buffer[120*4][160];

    	error = STSgetData(handle, &header,  (char *)buffer, &size, 0, 0);
    	if (error == -1) {
		ROS_ERROR_STREAM("Could get frame: " << error);
		STSclose(handle);
		return 0;
	}

	/*
	 * Creating the pointcloud
	 */

	// Fill in the cloud data
	PointCloud::Ptr msg_non_filtered (new PointCloud);
	msg_non_filtered->header.frame_id = "tf_sentis_tof";
	msg_non_filtered->height = header.imageHeight;
	msg_non_filtered->width = header.imageWidth;
	msg_non_filtered->is_dense = true;
	
	PointCloud::Ptr msg_filtered (new PointCloud);
	msg_filtered->header.frame_id = "tf_sentis_tof";
	msg_filtered->width    = 1;
	msg_filtered->height   = header.imageWidth*header.imageHeight;
	msg_filtered->is_dense = false;

	int countWidth=0;

	for (int i= 0 ; i < header.imageHeight; i++) {
        	for(int j=0; j<header.imageWidth; j++) {
			pcl::PointXYZI temp_point;
			temp_point.x = buffer[i+120][j]/1000.f;  //z
		 	temp_point.y = buffer[i+(120*2)][j]/1000.f; // y

			if (buffer[i][j]<0) {
				temp_point.z = nanf("");
		   	} else {
				temp_point.z = buffer[i][j]/1000.f; //x
		    	}
			temp_point.intensity = buffer[i+(120*3)][j];
			if(amplitudeFilterOn==true && buffer[i+(120*3)][j]>amplitudeThreshold) {
				msg_filtered->points.push_back(temp_point);
				countWidth++;
			}
			msg_non_filtered->points.push_back(temp_point);
			//ROS_INFO_STREAM(temp_point);
		}
	}

	msg_filtered->height   = countWidth;
	
	 /*
	  * Publishing the messages
	  */
	 if(amplitudeFilterOn){
		#if ROS_VERSION > ROS_VERSION_COMBINED(1,9,49)
			msg_filtered->header.stamp = ros::Time::now().toNSec();
		#else
			msg_filtered->header.stamp = ros::Time::now();
		#endif
			pub_filtered.publish (msg_filtered);
	 }

	#if ROS_VERSION > ROS_VERSION_COMBINED(1,9,49)
		msg_non_filtered->header.stamp = ros::Time::now().toNSec();
	#else
		msg_non_filtered->header.stamp = ros::Time::now();
	#endif
		pub_non_filtered.publish (msg_non_filtered);

	return 1;
}

/**
 *
 * @brief Main function
 *
 * @param [in] int
 * @param [in] char *
 *
 */
int main(int argc, char *argv[]) {
	ROS_INFO("Starting sentis_tof_m100 ros...");
	ros::init (argc, argv, "sentis_tof_m100");
	ros::NodeHandle nh;

	dynamic_reconfigure::Server<sentis_tof_m100::sentis_tof_m100Config> srv;
	dynamic_reconfigure::Server<sentis_tof_m100::sentis_tof_m100Config>::CallbackType f;

	f = boost::bind(&callback, _1, _2);

	if(initialize(argc, argv,nh)){
		first = true;
		srv.setCallback(f);
		first = false;
		ROS_INFO("Initalized Camera... Reading Data");
		ros::Rate loop_rate(10);
		while (nh.ok())
		{
			publishData();
			ros::spinOnce ();
			loop_rate.sleep ();
		}
	} else {
		ROS_WARN("Cannot Initialize Camera. Check the parameters and try again!!");
		return 0;
	}

	STSclose(handle);
	return 0;
}
