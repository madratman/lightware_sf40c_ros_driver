#include <ros/ros.h>
#include <string>
#include <iostream>
#include <cstdio>

#include <std_msgs/String.h>
#include "serial/serial.h"
#include <sensor_msgs/LaserScan.h>


#define REPLY_SIZE 20
#define TIMEOUT 35

const unsigned int sensor_frequency = 60; /*sensor frequency in hz*/

std::string serial_port;

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

void enumerate_ports()
{
	vector<serial::PortInfo> devices_found = serial::list_ports();
	vector<serial::PortInfo>::iterator iter = devices_found.begin();
	while( iter != devices_found.end() )
	{
		serial::PortInfo device = *iter++;
		printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
	     device.hardware_id.c_str() );
	}
}

void print_usage()
{
	cerr << "Usage: test_serial {-e|<serial port address>} ";
    cerr << "<baudrate> [test string]" << endl;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "sf40_node");
	ros::NodeHandle nh;
	ros::Publisher laser_publisher = nh.advertise<sensor_msgs::LaserScan>("laser_scan", 5);
	ros::Rate loop_rate(sensor_frequency); 
	
	string port(argv[1]);

	if( port == "-e" ) 
	{
		enumerate_ports();
		return 0;
	}
	else if( argc < 3 ) 
	{
		print_usage();
		return 1;
	}

	// Argument 2 is the baudrate
	unsigned long baud = 0;
	#if defined(WIN32) && !defined(__MINGW32__)
	sscanf_s(argv[2], "%lu", &baud);
	#else
	sscanf(argv[2], "%lu", &baud);
	#endif

  	serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));
	char reply[REPLY_SIZE];
	
	sensor_msgs::LaserScan laser_scan_msg;

    //setting default device path for the sensor
	nh.param("serial_port", serial_port, std::string("/dev/ttyUSB0"));
	
	cout << "Is the serial port open?";
	if(my_serial.isOpen())
	cout << " Yes." << endl;
	else
	cout << " No." << endl;

	// Get the Test string
	int count = 0;
	string test_string;
	if (argc == 4) 
	{
		test_string = argv[3];
	} 
	else
	{
		test_string = "Testing.";
	}

	while(ros::ok()) {
		
	    size_t bytes_wrote = my_serial.write(test_string);
	    string result = my_serial.read(test_string.length()+1000);

	    //	count is bakar
    	cout << "Iteration: " << count << ", Bytes written: ";
    	cout << bytes_wrote << ", Bytes read: ";
    	cout << result.length() << ", String read: " << result << endl;
	    count += 1;

	    // TODO catch the error

		// parse the response from the lidar to make a laser scan message with angles and 
		// corresponding distances

		laser_scan_msg.header.frame_id="sf40_frame";
		laser_scan_msg.header.stamp=ros::Time::now();
		
		laser_publisher.publish(laser_scan_msg);

		// RIP, dear loop
		loop_rate.sleep(); 
	}
	
	ros::spin();
	return 0; 
}
