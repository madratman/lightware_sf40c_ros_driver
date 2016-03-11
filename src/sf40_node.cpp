#include <ros/ros.h>
#include <string>
#include <iostream>
#include <cstdio>

#include <std_msgs/String.h>
#include "serial/serial.h"
#include <sensor_msgs/LaserScan.h>

#define REPLY_SIZE 20
#define TIMEOUT 35

const unsigned int sensor_frequency = 1;
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

// Function to split the string, coz this is fucking cpp
vector<string> &split(const string &s, char delim, vector<string> &elems) {
    std::stringstream ss(s);
    string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

vector<string> split(const string &s, char delim) {
    vector<string> elems;
    split(s, delim, elems);
    return elems;
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
		
		//.For MMI mode, we need the <CR><LF> regex
		// Use teststring = "?TM,360,0" for a laser scanner response map, delimited by commas.
	    size_t bytes_wrote = my_serial.write(test_string + "\r\n");

	    // For HMI mode, well thug life
	    // size_t bytes_wrote = my_serial.write(test_string);

	    string response_string = my_serial.read(test_string.length()+20000); 
	    // TODO should I care about the exact size of response_string, it returns 1657 readings. 
	    // TODO MMI doesn't tell the angle, although it should be 0.22 deg of resolution, slightly varies in 0.21-0.23 sometimes
	    // TODO maybe the above should be a stream, why to save in a string? Is there a way
	    // to parse and dump into laser_scan_msg on the fly?

    	cout << "Iteration: " << count << ", Bytes written: ";
    	cout << bytes_wrote << ", Bytes read: ";
    	cout << response_string.length() << ", String read: " << response_string << endl;
	    count += 1;

	    // TODO catch the error

		// parse the response from the lidar to make a laser scan message with angles and corresponding distances
	    vector<string> laser_distances_string = split(response_string, ',');

	    // Delete the first three elements from response vector as they are not distances. 
	    for (int i=0; i<3; i++)
	    {
	    	laser_distances_string.erase(laser_distances_string.begin());
	    }

	    // cout << "laser_distances_string.length()" << laser_distances_string.size() << endl;
	    vector<float> laser_distances_float(laser_distances_string.size());
	    std::transform(laser_distances_string.begin(), laser_distances_string.end(), laser_distances_float.begin(), 
							[](const std::string &arg) { return std::stof(arg); }); 
		laser_scan_msg.header.stamp = ros::Time::now();
		laser_scan_msg.header.frame_id = "map";
		laser_scan_msg.angle_min = 0.0;
		laser_scan_msg.angle_max = 2*M_PI;
		laser_scan_msg.angle_increment = 2*M_PI/1654; //TODO check this
		laser_scan_msg.time_increment = 1.0; // TODO time between measurements [seconds] - if your scanner is moving, this will be used in interpolating position of 3d points
		laser_scan_msg.scan_time = 1.0/36000; // TODO time between scans [seconds]
		laser_scan_msg.range_min = 0.0;
		laser_scan_msg.range_max = 100.0; //TODO check
		laser_scan_msg.ranges = laser_distances_float; //TODO allocate might cause segfault
		//laser_scan_msg.intensities; //intensity data [device-specific units].  If your device does not provide intensities, please leave the array empty.

		laser_publisher.publish(laser_scan_msg);

		loop_rate.sleep(); 
	}
	
	ros::spin();
	return 0; 
}
