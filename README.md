### What is this repository for? ###
ROS driver for [Lightware SF-40/C](http://www.lightware.co.za/shop/en/products/45-sf40c-100-m.html). (Official doc of the scanner is in the link in the "Download" tab)

### Set up ###
First please ensure you're in MMI mode via Minicom. `Esc` switches b/w HMI and MMI modes

```
catkin_make
rosrun lightware_sf40_ros sf40_node <device-name> <baud-rate> "argument"
rosrun rviz rviz
```
<device-name> is the name of the device e.g. /dev/ttyUSB0 and <baud-rate> is the baud rate e.g. 115200
Now, "argument" is any MMI command mentioned in the Lightware doc(linked above). If you want a map spanning 360 degrees, with 0 degrees as the forward direction, the most common use case, then "argument" is ?TM,360,0

`rosrun lightware_sf_40_ros sf40_node ?TM,360,0`


In rviz, change frame name to "map", and listen to the `laser_scan` topic. The laser scan should show up.

The time stamp of message is fake, it just uses a `ros::Time::now()`. ~~Syncing with the encoder is a TODO at the time of writing~~ clock sync/slaving the laser to a cam isn't possible

### How to change spinning rate, other stuff mentioned in the Lightware doc and debug, etc ###
Yeah, so it's not a complete driver in that sense. I just use [minicom](http://linux.die.net/man/1/minicom) to change these things. `Esc` in minicom will switch b/w MMI and HMI modes. 
An alternative is to write a publisher for the specific MMI/HMI command you want. Find this line in `sf40_node.cpp` 
`size_t bytes_wrote = my_serial.write(test_string + "\r\n");`
Note here `\r\n` is basically the CRLF suffix for MMI mode, for HMI commands just change to 
`size_t bytes_wrote = my_serial.write(ENTER_YOUR_HMI_STRING_HERE);`


### Info ###
This wrapper depends on [serial library](https://github.com/wjwwood/serial) by William Woodall
