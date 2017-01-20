PSES - Lane Detector

## Requirements:
* ROS (code tested with ROS Indigo)
* MRPT:
```sh
$ sudo add-apt-repository ppa:joseluisblancoc/mrpt
$ sudo apt-get update
$ sudo apt-get install libmrpt-dev
```

## Use:
* To use lane detector with the PSES-Car in a real scenario:
```sh
$ roslaunch lane_detector lane_follower.launch
```
* To use lane detector with an image dataset:
```sh
$ roslaunch lane_detector lane_follower_simulation.launch
```

##Subscribed Topics:
/image (sensor_msgs/Image)
/camera_info (sensor_msgs/CameraInfo)
/lane_detector/driving_orientation (std_msgs::Int32)

##Published Topics:
/lane_detector/lane (lane_detector::Lane)

## ROS-Message lane_detector::Lane:
header (Header): Header with time stamp
left_line (geometry_msgs/Point32[]): Sorted Array of points with the x,y coordinates (in meters) of the left line.  
right_line (geometry_msgs/Point32[]): Sorted Array of points with the x,y coordinates (in meters) of the right line.
guide_line (geometry_msgs/Point32[]): Sorted Array of points with the x,y coordinates (in meters) of the guide (middle) line.

## Additional Requirements:
* The used camera has to be calibrated and should send the calibration data over the camera_info topic.
* The camera extrinsics (height and pitch angle) have to be known and should be set with dynamic reconfigure with:
```sh
$ rosrun rqt_reconfigure rqt_reconfigure
```
* The used camera has to have both lane marks in sight while driving a curve.
