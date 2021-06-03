# MARS_detect

## Overview

This package consist of files for generating and launching STag and ArUco marker related software.

## aruco_detect

This node finds ArUco markers in images stream and publishes their vertices
(corner points) and estimates 3D transforms from the camera to the fiducials.
It is based on the [`Aruco`](http://docs.opencv.org/trunk/d5/dae/tutorial_aruco_detection.html)
contributed module to OpenCV. It is an alternative to fiducial_detect

Documentation is in [the ROS wiki page](http://wiki.ros.org/aruco_detect).

#### Dictionary ID

You can generate ArUco markers from any dictionary you prefer. 
Make sure you set the corresponding `dictionary` parameter in launch file.
We performed most of the test with dictionary 7, which suited our needs.

## stag_detect

This node finds STag markers in images stream and estimates 3d transforms from the camera to the fiducials.
It is based on the [`STag`](https://github.com/usrl-uofsc/stag_ros), which documents also how well they perform according to ArUco.
We modified STag markers detection node for our needs and will not work with the codebase from original repository.

#### Library HD

You can generate STag markers from any library you prefer. 
Make sure you set the corresponding `libraryHD` parameter in launch file.
We performed most of the test with library HD15, which suited our needs.
