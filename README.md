# Translate point cloud data

## General information
This code is a small programm to shift point cloud data by given arguments,x,y,z (units in meters).
It is used in combination with DetectWindows programm https://github.com/svensMPG/DetectWindows in order to shift the bounding box of windows (in local coordinates) to the global coordinates again.

This code here can easily be integrated into DetectWindows, however, there was not time until now.

## Usage
**Input**: A PCD or PLY file containing local coordinates (in my case, output from DetectWindows).

**Output**: a new point cloud with the coordinates shifted by the given arguments for x,y,z. 

**run**: ./translatePC localCoordinateInputPC.pcd -x 358862.829062 -y 5701334.098114 -z 122.809974

## License
GNU GENERAL PUBLIC LICENSE Version 3
(c) 2018, Sven Schneider

