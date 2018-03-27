# Graph-based Stairway Detection using PointCloud Data -- GUI Version

![alt text](https://github.com/ThomasWestfechtel/StairwayDetection/blob/master/pics/resultExample.png "Staiway Detection Example")

To install the repository:

Go to main directory

```
mkdir build
cd build

cmake ..
make -j8
./pcl_viewer
```

There are 2 example pcd files in the example folder to test the algorihtm. When loading point cloud, the main window does not change its view automatically and has to be changed using keyboard/mouse to alighn the point cloud in the window.

Depending on the accuracy of the employed LIDAR some parameters need to be tuned accordingly. Most important are the parameters of the region growing algorithm in src/regiongrowing.h

This is the GUI version of the Stairway Detection. A stand alone version can be found at:
https://github.com/ThomasWestfechtel/StairwayDetection

The processing steps of the algorihm are explained in "3D graph based stairway detection and localization for mobile robots" (http://ieeexplore.ieee.org/document/7759096/).

![alt text](https://github.com/ThomasWestfechtel/StairwayDetection/blob/master/pics/stairGraph.png "Graph-based Detection")
