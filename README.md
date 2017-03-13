# Object-3d reconstruction
## Final project for the exam "Transactional systems and data mining" prepared in 3/2014

## Description
Using Kinect device and PCL (Point Cloud Library) for the reconstruction of 3d objects

### Kinect 
Microsoft Kinect is a Microsoft Device that can be used together with an XBox360 console. It allows the player to have control and interact with the system without the need to wear or hold any controller. 

![alt text](https://github.com/stefano9o/object_3d_reconstruction/blob/master/markdown/images/kinect.png "Kinect")

This device is capable of obtaining a 3-dimensional map of the environment in which it is located by using a depth sensor. 

![alt text](https://github.com/stefano9o/object_3d_reconstruction/blob/master/markdown/images/3dMap.png "3d Map")

### PCL
The Point Cloud Library (PCL) is a standalone, large scale, open project for 2D/3D image and point cloud processing. PCL provides a lot of structures and algorithms to execute operations 
of filtering, estimating, segmentation, surface reconstruction, and so on.

### The work
A set of pictures (plus depth information) are taken from a range of different perspectives 
To create a 3-dimensional model of an object.

Outline
- Image acquisition
- Object isolation
- Image blending
- Noise removal
- (Surface reconstruction)

### Image acquisition
PCL provides a lot of classes and functions for capturing point clouds from a variety of sensing devices. 
In particular, the OpenNI Grabber Framework allows one to acquire data streams from OpenNi compatible cameras.
We have acquired eight pictures from different perspectives (one of these every 45 degrees)

![alt text](https://github.com/stefano9o/object_3d_reconstruction/blob/master/markdown/images/acquisition.png "Acquisition")

### Object isolation 
The position and the dimension of the object to be acquired have been assumed as known.
Each one of these images has been cleaned from points that don't belong to the object. To
do that we have used three parameters: rangeZ, rangeX1 and rangeX2.

![alt text](https://github.com/stefano9o/object_3d_reconstruction/blob/master/markdown/images/isolation1.png "Remove extra points")

As can be seen from the previous image, it is also necessary to remove the floor from the image to isolate the object completely. To do that, we have used RANSAC (RANdom SAmple Consensus) that is an iterative
method to estimate parameters of a mathematical model (a plane in our case) from a set of observed data.

![alt text](https://github.com/stefano9o/object_3d_reconstruction/blob/master/markdown/images/isolation2.png "Remove plane")

### Image blending 
The basic idea of the implemented algorithm is as follows:
- rotate the i-th image by 45 degrees around the centre of gravity;
- use the "Iterative Closest Point Non-Linear" algorithm in order to align the i-th with the (i+1)-th image.

The "Iterative Closest Point Non-Linear" at the beginning looks for the correspondence between two points from the two images. Then the algorithm carries out rotations of the i+1-th image, in order to minimise the distance error of every correspondence. 

![alt text](https://github.com/stefano9o/object_3d_reconstruction/blob/master/markdown/images/fusion.jpg "Fusion")

### Noise removal
The image blending produces the so-called salt-and-pepper noise. This kind of noise is characterised by the presence of some unrelated points whose value is highly different from the close points.  
We have used the "outlier filter" in order to remove this noise.

![alt text](https://github.com/stefano9o/object_3d_reconstruction/blob/master/markdown/images/noiseRemoval.png "Noise removal")

### Surface reconstruction
PCL provides several API for doing this process. After some trial, it has been chosen to use the ConcaveHull algorithm.

![alt text](https://github.com/stefano9o/object_3d_reconstruction/blob/master/markdown/images/surfaceReconstruction.png "Surface reconstruction")

### Future works
- Use more Kinect devices.
- Improve the "surface reconstruction" process.
- Use smoothing algorithm.
- Save in a format suitable for the 3d printer.
- Using new features offered by Kinect 3.0

### Appendix
the folder "scansioni" include several depth-image acquired from the Kinect.
