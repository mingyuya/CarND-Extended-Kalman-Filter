# Writeup for P1 of Self-Driving Car Nanodegree Term2: Extended Kalman Filter
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

### Matt, Min-gyu, Kim
---

[image1]: ./kalman_filter_process.png "KF_PROCESS"
[image2]: ./build/result/screenshot/screenshot_tracking.png "SCREENSHOT"

### Brief Description

I built the [Extended Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) which takes inputs from sensor fusion (LiDAR and RADAR). Here is the processing flow of the filter.

![alt text][image1]

In addition to the typical kalman filter, I have to linearize the inputs from RADAR sensor. It was implemented by Jacobian matrix which can be found in `tools.cpp`

---

### Simulation

The compiled program can communicate with the simulator by [uWebSocket](https://github.com/uNetworking/uWebSockets).
Here is the captured image during the simulation.

![alt text][image2]

The filter successfully tracked the object - green triangles - as you can see in the image.
