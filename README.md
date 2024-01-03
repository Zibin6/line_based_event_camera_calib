# line_based_event_camera_calib


## Introduction

This repository contains the key code of our line_based event camera calibration algorithms.

Details of the algorithm can be found in our paper:

"LECalib: Line-based Event Camera Calibration"

Zibin Liu, Banglei Guan, Yang Shang, Zhenbao Yu, Yifei Bian and Qifeng Yu.

2024


## Main Function


Simulation test of random data can be found in 

```
demo.m               : algorithm testing
noise_test_both.m    : accuracy w.r.t varying image noise
linenumber_test_both : accuracy w.r.t varying number of lines
f_test_both.m        : accuracy w.r.t varying focal length
w_test_both.m        : accuracy w.r.t varying measurement error of angular and linear velocities

```

Real-world test of DAVIS346 calibration can be found in 

```
test_box.m      : DAVIS346 calibration using a box

```

## Contact

Please contact the authors for requests.

This work was developed at National University of Defense Technology, 
Hunan Provincial Key Laboratory of Image Measurement and Vision Navigation.
