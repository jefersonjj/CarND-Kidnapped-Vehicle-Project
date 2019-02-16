# **Kidnapped Vehicle Project** 

## Writeup

This is my writeup for the project "Kidnapped Vehicle" of Self Driving Car Nanadegree on Udacity.

---
[//]: # (Image References)

[image1]: ./output/KidnappedVehicle.gif "Kidnapped Vehicle (Animation)"

---

### Contents

* [About Kidnapped Vehicle Project](#About-Kidnapped-Vehicle-Project)
* [Project code](#Project-code)
* [Output video](#Output-video)
* [Some notes](#Some-notes)

---
## About Kidnapped Vehicle Project

The goals / steps of this project are the following:

* Implement a 2 dimensional particle filter in C++ given a map, some initial localization information and observation/control data at each time step.
* More detail explanation can be found in [README](https://github.com/pl80tech/CarND-Kidnapped-Vehicle-Project/blob/master/README.md)

---
## Project code

Here is my working repository for this project:

https://github.com/pl80tech/CarND-Kidnapped-Vehicle-Project

It is imported and frequently updated (cherry-pick or merge) from below original repository:

https://github.com/udacity/CarND-Kidnapped-Vehicle-Project

---
## Output video

Here are the links to the output videos which were uploaded on Github and Youtube (for easy reference). Click on the thumbnail image to jump to the video on Youtube.

| Link on Youtube              | Link on Github               |
|:----------------------------:|:----------------------------:|
|[![ProjectVideo](https://i.ytimg.com/vi/2Qa63TRbtpw/hqdefault.jpg)](http://www.youtube.com/watch?v=2Qa63TRbtpw)|[/output/KidnappedVehicle.mp4](https://github.com/pl80tech/CarND-Kidnapped-Vehicle-Project/blob/master/output/KidnappedVehicle.mp4)|

---
## Some notes

### Debug option

 Add definition DEBUG in [helper_functions.h](https://github.com/pl80tech/CarND-Kidnapped-Vehicle-Project/blob/master/src/helper_functions.h) as below before compiling the sources to show debug messages with detail information.
 ```C++
 // Flag to enable debug messages with detail information
 // Comment out to disable
 #define DEBUG
 ```
 Another way is to add below description to [CMakeLists.txt](https://github.com/pl80tech/CarND-Kidnapped-Vehicle-Project/blob/master/CMakeLists.txt) to compile the sources with definition DEBUG.
 ```txt
 add_definitions(-DDEBUG)
 ```

### Test code

Folder [/test_code/](https://github.com/pl80tech/CarND-Kidnapped-Vehicle-Project/tree/master/test_code) includes the code (Jupyter notebook, C++) for the quiz on lesson 2 ~ 6. Refer to the description on [runtest.sh](https://github.com/pl80tech/CarND-Kidnapped-Vehicle-Project/blob/master/test_code/runtest.sh) about how to compile and run the C++ code.