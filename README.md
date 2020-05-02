Ball Plate Control System
=============
> This is my undergraduate graduation design. It is a ball plate control system. 
>
> The video about this project is located [here](https://youtu.be/u6pzwO9xneA). 

_____

![](https://github.com/505788057/Ball-Tracking/blob/master/fig/struction.png)

Description
----------
In the first part, I mainly detect the presence of a silver ball using computer vision techniques.  Through GaussianBlur , I get the frame which has little noisy and then I transfer the frame from BGR color style into HSV color style. After erode ,dilate and mask, I get the two value frame which consists of black and white. Finally I get the (x,y)  and center of ball though Canny algorithm.

The second part is controlling. I use PD controller to control the system. After calculating, I get a suitable signal to motor driver though serial communication. Make the motor turn and produce a acceleration for silver ball. The velocity of silver ball is processed by filter to make the velocity more smoother. 

If you want to learn it on your PC. You can download it directly or use `git clone` in git IDE. 

```
$ git clone https://github.com/505788057/Ball-Tracking.git
```
Prerequisites
-------------
- Python            3.7.4
- imutils           0.5.3
- opencv-python     4.1.2.30
- numpy             1.16.5
- matplotlib    3.1.1

Interface
---------
![](https://github.com/505788057/Ball-Tracking/blob/master/fig/interface.png)
### Function introduction
- HSV Parameter debugging
  
  For targeting different colored balls we need to set the different threshold.
  
- PID Parameter debugging & Graphic drawing

- Ball Tracking & real time showing

- Ball control interface

- System control interface



Reference
---------
1. [Pyimagesearch](https://www.pyimagesearch.com/)
2. [OpenCV-Python图像处理教程](https://github.com/ex2tron/OpenCV-Python-Tutorial)
3. [Ball Balancing PID System](https://www.instructables.com/id/Ball-Balancing-PID-System/)
