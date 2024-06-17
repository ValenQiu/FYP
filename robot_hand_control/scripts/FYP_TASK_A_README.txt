To run the hand gesture control robotic hand:

1. open a terminal, run:

```
roscore
```

2. open a new terminal, run:

```
sudo leapd
```

output:
[Info] WebSocket server started
[Info] Secure WebSocket server started
[Info] Leap Motion Controller detected: S323A017625
[Info] Firmware is up to date.

3. open a new terminal, activate the serial port:

```
sudo chmod a+rw /dev/ttyUSB0
```

need to run it every time reconnect to the hand

4. run command:

```
rosrun leap_motion hand_control.py 
```



In this code, we have the folloing important files:

1. HandController_FYP.py
the class of hand controller, which including open the serial port and send message to the hand

2. hand_control.py
the code for hand gesture recognition (right hand only), and data smoothing (averaging and Gaussian filter)
then, calculate the finger vectors and then calculate the finger angle between the finger vector and the 
palm normal

TO DO:
make the code 2 into a class, intergrate all functions
add functions for left hand, which will be used to control the motion modes (ON / OFF)

