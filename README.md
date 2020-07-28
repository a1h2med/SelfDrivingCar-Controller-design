# Self Driving Car Control Design

## Self Driving Cars Longitudinal and Lateral Control Design 

In this project, I've implemented a controller in Python and use it to drive a car autonomously in race track in carla simulator.

### Project notes:

In this projects there was waypoints collected through out the race track, gathered in a file, and this file is passed as an input.  
this file should has the reference speed and angles.

Since the controller reference contains both position and speed, we need to implement both __Longitudinal and Lateral Control__ to make sure that we are tracking the (race track correctly).


The output of the controller will be the vehicle __throttle__, __brake__ and __steering angle__ commands.

please note that (throttle and brake) comes from Longitudinal speed control and (steering) comes from Lateral Control.

If anyone wants to add his/her controller, you should edit __controller2d.py__ which exists inside python client folder.
this file has some functions, but you are interested in __update_controls__ function, in which you can edit and implement your controller design.

###  The Race Tack including waypoints:


### Some concepts and formulas:

### Solution Figures:

### Final Notes:

This project is the final project from (self driving car specialization) on coursera.
In the __Longitudinal_control__ it's not required to add Low level controller, as it's linear process so you can neglect its effect.