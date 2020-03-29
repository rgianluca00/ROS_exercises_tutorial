# Exercise_2

Write a node that simulates a high-level controller that applies a saturation on a custom control command message. The turtle behaviour under saturation effects is verified by examining rqt_plot.

## Usage

Launch the exercise with the following command
```
roslaunch exercise_2 exercise_2.launch min_linear:=-3 max_linear:=3 min_angular:=-3 max_angular:=3
```

## NOTES

* To change from terminal max e min limits to be applied as saturations for linear and angular velocity, change values in the previous launch command (min_linear:=<desired_value> max_linear:=<desired_value> min_angular:=<desired_value> max_angular:=<desired_value>)

* To set different values for custom control velocities, go to exercise_2.launch. There, change values inside {linear_velocity: <desired_value>, angular_yaw: <desired_value>}


