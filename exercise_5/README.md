# Exercise_5

Spawn 3 turtles: one is the robot, the other two are the fixed obstacles. Move the robot to a goal configuration by not colliding with the two obstacles. Simulate various scenarios for the two obstacles possible configurations.

## Usage

The following tests were made with fixed parameters for attractive and repulsive gain, radius of influence, distance and yaw thresholds, proportional control gains.

### Test_1

This is a standard configuration to test attractive and repulsive forces. Here the robot manages to reach its goal without collisions or entering Local Minima. 

Launch the exercise with the following command.
```
roslaunch exercise_5 exercise_5_test1.launch
```

### Test_2

Here the robot manages to reach its goal without collisions. The distance between the two turtle-obstacles is 2.5 on the x-axis, still enough for preventing the robot from enter in a Local Minima situation.

Launch the exercise with the following command
```
roslaunch exercise_5 exercise_5_test2.launch
```

### Test_2 (+ Local Minima)

Here the robot does not reach its goal without collisions. The distance between the two turtle-obstacles is now 2.4 on the x-axis, not enough for preventing the robot from enter in a Local Minima situation for the chosen goal configuration. The robot is stuck in a Local Minima case beacuse total force equals zero, this depends on the two obstacles positions related to the chosen goal configuration, and therefore on the total force field that is being generated. 

Launch the exercise with the following command
```
roslaunch exercise_5 exercise_5_test2_LM.launch
```

### Test_3

Here again the robot reaches its goal, slaloming without collisions. 

Launch the exercise with the following command
```
roslaunch exercise_5 exercise_5_test3.launch
```

### Test_4

Here again the robot reaches its goal, passing through an opening between two obstacles, without collisions. 

Launch the exercise with the following command
```
roslaunch exercise_5 exercise_5_test4.launch
```

## NOTES

* To change obstacle position, go to exercise_5_test<number>.launch and change:

- "{x: <desired_x_coord>, y: <desired_y_coord>, theta: <desired_theta_coord>, name: obstacle1}", for obstacle1
- "{x: <desired_x_coord>, y: <desired_y_coord>, theta: <desired_theta_coord>, name: obstacle2}", for obstacle2

* To change goal configuration, go to exercise_5_test<number>.launch and change:

<param name = "goal_x_coord" value = "<desired_value>"/> 
<param name = "goal_y_coord" value = "<desired_value>"/> 
