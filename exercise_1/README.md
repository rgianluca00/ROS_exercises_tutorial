#Exercise_1

Visualize turtle pose on rviz while controlling it through keyboard using turtlesim node.

##Usage

Launch the exercise with the following command
```
roslaunch exercise_1 exercise_1.launch
```
##NOTES

* Click on the terminal where the previous command was launched in order to move the turtle both on turtlesim simulation window and on rviz

* The rviz config file with alle the initial ambient configuration is stored in the folder rviz. According to the TF transformation created in the node turtle_broadcaster, "world" is the fixed frame, while "turtle 1" is its child frame and it moves accordingly with the turtle on the turtlesim simulation window.
