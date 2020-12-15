# Path Planning
## Short explanation
After launching the launch-file. A map is visible in rviz. By choosing a "2D Nav Goal" the algorithm gets a goal assigned. If that goal is is in a free cell the robot is able to navigate to the goal by using A* to find the best possible path. The node terminates after finding a path, but a new node starts right away if the robot should go to another goal. This can be changed by editing the launch file to disable respawn for the navigation_node.

## Start Guide
For this to work all the needed nodes have to be able to execute. Afterwards the program can simply be launched by typing in 

```bash
roslaunch pcimr_navigation navigation.launch
```
