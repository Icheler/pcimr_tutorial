# Tutorial 03 - Localization

The robot works in package pcimr_localization as the localization_node.py. Working with the simulation, the robot is capable of reaching all goals with random start positions and is able to cope with larger uncertanties in the move_model. 

The code can be run by using the command: 
```bash
    roslaunch pcimr_localization localization.launch
```

## Did anything have to change after adding random start locations and using a different motion model?
For me it didn't. I had a minor bug in the move model which checked the wrong locations, after fixing that my algorithm was capable of localization after being 'kidnapped' since I added a low ground probability for all cells to allow for that scenario. If the robot works properly it does not interfer, for the kidnapped problem the robot is able to find a solution this way.