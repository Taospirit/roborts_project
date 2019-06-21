# Module Information
The script dir contains all the code related to decision
## Battle.py
This script contains all the information related to the environment, such as robot poses and enemy poses.  It servers as the input to our decision node.
## controller.py
This script contains all the action related to robots, for example, such as navigation and shooting. It servers as the controller to all commands and the output of our decision node.
Do not need to run manually.
## pose_update.py
This script updates the robot pose relative to the map.
## shoot_enemy.py
This script determines the shoot mode according to enemy distances.
Need to add shoot freq in continuous mode
## enemy_detect.py
This script publishes the fake enemy pose for simulation.
## decision_node.py
This script implements the detailed the decision by using behavior trees.
