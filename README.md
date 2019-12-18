# xbox360_controller_interface

## Usage
  Control our quadruped robot
 `roslaunch xbox360_controller_interface joy_control.launch`
  **Hint:** source file is 'quadruped_joy_control.cpp'
###1
After launch the corresponding launch file, select the proper action in Free Gait Actions, and click send button (the action goal is send to server and planner is calculating the curves in specific time.) Click the switch button to run the base_balance_controller, now the robot in the model is running.
###2
you can operate in the xbox_360_handle. the cross up key is the standleg_flag to control the robot state.With LB and RB, you can control the robot pose by moving the left joy_stick and right joy_stick(left-right). At any time, you can press button A the store this state as the initial state.
