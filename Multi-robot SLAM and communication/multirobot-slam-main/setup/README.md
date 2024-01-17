# Problems setup

See `test_multiRobotSLAMSolver.m` for usage of `multiRobotSLAM` and `multiRobotSLAMSolver` classes.

+ `multiRobotSLAMSolver` defines available algorithms.
+ `multiRobotSLAM` defines the problem setup, including true map, number of robots, etc.
+ `driveWheelRobot` defines the drive wheel robots.
  + Each robot has one range finder `rangeFinderSensor` and one controller `driveWheelRobotController`.
  + Each robot keeps track of an estimated map and an estimated state.
  + Each robot keeps an copy of true state.