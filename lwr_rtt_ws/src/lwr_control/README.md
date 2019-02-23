lwr_control
============

# Launch in Simulation

```bash
# Start Gazebo + your controller
roslaunch lwr_control run.launch sim:=true
```

# Launch on Hardware

```bash
# Start RTnet connection
rosrun lwr_scrits rtnet start
# Start your controller
roslaunch lwr_control run.launch
```
