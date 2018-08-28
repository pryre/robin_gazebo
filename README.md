# robin_gazebo
Hardware in the Loop interface for using the Robin firmware with the Gazebo simulator

## Installing Dependencies
Before any steps can be made towards setting up the simulator interface, some packages must first be installed:

```sh
sudo apt install ros-kinetic-simulators ros-kinetic-mavros-msgs ros-kinetic-hector-gazebo-plugins ros-kinetic-gazebo-ros-control
```

To run the full flight control example, also make sure you have built the following packages:
- [contrail](https://github.com/qutas/contrail)
- [mavel](https://github.com/qutas/mavel)
- [rqt_mavros_gui](https://github.com/qutas/rqt_mavros_gui)
- [rqt_robin_gcs](https://github.com/qutas/rqt_robin_gcs)
- [rqt_generic_hud](https://github.com/qutas/rqt_generic_hud)

## Compiling
```sh
cd ~/catkin_ws/src
git clone https://github.com/qutas/robin_gazebo
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Updating
```sh
roscd robin_gazebo
git pull
cd ~/catkin_ws
catkin_make
```

## Running the simulator
To launch the primary simulation (of a quadcopter X4):
```sh
roslaunch robin_gazebo robin_sim_gazebo_quad.launch
```

The parameters `rviz` and `rqt` can also be passed to load an example GCS interface:
```
roslaunch robin_gazebo robin_sim_gazebo_quad.launch rviz:=true rqt:=true
```

The additional file, `robin_sim_gazebo_hex.launch`, is also provided as a means of simulating a hexarotor X6.

## Running the Example Controller
An example high-level controller is provided through the `contrail` and `mavros` packages:
```sh
roslaunch robin_gazebo robin_sim_controller.launch
```

## Performing a Simulated Flight
No low-level control/autopilot interfaces are provided as part of this package, however it was designed with the use case of the [Robin Hardware in the Loop Simulation](https://github.com/qutas/robin/blob/master/documents/SIMULATION.md#hardware-in-the-loop-hitl), so it is recommeneded that you check those documents for more information.

Assuming you have completed the rest of the setup for the HitL simulation, you should only need to arm the flight controller for it to perform the simulation. This can be done using the MAVROS CLI interfaces, or by using the `rqt_mavros_gui` arming button.


