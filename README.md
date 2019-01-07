# robin_gazebo
Simulation interface for using the Robin firmware with the Gazebo simulator.

## Installing Dependencies
Before any steps can be made towards setting up the simulator interface, some packages must first be installed:

```sh
sudo apt install ros-kinetic-simulators ros-kinetic-mavros-msgs ros-kinetic-hector-gazebo-plugins ros-kinetic-gazebo-ros-control
```

To run the full flight control example, also make sure you have built the following packages:
- [contrail](https://github.com/qutas/contrail)
- [mavel](https://github.com/qutas/mavel)
- [robin](https://github.com/qutas/robin)
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
roslaunch robin_gazebo robin_sim_quad_full.launch
```

The `robin_sim_quad_full.launch` is actually a combination of three other launch files that to do the heavy lifing. It is not necessary to run all of these files along with `robin_sim_quad_full.launch`, but they can be all run individually if desired. As a brief summary:
- `robin_sim_quad.launch`: Launches Gazebo, and spawns in the quadrotor model (also launches some helper nodes).
- `robin_sim_fcu.launch`: Launches `mavros` and `robin` to perform the SITL autopilot functions.
- `robin_sim_controller.launch`: Launches `mavel` to perform the position control.
- `robin_sim_gcs.launch`: Launches `rviz` and `rqt` to allow for user interaction.

The additional file, `robin_sim_hex_full.launch`, is also provided as a means of simulating a hexarotor X6.

## Loading the default parameters
Each time the `robin` package is compiled, you will need to configure/calibrate the autopilot. To make this less painful, a set of "flight-ready" parameters have been included with this package.

To set the "flight-ready" parameters, run the following commands:
```
roscd robin_gazebo/config/
rosrun mavros mavparam -n /robin/mavros load -qgc robin.params
```

Then, using the GUI, press `Write EEPROM`. At this point, the simulator will need to be restarted, as some parameters won't take effect until this is done.

If done correctly, you should see some messages in the command window that look like the following:
```
[ WARN] [1546845273.869100013, 0.011000000]: FCU: [MIXER] Using mixer QUAD X
[ WARN] [1546845273.879974512, 0.022000000]: FCU: [MIXER] Layout: [M,M,M,M,-,-,-,-]
[ERROR] [1546845273.891000988, 0.033000000]: FCU: [SENSOR] HARDWARE IN THE LOOP ENABLED!
[ WARN] [1546845273.902062289, 0.044000000]: FCU: [SENSOR] Using HIL, some sensors disabled
```

This indicates that HIL mode is active, and that the system is running in quadrotor mode.

## First Flights
Before the flight starts, it is important to know that `mavel` must be and started first and connected to the autopilot before the flight can commence. If you get "control change denied" or "arming denied", wait a few moments and try again. The startup process usually takes about 10 seconds.

To start the first flight, perform the following actions:
1. Using the GUI, set the flight mode to `OFFBOARD`.
2. Using the GUI, arm the UAV.
3. In a new terminal, load the takeoff flight plan: `roslaunch robin_gazebo plan_loader.launch move:=takeoff`

At this point, you should see the UAV takeoff and start flight. You will also notice that once the flight plan is completed (i.e. the quadrotor makes it to the goal), the plan loader will automatically exit. At this point, you can now safely load in a new flight plan (new flight plans will override currently running plans if you don't wait). Try loading some of the following plans that are provided as part of the `contrail` package:
```
roslaunch robin_gazebo plan_loader.launch move:=home_circle_start
roslaunch robin_gazebo plan_loader.launch move:=circle
roslaunch robin_gazebo plan_loader.launch move:=circle_end_home
roslaunch robin_gazebo plan_loader.launch move:=square
roslaunch robin_gazebo plan_loader.launch move:=home_north
roslaunch robin_gazebo plan_loader.launch move:=home
roslaunch robin_gazebo plan_loader.launch move:=land
```

At the end of the demo, once the UAV has landed, it is a good idea to disarm the UAV. This is simply to get into a good habit for the real world, as the UAV will technically keep trying to perform the position control even while it is on the ground. Once you disarm the UAV, you will need to restart `mavel` (or in this case, the entire simulator), as the system is no longer in an expected state.

## Multiple UAV
To perform run the simulation with multiple UAV active, run the following commands in separate windows:


#### Start the simulation environment
```sh
roscore
roslaunch robin_gazebo robin_sim_gazebo.launch
```

#### Spawning a multirotor
Spawning a new instance of a quadrotor can be done as listed below. Note that the parameters `model_id`, used to configure the ROS namespace, and `fcu_udp_set`, which is used to set the UDP ports used by the FCU. For the full system to work correctly, both parameters **must** be unique for each multirotor. The `fcu_udp_set` can simply be increased by 1 to asign a unique set of UDP ports.

```sh
roslaunch robin_gazebo robin_sim_quad_full.launch gazebo:=false rviz:=false rqt:=false fcu_sys:=1 model_id:=robin1 spawn_x:=0.0 spawn_y:=0.0
roslaunch robin_gazebo plan_loader.launch model_id:=robin1 move:=home
```
One thing to consider is that each parameters for a multirotor are tied to its `model_id`, meaning that you will have to set the initial parameters the fist time you spawn each multirotor. The other thing to consider is the spawn point for each multirotor, as you do not want to spawn each one on-top of eachother (adding 0.5 to an axis should be enough in most cases). Lastly, you probably only want a limited number of GUI's open, so enable/disable them as desired.

---
**Note:** As of ~Jan. 2019, mavros will only connect to the FCU if the `fcu_sys` parameter matches the FCU parameter MAV_SYS_ID. This means that initially, you will have to spawn in one UAV at a time, with the correct `model_id`, set the MAV_SYS_ID parameter, then restart that system with the correct `fcu_sys` parameter to match.
---

This setup will also allow for multiple UAVs to be displayed via a GCS like QGroundControl.
