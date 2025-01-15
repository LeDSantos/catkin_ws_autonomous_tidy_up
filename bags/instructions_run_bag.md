# Recording a ros bag

You will need **one extra terminal** to record. Execute the commands at the singularity_files folder:

``` sh
$ singularity shell --nv ros-melodic-pal_public_docker_custom.sif
$ cd ..
$ source devel/setup.bash
$ source ./run_local_master.sh ## for simulation
```

You can use [run_tiago_master.sh](../run_tiago_master.sh) in case that the real robot is connected to the computer, but edit the file to your robot and your computer IP.

Before starting the simulation or real experiment, execute this:

``` sh
$ cd bags
$ rosbag record rosout tf mask_rcnn_73b2_kitchen/output/viz potencial_visualise semantic_map_visualise move_base/current_goal move_group/monitored_planning_scene start_demo continue_execution robot_pose coverage_acessible_area execution_stage place_poses
```

The result will be a *.bag file.

# Running a ros bag

To run the real robot experiment bag, download the 2023-07-20-16-03-07.bag from [here](https://drive.google.com/file/d/1htd3FVzeeuflL-QukyV0_wzlawu_1NDG/view?usp=sharing) and unzip it on the bags folder.

You will need **four terminals** to run a bag. Prepare each terminal with the same comands on the singularity_files folder as cited above.

**First terminal**, iniciate the ros system:

``` sh
$ roscore
```

**Second terminal**, load the robot model and open Rviz:

``` sh
$ roslaunch autonomous_tidy_up load_robot_model_for_bag.launch
$ rviz rviz
```

You can select autonomous_tidy_up.rviz that is on configs_rviz folder to visualize the robot and all the important topics at Rviz.

**Third terminal**, play the bag:

``` sh
$ cd bags
$ rosbag play 2023-07-20-16-03-07.bag
```