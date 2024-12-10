// C++ standard headers
#include <exception>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>

// FOR COMUNICATION
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <shape_msgs/SolidPrimitive.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib/client/simple_action_client.h>

// Specific for TIAGO
#include <pal_navigation_msgs/SaveMap.h>
#include <play_motion_msgs/PlayMotionAction.h>

// Transforms
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>

// FOR PLC
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>

// Include PointCloud2 message
#include <sensor_msgs/PointCloud2.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/collision_detection/collision_robot.h>

// Grasp
#include <moveit_grasps/grasp_generator.h>
#include <moveit_grasps/grasp_filter.h>
#include <moveit_grasps/grasp_data.h>
#include <moveit_grasps/grasp_planner.h>
#include <moveit_grasps/grasp_candidate.h>

// Parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// For gazebo
#include <gazebo_ros_link_attacher/Attach.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>

// Voronoi
#include <autonomous_tidy_up/thinning.h>

//Semantic report
using std::cout;
using std::cin;
using std::endl;
#include <sstream>
using std::string;
#include <iostream>
#include <chrono>

namespace autonomous_tidy_up {

enum CellType{kFree, kObstacle, kExpandedObstacle, kUnknown, kCostmapObstacle, kNOTTable, kUnvisited, kPath, kUnvalidGoal, kInvalidBasePose};

enum ObjectType{ObjectTable, ObjectOnFloor, NoObject};
enum ObjectState{StateHolding, StateOnFloor, StateInvalid, StatePlaced, StateInvalidGrasp, StateInvalidPlan, StateNotGripped, StatePlaceFail, StatePlaceInvalid,
                 StateTableTooSmall, StateNotAcessible};

struct ObjectDetected{

  moveit_msgs::CollisionObject object;
  bool see_many_times = false;
  ObjectState state = StateOnFloor;
  int manipulation_tries = 0;
};

struct Cell{
  int LAST_TIME_ANALYSED = 0;
  int8_t ORIGINAL_MAP;
  CellType CELL_TYPE = kUnvisited;
  ObjectType OBJECT_TYPE = NoObject;
  std::string NAME;
  bool IS_ACCESSIBLE = false;
  bool VISITED = false;
  bool GRID_VISITED = false;
  bool VORONOI_CELL = false;
  bool VORONOI_VISITED = false;
  bool ROBOT_VISITED = false;
  double POTENTIAL = 1.0;
  bool BOUNDARY_CONDITION = false;
};  

enum ExecutionStage{StageWaitingStart, StageStart, StageExploration, StageManipulation, StageEnd};

}