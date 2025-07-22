#include <autonomous_tidy_up/autonomous_tidy_up.hpp>

#define MAX_FINGER_JOINT_VALUE 0.044

#define WITH_STOPS false

#define DEBUG false

#define MIN_TRYS_NUMBER 2
#define MAX_TRYS_NUMBER 3
#define MAX_NUM_VALID_GRASPS 30

#define MIN_OBJECT_HEIGHT 0.02

#define SIMULATION true

#define BASE_STATIC false

#define SIZE_TO_MARK 0.1

#define MAX_VALID_DIST_FOR_POINT_CLOUD_TO_ROBOT_VISITED 1.5

#define PADDINGarm_5_link 0.01

#if SIMULATION
  #define EXTRA_HEIGHT 0.01
  #define MIN_EXTRA_HEIGHT 0.01
  #define SIMULATIONvariation_z 0.02
#else
  #define EXTRA_HEIGHT 0.04 // difference in gripper position on real robot
  #define MIN_EXTRA_HEIGHT 0.01
  #define SIMULATIONvariation_z 0.05
#endif

#define OBJECT_EXPANSION  0.02
#define TABLE_EXPANSION   0.10// to prevent fingers from catching the edge of the table along with the object

#define MAX_TRYS 3

// ITU mode - Immediate Tidy-Up
#define EXPLORE_THEN_TIDY_UP false

// ETU mode - Explore then Tidy-Up
// #define EXPLORE_THEN_TIDY_UP true

#define ONLY_CRITICAL_PAUSES true

// Select strategy
#define STRATEGY_BVP_CLASSIC false//true
#define STRATEGY_VORONOI true//false
#define STRATEGY_GRID false

// BVP with Voronoi otimization
#define STRATEGY_BVP_PLUS_VORONOI false//true

// To save the created map
#define GMAPPING_EXPLORATION true

#if STRATEGY_GRID
  #define BASE_EXPANSION -0.1
#endif

#if STRATEGY_VORONOI
  #define BASE_EXPANSION 0.1
#endif

#if STRATEGY_BVP_CLASSIC
  #define BASE_EXPANSION 0.0
#endif

#if STRATEGY_GRID
#elif STRATEGY_VORONOI
#elif STRATEGY_BVP_CLASSIC
#else
#define BASE_EXPANSION 0.0 // without any strategy
#endif

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction> PlayMotionClient;
typedef moveit::planning_interface::MoveGroupInterface MoveitInterface;
typedef moveit::planning_interface::PlanningSceneInterface PlanningScene;

using std::string;
#include <iostream>

std::string return_current_time_and_date(){
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%X");
    return ss.str();
}

namespace autonomous_tidy_up
{
static const std::string LOGNAME = "autonomous_tidy_up";

namespace
{
  bool isStateValid(const planning_scene::PlanningScene* planning_scene,
                  robot_state::RobotState* robot_state,
                  const robot_model::JointModelGroup* group, const double* ik_solution)
{
  // ROS_WARN("CHECK STATUS");
  robot_state->setJointGroupPositions(group, ik_solution);
  robot_state->update();
  bool result = !planning_scene->isStateColliding(*robot_state, group->getName());
  // ROS_WARN("CHECK STATUS %s of %s", (result? "OK" : "colide"), group->getName().c_str());
  return result;
}

void waitForNextStep(moveit_visual_tools::MoveItVisualToolsPtr visual_tools, std::string prompt)
{
  visual_tools->prompt(prompt);
}
}

class AutonomousTidyUp
{
private:
  // A shared node handle
  ros::NodeHandle nh_;
  std::string ee_group_name_, dir_arq;
  std::string planning_group_name_;
  MoveBaseClient *ac;
  PlayMotionClient *client_arm;
  ros::ServiceClient grasp_client;
  ros::ServiceClient attach_client;
  ros::ServiceClient detach_client;
  ros::ServiceClient save_map_client;

  ros::ServiceClient make_plan_client;
  nav_msgs::GetPlan make_plan_srv;

  std_srvs::Empty empty_srv;
  bool successful_grasp;
  PlanningScene planning_scene_interface;
  const std::string ROBOT_DESCRIPTION{"robot_description"};
  MoveitInterface *group_arm_torso;
  MoveitInterface *group_gripper;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;

  // Tool for visualizing things in Rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  // MoveIt! Grasps
  moveit_grasps::GraspGeneratorPtr grasp_generator_;
  // Robot-specific data for generating grasps
  moveit_grasps::GraspDataPtr grasp_data_;
  // For planning approach and retreats
  moveit_grasps::GraspPlannerPtr grasp_planner_;
  // For selecting good grasps
  moveit_grasps::GraspFilterPtr grasp_filter_;
  // All the motion planning components
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;
  // Robot
  robot_model::RobotModelPtr robot_model_;
  // Arm
  const robot_model::JointModelGroup* arm_jmg_;

  geometry_msgs::Pose pose_inicial_obj;

  ros::Publisher block_planning_scene_pub, start_detection_pub, start_detection_only_object_pub, start_detection_only_table_pub, inspect_surroundings_pub, start_table_detection_pub, start_object_detection_pub, look_to_detect_objects_pub, only_pub_octomap_pub;
  std_msgs::Bool block_planning_scene_msg, start_detection_msg, start_detection_only_object_msg, start_detection_only_table_msg, inspect_surroundings_msg, start_table_detection_msg, start_object_detection_msg, look_to_detect_objects_msg, only_pub_octomap_msg;
  ros::Publisher semantic_map_visualise_pub, potencial_visualise_pub;
  nav_msgs::OccupancyGrid semantic_map_visualise_msg, potencial_visualise_msg;

  ros::Publisher place_poses_pub;
  geometry_msgs::PoseArray place_poses_msg;

  ros::Publisher execution_stage_pub;
  std_msgs::Int32 execution_stage_msg;

  ros::Publisher coverage_acessible_area_pub, n_acessible_cells_pub, n_visited_cells_pub;
  std_msgs::Float32 coverage_acessible_area_msg;
  std_msgs::Int32 n_acessible_cells_msg, n_visited_cells_msg;

  ros::Subscriber table_on_map_sub, object_on_map_sub, cloud_sub, move_base_status_sub;
  ros::Subscriber object_on_octomap_sub, stop_movement_sub;
  bool object_on_octomap = false, stop_movement = false;
  bool block_all_movement = false;

  bool valid_status_last_goal = true;

  // Basic move arm-torso to Joint Value Target.
  bool MoveGroupArmTorso();
  // Basic move gripper to Joint Value Target.
  bool MoveGroupGripper();
  bool real_object_lost;
  bool holding_object;

  ros::Timer timerStillHoldingObject, timerContinuousHeadMovement;
  ros::AsyncSpinner *spinner;

  std::string object_name_to_gazebo_ros_link_attacher = "unit_box_custom";

  geometry_msgs::Pose robot_pose, goal_point_, goal_point_not_valid;
  nav_msgs::OccupancyGrid map_recevied, local_costmap, global_costmap;

  std::vector<Cell> semantic_map;

  float map_resolution_, map_origin_x_, map_origin_y_, map_origin_z_, map_width_, map_height_;
  float global_costmap_resolution_, global_costmap_origin_x_, global_costmap_origin_y_, global_costmap_origin_z_, global_costmap_width_, global_costmap_height_;  
  float localcostmap_resolution_, localcostmap_origin_x_, localcostmap_origin_y_, localcostmap_origin_z_, localcostmap_width_, localcostmap_height_;
  int map_size = 0;
  int global_costmap_size = 0;

  // Tools
  int MapIndicesToVectorIndex(int i, int j);
  int LocalcostmapIndicesToVectorIndex(int i, int j);
  int GlobalcostmapIndicesToVectorIndex(int i, int j);
  float constrainAngle(float x);
  inline double getYaw(const geometry_msgs::Quaternion& q);
  inline double ComputeYaw(const geometry_msgs::Quaternion& q_gm);
  std::tuple<int, int> VectorIndexToMatrixIndices(int index);

  tf::StampedTransform transform;
  tf::TransformListener listener;

  std::string frame_input_cloud;
  pcl::PointCloud<pcl::PointXYZ> input_cloud;

  tf::StampedTransform transform_frame_input_cloud_to_map;
  tf::TransformListener listener_frame_input_cloud_to_map;

  bool last_goal_not_valid = false;
  bool first_interation = true;
  int  global_counter = 1;
  bool block_mark_as_visited = false;

  int smallest_i, smallest_j, largest_i, largest_j;

  ros::Time start_time, end_time, start_exploration_time, start_manipulation_time;
  float walking_distance = 0.0;

public:

  static const int NO_REAL_OBJECT = 123;
  static const int NO_PLACE_POSITION = 124;

  #if SIMULATION
  float base_radius = 0.3;
  #else
  float base_radius = 0.4;
  #endif

  std::vector<ObjectDetected> objects_on_the_floor_recevied, objects_wrong_size;
  std::vector<ObjectDetected> tables_recevied;
  bool no_acessible_objects;

  // Transforms
  std::tuple<int, int> transformCoordinateOdomToMap(float x, float y);
  std::tuple<int, int> transformCoordinateOdomToGlobalcostmap(float x, float y);
  std::tuple<float, float> transformCoordinateMapToOdom(int i, int j);
  std::tuple<int, int> transformCoordinateLocalcostmapToMap(int i, int j);
  std::tuple<float, float> transformCoordinateLocalcostmapToOdom(int i, int j);
  bool TRYtransformCoordinateLocalcostmapToOdom();

  ros::Subscriber gripper_state_sub, object_name_to_gazebo_ros_link_attacher_sub;
  control_msgs::JointTrajectoryControllerState gripper_state;
  ros::Subscriber planning_scene_ready_sub, continue_execution_sub;
  bool planning_scene_ready, continue_execution;
  ros::Subscriber odom_sub, robot_pose_sub, local_costmap_sub, map_sub, global_costmap_sub, global_costmap_update_sub;
  
  // Callbacks
  void gripper_state_callback(const control_msgs::JointTrajectoryControllerState &msg);
  void object_name_to_gazebo_ros_link_attacher_callback(const std_msgs::String &msg);
  void planning_scene_ready_callback(const std_msgs::Bool& msg);
  void continue_execution_callback(const std_msgs::Bool& msg);
  void odom_callback(const nav_msgs::Odometry& msg);
  void robot_pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg);
  void move_base_status_callback(const actionlib_msgs::GoalStatusArray& msg);
  void local_costmap_callback(const nav_msgs::OccupancyGrid& msg);
  void global_costmap_callback(const nav_msgs::OccupancyGrid& msg);
  void global_costmap_update_callback(const map_msgs::OccupancyGridUpdate& msg);
  void table_on_map_callback(const moveit_msgs::CollisionObject& msg);
  void object_on_map_callback(const moveit_msgs::CollisionObject& msg);
  void object_on_octomap_callback(const std_msgs::Bool& msg);
  void map_callback(const nav_msgs::OccupancyGrid& msg);
  void stop_movement_callback(const std_msgs::Bool& msg);
  void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input); // to mask visited area
  

  // Comunication: start, stop and wait. Block and unblock.
  void StartDetectionOnlyObject();
  void StartObjectDetection();
  void StopObjectDetection();

  void StartDetectionOnlyTable();
  void StartTableDetection();
  void StopTableDetection();

  void StartInspectSurroundings();
  void StartOnlyPubOctomap();

  void WaitToPlanningSceneReady();
  void WaitToContinueExecution();
  void WaitToContinueExecutionCritical();

  void BlockPlanningScene();
  void UnblockPlanningScene();

  // Time events
  void StillHoldingObject(const ros::TimerEvent& event);
  void ContinuousHeadMovement(const ros::TimerEvent& event);

  // Gazebo related
  bool AttachGazebo(bool right = true);
  bool DetachGazebo(bool right = true);

  // Setups
  
  AutonomousTidyUp(/* args */);
  bool loadSceneForGrasps();
  void setupGraspPipeline();
  bool SetupServers();
  bool SetupPlanningScene();
  bool SetupMoveitInterface();
  void set_local_planner_max_vel(double new_lin_vel, double new_ang_vel);
  void set_local_planner_xy_goal_tolerance(double new_xy_goal_tolerance);
  ~AutonomousTidyUp();

  //Goals and motions
  void SendGoalBase(float x, float y, float z, float roll, float pitch, float yaw);
  bool SendAndSaveGoalBase(geometry_msgs::Pose goal_pose);
  bool SendAndSaveGoalBase(float x, float y, float z, float roll, float pitch, float yaw);
  void SendAndSaveGoalBaseWithoutWait(geometry_msgs::Pose goal_pose);
  void SendAndSaveGoalBaseWithoutWait(float x, float y, float z, float roll, float pitch, float yaw);
  void SaveGoalNotValid(float x, float y, float z, float roll, float pitch, float yaw);
  bool SendMotionArm(string motion);
  void SendMotionArmWithoutWait(string motion);
  bool SendBIGhead_look_aroundSLOW2_steps();
  int step_head_motion = 0;
  void openGripper(trajectory_msgs::JointTrajectory& posture);
  // void closedGripper(trajectory_msgs::JointTrajectory& posture);
  bool LiftTorsoJoint(float joint_value);
  bool LiftEndEffector(float lift_value);
  bool MovementOpenEndEffector();
  bool MovementHome();
  bool MovementArmUpToWalk();
  bool MoveGripperToGoal(geometry_msgs::Pose new_goal);

  // Manipulation
  bool getIKSolution(const Eigen::Isometry3d& target_pose, robot_state::RobotState& solution, const std::string& link_name);
  bool planPreApproach(const robot_state::RobotState& goal_state, moveit_msgs::MotionPlanResponse& pre_approach_plan);
  bool planFullGrasp(std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates, std::vector<moveit_grasps::GraspCandidatePtr>& valid_grasp_candidate, std::vector<moveit_msgs::MotionPlanResponse>& pre_approach_plan);
  bool PickWithAutoGrasp(std::string object_name, int n_object = 0);
  void setACMFingerEntry(const std::string& object_name, const std::string& surface_name, bool allowed);
  void setACMFingerEntry(const std::string& object_name, bool allowed);
  bool PickObject(std::string object_name, int n_object = 0);
  moveit::core::MoveItErrorCode RealPick(std::string object_name_grasp, std::vector<moveit_grasps::GraspCandidatePtr> valid_grasp_candidate,
              std::vector<moveit_msgs::MotionPlanResponse> pre_approach_plan, int available_grasps, int n_object = 0);
  moveit::core::MoveItErrorCode placeOnTheTableTop(std::string object_name, std::string table_name, moveit_msgs::CollisionObject object_to_place, int n_object = 0);

  // Semantics
  bool FindObject(std::string object_name);
  moveit_msgs::CollisionObject GetObject(std::string object_name);
  geometry_msgs::Pose GetObjectPose(std::string object_name);
  bool startSemanticMap();
  void UpdateSemanticMap();
  void AddObjectsAndTablesReceviedToSemanticMap();
  void RemoveObjectsAndTablesReceviedToSemanticMap();
  void MarkAsPlacedObject(moveit_msgs::CollisionObject obj, int n_object);
  void ChangeObjectState(int n_object, ObjectState state);
  void ChangeTableState(int n_table, ObjectState state);
  bool AddInfoToSemanticMap(moveit_msgs::CollisionObject object, ObjectType type, float size_tolerance, float size_min, bool mark_as_unvised = false);
  std::vector<int> GetSimilarObjects(moveit_msgs::CollisionObject new_object, ObjectType type, float tolerance);
  void AddNewDetectedObject(ObjectDetected new_obj, ObjectType type, float tolerance);
  inline bool CheckObjectOnFloor(moveit_msgs::CollisionObject object);
  float CoverageAcessibleArea();
  bool NoTablesNear(int i, int j);
  void PubSemanticMapVisualise();
  inline bool ValidObjectOnFloor(int n_object);
  inline bool ValidTable(int n_table);
  void PubJustCoverage();

  ExecutionStage stage;
  inline void ChangeStage(ExecutionStage new_stage);

  // Base navigation
  bool PathExist(geometry_msgs::Pose pose);
  bool PathExist(float x, float y, float z, float roll, float pitch, float yaw);
  int LengthPath(geometry_msgs::Pose pose);
  std::tuple<geometry_msgs::Pose, moveit_msgs::CollisionObject> GoalNearestObjectOnFloor();
  std::tuple<geometry_msgs::Pose, moveit_msgs::CollisionObject, int> GoalNearestTable();
  geometry_msgs::Pose FindFreePoseNearObject(moveit_msgs::CollisionObject object);
  geometry_msgs::Pose FindFreePoseNearTable(moveit_msgs::CollisionObject object_table, float dist_from_table);
  inline bool InsideMap(float x, float y);
  inline bool InsideGlobalcostmap(float x, float y);

  //////////GRID FUNCTIONS
  bool FreeArea(int n_center, int m_center, float dist);
  float PositionOnGrid(float pos, float dist);
  bool SimpleTrajectory(float min_dist);
  void PubVisitedGrid();
  int preference_wall=1;
  bool finded_wall=false;
  bool visited_all_grid_goals = false;

  //////////VORONOI FUNCTIONS
  void ComputeVoronoiDiagram();
  geometry_msgs::Pose FindFreePoseVoronoi();
  void MarkUnderBaseAsVisitedVoronoi();
  bool visited_all_voronoi_goals = false;
  bool HasUnvisitedVoronoiNeighbors(int i_center, int j_center);
  void PubVoronoi();

  // BVP FUNCTIONS
  void InitializePotentialField();
  void MarkUnderBaseAsVisited();
  void UpdatePotentialField();
  void ComputeNextGoalInBVP();
  int flat_potential_field = 0;
  void MarkAsExpandedObstacle(float x, float y);
  void MarkAsInvalidBasePose(float x, float y);
  void MarkAsObstacle(float x, float y);
  void MarkAsHightPot(float x, float y);
  bool ViableGoal(float x, float y);
  void PubPotentialMap();

  // Control
  void ExploreToFindObjects(); // EXPLORATION
  void CollectAllObjectsOnTheFloor(); // MANIPULATION
  bool DetectObjectAgainAndPick();
  bool DetectTableAgainAndPlace(moveit_msgs::CollisionObject table_near, moveit_msgs::CollisionObject object_to_place, int n_object = 0, int n_table = 0);
  bool ObjectAndTableAvailable();
  int RemainingObjectsToPick();

  // Prints and report file
  void printSceneObjects();
  void PrintStateAllObjects();
  void PrintStateAllObjectsOnFile();
  void PrintStateAllTables();
  void PrintStateAllTablesOnFile();
  void PrintInfoAllTables();
  std::ofstream semantic_report_file;
  bool OpenReportFile();
  bool CloseReportFile();
  void SaveGmappinMap();

  // Real robot on the lab
  void AddFixedLabTable();
};

///////////////////////////////////////
//                                   //
//                Tools              //
//                                   //
///////////////////////////////////////

int AutonomousTidyUp::MapIndicesToVectorIndex(int i, int j) { return i + j * map_width_; }
int AutonomousTidyUp::LocalcostmapIndicesToVectorIndex(int i, int j) { return i + j * localcostmap_width_; }
int AutonomousTidyUp::GlobalcostmapIndicesToVectorIndex(int i, int j) { return i + j * global_costmap_width_; }

std::tuple<int, int> AutonomousTidyUp::VectorIndexToMatrixIndices(int index) {
  int temp_j = index / map_width_;
  int temp_i = index - temp_j * map_width_;  
  return std::make_tuple(temp_i, temp_j);
}

inline double AutonomousTidyUp::getYaw(const geometry_msgs::Quaternion& q)
{
  // Source: http://docs.ros.org/en/jade/api/tf2/html/impl_2utils_8h_source.html
  double yaw;

  double sqw;
  double sqx;
  double sqy;
  double sqz;

  sqx = q.x * q.x;
  sqy = q.y * q.y;
  sqz = q.z * q.z;
  sqw = q.w * q.w;

  // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
  double sarg = -2 * (q.x*q.z - q.w*q.y) / (sqx + sqy + sqz + sqw); /* normalization added from urdfom_headers */

  if (sarg <= -0.99999) {
    yaw   = -2 * atan2(q.y, q.x);
  } else if (sarg >= 0.99999) {
    yaw   = 2 * atan2(q.y, q.x);
  } else {
    yaw   = atan2(2 * (q.x*q.y + q.w*q.z), sqw + sqx - sqy - sqz);
  }
  return yaw;
};

inline double AutonomousTidyUp::ComputeYaw(const geometry_msgs::Quaternion& q_gm) {
  return getYaw(q_gm);
}

float AutonomousTidyUp::constrainAngle(float x){
  // Source: https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code
  float new_x = fmod(x + M_PI, 2.0 * M_PI);
  if (new_x < 0.0)
    new_x += 2.0 * M_PI;
  return new_x - M_PI; /// [-PI, +PI)
}

///////////////////////////////////////
//                                   //
//        Basic move group to        //
//         Joint Value Target        //
///////////////////////////////////////

bool AutonomousTidyUp::MoveGroupArmTorso(){

  group_arm_torso->setStartStateToCurrentState();
  group_arm_torso->setMaxVelocityScalingFactor(1.0);

  MoveitInterface::Plan my_plan;
  bool success = bool(group_arm_torso->plan(my_plan));

  if (!success){
    // throw std::runtime_error("No plan found");
    ROS_ERROR("No plan found for group_arm_torso");
    return false;
  }

  #if DEBUG
  ROS_WARN_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");
  // Execute the plan
  ros::Time start = ros::Time::now();
  #endif

  moveit::planning_interface::MoveItErrorCode e = group_arm_torso->move();
  if (!bool(e)){
    // throw std::runtime_error("Error executing plan");
    ROS_ERROR("Error executing plan for group_arm_torso %d", e.val);
    WaitToContinueExecution();
    return false;
  }

  #if DEBUG
  ROS_WARN_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
  #endif

  return true;
}

bool AutonomousTidyUp::MoveGroupGripper()
{
  #if DEBUG
  ROS_WARN("Available Planning Groups:");

  const std::vector<std::string> &names = group_gripper->getJointNames();

  int num = names.size();
  for (int i = 0; i < num; i++)
    ROS_WARN(">>>>> %s", names[i].c_str());

  ROS_WARN("%d ..... %s", num, group_gripper->getName().c_str());

  const std::vector<std::string> &names2 = group_gripper->getJointModelGroupNames();

  int num2 = names2.size();
  for (int i2 = 0; i2 < num2; i2++)
    ROS_WARN(">.....>>> %s", names2[i2].c_str());

  #endif

  group_gripper->setStartStateToCurrentState();
  group_gripper->setMaxVelocityScalingFactor(1.0);

  MoveitInterface::Plan my_plan;
  bool success = bool(group_gripper->plan(my_plan));

  if (!success){
    // throw std::runtime_error("No plan found");
    ROS_ERROR("No plan found for group_gripper");
    return false;
  }

  #if DEBUG
  ROS_WARN_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");
  // Execute the plan
  ros::Time start = ros::Time::now();
  #endif

  moveit::planning_interface::MoveItErrorCode e = group_gripper->move();
  if (!bool(e)){
    ROS_ERROR("Error executing plan for group_gripper");
    return false;
  }

  #if DEBUG
  ROS_WARN_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
  #endif
  return true;
}

///////////////////////////////////////
//                                   //
//             Transforms            //
//                                   //
///////////////////////////////////////

// It transforms the coordinate system from the Odom to the Map
std::tuple<int, int> AutonomousTidyUp::transformCoordinateOdomToMap(float x, float y) {
  int j = y / map_resolution_ - map_origin_y_ / map_resolution_;
  int i = x / map_resolution_ - map_origin_x_ / map_resolution_;
  return std::make_tuple(i, j);
}

// It transforms the coordinate system from the Odom to the Globalcostmap
std::tuple<int, int> AutonomousTidyUp::transformCoordinateOdomToGlobalcostmap(float x, float y) {
  int j = y / global_costmap_resolution_ - global_costmap_origin_y_ / global_costmap_resolution_;
  int i = x / global_costmap_resolution_ - global_costmap_origin_x_ / global_costmap_resolution_;
  return std::make_tuple(i, j);
}

// It transforms the coordinate system from the Localcostmap to the Map
std::tuple<int, int> AutonomousTidyUp::transformCoordinateLocalcostmapToMap(int i, int j) {
  
  float x_odom = (i + localcostmap_origin_x_ / localcostmap_resolution_) * localcostmap_resolution_;
  float y_odom = (j + localcostmap_origin_y_ / localcostmap_resolution_) * localcostmap_resolution_;

  geometry_msgs::PointStamped point_in, point_out;
  point_in.header = local_costmap.header;
  // point_in.header.stamp = ros::Time::now();
  point_in.point.x = x_odom;
  point_in.point.y = y_odom;
  point_in.point.z = 0.0;

  try{
    listener.transformPoint("map", point_in, point_out);
  } catch (tf::TransformException ex){
    ROS_ERROR("transformCoordinateLocalcostmapToMap %s",ex.what());
  }
  // float x = (i + localcostmap_origin_x_ / localcostmap_resolution_) * localcostmap_resolution_;
  // float y = (j + localcostmap_origin_y_ / localcostmap_resolution_) * localcostmap_resolution_;
  return transformCoordinateOdomToMap(point_out.point.x, point_out.point.y);
}

// It transforms the coordinate system from the Map to the Odom
std::tuple<float, float> AutonomousTidyUp::transformCoordinateMapToOdom(int i, int j) {
  float x = (i + map_origin_x_ / map_resolution_) * map_resolution_;
  float y = (j + map_origin_y_ / map_resolution_) * map_resolution_;
  return std::make_tuple(x, y);
}

// It transforms the coordinate system from the Localcostmap to the Odom
std::tuple<float, float> AutonomousTidyUp::transformCoordinateLocalcostmapToOdom(int i, int j) {
  
  float x_odom = (i + localcostmap_origin_x_ / localcostmap_resolution_) * localcostmap_resolution_;
  float y_odom = (j + localcostmap_origin_y_ / localcostmap_resolution_) * localcostmap_resolution_;

  geometry_msgs::PointStamped point_in, point_out;
  point_in.header = local_costmap.header;
  // point_in.header.stamp = ros::Time::now();
  point_in.point.x = x_odom;
  point_in.point.y = y_odom;
  point_in.point.z = 0.0;

  try{
    listener.transformPoint("map", point_in, point_out);
  } catch (tf::TransformException ex){
    ROS_ERROR("transformCoordinateLocalcostmapToOdom %s",ex.what());
  }

  // float x = (i + localcostmap_origin_x_ / localcostmap_resolution_) * localcostmap_resolution_;
  // float y = (j + localcostmap_origin_y_ / localcostmap_resolution_) * localcostmap_resolution_;
  return std::make_tuple(point_out.point.x, point_out.point.y);
}

// It trys to transform Localcostmap to Odom
bool AutonomousTidyUp::TRYtransformCoordinateLocalcostmapToOdom() {
  
  float x_odom = localcostmap_origin_x_;
  float y_odom = localcostmap_origin_y_;

  geometry_msgs::PointStamped point_in, point_out;
  point_in.header = local_costmap.header;
  // point_in.header.stamp = ros::Time::now();
  point_in.point.x = x_odom;
  point_in.point.y = y_odom;
  point_in.point.z = 0.0;

  try{
    listener.transformPoint("map", point_in, point_out);
  } catch (tf::TransformException ex){
    ROS_ERROR("TRYtransformCoordinateLocalcostmapToOdom %s",ex.what());
    return false;
  }

  // float x = (i + localcostmap_origin_x_ / localcostmap_resolution_) * localcostmap_resolution_;
  // float y = (j + localcostmap_origin_y_ / localcostmap_resolution_) * localcostmap_resolution_;
  return true;
}


///////////////////////////////////////
//                                   //
//             Callbacks             //
//                                   //
///////////////////////////////////////

void AutonomousTidyUp::gripper_state_callback(const control_msgs::JointTrajectoryControllerState &msg){
  gripper_state = msg;
}

// EXAMPLE for one of the cups at the garage: rostopic pub /object_name_to_gazebo_ros_link_attacher std_msgs/String "data: 'cup_glass_test_6'"
void AutonomousTidyUp::object_name_to_gazebo_ros_link_attacher_callback(const std_msgs::String &msg){
  object_name_to_gazebo_ros_link_attacher = msg.data;
}

void AutonomousTidyUp::planning_scene_ready_callback(const std_msgs::Bool& msg){
  planning_scene_ready = msg.data;
}

void AutonomousTidyUp::continue_execution_callback(const std_msgs::Bool& msg){
  continue_execution = msg.data;
}

void AutonomousTidyUp::odom_callback(const nav_msgs::Odometry& msg){
  robot_pose = msg.pose.pose;
}

void AutonomousTidyUp::robot_pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg){
  geometry_msgs::Pose last_robot_pose = robot_pose;
  robot_pose = msg.pose.pose;
  walking_distance += sqrt(pow(robot_pose.position.x - last_robot_pose.position.x, 2) + pow(robot_pose.position.y - last_robot_pose.position.y, 2));
}

void AutonomousTidyUp::move_base_status_callback(const actionlib_msgs::GoalStatusArray& msg){
  if(msg.status_list.size() == 0) return;
  valid_status_last_goal =     (msg.status_list.back().status == actionlib_msgs::GoalStatus::ACTIVE)
                            or (msg.status_list.back().status == actionlib_msgs::GoalStatus::PENDING)
                            or (msg.status_list.back().status == actionlib_msgs::GoalStatus::SUCCEEDED);
}

// Uses the local_costmap to update the semantic map.
void AutonomousTidyUp::local_costmap_callback(const nav_msgs::OccupancyGrid& msg)
{
  ROS_WARN("local_costmap_callback");
  local_costmap = msg;
  localcostmap_width_ = local_costmap.info.width;
  localcostmap_height_ = local_costmap.info.height;
  localcostmap_origin_x_ = local_costmap.info.origin.position.x;
  localcostmap_origin_y_ = local_costmap.info.origin.position.y;
  localcostmap_origin_z_ = local_costmap.info.origin.position.z;
  localcostmap_resolution_ = local_costmap.info.resolution;

  // Transforms to map reference.
  try{
    listener.waitForTransform("map", "odom", local_costmap.header.stamp, ros::Duration(2.0));
    listener.lookupTransform("map", "odom", local_costmap.header.stamp, transform);
  } catch (tf::TransformException ex){
    ROS_ERROR("local_costmap %s",ex.what());
    return;
  }

  if ( int(localcostmap_width_*localcostmap_height_) <= 0)
  {
    ROS_WARN("local_costmap size 0");
    return;
  }  

  ROS_WARN("startSemanticMap local_costmap_callback %s", (startSemanticMap()? "OK" : "fail"));

  if (map_size != 0 && semantic_map.size() > 0)
  {
    UpdateSemanticMap();
  } else
  {
    ROS_WARN("No semantic map yet");
  }  
}

void AutonomousTidyUp::global_costmap_callback(const nav_msgs::OccupancyGrid& msg)
{
  global_costmap = msg;
  global_costmap_width_ = global_costmap.info.width;
  global_costmap_height_ = global_costmap.info.height;
  global_costmap_origin_x_ = global_costmap.info.origin.position.x;
  global_costmap_origin_y_ = global_costmap.info.origin.position.y;
  global_costmap_origin_z_ = global_costmap.info.origin.position.z;
  global_costmap_resolution_ = global_costmap.info.resolution;

  global_costmap_size = global_costmap_height_ * global_costmap_width_;
  ROS_WARN("global_costmap_callback...map_width_: %f   map_height_: %f", global_costmap_width_, global_costmap_height_);

}

// Updates part of the global_costmap.
void AutonomousTidyUp::global_costmap_update_callback(const map_msgs::OccupancyGridUpdate& msg)
{
  if (global_costmap_size == 0)
  {
    ROS_WARN("global_costmap_update_callback WITHOUT global_costmap...");
    return;
  }
  
  for (auto x = msg.x; x < msg.x + msg.width; x++) {
    for (auto y = msg.y; y < msg.y + msg.height; y++) {
      global_costmap.data[x + y * global_costmap.info.width] = msg.data[(x - msg.x) + (y - msg.y) * msg.width];
    }
  }
  ROS_WARN("global_costmap_update_callback...");
}

// Adds the new table to the list tables_recevied.
void AutonomousTidyUp::table_on_map_callback(const moveit_msgs::CollisionObject& msg)
{
  // ROS_WARN("table_on_map_callback");
  ObjectDetected table_recevied;
  table_recevied.object = msg;
  // table_recevied.see_many_times = false;

  if(!InsideMap(table_recevied.object.primitive_poses[0].position.x, table_recevied.object.primitive_poses[0].position.y))
    return;

  table_recevied.see_many_times = true;

  if (tables_recevied.size() <= 0) // First table, just adds to the list.
  {
    tables_recevied.push_back(table_recevied);
    ROS_WARN("FIRST fist time table");
    return;
  }

  AddNewDetectedObject(table_recevied, ObjectTable, 0.05); // Necessary to avoid duplicity or overlapping.
  return;    
}

// Adds the new object to the list object_recevied.
void AutonomousTidyUp::object_on_map_callback(const moveit_msgs::CollisionObject& msg)
{
  ROS_WARN("object_on_map_callback");
  ObjectDetected object_recevied, final_object;
  object_recevied.object = msg;
  object_recevied.see_many_times = false;

  while (object_recevied.object.id.size() < 15)
  {
    object_recevied.object.id.append("."); // To normalize the id size.
  }
  
  if(!InsideMap(object_recevied.object.primitive_poses[0].position.x, object_recevied.object.primitive_poses[0].position.y))
    return;

  if (!CheckObjectOnFloor(object_recevied.object))
  {
    ROS_WARN("Object too high or too low");
    return;
  }
  
  if (object_recevied.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] < 0.02
      or object_recevied.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] < 0.02
      or object_recevied.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] < 0.02)
  {
    ROS_WARN("Object too small");
    objects_wrong_size.push_back(object_recevied); // Separate list for report
    return;
  }

  #if SIMULATION
  if (object_recevied.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] > 0.10
      or object_recevied.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] > 0.10)
  #else
  if (object_recevied.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] > 0.15
      or object_recevied.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] > 0.15)
  #endif
  {
    ROS_WARN("Object too big");
    objects_wrong_size.push_back(object_recevied); // Separate list for report
    return;
  }

  AddNewDetectedObject(object_recevied, ObjectOnFloor, 0.15); // Adjusts the object's pose, avoids duplicity or overlapping.
}

void AutonomousTidyUp::object_on_octomap_callback(const std_msgs::Bool& msg){
  ROS_WARN("object_on_octomap_callback");
  object_on_octomap = msg.data;
}

// Receives the map and updates the semantic map.
void AutonomousTidyUp::map_callback(const nav_msgs::OccupancyGrid& msg)
{
  map_recevied = msg;

  map_width_ = map_recevied.info.width;
  map_height_ = map_recevied.info.height;
  map_origin_x_ = map_recevied.info.origin.position.x;
  map_origin_y_ = map_recevied.info.origin.position.y;
  map_origin_z_ = map_recevied.info.origin.position.z;
  map_resolution_ = map_recevied.info.resolution;

  map_size = map_recevied.info.height * map_recevied.info.width;
  ROS_WARN("map_callback...map_width_: %f   map_height_: %f", map_width_, map_height_);

  ROS_WARN("startSemanticMap map_callback %s", (startSemanticMap()? "OK" : "fail"));
  
  ROS_WARN("ORIGINAL_MAP updated");  
}

// Stops the robot's movement. Used only in real experiments.
void AutonomousTidyUp::stop_movement_callback(const std_msgs::Bool& msg) {

  stop_movement = msg.data;

  if (stage != StageExploration or stop_movement == false or block_all_movement == true)
  {
    return;
  }  

  actionlib::SimpleClientGoalState state = client_arm->getState();
  ROS_WARN_STREAM("Arm client state: " << state.toString());

  // Freezes the robot for 5 seconds.
  block_all_movement = true;
  client_arm->cancelAllGoals();
  SendAndSaveGoalBase(robot_pose);
  ros::Duration(5.0).sleep();
  block_all_movement = false;
}

// Marks the visited cells in the semantic map using the depth points near the robot.
void AutonomousTidyUp::cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
{
  if(block_mark_as_visited){
    return;
  }

  if (map_size <= 0 or semantic_map.size() <= 0)
    return;

  frame_input_cloud = input->header.frame_id;

  pcl::fromROSMsg(*input, input_cloud);

  // Transforms to map reference.
  try{
    listener_frame_input_cloud_to_map.waitForTransform("map", frame_input_cloud, input->header.stamp, ros::Duration(2.0));
    listener_frame_input_cloud_to_map.lookupTransform("map", frame_input_cloud, input->header.stamp, transform_frame_input_cloud_to_map);
  } catch (tf::TransformException ex){
    ROS_ERROR("cloudCB %s",ex.what());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> input_cloud_on_map;
  pcl_ros::transformPointCloud(input_cloud, input_cloud_on_map, transform_frame_input_cloud_to_map);
  float max_valid_dist = MAX_VALID_DIST_FOR_POINT_CLOUD_TO_ROBOT_VISITED;
  float pot2_max_valid_dist = pow(max_valid_dist, 2.0);

  // Analyses each point.
  for (const auto& point : input_cloud_on_map)
  {
    if (!InsideMap(point.x, point.y))
    {
      continue;
    }

    if ((pow(point.x - robot_pose.position.x, 2.0) + pow(point.y - robot_pose.position.y, 2.0)) > pot2_max_valid_dist)
    {
      continue; // Outside max distance.
    }

    int i, j;
    std::tie(i, j) = transformCoordinateOdomToMap(point.x, point.y);
    semantic_map[MapIndicesToVectorIndex(i,j)].ROBOT_VISITED = true;
  }
}


///////////////////////////////////////
//                                   //
//           Comunication            //
//  start/stop; wait. Block/unblock  //
///////////////////////////////////////

void AutonomousTidyUp::StartDetectionOnlyObject(){
  start_detection_only_object_msg.data = true;
  start_detection_only_object_pub.publish(start_detection_only_object_msg);
  ros::spinOnce();
  ROS_WARN("StartDetectionOnlyObject pub");
}

void AutonomousTidyUp::StartObjectDetection(){
  look_to_detect_objects_msg.data = true;
  look_to_detect_objects_pub.publish(look_to_detect_objects_msg);

  start_object_detection_msg.data = true;
  start_object_detection_pub.publish(start_object_detection_msg);
  ros::spinOnce();
  ROS_WARN("StartObjectDetection pub");
}

void AutonomousTidyUp::StopObjectDetection(){
  look_to_detect_objects_msg.data = false;
  look_to_detect_objects_pub.publish(look_to_detect_objects_msg);

  start_object_detection_msg.data = false;
  start_object_detection_pub.publish(start_object_detection_msg);
  ros::spinOnce();
  ROS_WARN("StopObjectDetection pub");
}

void AutonomousTidyUp::StartDetectionOnlyTable(){
  start_detection_only_table_msg.data = true;
  start_detection_only_table_pub.publish(start_detection_only_table_msg);
  ros::spinOnce();
  ROS_WARN("StartDetectionOnlyTable pub");
}

void AutonomousTidyUp::StartTableDetection(){
  start_table_detection_msg.data = true;
  start_table_detection_pub.publish(start_table_detection_msg);
  ros::spinOnce();
  ROS_WARN("StartTableDetection pub");
}

void AutonomousTidyUp::StopTableDetection(){
  start_table_detection_msg.data = false;
  start_table_detection_pub.publish(start_table_detection_msg);
  ros::spinOnce();
  ROS_WARN("StopTableDetection pub");
}

void AutonomousTidyUp::StartInspectSurroundings(){
  inspect_surroundings_msg.data = true;
  inspect_surroundings_pub.publish(inspect_surroundings_msg);
  ros::spinOnce();
  ROS_WARN("StartInspectSurroundings pub");
}

void AutonomousTidyUp::StartOnlyPubOctomap(){
  only_pub_octomap_msg.data = true;
  only_pub_octomap_pub.publish(only_pub_octomap_msg);
  ros::spinOnce();
  ROS_WARN("Start only_pub_octomap pub");
}

void AutonomousTidyUp::WaitToPlanningSceneReady(){
  planning_scene_ready = false;
  while (!planning_scene_ready && ros::ok())
  {
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    ROS_WARN("Wait planning_scene_ready...");
  }
}

void AutonomousTidyUp::WaitToContinueExecution(){
  #if ONLY_CRITICAL_PAUSES
  return;
  #else
  continue_execution = false;
  while (!continue_execution && ros::ok())
  {
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    ROS_WARN("Wait continue_execution...");
  }
  #endif
}

void AutonomousTidyUp::WaitToContinueExecutionCritical(){

  continue_execution = false;
  #if not SIMULATION
  while (!continue_execution && ros::ok())
  #endif
  {
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    ROS_WARN("Wait continue_execution CRITICAL...");
  }
}

void AutonomousTidyUp::BlockPlanningScene(){
  block_planning_scene_msg.data = true;
  block_planning_scene_pub.publish(block_planning_scene_msg);
  ros::spinOnce();
}

void AutonomousTidyUp::UnblockPlanningScene(){
  block_planning_scene_msg.data = false;
  block_planning_scene_pub.publish(block_planning_scene_msg);
  ros::spinOnce();
}

///////////////////////////////////////
//                                   //
//            Time events            //
//                                   //
///////////////////////////////////////

// Closes the gripper periodically to check if the robot is still holding an object and to maintain the grip.
void AutonomousTidyUp::StillHoldingObject(const ros::TimerEvent& event)
{
  if (!holding_object)
  {
    return;
  }
  
  bool result = grasp_client.call(empty_srv); // Sends the command to close the gripper.
  if (!result) {
    ROS_ERROR("Failed to call service grasp_client");
  }else
  {
    // Waits for the fingers to stop moving.
    do {
      ros::Duration(1.0).sleep();
      ros::spinOnce();
    }while (ros::ok() && (gripper_state.desired.velocities[0] != 0.0 or gripper_state.desired.velocities[1] != 0.0));
        
    float dist_fingers = gripper_state.desired.positions[0] + gripper_state.desired.positions[1];
    #if SIMULATION
    if (dist_fingers < 0.01)
    #else
    if (dist_fingers <= 0.003)
    #endif
    {
      ROS_ERROR("LOST SOMETHING .......................");
      holding_object = false;
    }else
    {
      ROS_WARN("HOLDING SOMETHING.......................");
    }
  }
}

// Sends the head motion periodically.
void AutonomousTidyUp::ContinuousHeadMovement(const ros::TimerEvent& event){

  if (!block_all_movement)
  {
    #if SIMULATION
    SendMotionArmWithoutWait("BIGhead_look_aroundSLOW2");
    #else
    SendBIGhead_look_aroundSLOW2_steps();
    #endif
  }
  
  return;
}

///////////////////////////////////////
//                                   //
//          Gazebo related           //
//                                   //
///////////////////////////////////////

// Attachs the object_name_to_gazebo_ros_link_attacher model to a finger.
bool AutonomousTidyUp::AttachGazebo(bool right)
{
  gazebo_ros_link_attacher::Attach attach_msg;
  attach_msg.request.model_name_1 = "tiago";
  if (right)
  {
    attach_msg.request.link_name_1 = "gripper_right_finger_link";
  } else
  {
    attach_msg.request.link_name_1 = "gripper_left_finger_link";
  }
  
  // Name received by the /object_name_to_gazebo_ros_link_attacher topic.
  attach_msg.request.model_name_2 = object_name_to_gazebo_ros_link_attacher;
  // Default name of the first link for the cups model used in simulation.
  attach_msg.request.link_name_2 = "link_0";

  bool result_gazebo = attach_client.call(attach_msg);
  if (!result_gazebo) {
    ROS_ERROR("Failed to call service attach_client");
  }else
  {
    ROS_ERROR("CALLED service attach_client");
  }
  return result_gazebo;
}

// Detachs the object_name_to_gazebo_ros_link_attacher model to a finger.
bool AutonomousTidyUp::DetachGazebo(bool right)
{
  gazebo_ros_link_attacher::Attach detach_msg;
  detach_msg.request.model_name_1 = "tiago";
  if (right)
  {
    detach_msg.request.link_name_1 = "gripper_right_finger_link";
  } else
  {
    detach_msg.request.link_name_1 = "gripper_left_finger_link";
  }

  // Name received by the /object_name_to_gazebo_ros_link_attacher topic.
  detach_msg.request.model_name_2 = object_name_to_gazebo_ros_link_attacher;
  // Default name of the first link for the cups model used in simulation.
  detach_msg.request.link_name_2 = "link_0";

  bool result_gazebo = detach_client.call(detach_msg);
  if (!result_gazebo) {
    ROS_ERROR("Failed to call service detach_client");
  }else
  {
    ROS_ERROR("CALLED service detach_client");
  }
  return result_gazebo;
}


///////////////////////////////////////
//                                   //
//              Setups               //
//                                   //
///////////////////////////////////////

/** @brief Opens the semantic report file, sets servers, planning scene, MoveIt interface and MoveIt Grasps pipeline.
 *  Also, sets publishers and subscribers. */
AutonomousTidyUp::AutonomousTidyUp(/* args */) : nh_("~")
{
  execution_stage_pub = nh_.advertise<std_msgs::Int32>("/execution_stage", 5);
  ChangeStage(StageWaitingStart);

  // Get arm info from param server
  const std::string parent_name = LOGNAME;  // for namespacing logging messages
  rosparam_shortcuts::get(parent_name, nh_, "planning_group_name", planning_group_name_);
  rosparam_shortcuts::get(parent_name, nh_, "ee_group_name", ee_group_name_);
  rosparam_shortcuts::get(parent_name, nh_, "dir_arq", dir_arq);
  ROS_WARN("Semantic report file %s", (OpenReportFile()? "OK" : "fail"));

  ROS_INFO_STREAM_NAMED("test", "End Effector: " << ee_group_name_);
  ROS_INFO_STREAM_NAMED("test", "Planning Group: " << planning_group_name_);

  spinner = new ros::AsyncSpinner(4);
  spinner->start();

  ROS_WARN("Servers %s", (SetupServers()? "OK" : "fail"));
  ROS_WARN("PlanningScene %s", (SetupPlanningScene()? "OK" : "fail"));

  ROS_WARN("MoveitInterface of %s is %s", planning_group_name_.c_str(), (SetupMoveitInterface()? "OK" : "fail"));
  ROS_WARN("loadSceneForGrasps %s", (loadSceneForGrasps()? "OK" : "fail"));
  setupGraspPipeline();
  ROS_WARN("GraspPipeline OK");

  no_acessible_objects = false;

  srand(10);

  // Starts with the gripper open.
  MovementOpenEndEffector();
  successful_grasp = false;
  real_object_lost = false;
  planning_scene_ready = false;

  block_planning_scene_pub = nh_.advertise<std_msgs::Bool>("/block_planning_scene", 5);
  start_detection_pub = nh_.advertise<std_msgs::Bool>("/start_detection", 5);
  start_detection_only_object_pub = nh_.advertise<std_msgs::Bool>("/start_detection_only_object", 5);
  start_detection_only_table_pub = nh_.advertise<std_msgs::Bool>("/start_detection_only_table", 5);

  inspect_surroundings_pub = nh_.advertise<std_msgs::Bool>("/inspect_surroundings", 5);
  only_pub_octomap_pub = nh_.advertise<std_msgs::Bool>("/only_pub_octomap", 5);

  place_poses_pub = nh_.advertise<geometry_msgs::PoseArray>("/place_poses", 5);
  
  gripper_state_sub = nh_.subscribe("/gripper_controller/state", 1, &AutonomousTidyUp::gripper_state_callback, this);
  planning_scene_ready_sub = nh_.subscribe("/planning_scene_ready", 1, &AutonomousTidyUp::planning_scene_ready_callback, this);
  continue_execution_sub = nh_.subscribe("/continue_execution", 1, &AutonomousTidyUp::continue_execution_callback, this);
  object_name_to_gazebo_ros_link_attacher_sub = nh_.subscribe("/object_name_to_gazebo_ros_link_attacher", 1, &AutonomousTidyUp::object_name_to_gazebo_ros_link_attacher_callback, this);
  
  // For camera_data_processing
  start_table_detection_pub = nh_.advertise<std_msgs::Bool>("/start_table_detection", 5);
  start_object_detection_pub = nh_.advertise<std_msgs::Bool>("/start_object_detection", 5);
  look_to_detect_objects_pub = nh_.advertise<std_msgs::Bool>("/look_to_detect_objects", 5);

  robot_pose_sub = nh_.subscribe("/robot_pose" , 1, &AutonomousTidyUp::robot_pose_callback, this);

  map_sub = nh_.subscribe("/map", 1, &AutonomousTidyUp::map_callback, this);
  global_costmap_sub = nh_.subscribe("/move_base/global_costmap/costmap", 1, &AutonomousTidyUp::global_costmap_callback, this);
  global_costmap_update_sub = nh_.subscribe("/move_base/global_costmap/costmap_updates", 1, &AutonomousTidyUp::global_costmap_update_callback, this);

  table_on_map_sub = nh_.subscribe("/table_on_map", 5, &AutonomousTidyUp::table_on_map_callback, this);
  object_on_map_sub = nh_.subscribe("/object_on_map", 5, &AutonomousTidyUp::object_on_map_callback, this);
  
  move_base_status_sub = nh_.subscribe("/move_base/status", 1, &AutonomousTidyUp::move_base_status_callback, this);

  object_on_octomap_sub = nh_.subscribe("/object_on_octomap", 5, &AutonomousTidyUp::object_on_octomap_callback, this);

  #if not SIMULATION // Used only for the real robot.
  stop_movement_sub = nh_.subscribe("/stop_movement", 1, &AutonomousTidyUp::stop_movement_callback, this);
  #endif

  semantic_map_visualise_pub = nh_.advertise<nav_msgs::OccupancyGrid>("/semantic_map_visualise", 5);
  potencial_visualise_pub = nh_.advertise<nav_msgs::OccupancyGrid>("/potencial_visualise", 5);

  // For metrics node
  coverage_acessible_area_pub = nh_.advertise<std_msgs::Float32>("/coverage_acessible_area", 5);
  n_acessible_cells_pub = nh_.advertise<std_msgs::Int32>("/n_acessible_cells", 5);
  n_visited_cells_pub = nh_.advertise<std_msgs::Int32>("/n_visited_cells", 5);

  holding_object = false;
  #if SIMULATION
  timerStillHoldingObject = nh_.createTimer(ros::Duration(2.0), &AutonomousTidyUp::StillHoldingObject, this);
  #else // The real robot needs more time between grips.
  timerStillHoldingObject = nh_.createTimer(ros::Duration(4.0), &AutonomousTidyUp::StillHoldingObject, this);
  #endif

  timerContinuousHeadMovement = nh_.createTimer(ros::Duration(2.0), &AutonomousTidyUp::ContinuousHeadMovement, this);
  if (timerContinuousHeadMovement.hasStarted()) timerContinuousHeadMovement.stop();

  // Custom padding for a better performance.
  collision_detection::CollisionRobotPtr collision_robot_final = psm_->getPlanningScene()->getCollisionRobotNonConst();
  std::map<std::string, double> padding_final = collision_robot_final->getLinkPadding();
  ROS_WARN("DEFAULT final on arm_5_link of %f", padding_final["arm_5_link"]);

  ROS_WARN("startSemanticMap %s", (startSemanticMap()? "OK" : "fail"));

  local_costmap_sub = nh_.subscribe("/move_base/local_costmap/costmap", 3, &AutonomousTidyUp::local_costmap_callback, this);

  cloud_sub = nh_.subscribe("/xtion/depth_registered/points", 3, &AutonomousTidyUp::cloudCB, this);

  UnblockPlanningScene();
  timerContinuousHeadMovement.stop();

  smallest_i = smallest_j = std::numeric_limits<int>::max();
  largest_i = largest_j = std::numeric_limits<int>::min();
}

// Prepares the scene for MoveIt Grasps.
bool AutonomousTidyUp::loadSceneForGrasps()
{
  // ---------------------------------------------------------------------------------------------
  // Load planning scene to share
  psm_.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION));
  
  // psm_.reset(new std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description"));
  if (!psm_->getPlanningScene())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Planning scene not configured");
    return false;
  }

  psm_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                          "grasping_planning_scene");
  psm_->getPlanningScene()->setName("grasping_planning_scene");

  collision_detection::CollisionRobotPtr collision_robot_final1_1 = psm_->getPlanningScene()->getCollisionRobotNonConst();///->setLinkPadding(teste, 0.5);
  std::map<std::string, double> padding_final1_1 = collision_robot_final1_1->getLinkPadding();
  
  collision_robot_final1_1->setLinkPadding("arm_5_link", PADDINGarm_5_link);

  psm_->getPlanningScene()->propogateRobotPadding();
  
  auto new_link_padding = psm_->getPlanningScene()->getCollisionRobot()->getLinkPadding();
  
  ROS_WARN("DEFAULT loadSceneForGrasps on arm_5_link of %f", new_link_padding["arm_5_link"]);
  
  robot_model_loader::RobotModelLoaderPtr robot_model_loader;
  robot_model_loader.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));

  // Load the robot model
  robot_model_ = robot_model_loader->getModel();
  arm_jmg_ = robot_model_->getJointModelGroup(planning_group_name_);

  // ---------------------------------------------------------------------------------------------
  // Load the Robot Viz Tools for publishing to Rviz
  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(robot_model_->getModelFrame(), "/rviz_visual_tools",
                                                                   psm_));
  visual_tools_->loadMarkerPub();
  visual_tools_->loadRobotStatePub("/display_robot_state");
  visual_tools_->loadTrajectoryPub("/display_planned_path");
  visual_tools_->loadSharedRobotState();
  visual_tools_->enableBatchPublishing();
  visual_tools_->deleteAllMarkers();
  visual_tools_->removeAllCollisionObjects();
  visual_tools_->hideRobot();
  visual_tools_->trigger();

  // Publish the global frame
  visual_tools_->publishAxis(Eigen::Isometry3d::Identity());
  visual_tools_->trigger();

  return true;
}

// Configures the MoveIt Grasp to generate grasps for the robot model used with custom weights.
void AutonomousTidyUp::setupGraspPipeline()
{
  // ---------------------------------------------------------------------------------------------
  // Load grasp data specific to our robot
  grasp_data_ = std::make_shared<moveit_grasps::GraspData>(nh_, ee_group_name_, visual_tools_->getRobotModel());
  if (!grasp_data_->loadGraspData(nh_, ee_group_name_))
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Failed to load Grasp Data parameters.");
    exit(-1);
  }

  // ---------------------------------------------------------------------------------------------
  // Load grasp generator
  grasp_generator_.reset(new moveit_grasps::GraspGenerator(visual_tools_));

  // Set the ideal grasp orientation for scoring
  std::vector<double> ideal_grasp_rpy = { 3.14, 0.0, 0.0 };
  grasp_generator_->setIdealTCPGraspPoseRPY(ideal_grasp_rpy);

  // Set custom grasp score weights
  moveit_grasps::GraspScoreWeights grasp_score_weights;
  grasp_score_weights.orientation_x_score_weight_ = 2.0;
  grasp_score_weights.orientation_y_score_weight_ = 2.0;
  grasp_score_weights.orientation_z_score_weight_ = 2.0;
  grasp_score_weights.translation_x_score_weight_ = 1.0;
  grasp_score_weights.translation_y_score_weight_ = 1.0;
  grasp_score_weights.translation_z_score_weight_ = 1.0;
  // Finger gripper specific weights.
  // Note that we do not need to set the suction gripper specific weights for our finger gripper.
  grasp_score_weights.depth_score_weight_ = 2.0;
  grasp_score_weights.width_score_weight_ = 2.0;
  grasp_generator_->setGraspScoreWeights(grasp_score_weights);

  // ---------------------------------------------------------------------------------------------
  // Load grasp filter
  grasp_filter_.reset(new moveit_grasps::GraspFilter(visual_tools_->getSharedRobotState(), visual_tools_));

  // ---------------------------------------------------------------------------------------------
  // Load grasp planner for approach, lift and retreat planning
  grasp_planner_.reset(new moveit_grasps::GraspPlanner(visual_tools_));

  #if WITH_STOPS
  // MoveIt Grasps allows for a manual breakpoint debugging tool to be optionally passed in
  grasp_planner_->setWaitForNextStepCallback(boost::bind(&waitForNextStep, visual_tools_, _1));
  #endif

  // -----------------------------------------------------
  // Load the motion planning pipeline
  planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(robot_model_, nh_, "planning_plugin", "request_"
                                                                                                           "adapter"));

}

// Starts the servers for motion, grasp, attach/detach gazebo and save map.
bool AutonomousTidyUp::SetupServers()
{
  // MoveBaseClient 
  ac = (new MoveBaseClient("move_base", true));
  while (!ac->waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
    if (!ros::ok())
      return false;
  }
  #if DEBUG
    ROS_WARN("move_base action server OK");
  #endif

  // PlayMotionClient 
  client_arm = (new PlayMotionClient("/play_motion", true));
  while (!client_arm->waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the play_motion action server to come up");
    if (!ros::ok())
      return false;
  }
  #if DEBUG
    ROS_WARN("play_motion action server OK");
  #endif

  grasp_client = nh_.serviceClient<std_srvs::Empty>("/parallel_gripper_controller/grasp");
  grasp_client.waitForExistence();
  ROS_WARN("parallel_gripper_controller/grasp READY");

  make_plan_client = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/GlobalPlanner/make_plan");
  make_plan_client.waitForExistence();
  ROS_WARN("/move_base/GlobalPlanner/make_plan READY");

  #if SIMULATION
  attach_client = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
  attach_client.waitForExistence();
  ROS_WARN("/link_attacher_node/attach READY");

  // EXAMPLE to call the service:
  // Singularity> rosservice call /link_attacher_node/attach "model_name_1: 'tiago'
  // link_name_1: 'gripper_right_finger_link'
  // model_name_2: 'unit_box_custom'
  // link_name_2: 'link'" 
  // ok: True
  // Singularity> 

  detach_client = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
  detach_client.waitForExistence();
  ROS_WARN("/link_attacher_node/detach READY");
  #endif

  #if GMAPPING_EXPLORATION
  save_map_client = nh_.serviceClient<pal_navigation_msgs::SaveMap>("/pal_map_manager/save_map");
  save_map_client.waitForExistence();
  #endif

  return true;
}

// Prepares the PlanningSceneMonitor and sets new padding for the
bool AutonomousTidyUp::SetupPlanningScene()
{
  psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(ROBOT_DESCRIPTION);
  // psm_.force_kill = false;
  ros::spinOnce();
  // update the planning scene monitor with the current state
  bool success = false;
  while (ros::ok() && !success){
    success = psm_->requestPlanningSceneState("/get_planning_scene");
    ROS_WARN_STREAM("Request planning scene " << (success ? "succeeded." : "failed."));
  }

  // keep up to date with new changes
  psm_->startSceneMonitor("/move_group/monitored_planning_scene");
  #if DEBUG
    psm_->getPlanningScene()->printKnownObjects();
  #endif

  collision_detection::CollisionRobotPtr collision_robot_final1_1 = psm_->getPlanningScene()->getCollisionRobotNonConst();///->setLinkPadding(teste, 0.5);
  std::map<std::string, double> padding_final1_1 = collision_robot_final1_1->getLinkPadding();
  
  collision_robot_final1_1->setLinkPadding("arm_5_link", PADDINGarm_5_link);

  psm_->getPlanningScene()->propogateRobotPadding();
  
  auto new_link_padding = psm_->getPlanningScene()->getCollisionRobot()->getLinkPadding();
  
  ROS_WARN("DEFAULT SetupPlanningScene inicial on arm_5_link of %f", new_link_padding["arm_5_link"]);
  
  return success;
}

// Configures group_arm_torso and group_gripper.
bool AutonomousTidyUp::SetupMoveitInterface()
{
  group_arm_torso = (new MoveitInterface(planning_group_name_));
  group_arm_torso->setPlanningTime(60.0);
  group_arm_torso->setPoseReferenceFrame("base_footprint");

  group_gripper = (new MoveitInterface(ee_group_name_));
  group_gripper->setPlanningTime(10.0);
  group_gripper->setPoseReferenceFrame("base_footprint");

  return true;
}

// Sets new max velocity.
void AutonomousTidyUp::set_local_planner_max_vel(double new_lin_vel, double new_ang_vel) {
  // Source: https://github.com/nlimpert/safety_controller/blob/061a20a6a025fb4c585907b8f3c9957382d510a6/src/safety_controller_node.cpp#L142

  dynamic_reconfigure::ReconfigureRequest dynreconf_srv_req;
  dynamic_reconfigure::ReconfigureResponse dynreconf_srv_resp;
  dynamic_reconfigure::DoubleParameter dynreconf_max_vel_x_param;
  dynamic_reconfigure::DoubleParameter dynreconf_max_vel_theta_param;
  // dynamic_reconfigure::DoubleParameter dynreconf_max_vel_y_param;
  dynamic_reconfigure::BoolParameter dynreconf_bool_param;
  dynamic_reconfigure::Config dynreconf_conf;

  dynreconf_max_vel_x_param.name = "max_vel_x";
  dynreconf_max_vel_x_param.value = new_lin_vel;
  dynreconf_max_vel_theta_param.name = "max_vel_theta";
  dynreconf_max_vel_theta_param.value = new_ang_vel;
  // dynreconf_max_vel_y_param.name = "max_vel_y";
  // dynreconf_max_vel_y_param.value = new_lin_vel; // y for omnidirectional.

  dynreconf_conf.doubles.push_back(dynreconf_max_vel_x_param);
  dynreconf_conf.doubles.push_back(dynreconf_max_vel_theta_param);
  // dynreconf_conf.doubles.push_back(dynreconf_max_vel_y_param);
  dynreconf_srv_req.config = dynreconf_conf;

  #if SIMULATION
  while (!ros::service::call("/move_base/TebLocalPlannerROS/set_parameters",
                          dynreconf_srv_req, dynreconf_srv_resp)) {
  #else
  while (!ros::service::call("/move_base/PalLocalPlanner/set_parameters",
                          dynreconf_srv_req, dynreconf_srv_resp)) {
  #endif
    ROS_WARN_THROTTLE(1.0, "Failed to send dynreconf");
    ros::spinOnce();
  }

  ROS_INFO("Sent dynreconf to LocalPlanner, new vel lin: %f ang: %f", new_lin_vel, new_ang_vel);
  dynreconf_conf.doubles.clear();
}

// Sets new goal tolerance.
void AutonomousTidyUp::set_local_planner_xy_goal_tolerance(double new_xy_goal_tolerance) {

  dynamic_reconfigure::ReconfigureRequest dynreconf_srv_req;
  dynamic_reconfigure::ReconfigureResponse dynreconf_srv_resp;
  dynamic_reconfigure::DoubleParameter dynreconf_xy_goal_tolerance_param;
  dynamic_reconfigure::BoolParameter dynreconf_bool_param;
  dynamic_reconfigure::Config dynreconf_conf;

  dynreconf_xy_goal_tolerance_param.name = "xy_goal_tolerance";
  dynreconf_xy_goal_tolerance_param.value = new_xy_goal_tolerance;

  dynreconf_conf.doubles.push_back(dynreconf_xy_goal_tolerance_param);
  
  dynreconf_srv_req.config = dynreconf_conf;

  #if SIMULATION
  while (!ros::service::call("/move_base/TebLocalPlannerROS/set_parameters",
                          dynreconf_srv_req, dynreconf_srv_resp)) {
  #else
  while (!ros::service::call("/move_base/PalLocalPlanner/set_parameters",
                          dynreconf_srv_req, dynreconf_srv_resp)) {
  #endif
    ROS_WARN_THROTTLE(1.0, "Failed to send dynreconf");
    ros::spinOnce();
  }

  ROS_INFO("Sent dynreconf to LocalPlanner, new xy_goal_tolerance: %f m", new_xy_goal_tolerance);
  dynreconf_conf.doubles.clear();

}

AutonomousTidyUp::~AutonomousTidyUp()
{
}

///////////////////////////////////////
//                                   //
//         Goals and motions         //
//                                   //
///////////////////////////////////////

void AutonomousTidyUp::SendGoalBase(float x, float y, float z, float roll, float pitch, float yaw) {

  move_base_msgs::MoveBaseGoal goal;
  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.position.z = z;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

  ROS_WARN("Sending goal");
  ac->sendGoal(goal);
  // Wait an infinite time for the results
  ac->waitForResult();

  // Check if the robot reached ts goal
  if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_WARN("Result: Hooray, reached drop off zone");
  else
    ROS_WARN("Result: The base failed to move forward");
}

bool AutonomousTidyUp::SendAndSaveGoalBase(geometry_msgs::Pose goal_pose) {

  move_base_msgs::MoveBaseGoal goal;
  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = goal_pose;

  goal_point_ = goal_pose;

  ROS_WARN("Sending goal");
  ac->sendGoalAndWait(goal);
  // Wait an infinite time for the results
  ac->waitForResult();

  // Check if the robot reached ts goal
  if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_WARN("Result: Hooray, reached drop off zone");
    return true;
  } else {
    ROS_WARN("Result: The base failed to move forward");
    return false;
  }
}

bool AutonomousTidyUp::SendAndSaveGoalBase(float x, float y, float z, float roll, float pitch, float yaw) {

  move_base_msgs::MoveBaseGoal goal;
  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.position.z = z;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

  goal_point_ = goal.target_pose.pose;

  ROS_WARN("Sending goal");
  ac->sendGoalAndWait(goal);
  // Wait an infinite time for the results
  ac->waitForResult();

  // Check if the robot reached ts goal
  if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_WARN("Result: Hooray, reached drop off zone");
    return true;
  } else {
    ROS_WARN("Result: The base failed to move forward");
    return false;
  }
}

void AutonomousTidyUp::SendAndSaveGoalBaseWithoutWait(geometry_msgs::Pose goal_pose) {

  move_base_msgs::MoveBaseGoal goal;
  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = goal_pose;

  goal_point_ = goal_pose;

  ROS_WARN("Sending goal");
  ac->sendGoal(goal);

}

void AutonomousTidyUp::SendAndSaveGoalBaseWithoutWait(float x, float y, float z, float roll, float pitch, float yaw) {

  move_base_msgs::MoveBaseGoal goal;
  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.position.z = z;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

  goal_point_ = goal.target_pose.pose;

  ROS_WARN("Sending goal");
  ac->sendGoal(goal);
}

void AutonomousTidyUp::SaveGoalNotValid(float x, float y, float z, float roll, float pitch, float yaw) {

  geometry_msgs::Pose goal;

  goal.position.x = x;
  goal.position.y = y;
  goal.position.z = z;
  goal.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

  goal_point_not_valid = goal;

}

bool AutonomousTidyUp::SendMotionArm(string motion) {

  actionlib::SimpleClientGoalState state = client_arm->getState();
  ROS_WARN_STREAM("Arm client state: " << state.toString());
  
  if (!state.isDone())
  {
    return false;
  }

  play_motion_msgs::PlayMotionGoal motion_arm;

  motion_arm.motion_name = motion.c_str();
  motion_arm.skip_planning = false;
  motion_arm.priority = 0;

  ROS_WARN_STREAM("Sending goal with motion: " << motion);
  client_arm->sendGoalAndWait(motion_arm);

  ROS_WARN("Waiting for result ...");
  bool actionOk = client_arm->waitForResult(ros::Duration(30.0));

  state = client_arm->getState();

  if (state.state_ != state.SUCCEEDED)
  {
    ROS_ERROR_STREAM("Action ERROR with state: " << state.toString());
    return false;
  } else
  {
    ROS_ERROR_STREAM("Action finished successfully with state: " << state.toString());
    return true;
  }
  
}

void AutonomousTidyUp::SendMotionArmWithoutWait(string motion) {

  actionlib::SimpleClientGoalState state = client_arm->getState();
  ROS_WARN_STREAM("Arm client state: " << state.toString());
  
  if (!state.isDone())
  {
    return;
  }

  play_motion_msgs::PlayMotionGoal motion_arm;

  motion_arm.motion_name = motion.c_str();
  motion_arm.skip_planning = false;
  motion_arm.priority = 0;

  ROS_WARN_STREAM("Sending goal with motion: " << motion);
  client_arm->sendGoal(motion_arm);

}

// Head motion in steps to allow the pauses for the real robot. When it stops, the movement continues in the next step.
bool AutonomousTidyUp::SendBIGhead_look_aroundSLOW2_steps() {

  actionlib::SimpleClientGoalState state = client_arm->getState();
  ROS_WARN_STREAM("Arm client state: " << state.toString());

  if (!state.isDone())
  {
    return false;
  }

  string motion = "BIGhead_look_aroundSLOW2_0";
  motion[25] += step_head_motion;

  play_motion_msgs::PlayMotionGoal motion_arm;

  motion_arm.motion_name = motion.c_str();
  motion_arm.skip_planning = false;
  motion_arm.priority = 0;

  ROS_WARN_STREAM("Sending goal with motion: " << motion);
  client_arm->sendGoal(motion_arm);

  step_head_motion = (step_head_motion+1) % 8; // [0, 7]
  return true;
}

void AutonomousTidyUp::openGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gripper_left_finger_joint";
  posture.joint_names[1] = "gripper_right_finger_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = MAX_FINGER_JOINT_VALUE;
  posture.points[0].positions[1] = MAX_FINGER_JOINT_VALUE;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

// void AutonomousTidyUp::closedGripper(trajectory_msgs::JointTrajectory& posture)
// {
//   posture.joint_names.resize(2);
//   posture.joint_names[0] = "gripper_left_finger_joint";
//   posture.joint_names[1] = "gripper_right_finger_joint";
//   posture.points.resize(1);
//   posture.points[0].positions.resize(2);
//   posture.points[0].positions[0] = 0.00;
//   posture.points[0].positions[1] = 0.00;
//   posture.points[0].time_from_start = ros::Duration(0.5);
// }

bool AutonomousTidyUp::LiftTorsoJoint(float joint_value){
  std::vector<double> group_variable_values;
  group_arm_torso->getCurrentState()->copyJointGroupPositions(group_arm_torso->getCurrentState()->getRobotModel()->getJointModelGroup(group_arm_torso->getName()), group_variable_values);

  if (joint_value >= 0.0 && joint_value < 0.35)
  {
    group_variable_values[0] = joint_value;
  }else
  {
    ROS_WARN("invalid joint_value - Lift Torso Joint FINISHED");
    return false;
  }
   
  group_arm_torso->setJointValueTarget(group_variable_values); 
  bool resul = MoveGroupArmTorso();
  ROS_WARN("Lift Torso Joint FINISHED");
  return resul;
}

// Lifts the end effector in the z axis.
bool AutonomousTidyUp::LiftEndEffector(float lift_value){

  geometry_msgs::PoseStamped current_pose = group_arm_torso->getCurrentPose();
  geometry_msgs::Pose new_pose;

  new_pose = current_pose.pose;
  new_pose.position.z = current_pose.pose.position.z + lift_value;

  ROS_WARN("LIFT THE GRIPPER");

  return MoveGripperToGoal(new_pose);
}

bool AutonomousTidyUp::MovementOpenEndEffector()
{
  std::vector<double> joints_value;
  joints_value.resize(2);
  joints_value[0] = MAX_FINGER_JOINT_VALUE;
  joints_value[1] = MAX_FINGER_JOINT_VALUE;

  group_gripper->setJointValueTarget(joints_value);
  bool resul = MoveGroupGripper();
  group_gripper->setJointValueTarget(joints_value);
  bool resul2 = MoveGroupGripper();
  ROS_WARN("Movement Open End Effector FINISHED");
  return (resul or resul2);
}

// Goes to the default robot arm/torso pose.
bool AutonomousTidyUp::MovementHome(){
  std::vector<double> group_variable_values;
  group_variable_values.resize(8);

  group_variable_values[0] = 0.3;
  group_variable_values[1] = DEG2RAD(  11.0);//// * M_PI / 180.0;
  group_variable_values[2] = DEG2RAD( -77.0);
  group_variable_values[3] = DEG2RAD( -11.0);
  group_variable_values[4] = DEG2RAD( 111.0);
  group_variable_values[5] = DEG2RAD( -90.0);
  group_variable_values[6] = DEG2RAD(  78.0);
  group_variable_values[7] = DEG2RAD(   0.0);  
   
  group_arm_torso->setJointValueTarget(group_variable_values);
  bool resul = MoveGroupArmTorso();
  ROS_WARN("Movement Home FINISHED");
  return resul;
}

// Goes to the better position outside the sensors reach to carry the object.
bool AutonomousTidyUp::MovementArmUpToWalk(){
  std::vector<double> group_variable_values;

  group_arm_torso->getCurrentState()->copyJointGroupPositions(group_arm_torso->getCurrentState()->getRobotModel()->getJointModelGroup(group_arm_torso->getName()), group_variable_values);

  float original_value_joint_1 = group_variable_values[1];

  group_variable_values[0] = 0.3;

  bool resul = false;
  if (!resul)
  {
    group_variable_values[0] = 0.3;
    if (original_value_joint_1 <= DEG2RAD( 80))
    { // object picked up to the robot's right
      group_variable_values[1] = DEG2RAD(  42.0);
      ROS_WARN("RIGHT.... up");
      group_variable_values[2] = DEG2RAD( -80.0);
      group_variable_values[3] = DEG2RAD(  85.0);
      group_variable_values[4] = DEG2RAD( 121.0);
      group_variable_values[5] = DEG2RAD( -88.0);
      group_variable_values[6] = DEG2RAD(  45.0);
      group_variable_values[7] = DEG2RAD( -38.0); 
    }else
    { // left
      group_variable_values[1] = DEG2RAD( 150.0);
      ROS_WARN("LEFT.... up");
      group_variable_values[2] = DEG2RAD( -84.0);
      group_variable_values[3] = DEG2RAD( -81.0);
      group_variable_values[4] = DEG2RAD( 121.0);
      group_variable_values[5] = DEG2RAD( -93.0);
      group_variable_values[6] = DEG2RAD(  41.0);
      group_variable_values[7] = DEG2RAD(  23.0); 
    } 

    group_arm_torso->setJointValueTarget(group_variable_values);
    resul = MoveGroupArmTorso();
  }

  ROS_WARN("Movement Arm Up To Walk FINISHED");
  return resul;
}

bool AutonomousTidyUp::MoveGripperToGoal(geometry_msgs::Pose new_goal) {

  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "base_footprint";
  goal.header.stamp = ros::Time::now();
  goal.pose = new_goal;

  group_arm_torso->setStartStateToCurrentState();
  group_arm_torso->setPoseTarget(goal);
  bool resul = MoveGroupArmTorso();
  ROS_WARN("Move Gripper To Goal FINISHED");
  return resul;
}


///////////////////////////////////////
//                                   //
//            Manipulation           //
//                                   //
///////////////////////////////////////

bool AutonomousTidyUp::getIKSolution( const Eigen::Isometry3d& target_pose,
                     robot_state::RobotState& solution, const std::string& link_name)
{
  boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRW> ls(new planning_scene_monitor::LockedPlanningSceneRW(psm_));
  moveit::core::GroupStateValidityCallbackFn constraint_fn = boost::bind(&isStateValid, static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get(), _1, _2, _3);

  // seed IK call with current state
  solution = (*ls)->getCurrentState();
  ls.reset();

  // Solve IK problem for arm
  // disable explicit restarts to guarantee close solution if one exists
  const double timeout = 0.1;
  return solution.setFromIK(arm_jmg_, target_pose, link_name, timeout, constraint_fn);
}

// Plans a pre-approach considering a tolerance.
bool AutonomousTidyUp::planPreApproach(const robot_state::RobotState& goal_state, moveit_msgs::MotionPlanResponse& pre_approach_plan)
{
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;

  double tolerance_above = 0.02;
  double tolerance_below = 0.02;
  req.group_name = arm_jmg_->getName();
  req.num_planning_attempts = 5;
  req.allowed_planning_time = 1.5;
  moveit_msgs::Constraints goal = kinematic_constraints::constructGoalConstraints(goal_state, arm_jmg_, tolerance_below, tolerance_above);

  req.goal_constraints.push_back(goal);
  boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRW> ls(new planning_scene_monitor::LockedPlanningSceneRW(psm_));

  // Gets the robot current state
  robot_state::RobotState rs = (*ls)->getCurrentState();
    
  robot_state::robotStateToRobotStateMsg(rs, req.start_state);

  planning_pipeline_->generatePlan(*ls, req, res);
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_INFO_NAMED(LOGNAME, "Failed to plan approach successfully");
    return false;
  }

  res.getMessage(pre_approach_plan);
  return true;
}

// Analises the grasp candidates to find valid grasps with valid pre-approach plan. DOES NOT CONSIDERS THE ENVIRONMENT, ONLY IF THE ARM REACHES.
bool AutonomousTidyUp::planFullGrasp(std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates,
                     std::vector<moveit_grasps::GraspCandidatePtr>& list_valid_grasp_candidate,
                     std::vector<moveit_msgs::MotionPlanResponse>& list_pre_approach_plan)
{
  moveit::core::RobotStatePtr current_state;
  moveit_grasps::GraspCandidatePtr valid_grasp_candidate;
  moveit_msgs::MotionPlanResponse pre_approach_plan;
  {
    boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRW> ls(new planning_scene_monitor::LockedPlanningSceneRW(psm_));
    current_state.reset(new moveit::core::RobotState((*ls)->getCurrentState()));
  }

  bool success = false;
  for (int num_valid_grasp = 0; !grasp_candidates.empty() && num_valid_grasp < MAX_NUM_VALID_GRASPS; grasp_candidates.erase(grasp_candidates.begin()))
  {
    valid_grasp_candidate = grasp_candidates.front();

    ROS_WARN("get valid_grasp_candidate %s", ((valid_grasp_candidate->getPreGraspState(current_state))? "OK": "fail"));
    if (!grasp_planner_->planApproachLiftRetreat(valid_grasp_candidate, current_state, psm_, false))
    {
      ROS_INFO_NAMED(LOGNAME, "failed to plan approach lift retreat");
      continue;
    }

    robot_state::RobotStatePtr pre_grasp_state = valid_grasp_candidate->segmented_cartesian_traj_[moveit_grasps::APPROACH].front();
    if (!planPreApproach(*pre_grasp_state, pre_approach_plan))
    {
      ROS_WARN_NAMED(LOGNAME, "failed to plan to pregrasp_state");
      continue;
    }

    num_valid_grasp++;
    list_valid_grasp_candidate.push_back(valid_grasp_candidate);
    list_pre_approach_plan.push_back(pre_approach_plan);

    success = true;
  }
  return success;
}

// Calculates the possible grasps and picks an object.
bool AutonomousTidyUp::PickWithAutoGrasp(std::string object_name, int n_object){

  setACMFingerEntry(object_name, true);

  geometry_msgs::Pose object_pose;
  double object_x_depth;
  double object_y_width;
  double object_z_height;

  std::vector<std::string> chose_object;
  chose_object.resize(1);
  chose_object[0] = object_name;

  std::map<std::string,moveit_msgs::CollisionObject> map_collision_objects;

  // Gets the object from the planning scene.
  map_collision_objects = planning_scene_interface.getObjects(chose_object);

  if (map_collision_objects.size() == 0)
  {
    ROS_WARN("Without collision object: %s", object_name.c_str());
    return false;
  }

  #if DEBUG
  ROS_WARN("found %d objects", map_collision_objects.size());
  ROS_WARN("object with id %s", map_collision_objects[object_name].id.c_str());
  ROS_WARN("object on frame %s", map_collision_objects[object_name].header.frame_id.c_str());
  ROS_WARN("object on x: %f", map_collision_objects[object_name].primitive_poses[0].position.x);
  #endif

  object_x_depth = map_collision_objects[object_name].primitives[0].dimensions[0];
  object_y_width = map_collision_objects[object_name].primitives[0].dimensions[1];
  object_z_height = map_collision_objects[object_name].primitives[0].dimensions[2];
  // #if DEBUG
  // ROS_WARN("object size %f x %f x %f", object_x_depth, object_y_width, object_z_height);
  
  pose_inicial_obj = map_collision_objects[object_name].primitive_poses[0];

  // Generates grasp candidates.
  std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates[map_collision_objects[object_name].primitive_poses.size()];

  // Configures the desired types of grasps.
  moveit_grasps::GraspCandidateConfig grasp_generator_config = moveit_grasps::GraspCandidateConfig();
  grasp_generator_config.disableAll();
  grasp_generator_config.enable_face_grasps_ = true;
  grasp_generator_config.generate_y_axis_grasps_ = true;
  grasp_generator_config.generate_x_axis_grasps_ = true;
  grasp_generator_config.generate_z_axis_grasps_ = true;

  // For each solid that composes the object.
  for (int i = 0; i < map_collision_objects[object_name].primitive_poses.size(); i++)
  {

    if(map_collision_objects[object_name].primitives[i].type == shape_msgs::SolidPrimitive::BOX){
      object_pose = map_collision_objects[object_name].primitive_poses[i];
      object_x_depth = map_collision_objects[object_name].primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_X];
      object_y_width = map_collision_objects[object_name].primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
      object_z_height = MIN_OBJECT_HEIGHT + map_collision_objects[object_name].primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_Z];
      // #if DEBUG
      ROS_WARN("object BOX %d size %f x %f x %f", i, object_x_depth, object_y_width, object_z_height);
      // #endif
    }else if (map_collision_objects[object_name].primitives[i].type == shape_msgs::SolidPrimitive::CYLINDER)
    {
      object_pose = map_collision_objects[object_name].primitive_poses[i];
      object_x_depth = 2.0 * map_collision_objects[object_name].primitives[i].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
      object_y_width = 2.0 * map_collision_objects[object_name].primitives[i].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
      object_z_height = map_collision_objects[object_name].primitives[i].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT];
      ROS_WARN("object CYLINDER %d size %f x %f x %f", i, object_x_depth, object_y_width, object_z_height);
      
    }
    
    if ((object_x_depth  > MAX_FINGER_JOINT_VALUE * 2.0 - 0.02) &&
        (object_y_width  > MAX_FINGER_JOINT_VALUE * 2.0 - 0.02))// &&
        // (object_z_height > MAX_FINGER_JOINT_VALUE * 2.0 - 0.01) )
    {
      if (object_x_depth <= object_y_width)
      {
        object_x_depth = MAX_FINGER_JOINT_VALUE * 2.0 - 0.02;
      } else
      {
        object_y_width = MAX_FINGER_JOINT_VALUE * 2.0 - 0.02;
      }
      ROS_ERROR("ADJUSTED DIMENSIONS");
    }
    
    if (object_z_height > 0.10) // the gripper is 0.10 long
    {
      object_z_height = object_z_height + EXTRA_HEIGHT;
    } else
    {      
      object_z_height = object_z_height + MIN_EXTRA_HEIGHT;
    }
  
    if (!grasp_generator_->generateGrasps(visual_tools_->convertPose(object_pose), object_x_depth, object_y_width,
                                          object_z_height, grasp_data_, grasp_candidates[i], grasp_generator_config))
    {
      ROS_WARN("Grasp generator failed to generate any valid grasps for %d", i);
      ChangeObjectState(n_object, StateInvalidGrasp);   
      return false;
    }else{
      ROS_WARN("%d VALID grasps for %d", grasp_candidates[i].size(), i);
    }
  }

  std::vector<moveit_grasps::GraspCandidatePtr> all_grasp_candidates;
  for (int i = 0; i < map_collision_objects[object_name].primitive_poses.size(); i++)
  {
    all_grasp_candidates.insert(all_grasp_candidates.end(), grasp_candidates[i].begin(), grasp_candidates[i].end());
  }

  if (all_grasp_candidates.size() == 0)
  {
    ROS_WARN("Without grasp candidates");
    ROS_WARN("--------->>>>>>>>Object %s Without grasp candidates", objects_on_the_floor_recevied[n_object].object.id.c_str());
    ChangeObjectState(n_object, StateInvalidGrasp);
    return false;
  }  

  // Generating a seed state for filtering grasps.
  robot_state::RobotStatePtr seed_state(new robot_state::RobotState(*visual_tools_->getSharedRobotState()));
  Eigen::Isometry3d eef_mount_grasp_pose = visual_tools_->convertPose(object_pose) * grasp_data_->tcp_to_eef_mount_.inverse();
  if (!getIKSolution( eef_mount_grasp_pose, *seed_state, grasp_data_->parent_link_->getName()))
  {
    ROS_WARN("The ideal seed state is not reachable. Using start state as seed.");
  }else{
    // ROS_WARN("OK");
  }

  // Filtering grasps:
  // Note: This step also solves for the grasp and pre-grasp states and stores them in grasp candidates)
  bool filter_pregrasps = true;
  if (!grasp_filter_->filterGrasps(all_grasp_candidates, psm_, arm_jmg_, seed_state, filter_pregrasps))
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Filter grasps failed");
    ROS_WARN("--------->>>>>>>>Object %s Filter grasps failed", objects_on_the_floor_recevied[n_object].object.id.c_str());
    ChangeObjectState(n_object, StateInvalidGrasp);
    return false;
  }

  if (!grasp_filter_->removeInvalidAndFilter(all_grasp_candidates))
  {
    ROS_WARN_NAMED(LOGNAME, "Grasp filtering removed all grasps");
    ROS_WARN("--------->>>>>>>>Object %s Grasp filtering removed all grasps", objects_on_the_floor_recevied[n_object].object.id.c_str());
    ChangeObjectState(n_object, StateInvalidGrasp);
    return false;
  }
  ROS_INFO_STREAM_NAMED(LOGNAME, "" << all_grasp_candidates.size() << " remain after filtering");

  // Plans free-space approach, cartesian approach, lift and retreat trajectories.
  std::vector<moveit_grasps::GraspCandidatePtr> selected_grasp_candidate;
  std::vector<moveit_msgs::MotionPlanResponse> pre_approach_plan;
  if (!planFullGrasp(all_grasp_candidates, selected_grasp_candidate, pre_approach_plan))
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Failed to plan grasp motions");
    ROS_WARN("--------->>>>>>>>Object %s Failed to plan grasp motions", objects_on_the_floor_recevied[n_object].object.id.c_str());
    ChangeObjectState(n_object, StateInvalidPlan);
    return false;
  }

  ROS_WARN("%d plan grasp motions", selected_grasp_candidate.size());

  // Executes the picking with the calculated grasps.
  setACMFingerEntry(object_name, true);
  moveit::core::MoveItErrorCode resul = RealPick(object_name, selected_grasp_candidate, pre_approach_plan, selected_grasp_candidate.size(), n_object);
  setACMFingerEntry(object_name, true);

  if (resul == autonomous_tidy_up::AutonomousTidyUp::NO_REAL_OBJECT)
  {
    real_object_lost = true;
  }
  
  return (resul == moveit_msgs::MoveItErrorCodes::SUCCESS) or (planning_scene_interface.getAttachedObjects().size() == 1);
}

// To set collision checking between fingers and object.
void AutonomousTidyUp::setACMFingerEntry(const std::string& object_name, bool allowed)
{
  planning_scene_monitor::LockedPlanningSceneRW scene(psm_);  // Lock planning scene

  // Get links of end effector
  const std::vector<std::string>& ee_links = grasp_data_->ee_jmg_->getLinkModelNames();

  // Set collision checking between fingers and object
  for (std::size_t i = 0; i < ee_links.size(); ++i)
  {
    scene->getAllowedCollisionMatrixNonConst().setEntry(object_name, ee_links[i], allowed);
  }
  scene->getAllowedCollisionMatrixNonConst().setEntry(object_name, "gripper_left_finger_link", allowed);
  scene->getAllowedCollisionMatrixNonConst().setEntry(object_name, "gripper_right_finger_link", allowed);
}

// To set collision checking between fingers and object; object and surface.
void AutonomousTidyUp::setACMFingerEntry(const std::string& object_name, const std::string& surface_name, bool allowed)
{
  planning_scene_monitor::LockedPlanningSceneRW scene(psm_);  // Lock planning scene

  // Get links of end effector
  const std::vector<std::string>& ee_links = grasp_data_->ee_jmg_->getLinkModelNames();

  // Set collision checking between fingers and object
  for (std::size_t i = 0; i < ee_links.size(); ++i)
  {
    scene->getAllowedCollisionMatrixNonConst().setEntry(object_name, ee_links[i], allowed);
  }

  scene->getAllowedCollisionMatrixNonConst().setEntry(object_name, "gripper_left_finger_link", allowed);
  scene->getAllowedCollisionMatrixNonConst().setEntry(object_name, "gripper_right_finger_link", allowed);

  // Set collision checking between surface and object
  scene->getAllowedCollisionMatrixNonConst().setEntry(object_name, surface_name, allowed);
}

// Picks an object and goes to the walking pose holding an object.
bool AutonomousTidyUp::PickObject(std::string object_name, int n_object)
{
  std::vector<std::string> objects;
  objects.resize(1);
  objects[0] = object_name;

  geometry_msgs::Pose initial_object_pose = GetObjectPose(object_name);
  printSceneObjects();

  int i = 0;
  bool output = false;
  bool success = false;

  double place_pose[3] = {initial_object_pose.position.x, initial_object_pose.position.y, initial_object_pose.position.z};

  holding_object = false;
  real_object_lost = false;

  BlockPlanningScene();
  do
  {
    i = 0;
    output = false;
    success = false;
    do
    {
      output = PickWithAutoGrasp(object_name, n_object);
      ROS_WARN("pick try %d", i);
      if(output) success = true;
      i++;
      ros::Duration(1.0).sleep();
      if(real_object_lost) break;
    } while (ros::ok() && (i < MIN_TRYS_NUMBER? true : (!success &&  i < MAX_TRYS_NUMBER)));
    
    if (real_object_lost)
    {
      printSceneObjects();
      ROS_WARN("Pick Object END real_object_lost %s------------------------------", (success? "OK": "with problem"));
      return false;
    }else
    {
      holding_object = true;
    }
    
  } while (!holding_object && ros::ok());
  
  ros::Duration(5.0).sleep();

  if (success)
  {
    // Goes to the walking pose holding an object.
    bool resul = false;
    int trys = 0;
    do
    {
      resul = MovementArmUpToWalk();
      ros::Duration(1.0).sleep();
      ros::spinOnce();
    } while (!resul && ros::ok());
    
    ros::Duration(4.0).sleep();
    ros::spinOnce();
    ros::Duration(4.0).sleep();
    ros::spinOnce();
    #if not SIMULATION
    if (!holding_object)
    {
      ROS_WARN("Lost the object...");
      ChangeObjectState(n_object, StateNotGripped);
      success = false;

      std::string object_name_grasp = "object1";

      while (!MovementOpenEndEffector() && ros::ok())
      {
        ROS_WARN("Try to open the gripper");
        planning_scene_interface.removeCollisionObjects(std::vector<std::string>{object_name_grasp});
        ros::Duration(1.0).sleep();
        ros::spinOnce();
      };
      group_arm_torso->detachObject();
      ros::Duration(1.0).sleep();
      ros::spinOnce();
      group_arm_torso->detachObject();
      ros::Duration(1.0).sleep();
      ros::spinOnce();
      group_arm_torso->detachObject();
      ros::Duration(1.0).sleep();
      ros::spinOnce();
      planning_scene_interface.removeCollisionObjects(std::vector<std::string>{object_name_grasp});
      ros::Duration(1.0).sleep();
      ros::spinOnce();
      planning_scene_interface.removeCollisionObjects(std::vector<std::string>{object_name_grasp});
      ros::Duration(1.0).sleep();
      ros::spinOnce();

      trys = 0;
      do
      {
        resul = MovementHome();
        ros::Duration(1.0).sleep();
        ros::spinOnce();
        trys++;
      } while (trys < MAX_TRYS && resul == false && ros::ok());

      if (!resul)
      {
        ROS_ERROR("MovementHome with problem");
        WaitToContinueExecution();
        MovementHome();
      }
    } else
    {
      ROS_WARN("Still holding the object...");
    }
    #endif
  }
  
  LiftTorsoJoint(0.1);
  
  ros::Duration(10.0).sleep();

  printSceneObjects();

  UnblockPlanningScene();

  ROS_WARN("Pick Object END %s------------------------------", (success? "OK": "with problem"));
  return success;
}

// Picks an object.
moveit::core::MoveItErrorCode AutonomousTidyUp::RealPick(std::string object_name_grasp, std::vector<moveit_grasps::GraspCandidatePtr> valid_grasp_candidate,
              std::vector<moveit_msgs::MotionPlanResponse> pre_approach_plan, int available_grasps, int n_object)
{
  group_arm_torso->setPlanningTime(5.0*60.0);

  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(available_grasps);

  // Configurates grasps for a zero retreat and open gripper to use the GripperGrasper after the picking.
  for (int i = 0; i < available_grasps; i++)
  {  
    grasps[i] = valid_grasp_candidate[i]->grasp_;

    grasps[i].pre_grasp_posture.points[0].time_from_start = ros::Duration(0.5);
    grasps[i].grasp_posture.points[0].time_from_start = ros::Duration(0.5);

    // grasps[i].post_grasp_retreat.min_distance = 0.1;

    openGripper(grasps[i].pre_grasp_posture);
    openGripper(grasps[i].grasp_posture);

    grasps[i].post_grasp_retreat.direction.vector.x = 0.0;
    grasps[i].post_grasp_retreat.direction.vector.y = 0.0;
    grasps[i].post_grasp_retreat.direction.vector.z = 0.0;
    grasps[i].post_grasp_retreat.min_distance = 0.0;
    grasps[i].post_grasp_retreat.desired_distance = 0.0;
  }

  ROS_WARN("PICK auto STARTED");
  int i = 0;

  moveit::core::MoveItErrorCode output = moveit_msgs::MoveItErrorCodes::FAILURE;
  successful_grasp = false;
  bool success = false;
  // Tries the whole picking process several times until success.
  do
  {
    geometry_msgs::Pose initial_object_pose = GetObjectPose(object_name_grasp);
    setACMFingerEntry(object_name_grasp, true);
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    setACMFingerEntry(object_name_grasp, true);
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    ros::Duration(1.0).sleep();

    // PICKING
    group_arm_torso->setStartStateToCurrentState();
    output = group_arm_torso->pick(object_name_grasp.c_str(), grasps);
    setACMFingerEntry(object_name_grasp, true);

    ros::spinOnce();
    std::vector<std::string> chose_object;
    chose_object.resize(1);
    chose_object[0] = object_name_grasp;

    std::map<std::string,moveit_msgs::CollisionObject> map_collision_objects;
    map_collision_objects = planning_scene_interface.getObjects(chose_object);

    bool movement_result = false;

    if (output == moveit_msgs::MoveItErrorCodes::SUCCESS && map_collision_objects.size() == 0)
    {
      // Calling GripperGrasper.
      bool result = grasp_client.call(empty_srv);
      if (!result) {
        ROS_ERROR("Failed to call service grasp_client");
        successful_grasp = false;
      }else
      {
        ros::Duration(1.0).sleep();
        // Wait for the fingers to stop moving
        while (ros::ok() && (gripper_state.desired.velocities[0] != 0.0 or gripper_state.desired.velocities[1] != 0.0))
        {
          ros::spinOnce();
          ros::Duration(1.0).sleep();
        }
        ROS_WARN("GRASP CLIENT.......................");
        
        float dist_fingers = gripper_state.desired.positions[0] + gripper_state.desired.positions[1];
        #if SIMULATION
        if (dist_fingers < 0.000001)
        #else
        if (dist_fingers <= 0.003)// The real robot's fingers never close fully
        #endif
        {
          ROS_WARN("GRIPPED NOTHING.......................");
          successful_grasp = false;
          ChangeObjectState(n_object, StateNotGripped);
        }else
        {
          ROS_WARN("GRIPPED SOMETHING.......................");
          successful_grasp = true;
          ChangeObjectState(n_object, StateHolding);

          #if SIMULATION
            AttachGazebo();   
            ros::Duration(2.0).sleep();
          #endif
        }
      }

      if (!successful_grasp)
      {
        holding_object = false;
        ros::spinOnce();
        ros::Duration(2.0).sleep();// to make sure there are no holdings left in the queue
        ros::spinOnce();
        holding_object = false;
        ros::spinOnce();
        ros::Duration(2.0).sleep();
        ros::spinOnce();
        holding_object = false;
        ros::spinOnce();
        ros::Duration(2.0).sleep();
        ros::spinOnce();
        setACMFingerEntry(object_name_grasp, true);
         while (!MovementOpenEndEffector() && ros::ok())
        {
          ROS_WARN("Try to open the gripper");
          planning_scene_interface.removeCollisionObjects(std::vector<std::string>{object_name_grasp});
          ros::Duration(1.0).sleep();
          ros::spinOnce();
        };
        group_arm_torso->detachObject();
        ros::Duration(1.0).sleep();
        ros::spinOnce();
        group_arm_torso->detachObject();
        ros::Duration(1.0).sleep();
        ros::spinOnce();
        group_arm_torso->detachObject();
        ros::Duration(1.0).sleep();
        ros::spinOnce();
        setACMFingerEntry(object_name_grasp, true);

        // Goes back to default arm/torso pose.
        int trys = 0;
        do
        {
          movement_result = MovementHome();
          ros::Duration(1.0).sleep();
          ros::spinOnce();
          trys++;
        } while (trys < MAX_TRYS && movement_result == false && ros::ok());

        if (!movement_result)
        {
          ROS_ERROR("MovementHome with problem");
          WaitToContinueExecution();
          MovementHome();
        }
        output = autonomous_tidy_up::AutonomousTidyUp::NO_REAL_OBJECT;
        break;
      }
    } else if (output != output.SUCCESS)
    {
      if (output == output.PLANNING_FAILED)
      {
        ChangeObjectState(n_object, StateInvalidPlan);
      }
      if (output == output.CONTROL_FAILED or output == output.FAILURE)
      {
        ChangeObjectState(n_object, StateInvalid);
      } else
      {
        ROS_WARN("UNSUCCESSFUL output with result %d", output.val);
      }
    }
    setACMFingerEntry(object_name_grasp, true);

    ros::Duration(5.0).sleep();

    // Lifts the gripper or the torso to avoid touching the floor with the holding object.
    do
    {
      movement_result = LiftEndEffector(0.05);
      if(!movement_result)
        movement_result = LiftTorsoJoint(0.34);
      ros::Duration(0.25).sleep();
      ros::spinOnce();
    } while (!movement_result && ros::ok());
    
    setACMFingerEntry(object_name_grasp, true);
    ROS_WARN("PICK auto %d with result %d", i, output.val);
    if(output == moveit_msgs::MoveItErrorCodes::SUCCESS && successful_grasp) success = true;
    i++;
    ros::Duration(6.0).sleep();

    if (!FindObject(object_name_grasp))
    {
      break;
    }    
  }while (ros::ok() && (i < MIN_TRYS_NUMBER? true : (!success &&  i < MAX_TRYS_NUMBER)) && FindObject(object_name_grasp));
  
  ROS_WARN("PICK auto FINISHED");
  return output;
}

// Places an object on a table top.
moveit::core::MoveItErrorCode AutonomousTidyUp::placeOnTheTableTop(std::string object_name, std::string table_name, moveit_msgs::CollisionObject object_to_place, int n_object)
{
  std::vector<moveit_msgs::PlaceLocation> place_location;

  double step = 0.05; // Distance possible possible place poses.

  moveit_msgs::CollisionObject object_table = GetObject(table_name);

  ROS_WARN("%s on .... %s", object_table.id.c_str(), object_table.header.frame_id.c_str());

  // CONSIDER TABLE ROTATION
  double offset_vector[2], offset_vector_new_line[2], p_min[2], p_max[2], vector_diag_inf[2], p_init_line[2];
  double size_box_x, size_box_y;

  double max_dim_object = std::max(object_to_place.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X], object_to_place.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y]);

  // Considers a safety margin to avoid placing on the edge.
  float theta = constrainAngle(tf::getYaw(object_table.primitive_poses[0].orientation));
  size_box_x = object_table.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] - max_dim_object - 2.0 * TABLE_EXPANSION - 0.2;
  size_box_y = object_table.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] - max_dim_object - 2.0 * TABLE_EXPANSION - 0.2;

  double ang_diag_inf;

  ang_diag_inf = M_PI - atan2(size_box_y/2.0, size_box_x/2.0);

  vector_diag_inf[0] = (sqrt(size_box_x*size_box_x + size_box_y*size_box_y) / 2.0) * cos(theta + ang_diag_inf);
  vector_diag_inf[1] = (sqrt(size_box_x*size_box_x + size_box_y*size_box_y) / 2.0) * sin(theta + ang_diag_inf);

  p_min[0] = object_table.primitive_poses[0].position.x + vector_diag_inf[0];
  p_min[1] = object_table.primitive_poses[0].position.y + vector_diag_inf[1];
  p_max[0] = object_table.primitive_poses[0].position.x - vector_diag_inf[0];
  p_max[1] = object_table.primitive_poses[0].position.y - vector_diag_inf[1];


  offset_vector[0] = step * cos(theta - M_PI/2.0);
  offset_vector[1] = step * sin(theta - M_PI/2.0);

  offset_vector_new_line[0] = step * cos(theta);
  offset_vector_new_line[1] = step * sin(theta);

  ROS_WARN("BEGIN PLACE");

  int ang_variations = 2;
  int tam = int(size_box_x/step) * int(size_box_y/step) * ang_variations * ang_variations * ang_variations;

  if (tam == 0)
  {
    return autonomous_tidy_up::AutonomousTidyUp::NO_PLACE_POSITION;
  }
  
  place_location.resize(tam);
  double x, y, z;

  z = 0.03 + SIMULATIONvariation_z + object_table.primitive_poses[0].position.z + (object_table.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] / 2.0)+ (object_to_place.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] / 2.0);
  x = p_min[0];
  y = p_min[1];

  ROS_WARN("Ang %f deg, Placing from x: %f y: %f --- until x: %f y: %f --- using z: %f", theta * (180.0/M_PI), x, y, p_max[0], p_max[1], z);
  std::vector<geometry_msgs::Pose> all_poses;
  
  // Calculates possible place poses on table top.
  int i = 0;
  for (int i_x = 0; i_x < int(size_box_x/step) && i < tam; i_x++)
  {
    p_init_line[0] = x;
    p_init_line[1] = y;
    for (int i_y = 0; i_y < int(size_box_y/step) && i < tam; i_y++)
    {
      for (int i_ang = 0; i_ang < ang_variations && i < tam; i_ang++)
      {
      for (int i_roll = 0; i_roll < ang_variations && i < tam; i_roll++)
      {
      for (int i_pitch = 0; i_pitch < ang_variations && i < tam; i_pitch++)
      {
        place_location[i].place_pose.header.frame_id = "base_footprint";
        place_location[i].place_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(float(i_roll)*M_PI/2.0, float(i_pitch)*M_PI/2.0, theta + float(i_ang)*M_PI/2.0);
        place_location[i].place_pose.pose.position.x = x;
        place_location[i].place_pose.pose.position.y = y;
        place_location[i].place_pose.pose.position.z = z;
        all_poses.push_back(place_location[i].place_pose.pose);

        // ROS_WARN("place %d OBJECT at x: %f .. y: %f .. z: %f .. frame %s", i, place_location[i].place_pose.pose.position.x, place_location[i].place_pose.pose.position.y, place_location[i].place_pose.pose.position.z, place_location[i].place_pose.header.frame_id.c_str());
        
        // Setting pre-place approach.
        place_location[i].pre_place_approach.direction.header.frame_id = "base_footprint";
        place_location[i].pre_place_approach.direction.vector.z = -1.0;
        place_location[i].pre_place_approach.min_distance = 0.01;
        place_location[i].pre_place_approach.desired_distance = 0.10;

        // Setting post-grasp retreat.
        place_location[i].post_place_retreat.direction.header.frame_id = "base_footprint";
        place_location[i].post_place_retreat.direction.vector.z = 1.0;
        place_location[i].post_place_retreat.min_distance = 0.15;
        place_location[i].post_place_retreat.desired_distance = 0.20;

        openGripper(place_location[i].post_place_posture);
        i++;
      }
      }
      }

      x += offset_vector[0];
      y += offset_vector[1];
    }
    x = p_init_line[0] + offset_vector_new_line[0];
    y = p_init_line[1] + offset_vector_new_line[1];
  }


  place_poses_msg.header.frame_id = "base_footprint";
  place_poses_msg.header.stamp = ros::Time::now();
  place_poses_msg.poses = all_poses;
  place_poses_pub.publish(place_poses_msg);

  // Set support surface as table.
  group_arm_torso->setSupportSurfaceName(table_name);

  group_arm_torso->setPlanningTime(5.0*60.0);
  group_arm_torso->setMaxVelocityScalingFactor(0.25);// It's holding the object, go slower
  setACMFingerEntry(object_name, true);

  // Calls place to place the object using the place locations given.
  moveit::core::MoveItErrorCode resul;
  group_arm_torso->setStartStateToCurrentState();
  resul = group_arm_torso->place(object_name, place_location);

  ROS_ERROR("RESULT: %d........of %d positions............", resul, tam);

  if (resul.val != resul.SUCCESS)
  {
    WaitToContinueExecution();
    #if SIMULATION
    if (resul.val == resul.CONTROL_FAILED or resul.val == resul.TIMED_OUT)
    {
      AttachGazebo((objects_on_the_floor_recevied[n_object].manipulation_tries / 2) != 0); /// try 0, false, glue left
      DetachGazebo((objects_on_the_floor_recevied[n_object].manipulation_tries / 2) == 0); /// try 0, true, unglue right

      bool result = grasp_client.call(empty_srv);
      if (!result) {
        ROS_ERROR("Failed to call service grasp_client");
        successful_grasp = false;
      }else
      {

        ros::Duration(1.0).sleep();
        // Wait for the fingers to stop moving
        while (ros::ok() && (gripper_state.desired.velocities[0] != 0.0 or gripper_state.desired.velocities[1] != 0.0))
        {
          ros::spinOnce();
          ros::Duration(1.0).sleep();
        }
        ROS_WARN("GRASP CLIENT.......................");
        
        float dist_fingers = gripper_state.desired.positions[0] + gripper_state.desired.positions[1];
        if (dist_fingers < 0.000001)
        {
          ROS_WARN("GRIPPED NOTHING 2.......................");
        }else
        {
          ROS_WARN("GRIPPED SOMETHING 2.......................");
        }

        ROS_WARN("Second attempt at placement.......................");
        group_arm_torso->setStartStateToCurrentState();
        resul = group_arm_torso->place(object_name, place_location);
      }
    }
    #endif
  } else
  {
    holding_object = false;
    MovementOpenEndEffector();
    MovementOpenEndEffector();
    MovementOpenEndEffector();
  }  

  ROS_WARN("PLACE of %s FINISHED", object_name.c_str());
  group_arm_torso->setMaxVelocityScalingFactor(1.0);

  return resul;
}

///////////////////////////////////////
//                                   //
//             Semantics             //
//                                   //
///////////////////////////////////////

// Checks if an object is on the planning scene.
bool AutonomousTidyUp::FindObject(std::string object_name){

  auto all_objects_names = planning_scene_interface.getKnownObjectNames();
    
  if (std::find(all_objects_names.begin(), all_objects_names.end(), object_name) != all_objects_names.end())
  {
    return true;
  }else
  {
    ROS_WARN("NO OBJECT: %s", object_name.c_str());
    return false;
  }
}

// Gets the object from the planning scene.
moveit_msgs::CollisionObject AutonomousTidyUp::GetObject(std::string object_name)
{
  std::vector<std::string> objects;
  objects.resize(1);
  objects[0] = object_name;

  std::map< std::string, moveit_msgs::CollisionObject> collision_object;

  collision_object = planning_scene_interface.getObjects(objects);

  ROS_WARN("Get object: %s", object_name.c_str());

  return collision_object[object_name];
}

// Gets the object pose from the planning scene.
geometry_msgs::Pose AutonomousTidyUp::GetObjectPose(std::string object_name)
{
  std::vector<std::string> objects;
  objects.resize(1);
  objects[0] = object_name;

  std::map< std::string,geometry_msgs::Pose> object_pose;

  object_pose = planning_scene_interface.getObjectPoses(objects);

  ROS_WARN("Find object pose: %s", object_name.c_str());
  pose_inicial_obj = object_pose[object_name];

  return object_pose[object_name];
}

// Starts the semantic map with map and global_costmap data.
bool AutonomousTidyUp::startSemanticMap(){

  do
  {
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    ROS_WARN("Waiting map or global costmap...%d %d", map_size, global_costmap_size);
  }while (ros::ok() && map_size == 0 or global_costmap_size == 0);
  
  ROS_WARN("Map or global costmap...%d %d", map_size, global_costmap_size);
  semantic_map.resize(map_size);
  float x, y;
  int i_globalcostmap, j_globalcostmap, index_map, index_globalcostmap;
  for (int i = 0; i < map_width_; i++)
  {
    for (int j = 0; j < map_height_; j++)
    {
      index_map = MapIndicesToVectorIndex(i,j);
      semantic_map[index_map].ORIGINAL_MAP = map_recevied.data[index_map];
      std::tie(x, y) = transformCoordinateMapToOdom(i, j);

      if (!InsideGlobalcostmap(x, y))
      {
        continue;
      }
      std::tie(i_globalcostmap, j_globalcostmap) = transformCoordinateOdomToGlobalcostmap(x, y);
      index_globalcostmap = GlobalcostmapIndicesToVectorIndex(i_globalcostmap, j_globalcostmap);          

      if (global_costmap.data[index_globalcostmap] > 90 or map_recevied.data[index_map] > 90)
      {
        semantic_map[index_map].CELL_TYPE = kObstacle;
      } else if (global_costmap.data[index_globalcostmap] < 0 or map_recevied.data[index_map] < 0)
      {
        semantic_map[index_map].CELL_TYPE = kUnknown;
      } else if (semantic_map[index_map].CELL_TYPE != kCostmapObstacle and 
                  semantic_map[index_map].CELL_TYPE != kExpandedObstacle and 
                  semantic_map[index_map].CELL_TYPE != kInvalidBasePose)
      {
        semantic_map[index_map].CELL_TYPE = kFree;
      }
    }
  }
  ROS_WARN("semantic_map size %d", map_size);
  
  return true;
}

// Updates the semantic map with local_costmap data.
void AutonomousTidyUp::UpdateSemanticMap()
{
  // ROS_WARN("UpdateSemanticMap");
  int i_map, j_map;
  float x_test, y_test;

  if (!TRYtransformCoordinateLocalcostmapToOdom())
  {
    return;
  }  

  for (int i = 0; i < localcostmap_width_; i++)
  {
    for (int j = 0; j < localcostmap_height_; j++)
    {
      std::tie(x_test, y_test) = transformCoordinateLocalcostmapToOdom(i, j);
      if (!InsideMap(x_test, y_test)) continue;
      
      std::tie(i_map, j_map) = transformCoordinateLocalcostmapToMap(i, j);
      int index = MapIndicesToVectorIndex(i_map, j_map);
      semantic_map[index].VISITED = true;

      if (semantic_map[index].CELL_TYPE == kFree
       or semantic_map[index].CELL_TYPE == kCostmapObstacle
       or semantic_map[index].CELL_TYPE == kExpandedObstacle)
      {
        if (local_costmap.data[LocalcostmapIndicesToVectorIndex(i, j)] > 99) // used only to check the pose of detected objects
        {
          semantic_map[index].CELL_TYPE = kCostmapObstacle;
        } else if (local_costmap.data[LocalcostmapIndicesToVectorIndex(i, j)] > 60)
        {
          semantic_map[index].CELL_TYPE = kExpandedObstacle;
        } else if (local_costmap.data[LocalcostmapIndicesToVectorIndex(i, j)] >= 0)
        {
          semantic_map[index].CELL_TYPE = kFree;
        } else
        {
          semantic_map[index].CELL_TYPE = kUnknown;
        }
      }
    }
  }

}

// Adds tables and objects, that are stored in lists, to the semantic map.
void AutonomousTidyUp::AddObjectsAndTablesReceviedToSemanticMap()
{
    int tables_added = 0;
    for (int index = 0; index < tables_recevied.size(); index++)
    {
      // if (tables_recevied[index].see_many_times)
      // { 
        tables_recevied[index].object.id.resize(7);
        if (index < 10)
        {
          tables_recevied[index].object.id[5] = '0';
        } else
        {
          tables_recevied[index].object.id[5] = '0' + int(index/10.0);
        }        
        tables_recevied[index].object.id[6] = '0' + int(index%10);        
        ROS_WARN("...%s", tables_recevied[index].object.id.c_str());
        if (!ValidTable(index))
        {
          continue;
        }
        if (AddInfoToSemanticMap(tables_recevied[index].object, ObjectTable, 0.05, 0.25))
        {
          tables_added++;
        }
      // }      
    }
    ROS_ERROR("%d tables on map, %d detected in total", tables_added, tables_recevied.size());

    int obj_added = 0;
    for (int index = 0; index < objects_on_the_floor_recevied.size(); index++)
    {
        objects_on_the_floor_recevied[index].object.id.resize(15);
        if (index < 10)
        {
          objects_on_the_floor_recevied[index].object.id[10] = '0';
        } else
        {
          objects_on_the_floor_recevied[index].object.id[10] = '0' + int(index/10.0);
        }        
        objects_on_the_floor_recevied[index].object.id[11] = '0' + int(index%10);
        if (!ValidObjectOnFloor(index))
        {
          continue;
        }
        AddInfoToSemanticMap(objects_on_the_floor_recevied[index].object, ObjectOnFloor, 0.05, 0.00);
        ROS_WARN("...%s", objects_on_the_floor_recevied[index].object.id.c_str());
        obj_added++;
    }
    ROS_ERROR("%d objects on floor on map, %d detected in total", obj_added, objects_on_the_floor_recevied.size());
}

// Removes tables and objects, that are stored in lists, from the semantic map.
void AutonomousTidyUp::RemoveObjectsAndTablesReceviedToSemanticMap()
{
    for (int index = 0; index < tables_recevied.size(); index++)
    {
      // if (tables_recevied[index].see_many_times)
      // {        
        AddInfoToSemanticMap(tables_recevied[index].object, NoObject, 0.075, 0.25);
      // }      
    }

    for (int index = 0; index < objects_on_the_floor_recevied.size(); index++)
    {
      if (!ValidObjectOnFloor(index))
      {
        continue;
      }  
      AddInfoToSemanticMap(objects_on_the_floor_recevied[index].object, NoObject, 0.05, 0.00);  
    }
}

// Marks as placed object and cleans the previous pose on the semantic map.
void AutonomousTidyUp::MarkAsPlacedObject(moveit_msgs::CollisionObject obj, int n_object)
{
  int old_size = objects_on_the_floor_recevied.size();

  AddInfoToSemanticMap(obj, NoObject, 0.075, 0.00, true);

  objects_on_the_floor_recevied[n_object].state = StatePlaced;
  ROS_WARN("--------->>>>>>>>Object placed");
}

// Changes the object state and increments the manipulation tries. If the new state is placed, cleans the previous pose on the semantic map.
void AutonomousTidyUp::ChangeObjectState(int n_object, ObjectState state)
{
  objects_on_the_floor_recevied[n_object].state = state;

  ROS_WARN("Object %s change to state", objects_on_the_floor_recevied[n_object].object.id.c_str());

  switch (objects_on_the_floor_recevied[n_object].state)
  {
    case StateOnFloor:
      ROS_WARN(" on floor");
      break;
    case StateHolding:
      ROS_WARN(" holding");
      break;
    case StateInvalid:
      ROS_WARN(" invalid");
      objects_on_the_floor_recevied[n_object].manipulation_tries += 1;
      break;
    case StateInvalidGrasp:
      ROS_WARN(" invalid grasp");
      objects_on_the_floor_recevied[n_object].manipulation_tries += 1;
      break;
    case StateInvalidPlan:
      ROS_WARN(" invalid plan");
      objects_on_the_floor_recevied[n_object].manipulation_tries += 1;
      break;
    case StateNotGripped:
      ROS_WARN(" not gripped");
      objects_on_the_floor_recevied[n_object].manipulation_tries += 1;
      break;
    case StatePlaceInvalid:
      ROS_WARN(" place invalid");
      objects_on_the_floor_recevied[n_object].manipulation_tries += 1;
      MarkAsExpandedObstacle(robot_pose.position.x, robot_pose.position.y);
      break;
    case StatePlaceFail:
      ROS_WARN(" not placed");
      objects_on_the_floor_recevied[n_object].manipulation_tries += 1;
      break;
    case StateNotAcessible:
      ROS_WARN(" not acessible");
      break;
    case StatePlaced:
      ROS_WARN(" PLACED");
      break;
    
    default:
      ROS_WARN(" invalid state %d", objects_on_the_floor_recevied[n_object].state);
      break;
  }

  if (state == StatePlaced)
  {
    AddInfoToSemanticMap(objects_on_the_floor_recevied[n_object].object, NoObject, 0.075, 0.00, true);
  }  
}

// Changes the table state.
void AutonomousTidyUp::ChangeTableState(int n_table, ObjectState state)
{
  tables_recevied[n_table].state = state;

  ROS_WARN("Table %s change to state", tables_recevied[n_table].object.id.c_str());

  switch (tables_recevied[n_table].state)
  {
    case StateTableTooSmall:
      ROS_WARN(" too small");
      break;
    default:
      ROS_WARN(" invalid state");
      break;
  }
}

// Modifies the semantic map to add new object/table information.
bool AutonomousTidyUp::AddInfoToSemanticMap(moveit_msgs::CollisionObject object, ObjectType type, float size_tolerance, float size_min, bool mark_as_unvised)
{
  if(object.id.length() == 0) return false;

  float angle = ComputeYaw(object.primitive_poses[0].orientation);

  float size_x = object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X];
  float size_y = object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
  float size_z = object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z];

  if (size_x < size_min or size_y < size_min or size_z < size_min)
  {
    return false;
  }    

  int i_map, j_map;

  float max_x, max_y, min_x, min_y;
  // diag is the vector that starts in the object center and points to the vectice on the first quad
  float ang_diag = constrainAngle(atan2(size_y, size_x));
  float mod_diag = sqrt(size_x*size_x + size_y*size_y) / 2.0;
  float mod_object_pose = sqrt(object.primitive_poses[0].position.x*object.primitive_poses[0].position.x + object.primitive_poses[0].position.y*object.primitive_poses[0].position.y);
  float ang_object_pose = constrainAngle(atan2(object.primitive_poses[0].position.y, object.primitive_poses[0].position.x));

  float angle_robot = ComputeYaw(robot_pose.orientation);

  float center_x, center_y;

  if (object.header.frame_id.compare("map") == 0)
  {
    center_x = object.primitive_poses[0].position.x;
    center_y = object.primitive_poses[0].position.y;
    angle_robot = 0.0;
  } else {
    center_x = robot_pose.position.x + (mod_object_pose * cos(constrainAngle(ang_object_pose + angle_robot)));
    center_y = robot_pose.position.y + (mod_object_pose * sin(constrainAngle(ang_object_pose + angle_robot)));
  }
    
  float vertices[4][2];

  vertices[0][0] = center_x + (mod_diag * cos(constrainAngle(ang_diag + angle + angle_robot)));
  vertices[0][1] = center_y + (mod_diag * sin(constrainAngle(ang_diag + angle + angle_robot)));
  vertices[1][0] = center_x + (mod_diag * cos(constrainAngle(-ang_diag + angle + angle_robot)));
  vertices[1][1] = center_y + (mod_diag * sin(constrainAngle(-ang_diag + angle + angle_robot)));
  vertices[2][0] = center_x - (mod_diag * cos(constrainAngle(ang_diag + angle + angle_robot)));
  vertices[2][1] = center_y - (mod_diag * sin(constrainAngle(ang_diag + angle + angle_robot)));
  vertices[3][0] = center_x - (mod_diag * cos(constrainAngle(-ang_diag + angle + angle_robot)));
  vertices[3][1] = center_y - (mod_diag * sin(constrainAngle(-ang_diag + angle + angle_robot)));

  max_x = vertices[0][0];
  max_y = vertices[0][1];
  min_x = vertices[0][0];
  min_y = vertices[0][1];
    
  for (int it = 1; it < 4; it++)
  {
    max_x = std::fmax(max_x, vertices[it][0]);
    max_y = std::fmax(max_y, vertices[it][1]);
    min_x = std::fmin(min_x, vertices[it][0]);
    min_y = std::fmin(min_y, vertices[it][1]);
  }    

  // to ensure that it does not fall off the map
  max_x = std::fmin(max_x, map_origin_x_ +  map_width_ * map_resolution_);
  max_y = std::fmin(max_y, map_origin_y_ + map_height_ * map_resolution_);
  min_x = std::fmax(min_x, map_origin_x_ -  map_width_ * map_resolution_);
  min_y = std::fmax(min_y, map_origin_y_ - map_height_ * map_resolution_);

  int i_map_min, j_map_min, i_map_max, j_map_max;
  std::tie(i_map_min, j_map_min) = transformCoordinateOdomToMap(min_x, min_y);
  std::tie(i_map_max, j_map_max) = transformCoordinateOdomToMap(max_x, max_y);

  // ROS_WARN("%s added....i from %d until %d; j from %d until %d", object.id.c_str(), i_map_min, i_map_max, j_map_min, j_map_max);

  int total_n_cells = 0, n_cells = 0;
  n_cells = 0;
  float x, y;
  for (int i_map = std::min(i_map_min, i_map_max); i_map < std::max(i_map_min, i_map_max); i_map++)
  {
    for (int j_map = std::min(j_map_min, j_map_max); j_map < std::max(j_map_min, j_map_max); j_map++)
    {

      std::tie(x, y) = transformCoordinateMapToOdom(i_map, j_map);
      if (!InsideMap(x, y)) continue;

      float dist[4];
      // calculates the distance from (x,y) to the object's side between vertices[it] and [it+1]
      for (int it = 0; it < 3; it++)
      {
        dist[it] = fabs((vertices[it+1][0] - vertices[it][0])*(vertices[it][1] - y) - (vertices[it][0] - x)*(vertices[it+1][1] - vertices[it][1]))
                   / sqrt( pow(vertices[it+1][0] - vertices[it][0], 2.0) + pow(vertices[it+1][1] - vertices[it][1], 2.0));
      }
      // with vertices[3] and vertices[0]
      dist[3] = fabs((vertices[0][0] - vertices[3][0])*(vertices[3][1] - y) - (vertices[3][0] - x)*(vertices[0][1] - vertices[3][1]))
                / sqrt( pow(vertices[0][0] - vertices[3][0], 2.0) + pow(vertices[0][1] - vertices[3][1], 2.0));

      // For the point to be inside the object on the map, the sum of the distances from the point to the opposite sides must be equal to the side of the object.
      // Tolerance added.
      if (dist[0] + dist[2] <= size_x + 0.05 && dist[1] + dist[3] <= size_y + 0.05)
      {
        semantic_map[MapIndicesToVectorIndex(i_map,j_map)].OBJECT_TYPE = type;
        semantic_map[MapIndicesToVectorIndex(i_map,j_map)].NAME = object.id;
        if (mark_as_unvised)
        {
          semantic_map[MapIndicesToVectorIndex(i_map,j_map)].ROBOT_VISITED = false;
          semantic_map[MapIndicesToVectorIndex(i_map,j_map)].CELL_TYPE = kFree;
        }
        
        n_cells++;
      } else
      {
        // semantic_map[MapIndicesToVectorIndex(i_map,j_map)].CELL_TYPE = kNOTTable;
      }
        
      total_n_cells++;
    }
  }
  if (type != NoObject) ROS_WARN("SIZE X: %f and Y: %f at x: %f and y: %f ang: %f deg", size_x, size_y, center_x, center_y, RAD2DEG(angle));
  // if (type != NoObject) ROS_WARN("%s added....%d cells de %d", object.id.c_str(), n_cells, total_n_cells);

  int i_center, j_center, index;

  if (n_cells < 9)
  {
    // Too few object cells added, add at least 9.
    std::tie(i_center, j_center) = transformCoordinateOdomToMap(center_x, center_y);

    if(InsideMap(center_x - map_resolution_, center_y - map_resolution_) && InsideMap(center_x + map_resolution_, center_y + map_resolution_))
    {
      for (int i = i_center - 1; i <= i_center + 1; i++)
      {
        for (int j = j_center - 1; j <= j_center + 1; j++)
        {   
    
          index = MapIndicesToVectorIndex(i, j);
          semantic_map[index].OBJECT_TYPE = type;
          semantic_map[index].NAME = object.id;
          if (mark_as_unvised)
          {
            semantic_map[index].ROBOT_VISITED = false;
            semantic_map[index].CELL_TYPE = kFree;
          }
        }
      }
      if (type != NoObject) ROS_WARN("%s added....single cell", object.id.c_str());
    } else
    {
      if (type != NoObject) ROS_WARN("%s outside map....single cell", object.id.c_str());
    }
  }

  if (type == ObjectOnFloor) // Add ExpandedObstacle near object on the floor.
  {
    float ray = 0.3;
    int ray_cell = ray / map_resolution_;
    int pot2_ray_cell = pow(ray_cell, 2.0);
    if (!InsideMap(center_x, center_y))
    {
      return false;
    }
    int i_min, i_max, j_min, j_max;
    std::tie(i_center, j_center) = transformCoordinateOdomToMap(center_x, center_y);

    i_min = std::max(0, i_center - ray_cell);
    j_min = std::max(0, j_center - ray_cell);
    i_max = std::min(int(map_width_ - 1),  i_center + ray_cell);
    j_max = std::min(int(map_height_ - 1), j_center + ray_cell);

    for (int i = i_min; i <= i_max; i++)
    {
      for (int j = j_min; j <= j_max; j++)
      {
        if ((pow(i_center - i, 2.0) + pow(j_center - j, 2.0)) < pot2_ray_cell)
        {
          index = MapIndicesToVectorIndex(i, j);
          if (semantic_map[index].CELL_TYPE == kFree)
          {
            semantic_map[index].CELL_TYPE = kExpandedObstacle;
          }
        }        
      }
    }
  }
    
  if (type == NoObject) // Clears for navigation strategies.
  {
    float ray;
    if (mark_as_unvised)
    {
      ray = 0.6;
    } else
    {
      ray = 0.3;
    }   
    
    int ray_cell = ray / map_resolution_;
    int pot2_ray_cell = pow(ray_cell, 2.0);
    if (!InsideMap(center_x, center_y))
    {
      return false;
    }
    int i_min, i_max, j_min, j_max;
    std::tie(i_center, j_center) = transformCoordinateOdomToMap(center_x, center_y);

    i_min = std::max(0, i_center - ray_cell);
    j_min = std::max(0, j_center - ray_cell);
    i_max = std::min(int(map_width_ - 1),  i_center + ray_cell);
    j_max = std::min(int(map_height_ - 1), j_center + ray_cell);

    for (int i = i_min; i <= i_max; i++)
    {
      for (int j = j_min; j <= j_max; j++)
      {
        if ((pow(i_center - i, 2.0) + pow(j_center - j, 2.0)) < pot2_ray_cell)
        {
          index = MapIndicesToVectorIndex(i, j);
          if (mark_as_unvised)
          {
            semantic_map[index].CELL_TYPE = kFree;
            semantic_map[index].ROBOT_VISITED = false;
            #if STRATEGY_VORONOI
            semantic_map[index].VORONOI_VISITED = false;
            #endif
            #if STRATEGY_GRID
            semantic_map[index].GRID_VISITED = false;
            #endif
          } else if (semantic_map[index].CELL_TYPE == kExpandedObstacle)
          {
            semantic_map[index].CELL_TYPE = kFree;
          }
        }        
      }
    }
  }

  return true;      
}

// Compares the new_object with the list to find similar objects near it.
std::vector<int> AutonomousTidyUp::GetSimilarObjects(moveit_msgs::CollisionObject new_object, ObjectType type, float tolerance)
{
  std::vector<int> indexs;

  float angle = ComputeYaw(new_object.primitive_poses[0].orientation);

  float size_x = new_object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X];
  float size_y = new_object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
  float size_z = new_object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z];

  float ang_diag = constrainAngle(atan2(size_y, size_x));
  float mod_diag = sqrt(size_x*size_x + size_y*size_y) / 2.0;

  float center_x, center_y;

  center_x = new_object.primitive_poses[0].position.x;
  center_y = new_object.primitive_poses[0].position.y;
    
  float vertices[4][2];

  vertices[0][0] = center_x + (mod_diag * cos(constrainAngle(ang_diag + angle)));
  vertices[0][1] = center_y + (mod_diag * sin(constrainAngle(ang_diag + angle)));
  vertices[1][0] = center_x + (mod_diag * cos(constrainAngle(-ang_diag + angle)));
  vertices[1][1] = center_y + (mod_diag * sin(constrainAngle(-ang_diag + angle)));
  vertices[2][0] = center_x - (mod_diag * cos(constrainAngle(ang_diag + angle)));
  vertices[2][1] = center_y - (mod_diag * sin(constrainAngle(ang_diag + angle)));
  vertices[3][0] = center_x - (mod_diag * cos(constrainAngle(-ang_diag + angle)));
  vertices[3][1] = center_y - (mod_diag * sin(constrainAngle(-ang_diag + angle)));

  std::vector<ObjectDetected>* objects_vector;

  if (type == ObjectTable)
  {
    objects_vector = &tables_recevied;
  } else if (type == ObjectOnFloor)
  {
    objects_vector = &objects_on_the_floor_recevied;
  } else
  {
    return indexs;
  }
  
  // Compares new_object with the objects on the vector.
  for (int i = 0; i < objects_vector->size(); i++)
  {
    moveit_msgs::CollisionObject object = (*objects_vector)[i].object;
    if (type == ObjectOnFloor && !ValidObjectOnFloor(i))
    {
      continue;
    }
    if (type == ObjectTable && !ValidTable(i)) continue;
    
    bool inside = false;
    if (object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] * object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] < size_x * size_y)
    { /// new_object is bigger than object, so, verifiies if object is inside new_object.
      float x = object.primitive_poses[0].position.x;
      float y = object.primitive_poses[0].position.y;

      if (!InsideMap(x, y)) continue;

      // calculates the distance from object center to the new_object's side between vertices[it] and [it+1]
      float dist[4];
      for (int it = 0; it < 3; it++)
      {
        dist[it] = fabs((vertices[it+1][0] - vertices[it][0])*(vertices[it][1] - y) - (vertices[it][0] - x)*(vertices[it+1][1] - vertices[it][1]))
                   / sqrt( pow(vertices[it+1][0] - vertices[it][0], 2.0) + pow(vertices[it+1][1] - vertices[it][1], 2.0));
      }
      // with vertices[3] and vertices[0]
      dist[3] = fabs((vertices[0][0] - vertices[3][0])*(vertices[3][1] - y) - (vertices[3][0] - x)*(vertices[0][1] - vertices[3][1]))
                / sqrt( pow(vertices[0][0] - vertices[3][0], 2.0) + pow(vertices[0][1] - vertices[3][1], 2.0));

      // For the object center point to be inside new_object,
      // the sum of the distances from the point to the opposite sides of new_object must be equal to the side of new_object.
      // Tolerance added.
      inside = (dist[0] + dist[2] <= size_x + tolerance && dist[1] + dist[3] <= size_y + tolerance);    

    } else
    { /// object is bigger than new_object, so, verifiies if new_object is inside object.
      
      float angle_object = ComputeYaw(object.primitive_poses[0].orientation);

      float size_x_object = object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X];
      float size_y_object = object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
      float size_z_object = object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z];

      float ang_diag_object = constrainAngle(atan2(size_y_object, size_x_object));
      float mod_diag_object = sqrt(size_x_object*size_x_object + size_y_object*size_y_object) / 2.0;

      float center_x_object, center_y_object;

      center_x_object = object.primitive_poses[0].position.x;
      center_y_object = object.primitive_poses[0].position.y;
        
      float vertices_object[4][2];

      vertices_object[0][0] = center_x_object + (mod_diag_object * cos(constrainAngle(ang_diag_object + angle_object)));
      vertices_object[0][1] = center_y_object + (mod_diag_object * sin(constrainAngle(ang_diag_object + angle_object)));
      vertices_object[1][0] = center_x_object + (mod_diag_object * cos(constrainAngle(-ang_diag_object + angle_object)));
      vertices_object[1][1] = center_y_object + (mod_diag_object * sin(constrainAngle(-ang_diag_object + angle_object)));
      vertices_object[2][0] = center_x_object - (mod_diag_object * cos(constrainAngle(ang_diag_object + angle_object)));
      vertices_object[2][1] = center_y_object - (mod_diag_object * sin(constrainAngle(ang_diag_object + angle_object)));
      vertices_object[3][0] = center_x_object - (mod_diag_object * cos(constrainAngle(-ang_diag_object + angle_object)));
      vertices_object[3][1] = center_y_object - (mod_diag_object * sin(constrainAngle(-ang_diag_object + angle_object)));

      if (!InsideMap(center_x, center_y)) continue;

      // calculates the distance from new_object center to the object's side between vertices_object[it] and [it+1]
      float dist_object[4];
      for (int it = 0; it < 3; it++)
      {
        dist_object[it] = fabs((vertices_object[it+1][0] - vertices_object[it][0])*(vertices_object[it][1] - center_y) - (vertices_object[it][0] - center_x)*(vertices_object[it+1][1] - vertices_object[it][1]))
                   / sqrt( pow(vertices_object[it+1][0] - vertices_object[it][0], 2.0) + pow(vertices_object[it+1][1] - vertices_object[it][1], 2.0));
      }
      // with vertices_object[3] and vertices_object[0]
      dist_object[3] = fabs((vertices_object[0][0] - vertices_object[3][0])*(vertices_object[3][1] - center_y) - (vertices_object[3][0] - center_x)*(vertices_object[0][1] - vertices_object[3][1]))
                / sqrt( pow(vertices_object[0][0] - vertices_object[3][0], 2.0) + pow(vertices_object[0][1] - vertices_object[3][1], 2.0));

      // For the new_object center point to be inside object,
      // the sum of the distances from the point to the opposite sides of object must be equal to the side of object.
      // Tolerance added.
      inside = (dist_object[0] + dist_object[2] <= size_x_object + tolerance && dist_object[1] + dist_object[3] <= size_y_object + tolerance);    
    }

    if (inside)
    {
      indexs.push_back(i);
    }
    
  }
  return indexs;
}

// A new object is added to the list, avoiding duplicity and overlapping. Objects on the floor can have its pose adjusted according to detected obstacles.
void AutonomousTidyUp::AddNewDetectedObject(ObjectDetected new_obj, ObjectType type, float tolerance)
{
  std::vector<int> objects_inside_indexs = GetSimilarObjects(new_obj.object, type, tolerance);

  std::vector<ObjectDetected>* objects_vector;

  if (type == ObjectTable)
  {
    objects_vector = &tables_recevied;
    // ROS_WARN("TABLE");
  } else if (type == ObjectOnFloor)
  { // Special treatment to get a better pose precision using the obstacles at the semantic map.
    objects_vector = &objects_on_the_floor_recevied;
    int i_map, j_map, i_localcostmap, j_localcostmap;
    float x_new_object = new_obj.object.primitive_poses[0].position.x, y_new_object = new_obj.object.primitive_poses[0].position.y;
    float size_x_new_obj = new_obj.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X], size_y_new_obj = new_obj.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y];

    std::tie(i_map, j_map) = transformCoordinateOdomToMap(x_new_object, y_new_object);

    float dist_aprox = 0.2, dist, min_dist = 100.0, x_temp, y_temp;
    int dist_max = dist_aprox / map_resolution_, i_min, j_min, i_max, j_max, i_min_dist, j_min_dist;

    // Seach area
    i_min = std::max(i_map - dist_max, 0);
    j_min = std::max(j_map - dist_max, 0);
    i_max = std::min(i_map + dist_max, int(map_width_ - 1));
    j_max = std::min(j_map + dist_max, int(map_height_ - 1));

    for (int i = i_min; i < i_max; i++)
    {
      for (int j = j_min; j < j_max; j++)
      {
        if (semantic_map[MapIndicesToVectorIndex(i,j)].CELL_TYPE == kCostmapObstacle)
        {
          std::tie(x_temp, y_temp) = transformCoordinateMapToOdom(i,j);
          dist = sqrt(pow(x_temp - x_new_object, 2.0) + pow(y_temp - y_new_object, 2.0));
          if (dist < min_dist) // Gets the closest CostmapObstacle to the object.
          {
            min_dist = dist;
            i_min_dist = i;
            j_min_dist = j;
          }          
        }        
      }      
    }

    if (min_dist < 100.0 && min_dist > (sqrt(size_x_new_obj*size_x_new_obj + size_y_new_obj*size_y_new_obj))/2.0)
    { // CostmapObstacle far from original object pose.
      // CHECKS CONTINUOUS OBSTACLE AREA ON COSTMAP
      std::deque<int> to_be_processed;
      int local_counter = global_counter + 30;
      int n_continuous_costmap_obst_cells = 0;

      to_be_processed.clear();
      to_be_processed.emplace_back(MapIndicesToVectorIndex(i_min_dist, j_min_dist));

      std::vector<int> i_continuous_costmap_obst_cells, j_continuous_costmap_obst_cells;

      while (!to_be_processed.empty()) {
        int map_index = to_be_processed.front();
        to_be_processed.pop_front();

        int i, j;
        std::tie(i, j) = VectorIndexToMatrixIndices(map_index);

        int size = 1;
        for (int l = j - size; l <= j + size; ++l) {
          for (int k = i - size; k <= i + size; ++k) {
            if (k >= 0 && k < map_width_ && l >= 0 && l < map_height_) {
              int current_index = MapIndicesToVectorIndex(k, l);

              if ((semantic_map[current_index].CELL_TYPE == kCostmapObstacle
                  ) &&
                  semantic_map[current_index].LAST_TIME_ANALYSED != local_counter) {

                n_continuous_costmap_obst_cells++;
                i_continuous_costmap_obst_cells.push_back(k);
                j_continuous_costmap_obst_cells.push_back(l);

                semantic_map[current_index].LAST_TIME_ANALYSED = local_counter;
                to_be_processed.emplace_back(current_index);
              }
            }
          }
        }
      }      

      float new_obj_area = (new_obj.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X]
                            * new_obj.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y]);
    
      int n_cells_new_obj = new_obj_area / (map_resolution_ * map_resolution_);
      float new_x_center, new_y_center, x_min_dist, y_min_dist, ang_min_dist;

      if (n_continuous_costmap_obst_cells <= n_cells_new_obj)
      { // Continuous obstacle area is smaller or equal to new object area, so, change object pose.
        float yaw_new_obj = ComputeYaw(new_obj.object.primitive_poses[0].orientation);
        std::tie(x_min_dist, y_min_dist)= transformCoordinateMapToOdom(i_min_dist, j_min_dist);

        // Shifts the object around the obstacle keeping the edge of the object on the obstacle.
        std::vector<float> center_variation;
        float max_sum = 0.0;

        center_variation.push_back(new_obj.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] / 2.0);
        center_variation.push_back(new_obj.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] / 2.0);
        center_variation.push_back(sqrt(pow(new_obj.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y], 2.0)
                                     + pow(new_obj.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X], 2.0)) / 2.0);
        for (int n_ang = 0; n_ang < 8; n_ang++)
        {
          float temp_ang = constrainAngle( yaw_new_obj + float(n_ang) * (M_PI / 4.0));
          for (int n = 0; n < center_variation.size(); n++)
          {
            float temp_x, temp_y;

            temp_x = x_min_dist + cos(temp_ang) * center_variation[n];
            temp_y = y_min_dist + sin(temp_ang) * center_variation[n];    

            float sum_dist_to_center = 0.0;
            float x_obst, y_obst;

            for (int it = 0; it < n_continuous_costmap_obst_cells; it++)
            {
              std::tie(x_obst, y_obst) = transformCoordinateMapToOdom(i_continuous_costmap_obst_cells[it], j_continuous_costmap_obst_cells[it]);
              sum_dist_to_center += sqrt( pow(temp_x - x_obst, 2.0) + pow(temp_y - y_obst, 2.0));
            }

            if (sum_dist_to_center > max_sum)
            { // The largest sum of distances from the center means that the points are on the edge.
              max_sum = sum_dist_to_center;
              new_obj.object.primitive_poses[0].position.x = temp_x;
              new_obj.object.primitive_poses[0].position.y = temp_y;
              ROS_ERROR("<<<<<<<<<<<CENTER ADJUSTED>>>>>>>>>>");
            }            
          }    
        }
      } else
      {
        ROS_ERROR("<<<<<<<<<<<localcostmap obstacle does not match object>>>>>>>>>>");
      }      
    } else
    {
      ROS_ERROR("<<<<<<<<<<<NEAR THE RIGHT POSE>>>>>>>>>>");
    }
    
    new_obj.state = StateOnFloor;

  } else
  {
    return;
  }

  if (objects_inside_indexs.size() == 0)
  {
    objects_vector->push_back(new_obj);
    ROS_WARN("fist time object...");
    return;
  } else if (objects_inside_indexs.size() == 1)
  { 
    moveit_msgs::CollisionObject similar_obj = (*objects_vector)[objects_inside_indexs[0]].object;

    if ((similar_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] * similar_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y])
        >
        (new_obj.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] * new_obj.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y]))
    { // The similar_object you already have is larger, keep the larger one.
      (*objects_vector)[objects_inside_indexs[0]].see_many_times = true;
      return;
    } else
    { // Replaces with new object.
      new_obj.see_many_times = true;
      if (type == ObjectOnFloor) new_obj.manipulation_tries += (*objects_vector)[objects_inside_indexs[0]].manipulation_tries;
      (*objects_vector)[objects_inside_indexs[0]] = new_obj;
      return;
    }

  } else // objects_inside_indexs.size() > 1
  {
    new_obj.see_many_times = true;

    float bigger_size = new_obj.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] * new_obj.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
    int index_bigger = -1;
    for (int it = 0; it < objects_inside_indexs.size(); it++)
    {
      moveit_msgs::CollisionObject similar_obj = (*objects_vector)[objects_inside_indexs[it]].object;
      if ((similar_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] * similar_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y])
          > bigger_size)
      {
        index_bigger = it;
        bigger_size = (similar_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] * similar_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y]);
      }    
    }
    // Keeps the bigger.
    if (index_bigger != -1)
    {
      if (type == ObjectOnFloor) new_obj.manipulation_tries += (*objects_vector)[objects_inside_indexs[index_bigger]].manipulation_tries;
      new_obj.object = (*objects_vector)[objects_inside_indexs[index_bigger]].object;
    }

    ROS_WARN("FUSION, IT WAS %d", objects_vector->size());
    // Erases the objects inside.
    for (; !objects_inside_indexs.empty(); objects_inside_indexs.pop_back())
    {
      objects_vector->erase(objects_vector->begin() + objects_inside_indexs.back());
    }
    ROS_WARN("FUSION, NOW %d...........", objects_vector->size());
    objects_vector->push_back(new_obj);
    return;    
  }
}

// Verifies if the object's base is on the floor.
inline bool AutonomousTidyUp::CheckObjectOnFloor(moveit_msgs::CollisionObject object)
{
  float base_height = (object.primitive_poses[0].position.z - (object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] / 2.0));
  return base_height < (0.05) && base_height >= (0.0);
}

// Calculates the coverage over the acessible area.
float AutonomousTidyUp::CoverageAcessibleArea()
{
  std::deque<int> to_be_processed;
  global_counter++;
  if(global_counter >= 1000001)
    global_counter = 1;
  int local_counter = global_counter + 10;

  MarkUnderBaseAsVisited();

  AddObjectsAndTablesReceviedToSemanticMap();

  int robot_i, robot_j;
  std::tie(robot_i, robot_j) = transformCoordinateOdomToMap(robot_pose.position.x, robot_pose.position.y);
  int initial_robot_position = MapIndicesToVectorIndex(robot_i, robot_j);

  int n_acessible_free_cells = 0, n_visited_cells = 0;

  // Sets the entire map as inaccessible.
  for (int it = 0; it < map_size; it++)
  {
    semantic_map[it].IS_ACCESSIBLE = false;
  }

  to_be_processed.clear();
  if (semantic_map[initial_robot_position].CELL_TYPE == kFree or semantic_map[initial_robot_position].CELL_TYPE == kInvalidBasePose) {
    to_be_processed.emplace_back(initial_robot_position);
  }

  // Access the continuous accessible area around the robot to count.
  while (!to_be_processed.empty()) {
    int map_index = to_be_processed.front();
    to_be_processed.pop_front();

    int i, j;
    std::tie(i, j) = VectorIndexToMatrixIndices(map_index);

    int size = 1;
    for (int l = j - size; l <= j + size; ++l) {
      for (int k = i - size; k <= i + size; ++k) {
        if (k >= 0 && k < map_width_ && l >= 0 && l < map_height_) {
          int current_index = MapIndicesToVectorIndex(k, l);
          if (semantic_map[current_index].CELL_TYPE != kUnknown) {
            semantic_map[current_index].IS_ACCESSIBLE = true;
          }
          if (((semantic_map[current_index].CELL_TYPE == kFree or semantic_map[current_index].CELL_TYPE == kInvalidBasePose)
                  && semantic_map[current_index].OBJECT_TYPE != ObjectTable) &&
              semantic_map[current_index].LAST_TIME_ANALYSED != local_counter) {

            n_acessible_free_cells++;

            if(semantic_map[current_index].ROBOT_VISITED) n_visited_cells++;

            semantic_map[current_index].LAST_TIME_ANALYSED = local_counter;
            to_be_processed.emplace_back(current_index);
          }
        }
      }
    }
  }

  RemoveObjectsAndTablesReceviedToSemanticMap();

  float resul = 1.0;

  if (n_acessible_free_cells > 0)
    resul = (float(n_visited_cells) / float(n_acessible_free_cells));
  else
  {
    ROS_ERROR("ERROR:::::::::::NO ACCESSIBLE CELLS");
    return 1.0;
  }  

  // Publishes data for the metrics node.
  coverage_acessible_area_msg.data = resul;
  coverage_acessible_area_pub.publish(coverage_acessible_area_msg);
  n_acessible_cells_msg.data = n_acessible_free_cells;
  n_acessible_cells_pub.publish(n_acessible_cells_msg);
  n_visited_cells_msg.data = n_visited_cells;
  n_visited_cells_pub.publish(n_visited_cells_msg);
  ros::spinOnce();

  return resul;
}

// Verifies if there is no table near.
bool AutonomousTidyUp::NoTablesNear(int i, int j)
{
  int size= SIZE_TO_MARK / map_resolution_;
  // if (!InsideMap(x - map_resolution_ * float(size), y - map_resolution_ * float(size))
  //  or !InsideMap(x + map_resolution_ * float(size), y + map_resolution_ * float(size))) return false;

  int lim_min_i, lim_max_i, lim_min_j, lim_max_j;

  lim_min_i = std::max(0, i-size);
  lim_min_j = std::max(0, j-size);
  lim_max_i = std::min(int(map_width_ - 1),  i+size);
  lim_max_j = std::min(int(map_height_ - 1), j+size);

  for(int count_i = lim_min_i; count_i <= lim_max_i; count_i++)
    for(int count_j = lim_min_j; count_j <= lim_max_j; count_j++){
      if (semantic_map[count_i + count_j * map_width_].OBJECT_TYPE == ObjectTable)
      {
        return false;
      }
    }
    
  return true;
}

void AutonomousTidyUp::PubSemanticMapVisualise(){

  semantic_map_visualise_msg.header.stamp = ros::Time::now();
  semantic_map_visualise_msg.info = map_recevied.info;

  for (int i = 0; i < map_size; i++)
  {
    switch (semantic_map[i].CELL_TYPE)
    {
    case kExpandedObstacle:
    case kObstacle:
    case kCostmapObstacle:
      semantic_map_visualise_msg.data[i] = 100;
      break;
    case kInvalidBasePose:
      semantic_map_visualise_msg.data[i] = 101;
      break;
    case kFree:
      semantic_map_visualise_msg.data[i] = 5;
      break;
    case kUnknown:
      semantic_map_visualise_msg.data[i] = 0;
      break;
    case kUnvisited:
      semantic_map_visualise_msg.data[i] = 0;
      break;
    default:
      semantic_map_visualise_msg.data[i] = 0;
      break;
    }
    switch (semantic_map[i].OBJECT_TYPE)
    {
    case ObjectTable:
      semantic_map_visualise_msg.data[i] = 50 + 10 * (semantic_map[i].NAME[5] - '0');
      break;
    case ObjectOnFloor:
      semantic_map_visualise_msg.data[i] = -5;
      break; }
  }
  semantic_map_visualise_pub.publish(semantic_map_visualise_msg);
  ROS_WARN("PUB semantic_map_visualise");
}

inline bool AutonomousTidyUp::ValidObjectOnFloor(int n_object) {
  return (objects_on_the_floor_recevied[n_object].state == StateOnFloor
       && objects_on_the_floor_recevied[n_object].manipulation_tries < 10);
}

inline bool AutonomousTidyUp::ValidTable(int n_table) {
  return (tables_recevied[n_table].state != StateTableTooSmall);
}

void AutonomousTidyUp::PubJustCoverage(){
  int size = 2;
  int i, j;

  potencial_visualise_msg.header = map_recevied.header;
  potencial_visualise_msg.info = map_recevied.info;
  potencial_visualise_msg.data.resize(map_size);

  // Copy global map to msg
  for (int m = 0; m < map_height_; m++) {
    int multi = m * map_width_;
    for (int n = 0; n < map_width_; n++) {
      int index = n + multi;
      if (!semantic_map[index].IS_ACCESSIBLE)
      {
        potencial_visualise_msg.data[index] = -1;
      } else if (semantic_map[index].ROBOT_VISITED)
      {
        potencial_visualise_msg.data[index] = 50;
      } else
      {
        potencial_visualise_msg.data[index] = 0;
      }        
    }
  }

  potencial_visualise_pub.publish(potencial_visualise_msg);
}

// Changes the execution stage and notifies the other nodes.
inline void AutonomousTidyUp::ChangeStage(ExecutionStage new_stage)
{
  ExecutionStage old_stage = stage;
  stage = new_stage;
  execution_stage_msg.data = stage;
  execution_stage_pub.publish(execution_stage_msg);
  ros::spinOnce();

  return;
}

///////////////////////////////////////
//                                   //
//          Base navigation          //
//                                   //
///////////////////////////////////////

// Uses make_plan_srv to verify if there is a path.
bool AutonomousTidyUp::PathExist(geometry_msgs::Pose pose){

  if (!InsideMap(pose.position.x, pose.position.y)) return false;

  bool there_is_a_plan = false;

  make_plan_srv.request.start.header.frame_id = "map";
  make_plan_srv.request.start.header.stamp = ros::Time::now();
  make_plan_srv.request.start.pose = robot_pose;

  make_plan_srv.request.goal.header.frame_id = "map";
  make_plan_srv.request.goal.header.stamp = ros::Time::now();
  make_plan_srv.request.goal.pose = pose;
  bool result = make_plan_client.call(make_plan_srv);
  if (!result) {
    ROS_ERROR("Failed to call service make_plan_client");
    there_is_a_plan = false;
  } else
  {
    there_is_a_plan = (make_plan_srv.response.plan.poses.size() > 0);
    // ROS_WARN("FOUND a plan: %s.........", (there_is_a_plan? "yes" : "no"));
  }
  return there_is_a_plan;
}

// Uses make_plan_srv to verify if there is a path.
bool AutonomousTidyUp::PathExist(float x, float y, float z, float roll, float pitch, float yaw){

  if (!InsideMap(x, y)) return false;

  bool there_is_a_plan = false;

  make_plan_srv.request.start.header.frame_id = "map";
  make_plan_srv.request.start.header.stamp = ros::Time::now();
  make_plan_srv.request.start.pose = robot_pose;

  make_plan_srv.request.goal.header.frame_id = "map";
  make_plan_srv.request.goal.header.stamp = ros::Time::now();
  make_plan_srv.request.goal.pose.position.x = x;
  make_plan_srv.request.goal.pose.position.y = y;
  make_plan_srv.request.goal.pose.position.z = z;
  make_plan_srv.request.goal.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

  bool result = make_plan_client.call(make_plan_srv);
  if (!result) {
    ROS_ERROR("Failed to call service make_plan_client");
    there_is_a_plan = false;
  } else
  {
    there_is_a_plan = (make_plan_srv.response.plan.poses.size() > 0);
    // ROS_WARN("FOUND a plan: %s.........", (there_is_a_plan? "yes" : "no"));
  }
  return there_is_a_plan;
}

// Uses make_plan_srv to acquire the path length.
int AutonomousTidyUp::LengthPath(geometry_msgs::Pose pose){
  int lenght = -1;

  if (!InsideMap(pose.position.x, pose.position.y)) return lenght;

  make_plan_srv.request.start.header.frame_id = "map";
  make_plan_srv.request.start.header.stamp = ros::Time::now();
  make_plan_srv.request.start.pose = robot_pose;

  make_plan_srv.request.goal.header.frame_id = "map";
  make_plan_srv.request.goal.header.stamp = ros::Time::now();
  make_plan_srv.request.goal.pose = pose;
  bool result = make_plan_client.call(make_plan_srv);
  if (!result) {
    ROS_ERROR("Failed to call service make_plan_client");
  } else
  {
    lenght = make_plan_srv.response.plan.poses.size();
  }
  return lenght;
}

// Finds the nearest object on the floor and the pose near it that is closest to the robot. Also finds inaccessible objects.
std::tuple<geometry_msgs::Pose, moveit_msgs::CollisionObject> AutonomousTidyUp::GoalNearestObjectOnFloor()
{
  float min_dist = 100.0;
  moveit_msgs::CollisionObject nearest_object;
  std::vector<float> dist_array;

  std::map<float, moveit_msgs::CollisionObject> dist_to_object;
  std::map<float, int> dist_to_n_object;

  for (int i = 0; i < objects_on_the_floor_recevied.size(); i++)
  {
    moveit_msgs::CollisionObject object = objects_on_the_floor_recevied[i].object;
    if(object.id.length() == 0) continue;

    if(!ValidObjectOnFloor(i))  continue;
    
    float angle = ComputeYaw(object.primitive_poses[0].orientation);

    float size_x = object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X];
    float size_y = object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
    float size_z = object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z];

    float dist_x_pos, dist_x_neg, dist_y_pos, dist_y_neg; // Calculates the distance to each vertex.
    
    dist_x_pos = sqrt(pow(robot_pose.position.x - (object.primitive_poses[0].position.x + (size_x/2.0) * cos(angle)), 2.0) + pow(robot_pose.position.y - (object.primitive_poses[0].position.y + (size_x/2.0) * sin(angle)), 2.0));
    dist_x_neg = sqrt(pow(robot_pose.position.x - (object.primitive_poses[0].position.x - (size_x/2.0) * cos(angle)), 2.0) + pow(robot_pose.position.y - (object.primitive_poses[0].position.y - (size_x/2.0) * sin(angle)), 2.0));
    dist_y_pos = sqrt(pow(robot_pose.position.x - (object.primitive_poses[0].position.x + (size_y/2.0) * cos(angle)), 2.0) + pow(robot_pose.position.y - (object.primitive_poses[0].position.y + (size_y/2.0) * sin(angle)), 2.0));
    dist_y_neg = sqrt(pow(robot_pose.position.x - (object.primitive_poses[0].position.x - (size_y/2.0) * cos(angle)), 2.0) + pow(robot_pose.position.y - (object.primitive_poses[0].position.y - (size_y/2.0) * sin(angle)), 2.0));
    
    float dist = std::fmin( std::fmin( dist_y_pos, dist_y_neg), std::fmin(dist_x_pos, dist_x_neg));

    dist_array.push_back(dist);

    dist_to_object.insert({dist, object});
    dist_to_n_object.insert({dist, i});  
  }

  // Orders objects by distance to the robot.
  std::sort(dist_array.begin(), dist_array.end());

  bool valid_pose_found = false;
  geometry_msgs::Pose pose;
  int n_object;
  for(int i = 0; i < dist_array.size() && !valid_pose_found; i++)
  {
    nearest_object = dist_to_object[dist_array[i]];
    n_object = dist_to_n_object[dist_array[i]];
    pose = FindFreePoseNearObject(nearest_object);
    if (pose.position.x != robot_pose.position.x && pose.position.y != robot_pose.position.y)
    {
      ROS_WARN("%s is the nearest object", nearest_object.id.c_str()); 
      valid_pose_found = true;
    } else
    {
      ChangeObjectState(n_object, StateNotAcessible);
    }    
  }
  
  if (!valid_pose_found)
  {
    ROS_ERROR("-----------NO ACCESSIBLE OBJECT ON THE FLOOR");
    nearest_object.id = string("no_object");
  }  

  return std::tie(pose, nearest_object);
}

// Finds the nearest table and the pose near it that is closest to the robot.
std::tuple<geometry_msgs::Pose, moveit_msgs::CollisionObject, int> AutonomousTidyUp::GoalNearestTable()
{
  float min_dist = 100.0;
  moveit_msgs::CollisionObject nearest_table;
  std::vector<float> dist_array;
  int index_nearest_table = -1;

  std::map<float, moveit_msgs::CollisionObject> dist_to_object;

  for (int i = 0; i < tables_recevied.size(); i++)
  {
    moveit_msgs::CollisionObject object = tables_recevied[i].object;
    if(object.id.length() == 0 or !ValidTable(i)) continue;
    
    float angle = ComputeYaw(object.primitive_poses[0].orientation);

    float size_x = object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X];
    float size_y = object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
    float size_z = object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z];

    if (size_x < 0.7 or size_y < 0.7)
    {
      continue;
    }

    float dist_x_pos, dist_x_neg, dist_y_pos, dist_y_neg; // Calculates the distance to each vertex.
    
    dist_x_pos = sqrt(pow(robot_pose.position.x - (object.primitive_poses[0].position.x + (size_x/2.0) * cos(angle)), 2.0) + pow(robot_pose.position.y - (object.primitive_poses[0].position.y + (size_x/2.0) * sin(angle)), 2.0));
    dist_x_neg = sqrt(pow(robot_pose.position.x - (object.primitive_poses[0].position.x - (size_x/2.0) * cos(angle)), 2.0) + pow(robot_pose.position.y - (object.primitive_poses[0].position.y - (size_x/2.0) * sin(angle)), 2.0));
    dist_y_pos = sqrt(pow(robot_pose.position.x - (object.primitive_poses[0].position.x + (size_y/2.0) * cos(angle)), 2.0) + pow(robot_pose.position.y - (object.primitive_poses[0].position.y + (size_y/2.0) * sin(angle)), 2.0));
    dist_y_neg = sqrt(pow(robot_pose.position.x - (object.primitive_poses[0].position.x - (size_y/2.0) * cos(angle)), 2.0) + pow(robot_pose.position.y - (object.primitive_poses[0].position.y - (size_y/2.0) * sin(angle)), 2.0));

    float dist = std::fmin( std::fmin( dist_y_pos, dist_y_neg), std::fmin(dist_x_pos, dist_x_neg));

    dist_array.push_back(dist);
    dist_to_object.insert({dist, object});
  }

  // Orders tables by distance to the robot.
  std::sort(dist_array.begin(), dist_array.end());

  bool valid_pose_found = false;
  geometry_msgs::Pose pose;
  float dist_to_table;
  for(int i = 0; i < dist_array.size() && !valid_pose_found; i++)
  {
    nearest_table = dist_to_object[dist_array[i]]; 
    dist_to_table = 0.35;
    valid_pose_found = false;
    do
    {
      pose = FindFreePoseNearTable(nearest_table, dist_to_table);
      if (pose.position.x != robot_pose.position.x && pose.position.y != robot_pose.position.y)
      {
        ROS_WARN("%s is the closest table, pose %f away from the table", nearest_table.id.c_str(), dist_to_table);
        valid_pose_found = true;
        for (int it = 0; it < tables_recevied.size(); it++)
        {
          if (nearest_table.id.compare(tables_recevied[it].object.id) == 0)
          {
            index_nearest_table = it;
            break;
          }        
        }      
      } else
      {
        // Increments the distance between the pose and the table.
        dist_to_table += 0.05;
        valid_pose_found = false;
      }
    } while (!valid_pose_found && dist_to_table <= 0.5);
  }

  if(!valid_pose_found) ROS_ERROR(">>>>>>>>>DIDN'T FIND A VALID POSE NEAR TABLES>>>>>>>>>"); 

  return std::tie(pose, nearest_table, index_nearest_table);
}

// Finds an accessible free pose around an object. Also publishes the possible goals.
geometry_msgs::Pose AutonomousTidyUp::FindFreePoseNearObject(moveit_msgs::CollisionObject object)
{
  int i_center, j_center, index;
  float min_dist = 100.0, dist;

  float dist_from_object = 0.4 + (object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X]
                                  + object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y])/4.0;
  double step_ang = DEG2RAD(10);

  float center_x, center_y;

  center_x = object.primitive_poses[0].position.x;
  center_y = object.primitive_poses[0].position.y;

  std::vector<geometry_msgs::Pose> all_poses;

  for (int n_try = 0; n_try < 3; n_try++)
  {
    all_poses.clear();
    // Calculates the poses around the object.
    for (int i = 0; float(i)*step_ang < 2.0*M_PI; i++)
    {
      geometry_msgs::Pose pose;
      pose.position.x = center_x + dist_from_object * cos(float(i)*step_ang);
      pose.position.y = center_y + dist_from_object * sin(float(i)*step_ang);
      pose.position.z = 0.0;
      pose.orientation =  tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, M_PI + float(i)*step_ang);
      all_poses.push_back(pose);
    }
    ROS_WARN("Possible goals for object %d", n_try);
    place_poses_msg.header.frame_id = "map";
    place_poses_msg.header.stamp = ros::Time::now();
    place_poses_msg.poses = all_poses;
    place_poses_pub.publish(place_poses_msg);
    ros::spinOnce();
    
    geometry_msgs::Pose pose_near_robot = robot_pose;
    min_dist = 100.0;

    float coverage = CoverageAcessibleArea();
    ROS_ERROR("Update IS_ACCESSIBLE: COVERAGE OF %f PERCENT OF THE AREA ", (100.0)*coverage);

    // Finds the closest acessible free goal.
    for (int it = 0; it < all_poses.size(); it++)
    {
      int i_temp, j_temp;
      std::tie(i_temp, j_temp) = transformCoordinateOdomToMap(all_poses[it].position.x, all_poses[it].position.y);
      int index = MapIndicesToVectorIndex(i_temp, j_temp);
      if (   semantic_map[index].IS_ACCESSIBLE && semantic_map[index].CELL_TYPE == kFree
          && semantic_map[index].OBJECT_TYPE == NoObject)
      {
        dist = float(LengthPath(all_poses[it])) * map_resolution_;
        if (dist < min_dist && ViableGoal(all_poses[it].position.x, all_poses[it].position.y) && dist > 0.0)
        {
          min_dist = dist;
          pose_near_robot = all_poses[it];
        }
      }
    }

    if (min_dist > 99.0)
    {
      ROS_WARN("DIDN'T FIND AN ACCESSIBLE POSE NEAR THE OBJECT try %d at %f m", n_try, dist_from_object);
      // Increments the distance between the pose and the object.
      dist_from_object += 0.1;
      // return robot_pose;
    } else
    {
      ROS_WARN(">>>>>>>>>>FOUND AN ACCESSIBLE POSE NEAR THE OBJECT try %d at %f m", n_try, dist_from_object);
      return pose_near_robot;
    }
  }
  ROS_WARN("DIDN'T FIND AN ACCESSIBLE POSE NEAR THE OBJECT");
  return robot_pose;
}

// Finds an accessible free pose around a table, dist_from_table away. Also publishes the possible goals.
geometry_msgs::Pose AutonomousTidyUp::FindFreePoseNearTable(moveit_msgs::CollisionObject object_table, float dist_from_table = 0.35)
{
  int i_center, j_center, index;
  float min_dist = 100.0, dist;

  double step = 0.1;
  
  ROS_WARN("%s on .... %s", object_table.id.c_str(), object_table.header.frame_id.c_str());

  // CONSIDER TABLE ROTATION
  double offset_vector[2], offset_vector_new_line[2], p_min[2], p_max[2], vector_diag_inf[2], p_init_line[2];
  double size_box_x, size_box_y;

  float theta = constrainAngle(tf::getYaw(object_table.primitive_poses[0].orientation));
  size_box_x = 2*dist_from_table + object_table.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X];
  size_box_y = 2*dist_from_table + object_table.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
  
  double ang_diag_inf;
  ang_diag_inf = M_PI - atan2(size_box_y/2.0, size_box_x/2.0);

  vector_diag_inf[0] = (sqrt(size_box_x*size_box_x + size_box_y*size_box_y) / 2.0) * cos(theta + ang_diag_inf);
  vector_diag_inf[1] = (sqrt(size_box_x*size_box_x + size_box_y*size_box_y) / 2.0) * sin(theta + ang_diag_inf);
  
  p_min[0] = object_table.primitive_poses[0].position.x + vector_diag_inf[0];
  p_min[1] = object_table.primitive_poses[0].position.y + vector_diag_inf[1];
  p_max[0] = object_table.primitive_poses[0].position.x - vector_diag_inf[0];
  p_max[1] = object_table.primitive_poses[0].position.y - vector_diag_inf[1];

  offset_vector[0] = step * cos(theta - M_PI/2.0);
  offset_vector[1] = step * sin(theta - M_PI/2.0);

  offset_vector_new_line[0] = step * cos(theta);
  offset_vector_new_line[1] = step * sin(theta);

  int tam = 2 * (int(size_box_x/step) + int(size_box_y/step));
  double x, y, z;

  z = 0.0;
  x = p_min[0] + float(3.0 + int(dist_from_table / step)) * offset_vector[0];
  y = p_min[1] + float(3.0 + int(dist_from_table / step)) * offset_vector[1];

  if (tam == 0)
  {
    return robot_pose;
  }

  std::vector<geometry_msgs::Pose> all_poses;

  // Calculates the poses on the four sides.
  int i = 0;
  // for (int i_x = 0; i_x < int(size_box_x/step) && i < tam; i_x++)
  // {
    // p_init_line[0] = x;
    // p_init_line[1] = y;
    for (int i_y = 0; i_y < (int(size_box_y/step) -3 - 2*int(dist_from_table / step)) && i < tam; i_y++)
    {
      geometry_msgs::Pose pose;
      pose.position.x = x;
      pose.position.y = y;
      pose.position.z = z;
      pose.orientation =  tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, theta);
      all_poses.push_back(pose);

      i++;
      x += offset_vector[0];
      y += offset_vector[1];
    }
    // x = p_init_line[0] + offset_vector_new_line[0];
    // y = p_init_line[1] + offset_vector_new_line[1];
  // }

  z = 0.0;
  x = p_max[0] - float(1.0 + int(dist_from_table / step)) * offset_vector[0];
  y = p_max[1] - float(1.0 + int(dist_from_table / step)) * offset_vector[1];
    for (int i_y = 0; i_y < (int(size_box_y/step) -3 - 2*int(dist_from_table / step)) && i < tam; i_y++)
    {
      geometry_msgs::Pose pose;
      pose.position.x = x;
      pose.position.y = y;
      pose.position.z = z;
      pose.orientation =  tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, theta + M_PI);
      all_poses.push_back(pose);

      i++;
      x -= offset_vector[0];
      y -= offset_vector[1];
    }

  z = 0.0;
  x = p_min[0] + float(1.0 + int(dist_from_table / step)) * offset_vector_new_line[0];
  y = p_min[1] + float(1.0 + int(dist_from_table / step)) * offset_vector_new_line[1];
  for (int i_x = 0; i_x < (int(size_box_x/step) - 3 - 2*int(dist_from_table / step)) && i < tam; i_x++)
  {
    p_init_line[0] = x;
    p_init_line[1] = y;
    // for (int i_y = 0; i_y < int(size_box_y/step) && i < tam; i_y++)
    // {
      geometry_msgs::Pose pose;
      pose.position.x = x;
      pose.position.y = y;
      pose.position.z = z;
      pose.orientation =  tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, theta - M_PI/2.0);
      all_poses.push_back(pose);

      i++;
    //   x += offset_vector[0];
    //   y += offset_vector[1];
    // }
    x = p_init_line[0] + offset_vector_new_line[0];
    y = p_init_line[1] + offset_vector_new_line[1];
  }

  z = 0.0;
  x = p_max[0] - float(1.0 + int(dist_from_table / step)) * offset_vector_new_line[0];
  y = p_max[1] - float(1.0 + int(dist_from_table / step)) * offset_vector_new_line[1];
  for (int i_x = 0; i_x < (int(size_box_x/step) - 3 - 2*int(dist_from_table / step)) && i < tam; i_x++)
  {
    p_init_line[0] = x;
    p_init_line[1] = y;
    // for (int i_y = 0; i_y < int(size_box_y/step) && i < tam; i_y++)
    // {
      geometry_msgs::Pose pose;
      pose.position.x = x;
      pose.position.y = y;
      pose.position.z = z;
      pose.orientation =  tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, theta + M_PI/2.0);
      all_poses.push_back(pose);

      i++;
    //   x += offset_vector[0];
    //   y += offset_vector[1];
    // }
    x = p_init_line[0] - offset_vector_new_line[0];
    y = p_init_line[1] - offset_vector_new_line[1];

  }

  ROS_WARN("Possible goals for table");
  place_poses_msg.header.frame_id = "map";
  place_poses_msg.header.stamp = ros::Time::now();
  place_poses_msg.poses = all_poses;
  place_poses_pub.publish(place_poses_msg);
  ros::spinOnce();

  int i_robot, j_robot;
  std::tie(i_robot, j_robot) = transformCoordinateOdomToMap(robot_pose.position.x, robot_pose.position.y);
  int i_min_dist, j_min_dist;
  i_min_dist = i_robot;
  j_min_dist = j_robot;

  geometry_msgs::Pose pose_near_robot = robot_pose;
  min_dist = 100.0;

  // Finds the closest acessible free goal.
  for (int it = 0; it < all_poses.size(); it++)
  {
    int i_temp, j_temp;
    std::tie(i_temp, j_temp) = transformCoordinateOdomToMap(all_poses[it].position.x, all_poses[it].position.y);
    int index = MapIndicesToVectorIndex(i_temp, j_temp);
    if (   semantic_map[index].IS_ACCESSIBLE && semantic_map[index].CELL_TYPE == kFree
        && semantic_map[index].OBJECT_TYPE == NoObject)
    {
      dist = float(LengthPath(all_poses[it])) * map_resolution_;
      if (dist < min_dist && ViableGoal(all_poses[it].position.x, all_poses[it].position.y) && dist > 0.0)
      {
        min_dist = dist;
        pose_near_robot = all_poses[it];
      }          
    }
  }
  
  if (min_dist > 99.0)
  {
    ROS_WARN("DIDN'T FIND AN ACCESSIBLE POSE NEAR THE TABLE");
    return robot_pose;
  } else
  {
    ROS_WARN(">>>>>>>>>>FOUND AN ACCESSIBLE POSE NEAR THE TABLE");
    return pose_near_robot;
  }
}

inline bool AutonomousTidyUp::InsideMap(float x, float y) {
  bool inside_x = (map_origin_x_ < x - map_resolution_) && (x + map_resolution_ < map_origin_x_ + (map_resolution_ * map_width_ ));
  bool inside_y = (map_origin_y_ < y - map_resolution_) && (y + map_resolution_ < map_origin_y_ + (map_resolution_ * map_height_));
  return (inside_x && inside_y);
}

// inline bool AutonomousTidyUp::InsideMap(int i, int j) {
//   float x, y;
//   std::tie(x, y) = transformCoordinateMapToOdom(i,j);
//   bool inside_x = (map_origin_x_ < x - map_resolution_) && (x + map_resolution_ < map_origin_x_ + (map_resolution_ * map_width_ ));
//   bool inside_y = (map_origin_y_ < y - map_resolution_) && (y + map_resolution_ < map_origin_y_ + (map_resolution_ * map_height_));
//   return (inside_x && inside_y);
// }

inline bool AutonomousTidyUp::InsideGlobalcostmap(float x, float y) {
  bool inside_x = (global_costmap_origin_x_ < x - global_costmap_resolution_) && (x + global_costmap_resolution_ < global_costmap_origin_x_ + (global_costmap_resolution_ * global_costmap_width_ ));
  bool inside_y = (global_costmap_origin_y_ < y - global_costmap_resolution_) && (y + global_costmap_resolution_ < global_costmap_origin_y_ + (global_costmap_resolution_ * global_costmap_height_));
  return (inside_x && inside_y);
}

///////////////////////////////////////
//                                   //
//          GRID FUNCTIONS           //
//                                   //
///////////////////////////////////////

// Checks if an area is free, accessible, and without objects.
bool AutonomousTidyUp::FreeArea(int n_center, int m_center, float dist){
  int size = (base_radius+BASE_EXPANSION)*dist/map_resolution_;
  
  if(m_center < 0 || m_center >= map_height_ || n_center >= map_width_ || n_center < 0)
    return false;

  int m_init = m_center - size;
  if(m_init<0) m_init=0;
  int n_init = n_center - size;
  if(n_init<0) n_init=0;

  for (int m = m_init; m < m_center + size && m < map_height_; m++) {
    int multi = m * map_width_;
    for (int n = n_init; n < n_center + size && n < map_width_; n++) {
      int map_index = n + multi;
      if(semantic_map[map_index].CELL_TYPE != kFree ||
          !semantic_map[map_index].IS_ACCESSIBLE ||
          semantic_map[map_index].OBJECT_TYPE != NoObject){
        return false;
      }
    }
  }
  return true;
}

// Adjusts a position to a dist grid.
float AutonomousTidyUp::PositionOnGrid(float pos, float dist){
  float pos_on_grid=0, pos_abs = abs(pos);
  for (pos_on_grid=0; pos_abs>=dist; pos_abs-=dist)
    pos_on_grid+=dist;
    
  if(pos_abs!=0 && pos_abs>=dist/2)
    pos_on_grid+=dist;
        
  if(pos<0)
    pos_on_grid=pos_on_grid*(-1);
    
  return pos_on_grid;
}

// Guides the robot to walk on a grid.
bool AutonomousTidyUp::SimpleTrajectory(float min_dist){
  int i_pref, j_pref, mult_ang;
  int current_index;
  float robot_yaw, goal_yaw, x_pref, y_pref, current_robot_x_on_grid, current_robot_y_on_grid;
  // float min_dist=1.0;// space between the goals on the grid
  geometry_msgs::Pose goal_pose, preference_pose;
  geometry_msgs::Quaternion new_quat;
  geometry_msgs::Pose computed_goal_point_;

  robot_yaw = ComputeYaw(robot_pose.orientation);
  if(robot_yaw<M_PI/4 && robot_yaw>-M_PI/4){
    mult_ang=0;
  }else if(robot_yaw>=M_PI/4 && robot_yaw<3*M_PI/4){
    mult_ang=1;
  }else if(robot_yaw>=3*M_PI/4 || robot_yaw<-3*M_PI/4){
    mult_ang=2;
  }else if(robot_yaw<=-M_PI/4 && robot_yaw>-3*M_PI/4){
    mult_ang=-1;
  }

  computed_goal_point_ = goal_point_;
  int size = base_radius*min_dist/map_resolution_;
  int i_g,j_g;
  std::tie(i_g, j_g)=transformCoordinateOdomToMap(goal_point_.position.x, goal_point_.position.y);
  if(!FreeArea(i_g, j_g, min_dist)){ // The current goal becomes inaccessible.
    for(int l = j_g - size; l <= j_g + size; ++l)
      for(int k = i_g - size; k <= i_g + size; ++k){
        if(k >= 0 && k < map_width_ && l >= 0 && l < map_height_)
          semantic_map[MapIndicesToVectorIndex(k, l)].GRID_VISITED=true;
      }
    // Sets the current robot's position on grid as goal.
    computed_goal_point_.position.x=PositionOnGrid(robot_pose.position.x, min_dist);
    computed_goal_point_.position.y=PositionOnGrid(robot_pose.position.y, min_dist);
    computed_goal_point_.orientation=robot_pose.orientation;
    ROS_WARN("goal robot pose corrected...x: %f   y: %f", computed_goal_point_.position.x, computed_goal_point_.position.y);
  }

  int i,j;
  std::tie(i, j)=transformCoordinateOdomToMap(computed_goal_point_.position.x, computed_goal_point_.position.y);
   
  float diff = sqrt(pow(computed_goal_point_.position.x - robot_pose.position.x, 2) + pow(computed_goal_point_.position.y - robot_pose.position.y, 2));
  if(!FreeArea(i, j, min_dist) || (diff < 0.5*min_dist && abs(constrainAngle(robot_yaw-ComputeYaw(computed_goal_point_.orientation)))<M_PI/4)){
    // Goal reached.
    for(int l = j - size; l <= j + size; ++l)
      for(int k = i - size; k <= i + size; ++k){
        if(k >= 0 && k < map_width_ && l >= 0 && l < map_height_)
          semantic_map[MapIndicesToVectorIndex(k, l)].GRID_VISITED=true;
      }

    // Calculate next goal on the preference direction.
    x_pref=computed_goal_point_.position.x;
    y_pref=computed_goal_point_.position.y;
    switch (mult_ang){
      case 0: y_pref += preference_wall*min_dist; break;
      case 1: x_pref -= preference_wall*min_dist; break;
      case 2: y_pref -= preference_wall*min_dist; break;
      case -1: x_pref += preference_wall*min_dist; break;    
      default: break;
    }
    goal_yaw=constrainAngle((mult_ang+preference_wall)*M_PI/2);
    std::tie(i_pref, j_pref)=transformCoordinateOdomToMap(x_pref, y_pref);
    current_index=MapIndicesToVectorIndex(i_pref, j_pref);
    if(FreeArea(i_pref, j_pref, min_dist) && !semantic_map[current_index].GRID_VISITED && finded_wall){
      // Follows the preference only after finding a wall.
      goal_pose.position.x = x_pref;
      goal_pose.position.y = y_pref;

      ROS_WARN("goal preference...");
    }else{ // Front
      x_pref=computed_goal_point_.position.x;
      y_pref=computed_goal_point_.position.y;
      switch (mult_ang){
        case 0: x_pref += min_dist; break;
        case 1: y_pref += min_dist; break;
        case 2: x_pref -= min_dist; break;
        case -1: y_pref -= min_dist; break;    
        default: break;
      }
      goal_yaw=mult_ang*M_PI/2;
      std::tie(i_pref, j_pref)=transformCoordinateOdomToMap(x_pref, y_pref);
      current_index=MapIndicesToVectorIndex(i_pref, j_pref);
      if(FreeArea(i_pref, j_pref, min_dist) && !semantic_map[current_index].GRID_VISITED){
        goal_pose.position.x = x_pref;
        goal_pose.position.y = y_pref;
        ROS_WARN("goal front...");
      }else{
        if(!finded_wall){ // Special goal, first goal after finding a wall
          finded_wall=true;
          x_pref=computed_goal_point_.position.x;
          y_pref=computed_goal_point_.position.y;
          switch (mult_ang){
            case 0: y_pref += min_dist; break;
            case 1: x_pref -= min_dist; break;
            case 2: y_pref -= min_dist; break;
            case -1: x_pref += min_dist; break;    
            default: break;
          }
          goal_yaw=constrainAngle((mult_ang+1)*M_PI/2);
          std::tie(i_pref, j_pref)=transformCoordinateOdomToMap(x_pref, y_pref);
          current_index=MapIndicesToVectorIndex(i_pref, j_pref);
          if(FreeArea(i_pref, j_pref, min_dist) && !semantic_map[current_index].GRID_VISITED){
            goal_pose.position.x = x_pref;
            goal_pose.position.y = y_pref;
            preference_wall=-1;
            ROS_WARN("special goal...changes the preference to -1");
          }else{
            x_pref=computed_goal_point_.position.x;
            y_pref=computed_goal_point_.position.y;
            switch (mult_ang){
              case 0: y_pref -= min_dist; break;
              case 1: x_pref += min_dist; break;
              case 2: y_pref += min_dist; break;
              case -1: x_pref -= min_dist; break;    
              default: break;
            }
            goal_yaw=constrainAngle((mult_ang-1)*M_PI/2);

            std::tie(i_pref, j_pref)=transformCoordinateOdomToMap(x_pref, y_pref);
            current_index=MapIndicesToVectorIndex(i_pref, j_pref);
            if(FreeArea(i_pref, j_pref, min_dist) && !semantic_map[current_index].GRID_VISITED){
              goal_pose.position.x = x_pref;
              goal_pose.position.y = y_pref;
              preference_wall=1;
              ROS_WARN("special goal...changes the preference to 1");
            }else{
              ROS_WARN("no goals on left or right after finding the wall forward");
              return false;
            }
          }
        }else{ // Counterpreference
          x_pref=computed_goal_point_.position.x;
          y_pref=computed_goal_point_.position.y;
          switch (mult_ang){
            case 0: y_pref += (-preference_wall)*min_dist; break;
            case 1: x_pref -= (-preference_wall)*min_dist; break;
            case 2: y_pref -= (-preference_wall)*min_dist; break;
            case -1: x_pref += (-preference_wall)*min_dist; break;    
            default: break;
          }
          goal_yaw=constrainAngle((mult_ang+(-preference_wall))*M_PI/2);
          std::tie(i_pref, j_pref)=transformCoordinateOdomToMap(x_pref, y_pref);
          current_index=MapIndicesToVectorIndex(i_pref, j_pref);

          if(FreeArea(i_pref, j_pref, min_dist) && !semantic_map[current_index].GRID_VISITED){
            goal_pose.position.x = x_pref;
            goal_pose.position.y = y_pref;
            preference_wall*=-1; // change the preference
            ROS_WARN("goal counterpreference, change the preference...");
          }else{
            // No goals on front, right or left
            x_pref=computed_goal_point_.position.x;
            y_pref=computed_goal_point_.position.y;
            std::tie(i_pref, j_pref)=transformCoordinateOdomToMap(x_pref, y_pref);

            std::vector<geometry_msgs::Pose> possible_goals;
            geometry_msgs::Pose point;
            int n_init, m_init;
            float x_on_grid, y_on_grid;

            // To find the nearest grid goal to the map origin
            x_on_grid = PositionOnGrid(map_origin_x_, min_dist);
            y_on_grid = PositionOnGrid(map_origin_y_, min_dist);
            if(x_on_grid < map_origin_x_)
              x_on_grid+=min_dist;
            if(y_on_grid < map_origin_y_)
              y_on_grid+=min_dist;
            std::tie(n_init, m_init)=transformCoordinateOdomToMap(x_on_grid, y_on_grid);

            // From that goal, find grid goals on the map
            for (int m = m_init; m < map_height_; m+=min_dist/map_resolution_) {
              int multi = m * map_width_;
              for (int n = n_init; n < map_width_; n+=min_dist/map_resolution_) {
                int map_index = n + multi;
                if(!semantic_map[map_index].GRID_VISITED && FreeArea(n, m, min_dist)){
                  std::tie(x_pref, y_pref)=transformCoordinateMapToOdom(n, m);
                  point.position.x = x_pref;
                  point.position.y = y_pref;
                  possible_goals.push_back(point);
                }                  
              }
            }
            ROS_WARN("Seach far goal...");
              
            if(possible_goals.empty()){
              ROS_ERROR(" --------- The GRID has finished !--------- ");
              PubVisitedGrid();
              return true;
            }else{
              // Finds the closest goal to the last goal.
              geometry_msgs::Pose near_goal, goal;
              near_goal=possible_goals.back();
              possible_goals.pop_back();
              while(!possible_goals.empty()){
                goal=possible_goals.back();
                possible_goals.pop_back();
                if(abs(goal.position.x - computed_goal_point_.position.x) + abs(goal.position.y - computed_goal_point_.position.y) < abs(near_goal.position.x - computed_goal_point_.position.x) + abs(near_goal.position.y - computed_goal_point_.position.y)){
                  near_goal=goal;
                }
              }
              ROS_WARN("near goal: x:%f y:%f", near_goal.position.x, near_goal.position.y);

              goal_yaw=atan2((near_goal.position.y - computed_goal_point_.position.y), (near_goal.position.x - computed_goal_point_.position.x));
              if(goal_yaw<M_PI/4 && goal_yaw>-M_PI/4){
                goal_yaw=0;
              }else if(goal_yaw>=M_PI/4 && goal_yaw<3*M_PI/4){
                goal_yaw=M_PI/2;
              }else if(goal_yaw>=3*M_PI/4 || goal_yaw<-3*M_PI/4){
                goal_yaw=M_PI;
              }else if(goal_yaw<=-M_PI/4 && goal_yaw>-3*M_PI/4){
                goal_yaw=-M_PI/2;
              }
              goal_pose=near_goal;
            }
          }
        }
      }
    }
    
    goal_pose.position.z = 0;
    computed_goal_point_ = goal_pose;
    computed_goal_point_.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, goal_yaw);
    // Set new goal
    ROS_WARN("Send grid goal...");
    SendAndSaveGoalBaseWithoutWait(computed_goal_point_);
  }else{
    ROS_WARN("Send grid goal again...");
    SendAndSaveGoalBaseWithoutWait(computed_goal_point_);
    current_robot_x_on_grid = PositionOnGrid(robot_pose.position.x, min_dist);
    current_robot_y_on_grid = PositionOnGrid(robot_pose.position.y, min_dist);

    float diff = sqrt(pow(current_robot_x_on_grid - robot_pose.position.x, 2) + pow(current_robot_x_on_grid - robot_pose.position.y, 2));
    if(diff < base_radius*min_dist){
      int i,j;
      std::tie(i, j)=transformCoordinateOdomToMap(current_robot_x_on_grid, current_robot_y_on_grid);

      if(!semantic_map[MapIndicesToVectorIndex(i, j)].GRID_VISITED){
        // reached a point on the grid that isn't a goal, set as GRID_VISITED
        // int size = 0.3*min_dist/map_resolution_;
        for(int l = j - size; l <= j + size; ++l)
          for(int k = i - size; k <= i + size; ++k){
            if(k >= 0 && k < map_width_ && l >= 0 && l < map_height_)
              semantic_map[MapIndicesToVectorIndex(k, l)].GRID_VISITED=true;
          }
      }
    }
  }
  PubVisitedGrid();
  return false;
}

void AutonomousTidyUp::PubVisitedGrid(){
  int size = 2;
  int i, j;

  potencial_visualise_msg.header = map_recevied.header;
  potencial_visualise_msg.info = map_recevied.info;
  potencial_visualise_msg.data.resize(map_size);

  // Copy global map to msg
  for (int m = 0; m < map_height_; m++) {
    int multi = m * map_width_;
    for (int n = 0; n < map_width_; n++) {
      int index = n + multi;
      if (!semantic_map[index].IS_ACCESSIBLE)
      {
        potencial_visualise_msg.data[index] = -1;
      } else if (semantic_map[index].ROBOT_VISITED && semantic_map[index].GRID_VISITED)
      {
        potencial_visualise_msg.data[index] = 100;
      } else if (semantic_map[index].ROBOT_VISITED)
      {
        potencial_visualise_msg.data[index] = 50;
      } else
      {
        potencial_visualise_msg.data[index] = 0;
      }        
    }
  }

  size = 2;
  std::tie(i, j) = transformCoordinateOdomToMap(goal_point_.position.x,
                                                  goal_point_.position.y);
  for (int l = j - size; l <= j + size; ++l)
    for (int k = i - size; k <= i + size; ++k) {
      if (k >= 0 && k < map_width_ && l >= 0 && l < map_height_)
        potencial_visualise_msg.data[MapIndicesToVectorIndex(k, l)] = 101;
    }

  std::tie(i, j) = transformCoordinateOdomToMap(robot_pose.position.x,
                                                robot_pose.position.y);
  potencial_visualise_msg.data[MapIndicesToVectorIndex(i, j)] = -1;

  potencial_visualise_pub.publish(potencial_visualise_msg);
}

///////////////////////////////////////
//                                   //
//         VORONOI FUNCTIONS         //
//                                   //
///////////////////////////////////////

// It computes the Voronoi Diagram based on the global_map
void AutonomousTidyUp::ComputeVoronoiDiagram() {
  unsigned char *map, *result;
  int external_border = 1;
  int width = map_width_ -1;
  int height = map_height_ -1;
  map = new unsigned char[width * height * 3];
  result = new unsigned char[width * height * 3];
  // translating the global_map information to the data type that the thinning class accepts
  int counter = 0;

  ROS_ERROR("initialized info");

  for (int y = height; y >= 1; y--) {
    for (int x = 1; x <= width; x++) {
      {
        int index = MapIndicesToVectorIndex(x, y);
        if (
          #if STRATEGY_VORONOI
          (semantic_map[index].ORIGINAL_MAP < 0 || semantic_map[index].ORIGINAL_MAP >= 90)
          #else
          semantic_map[index].CELL_TYPE != kFree
          #endif          
           || !semantic_map[index].IS_ACCESSIBLE
            || semantic_map[index].OBJECT_TYPE != NoObject
         || x == width ||
            x == 1 ||
            y == height ||
            y == 1
            ) {
          map[counter] = 0x00;
          map[counter + 1] = 0x00;
          map[counter + 2] = 0x00;
        } else 
        {
          map[counter] = 0xFF;
          map[counter + 1] = 0xFF;
          map[counter + 2] = 0xFF;
        }
        counter += 3;
      }
    }
  }

  // translating back the map output
  ROS_ERROR("translated map");
  
  Thinning t;
  ROS_ERROR("class initiated");
  t.thinningGUOandHALL(map, result, width, height);
  ROS_ERROR("Voronoi calculated");
  t.spurRemoval(5, result, width, height);
  ROS_ERROR("pruned");
  counter = 0;

  for (int y = height; y >= 1; y--) {
    for (int x = 1; x <= width; x++) {
      int index = MapIndicesToVectorIndex(x, y);

      semantic_map[index].VORONOI_CELL = (result[counter] == 0xFF);

      counter += 3;
    }
  }
  ROS_ERROR("saved on semantic map");

  delete[] map;
  delete[] result;
  ROS_ERROR("cleaned temporary maps");
}

// Finds an accessible free pose on Voronoi Diagram.
geometry_msgs::Pose AutonomousTidyUp::FindFreePoseVoronoi()
{
  int i_center, j_center, index;
  float min_dist = 100.0, dist, dist_to_search = 10.0, x_temp, y_temp;
  int cell_dist_to_search = dist_to_search / map_resolution_;
  int i_min, i_max, j_min, j_max, i_min_dist, j_min_dist;

  float min_dist_for_voronoi_goal = 0.5;

  double step = 0.1;
  
  std::tie(i_center, j_center) = transformCoordinateOdomToMap(robot_pose.position.x, robot_pose.position.y);
  
  i_min = std::max(i_center - cell_dist_to_search, 0);
  j_min = std::max(j_center - cell_dist_to_search, 0);
  i_max = std::min(i_center + cell_dist_to_search, int(map_width_ - 1));
  j_max = std::min(j_center + cell_dist_to_search, int(map_height_ - 1));

  // Finds the nearest cell.
  for (int i = i_min; i < i_max; i++)
  {
    for (int j = j_min; j < j_max; j++)
    {
      if (semantic_map[MapIndicesToVectorIndex(i,j)].VORONOI_CELL
         && !semantic_map[MapIndicesToVectorIndex(i,j)].VORONOI_VISITED
          && semantic_map[MapIndicesToVectorIndex(i,j)].IS_ACCESSIBLE
          && semantic_map[MapIndicesToVectorIndex(i,j)].CELL_TYPE == kFree
         )
      {
        if (HasUnvisitedVoronoiNeighbors(i,j)) // Avoids trying to reach a single Voronoi cell.
        {
          std::tie(x_temp, y_temp) = transformCoordinateMapToOdom(i,j);
          dist = sqrt(pow(x_temp - robot_pose.position.x, 2.0) + pow(y_temp - robot_pose.position.y, 2.0));
          if (dist < min_dist && dist >= min_dist_for_voronoi_goal && PathExist(x_temp, y_temp, 0.0, 0.0, 0.0, 0.0))
          {
            min_dist = dist;
            i_min_dist = i;
            j_min_dist = j;
          }
        } else
        {
          // semantic_map[MapIndicesToVectorIndex(i,j)].VORONOI_VISITED = true;
        }
      }
    }
  }

  if (min_dist > 99.0)
  {
    ROS_WARN("DIDN'T FIND A VORONOI POSE NEAR THE ROBOT");
    visited_all_voronoi_goals = true;
    return robot_pose;
  } else
  {
    geometry_msgs::Pose pose;
    float x_min_dist, y_min_dist, angle;

    std::tie(x_min_dist, y_min_dist) = transformCoordinateMapToOdom(i_min_dist, j_min_dist);
    angle = atan2(y_min_dist - robot_pose.position.y, x_min_dist - robot_pose.position.x);

    pose.position.x = x_min_dist;
    pose.position.y = y_min_dist;
    pose.position.z = 0.0;

    pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, angle);
  
    ROS_WARN(">>>>>>>>>>FOUND A VORONOI POSE NEAR THE ROBOT");
    return pose;
  }
}

void AutonomousTidyUp::MarkUnderBaseAsVisitedVoronoi()
{
  if(block_mark_as_visited){ 
    return;
  }

  int i_min, i_max, j_min, j_max, i_center, j_center;

  std::tie(i_center, j_center) = transformCoordinateOdomToMap(robot_pose.position.x, robot_pose.position.y);
    
  std::tie(i_min, j_min) = transformCoordinateOdomToMap(robot_pose.position.x - (base_radius+BASE_EXPANSION), robot_pose.position.y - (base_radius+BASE_EXPANSION));
  std::tie(i_max, j_max) = transformCoordinateOdomToMap(robot_pose.position.x + (base_radius+BASE_EXPANSION), robot_pose.position.y + (base_radius+BASE_EXPANSION));

  i_min = std::max(i_min, 0);
  j_min = std::max(j_min, 0);
  i_max = std::min(i_max, int(map_width_ - 1));
  j_max = std::min(j_max, int(map_height_ - 1));

  for (int j = j_min; j <= j_max; ++j) {
    for (int i = i_min; i <= i_max; ++i) {
      int map_index = i + j * map_width_;
      semantic_map[map_index].VORONOI_VISITED = true;
    }
  }
}

bool AutonomousTidyUp::HasUnvisitedVoronoiNeighbors(int i_center, int j_center){

  for (int i = i_center - 1; i <= i_center + 1; i++)
  {
    for (int j = j_center - 1; j <= j_center + 1; j++)
    {
      if (semantic_map[MapIndicesToVectorIndex(i,j)].VORONOI_CELL && !semantic_map[MapIndicesToVectorIndex(i,j)].VORONOI_VISITED)
      {
        return true;
      }      
    }
  }  
  return false;
}

void AutonomousTidyUp::PubVoronoi(){
  int size = 2;
  int i, j;

  potencial_visualise_msg.header = map_recevied.header;
  potencial_visualise_msg.info = map_recevied.info;
  potencial_visualise_msg.data.resize(map_size);

  // Copy global map to msg
  for (int m = 0; m < map_height_; m++) {
    int multi = m * map_width_;
    for (int n = 0; n < map_width_; n++) {
      int index = n + multi;
      if (!semantic_map[index].IS_ACCESSIBLE)
      {
        potencial_visualise_msg.data[index] = -1;
      } else if (semantic_map[index].VORONOI_CELL && semantic_map[index].VORONOI_VISITED)
      {
        potencial_visualise_msg.data[index] = 100;
      } else if (semantic_map[index].VORONOI_CELL)
      {
        potencial_visualise_msg.data[index] = 50;
      } else if (semantic_map[index].ROBOT_VISITED)
      {
        potencial_visualise_msg.data[index] = 75;
      } else
      {
        potencial_visualise_msg.data[index] = 0;
      }   
    }
  }

  size = 2;
  std::tie(i, j) = transformCoordinateOdomToMap(goal_point_.position.x,
                                                goal_point_.position.y);

  for (int l = j - size; l <= j + size; ++l)
    for (int k = i - size; k <= i + size; ++k) {
      if (k >= 0 && k < map_width_ && l >= 0 && l < map_height_)
        potencial_visualise_msg.data[MapIndicesToVectorIndex(k, l)] = 101;
    }

  std::tie(i, j) = transformCoordinateOdomToMap(robot_pose.position.x,
                                                robot_pose.position.y);
  potencial_visualise_msg.data[MapIndicesToVectorIndex(i, j)] = -1;

  potencial_visualise_pub.publish(potencial_visualise_msg);

}

///////////////////////////////////////
//                                   //
//           BVP FUNCTIONS           //
//                                   //
///////////////////////////////////////

// Initializes the potential field with high potential in obstacles and inaccessible regions. Low potential in accessible unvisited regions.
void AutonomousTidyUp::InitializePotentialField()
{
  MarkUnderBaseAsVisited();

  for (int j = 0; j < map_height_; ++j) {
    for (int i = 0; i < map_width_; ++i) {
      int map_index = i + j * map_width_;
      if(semantic_map[map_index].CELL_TYPE == kObstacle || semantic_map[map_index].CELL_TYPE == kExpandedObstacle
         || semantic_map[map_index].CELL_TYPE == kCostmapObstacle || semantic_map[map_index].CELL_TYPE == kUnknown
         || semantic_map[map_index].OBJECT_TYPE == ObjectTable || !semantic_map[map_index].IS_ACCESSIBLE){
        semantic_map[map_index].POTENTIAL = 1.0;
        semantic_map[map_index].BOUNDARY_CONDITION = true;
      }else if(semantic_map[map_index].ROBOT_VISITED == false){
        semantic_map[map_index].POTENTIAL = 0;
        semantic_map[map_index].BOUNDARY_CONDITION = true;
      }else{
        semantic_map[map_index].BOUNDARY_CONDITION = false;
      }
    }
  }

  if (last_goal_not_valid)
  {
    MarkAsHightPot(goal_point_not_valid.position.x, goal_point_not_valid.position.y);
  }

  int i,j;
  std::tie(i, j) = transformCoordinateOdomToMap(robot_pose.position.x, robot_pose.position.y);

  int size=1;
  for(int count_i=i-size; count_i<=i+size; count_i++)
    for(int count_j=j-size; count_j<=j+size; count_j++){
      int map_index = count_i + count_j * map_width_;
      semantic_map[map_index].POTENTIAL = 1.0;
      semantic_map[map_index].BOUNDARY_CONDITION = true;
    }
}

void AutonomousTidyUp::MarkUnderBaseAsVisited()
{
  if(block_mark_as_visited){ 
    return;
  }

  int i_min, i_max, j_min, j_max, i_center, j_center;

  std::tie(i_center, j_center) = transformCoordinateOdomToMap(robot_pose.position.x, robot_pose.position.y);
    
  std::tie(i_min, j_min) = transformCoordinateOdomToMap(robot_pose.position.x - (base_radius+BASE_EXPANSION), robot_pose.position.y - (base_radius+BASE_EXPANSION));
  std::tie(i_max, j_max) = transformCoordinateOdomToMap(robot_pose.position.x + (base_radius+BASE_EXPANSION), robot_pose.position.y + (base_radius+BASE_EXPANSION));

  i_min = std::max(i_min, 0);
  j_min = std::max(j_min, 0);
  i_max = std::min(i_max, int(map_width_ - 1));
  j_max = std::min(j_max, int(map_height_ - 1));

  for (int j = j_min; j <= j_max; ++j) {
    for (int i = i_min; i <= i_max; ++i) {
      int map_index = i + j * map_width_;
      semantic_map[map_index].ROBOT_VISITED = true;
      #if STRATEGY_GRID
      if(semantic_map[map_index].CELL_TYPE != kInvalidBasePose)
      #endif
        semantic_map[map_index].CELL_TYPE = kFree;
      }
    }
}

// Field updated using the relaxation method of Gauss-Seidel, considering finite-difference approximation.
void AutonomousTidyUp::UpdatePotentialField()
{
  double left, right, up, down;

  for (int j = 0; j < map_height_; ++j) {
    for (int i = 0; i < map_width_; ++i) {
      int map_index = i + j * map_width_;
      if(semantic_map[map_index].BOUNDARY_CONDITION == false){
        left  = semantic_map[map_index-1].POTENTIAL;
        right = semantic_map[map_index+1].POTENTIAL;
        up    = semantic_map[map_index-map_width_].POTENTIAL;
        down  = semantic_map[map_index+map_width_].POTENTIAL;
        semantic_map[map_index].POTENTIAL = (left + right + up + down)/4.0;
      }
    }
  }
}

// Computes the next goal on the potential field.
void AutonomousTidyUp::ComputeNextGoalInBVP()
{
  float diff = sqrt(pow(goal_point_.position.x - robot_pose.position.x, 2)
                  + pow(goal_point_.position.y - robot_pose.position.y, 2));
  int goal_i, goal_j;
  std::tie(goal_i, goal_j) = transformCoordinateOdomToMap(goal_point_.position.x,
                                                          goal_point_.position.y);

  ros::spinOnce();
  ROS_WARN("STATE ....... %s", (valid_status_last_goal? "ok" : "NOT VALID"));
  if (!PathExist(goal_point_) or !valid_status_last_goal)
  { // Invalid goal.
    ac->cancelGoal();
    SendAndSaveGoalBaseWithoutWait(robot_pose);
    ros::spinOnce();
  } else
    if(diff>0.5 && ViableGoal(goal_point_.position.x, goal_point_.position.y) && NoTablesNear(goal_i, goal_j))
      return; // Still going to the goal.

  int robot_i, robot_j;
  std::tie(robot_i, robot_j) = transformCoordinateOdomToMap(robot_pose.position.x,
                                                            robot_pose.position.y);
  
  int current_r_position_ = robot_i + robot_j * map_width_;
  int range = 10;
  int sqr_range=range*range;

  range = 1.5 / map_resolution_;
  sqr_range=range*range;
  int min_range = 10;
  int sqr_min_range=min_range*min_range;

  double min_pot=2;
  int chosen_index=-1, chosen_i = -1, chosen_j = -1;

  // Finds minimum potential.
  int k=3;
  if(chosen_i==-1 && chosen_j==-1){
    ROS_WARN("normal search");

    for (int j = -range; j <= range; ++j) {
      for (int i = -range; i <= range; ++i) {
        if (robot_i + i - k >= 0 && robot_i + i + k < map_width_ && robot_j + j - k >= 0 && robot_j + j +k < map_height_) {
          int sqr_dist = i*i + j*j;
          if(sqr_dist>sqr_range || sqr_dist<sqr_min_range)
            continue;

          int map_index = (robot_i + i) + (robot_j+j) * map_width_;

          if(
            #if STRATEGY_BVP_PLUS_VORONOI
            (semantic_map[map_index].VORONOI_CELL && semantic_map[map_index].CELL_TYPE == kFree &&
            semantic_map[map_index].POTENTIAL <= min_pot) || // voronoi cell has priority
            #endif
            (semantic_map[map_index].CELL_TYPE == kFree &&
            semantic_map[map_index].POTENTIAL < min_pot)){
            chosen_index = map_index;
            chosen_i = robot_i + i;
            chosen_j = robot_j + j;
            min_pot = semantic_map[map_index].POTENTIAL;
          }
        }
      }
    }
  }

  // Computing gradient descent.
  double left, right, up, down;
  left  = semantic_map[chosen_index-k].POTENTIAL;
  right = semantic_map[chosen_index+k].POTENTIAL;
  up    = semantic_map[chosen_index-k*map_width_].POTENTIAL;
  down  = semantic_map[chosen_index+k*map_width_].POTENTIAL;

  double dirX = -(right - left);
  double dirY = (up - down);
  double norm = sqrt(pow(dirX,2.0)+pow(dirY,2.0));
  if(norm>0){
      dirX /= norm;
      dirY /= norm;
  }else{
      dirX = dirY = 0;
  }

  float angle = atan2(dirY,dirX);
  ROS_WARN("norm %f dirX %f dirY %f Angle %f %f",norm,dirX,dirY,angle, angle*180.0/M_PI);

  // checking whether no unvisited point has been found, and hence, the exploration has finished
  if (norm < 1e-20) {  // it means that all points within the map have been visited, and
                            // hence, there's no attractive potential. 
                            // The algorithm should stop
    ROS_WARN(" --------- BVP EXPLORATION finished !--------- ");
    SendAndSaveGoalBaseWithoutWait(robot_pose);
    last_goal_not_valid = false;
    flat_potential_field++;
    return;
  } else
  {
    flat_potential_field = 0;
  }    

  int temp_i, temp_j;
  if (chosen_index != -1) {
    temp_i = chosen_i;
    temp_j = chosen_j;
    float x_goal, y_goal;
    std::tie(x_goal, y_goal) = transformCoordinateMapToOdom(temp_i, temp_j);

    // Better than gradient orientation, avoids turns.
    float angle = atan2(y_goal - robot_pose.position.y, x_goal - robot_pose.position.x);
    
    if (PathExist(x_goal, y_goal, 0.0, 0.0, 0.0, angle) && NoTablesNear(temp_i, temp_j))
    {
      SendAndSaveGoalBaseWithoutWait(x_goal, y_goal, 0.0, 0.0, 0.0, angle);
      last_goal_not_valid = false;
    }
      else
    {
      ROS_WARN("Goal NOT VALID");

      SaveGoalNotValid(x_goal, y_goal, 0.0, 0.0, 0.0, angle);
      last_goal_not_valid = true;
        
      MarkAsObstacle(x_goal, y_goal);
    }      
  }
}

void AutonomousTidyUp::MarkAsExpandedObstacle(float x, float y)
{
  int i,j;
  std::tie(i, j) = transformCoordinateOdomToMap(x, y);

  int size= SIZE_TO_MARK / map_resolution_;
  float temp_x, temp_y;

  for(int count_i=i-size; count_i<=i+size; count_i++)
    for(int count_j=j-size; count_j<=j+size; count_j++){        
      std::tie(temp_x, temp_y) = transformCoordinateMapToOdom(count_i,count_j);
      if (InsideMap(temp_x, temp_y))
      {
        int map_index = count_i + count_j * map_width_;
        if (semantic_map[map_index].CELL_TYPE == kFree)
        {
          semantic_map[map_index].CELL_TYPE = kExpandedObstacle;
        }
      }        
    }
  
  return;
}

void AutonomousTidyUp::MarkAsInvalidBasePose(float x, float y)
{
  int i,j;
  std::tie(i, j) = transformCoordinateOdomToMap(x, y);

  #if STRATEGY_GRID
  int size= 1.0 * SIZE_TO_MARK / map_resolution_;
  #else
  int size= 2.0 * SIZE_TO_MARK / map_resolution_;
  #endif
  float temp_x, temp_y;

  for(int count_i=i-size; count_i<=i+size; count_i++)
    for(int count_j=j-size; count_j<=j+size; count_j++){        
      std::tie(temp_x, temp_y) = transformCoordinateMapToOdom(count_i,count_j);
      if (InsideMap(temp_x, temp_y))
      {
        int map_index = count_i + count_j * map_width_;
        if (semantic_map[map_index].CELL_TYPE == kFree)
        {
          semantic_map[map_index].CELL_TYPE = kInvalidBasePose;
        }
      }
    }

  return;
}

void AutonomousTidyUp::MarkAsObstacle(float x, float y)
{
  int i,j;
  std::tie(i, j) = transformCoordinateOdomToMap(x, y);

  int size= SIZE_TO_MARK / map_resolution_;
  float temp_x, temp_y;

  for(int count_i=i-size; count_i<=i+size; count_i++)
    for(int count_j=j-size; count_j<=j+size; count_j++){        
      std::tie(temp_x, temp_y) = transformCoordinateMapToOdom(count_i,count_j);
      if (InsideMap(temp_x, temp_y))
      {
        int map_index = count_i + count_j * map_width_;
        if (semantic_map[map_index].CELL_TYPE == kFree)
        {
          semantic_map[map_index].CELL_TYPE = kObstacle;
        }
      }        
    }

  return;
}

void AutonomousTidyUp::MarkAsHightPot(float x, float y)
{
  int i,j;
  std::tie(i, j) = transformCoordinateOdomToMap(x, y);

  int size= SIZE_TO_MARK / map_resolution_;
  float temp_x, temp_y;

  for(int count_i=i-size; count_i<=i+size; count_i++)
    for(int count_j=j-size; count_j<=j+size; count_j++){        
      std::tie(temp_x, temp_y) = transformCoordinateMapToOdom(count_i,count_j);
      if (InsideMap(temp_x, temp_y))
      {
        int map_index = count_i + count_j * map_width_;
        semantic_map[map_index].POTENTIAL = 1.0;
        semantic_map[map_index].BOUNDARY_CONDITION = true;
      }        
    }
  
  return;
}

bool AutonomousTidyUp::ViableGoal(float x, float y)
{
  int i,j;
  std::tie(i, j) = transformCoordinateOdomToMap(x, y);

  int size= SIZE_TO_MARK / map_resolution_;

  if (!InsideMap(x - map_resolution_ * float(size), y - map_resolution_ * float(size))
   or !InsideMap(x + map_resolution_ * float(size), y + map_resolution_ * float(size))) return false;

  for(int count_i=i-size; count_i<=i+size; count_i++)
    for(int count_j=j-size; count_j<=j+size; count_j++){
      int map_index = count_i + count_j * map_width_;
      if (semantic_map[map_index].CELL_TYPE != kFree or semantic_map[map_index].OBJECT_TYPE != NoObject)
      {
        return false;
      }
    }

  return true;
} 

// It creates the map message from the potential field to visualize it.
void AutonomousTidyUp::PubPotentialMap() {
  int size = 2;
  int i, j;

  potencial_visualise_msg.header = map_recevied.header;
  potencial_visualise_msg.info = map_recevied.info;
  potencial_visualise_msg.data.resize(map_size);

  // Copy global map to msg
  for (int m = 0; m < map_height_; m++) {
    int multi = m * map_width_;
    for (int n = 0; n < map_width_; n++) {
      int index = n + multi;
      potencial_visualise_msg.data[index] = semantic_map[index].POTENTIAL*100;
    }
  }

  size = 2;
  std::tie(i, j) = transformCoordinateOdomToMap(goal_point_.position.x,
                                                goal_point_.position.y);

  for (int l = j - size; l <= j + size; ++l)
    for (int k = i - size; k <= i + size; ++k) {
      if (k >= 0 && k < map_width_ && l >= 0 && l < map_height_)
        potencial_visualise_msg.data[MapIndicesToVectorIndex(k, l)] = 101;
    }

  std::tie(i, j) = transformCoordinateOdomToMap(robot_pose.position.x,
                                                  robot_pose.position.y);
  potencial_visualise_msg.data[MapIndicesToVectorIndex(i, j)] = -1;
  potencial_visualise_pub.publish(potencial_visualise_msg);
}

///////////////////////////////////////
//                                   //
//              Control              //
//                                   //
///////////////////////////////////////

// Explores the environment aiming to find objects on the floor.
void AutonomousTidyUp::ExploreToFindObjects()
{
  #if SIMULATION
  set_local_planner_max_vel(0.15, 0.3);
  #else
  set_local_planner_max_vel(0.15, 0.3);
  #endif

  int voronoi_counter = 0;
  visited_all_voronoi_goals = false;
  visited_all_grid_goals = false;

  semantic_map_visualise_msg.header = map_recevied.header;
  semantic_map_visualise_msg.info = map_recevied.info;
  semantic_map_visualise_msg.data.resize(map_size);

  ROS_WARN("ExploreToFindObjects %d", global_counter);

  #if SIMULATION
  StartTableDetection();
  #endif
  StartObjectDetection();

  SendAndSaveGoalBase(robot_pose);

  block_mark_as_visited = false;
  first_interation = true;

  // Adjust height for exploration.
  LiftTorsoJoint(0.1);
  step_head_motion = 0;

  int initial_steps = 0;

  geometry_msgs::Pose second_robot_pose = robot_pose;
  second_robot_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, constrainAngle(ComputeYaw(robot_pose.orientation) + M_PI));

  // Starts exploration.
  ChangeStage(StageExploration);
  #if STRATEGY_BVP_CLASSIC
  flat_potential_field = 0;
  #endif

  // To clean previous invalid poses.
  #if (STRATEGY_GRID or STRATEGY_VORONOI)
  for (int i = 0; i < map_size; i++)
  {
    if (semantic_map[i].CELL_TYPE == kInvalidBasePose)
    {
      semantic_map[i].CELL_TYPE = kFree;
    }
  }  
  #endif

  while (ros::ok())
  {
    global_counter++;
    ros::Duration(0.25).sleep();
    ros::spinOnce();

    if (first_interation) // Initial turn.
    {
      if (!block_all_movement)
      {
      #if SIMULATION
      SendMotionArm("BIGhead_look_aroundSLOW2");
      ros::Duration(0.25).sleep();
      ros::spinOnce();
      #if not BASE_STATIC
      SendAndSaveGoalBase(robot_pose.position.x, robot_pose.position.y, robot_pose.position.z, 0.0, 0.0, constrainAngle(ComputeYaw(robot_pose.orientation) + M_PI));
      #endif
      ros::Duration(0.25).sleep();
      ros::spinOnce();
      SendMotionArm("BIGhead_look_aroundSLOW2");
      first_interation = false;
      stage = StageExploration;
      step_head_motion = 0;
      #else
        if( !SendBIGhead_look_aroundSLOW2_steps() ) continue;
        initial_steps++;
        ros::Duration(0.25).sleep();
        ros::spinOnce();
        if (initial_steps == 8)
        {
          #if not BASE_STATIC
          do
          {
            ROS_WARN("Send second_robot_pose");
            if (!block_all_movement) SendAndSaveGoalBase(second_robot_pose);
            ros::Duration(0.25).sleep();
            ros::spinOnce();
          } while (abs(constrainAngle(tf::getYaw(robot_pose.orientation)) - constrainAngle(tf::getYaw(second_robot_pose.orientation)))
                  >= 0.25 && ros::ok());        
          #endif
        }

        if (initial_steps == 16)
        {
          first_interation = false;
          step_head_motion = 0;
        }
        
      #endif
      }
      timerContinuousHeadMovement.start();
      float coverage_init = CoverageAcessibleArea();
      ROS_ERROR("INITIAL COVERAGE: %f PERCENT OF THE AREA", (100.0)*coverage_init);

      #if STRATEGY_VORONOI
      voronoi_counter++;
      MarkUnderBaseAsVisitedVoronoi();
      ComputeVoronoiDiagram();
      #endif
      continue;    
    }

    AddObjectsAndTablesReceviedToSemanticMap();
    PubSemanticMapVisualise();

    ///////////////// BVP /////////////////

    #if STRATEGY_BVP_CLASSIC

    #if STRATEGY_BVP_PLUS_VORONOI
    ComputeVoronoiDiagram();
    #endif

    InitializePotentialField();
    // Smoothes the field.
    for(int p=0;p<200;p++)
      UpdatePotentialField();
    PubPotentialMap();

    #if not BASE_STATIC
    if (!block_all_movement)
    {
      ComputeNextGoalInBVP();
    }
    #endif

    ///////////////// Voronoi /////////////////

    #elif STRATEGY_VORONOI
    MarkUnderBaseAsVisitedVoronoi();

    // Computes Voronoi diagram every 10 iterations to avoid frequent fluctuations.
    voronoi_counter++;
    if(voronoi_counter == 10){
      ROS_ERROR("CALCULATES VORONOI");
      ComputeVoronoiDiagram();
      voronoi_counter = 0; 
    }

    PubVoronoi();
    int i_v, j_v;
    std::tie(i_v, j_v) = transformCoordinateOdomToMap(goal_point_.position.x, goal_point_.position.y);
    if(!PathExist(goal_point_) or !semantic_map[MapIndicesToVectorIndex(i_v,j_v)].IS_ACCESSIBLE or semantic_map[MapIndicesToVectorIndex(i_v,j_v)].CELL_TYPE != kFree){
      MarkAsInvalidBasePose(goal_point_.position.x, goal_point_.position.y);
    }

    ROS_ERROR("VORONOI goal counter %d", voronoi_counter);
    SendAndSaveGoalBaseWithoutWait(FindFreePoseVoronoi());

    if(visited_all_voronoi_goals)
    {
      ROS_ERROR("Voronoi: FINAL COVERAGE: %f PERCENT OF THE AREA", (100.0)*CoverageAcessibleArea());
      break;
    }

    ///////////////// Grid /////////////////

    #elif STRATEGY_GRID

    int i_goal, j_goal;
    float min_dist = 1.0;
    std::tie(i_goal, j_goal)=transformCoordinateOdomToMap(goal_point_.position.x, goal_point_.position.y);

    if(!PathExist(goal_point_) || !FreeArea(i_goal, j_goal, min_dist)){
      MarkAsInvalidBasePose(goal_point_.position.x, goal_point_.position.y);
      ROS_ERROR("Goal without plan or not free area");
      ac->cancelAllGoals();
      SendAndSaveGoalBase(robot_pose);
    }

    float dist = sqrt(pow(goal_point_.position.x - robot_pose.position.x, 2.0) + pow(goal_point_.position.y - robot_pose.position.y, 2.0));
    float yaw_robot, yaw_goal_point;

    yaw_robot = ComputeYaw(robot_pose.orientation);
    yaw_goal_point = ComputeYaw(goal_point_.orientation);
    PubVisitedGrid();

    if ((dist < 0.2 && abs(constrainAngle(yaw_robot - yaw_goal_point)) < M_PI/4.0) || !PathExist(goal_point_))
      if(SimpleTrajectory(min_dist)) // Returns true when the strategy is over.
      {
        ROS_ERROR("Grid: FINAL COVERAGE: %f PERCENT OF THE AREA", (100.0)*CoverageAcessibleArea());
        visited_all_grid_goals = true;
        break;
      }

    ////////////////////////////////////////
    #else
    ROS_ERROR("WITHOUT STRATEGY");
    PubJustCoverage();
    #endif

    float coverage = CoverageAcessibleArea();
    ROS_ERROR("FINAL COVERAGE: %f PERCENT OF THE AREA ", (100.0)*coverage);

    #if EXPLORE_THEN_TIDY_UP
    if (coverage >= 1.0)
    #else
    if (ObjectAndTableAvailable() or coverage >= 1.0)
    #endif
    {
      break;
    }    

    #if STRATEGY_BVP_CLASSIC
    if (flat_potential_field >= 5)
    {
      ROS_ERROR("FLAT POTENTIAL FIELD");
      break;
    }
    
    #endif
    
    RemoveObjectsAndTablesReceviedToSemanticMap();

    ros::Duration(0.25).sleep();
    ros::spinOnce();
  }

  PrintInfoAllTables();

  // Stops the robot.
  SendAndSaveGoalBase(robot_pose);
  timerContinuousHeadMovement.stop();
  client_arm->cancelAllGoals();
  do {
    ros::Duration(0.25).sleep();
    ros::spinOnce();
  } while(!client_arm->getState().isDone());
  ROS_WARN("Head stoped");

  // Stops detection.
  StopTableDetection();
  StopObjectDetection();

  // Publishes the latest data.

  AddObjectsAndTablesReceviedToSemanticMap();
  PubSemanticMapVisualise();
  RemoveObjectsAndTablesReceviedToSemanticMap();

  #if STRATEGY_BVP_CLASSIC
  InitializePotentialField();
  for(int p=0;p<200;p++)
    UpdatePotentialField();
  PubPotentialMap();
  #endif

  #if STRATEGY_VORONOI
  MarkUnderBaseAsVisitedVoronoi();
  PubVoronoi();
  #endif

  #if STRATEGY_GRID
  PubVisitedGrid();
  #endif

  PrintInfoAllTables();

  ROS_WARN("------------ExploreToFindObjects OK %d------------", global_counter);  
}

// Manipulates the found objects on the floor by picking and placing them in tables.
void AutonomousTidyUp::CollectAllObjectsOnTheFloor()
{
  ChangeStage(StageManipulation);

  PrintStateAllObjects();
  PrintStateAllTables();

  ros::Duration(5.0).sleep();
  ros::spinOnce();
  geometry_msgs::Pose pose;
  moveit_msgs::CollisionObject object, table;

  std::vector<geometry_msgs::Pose> poses_to_revisit;

  moveit_msgs::CollisionObject object_to_place;
  bool success = false;

  no_acessible_objects = false;

  block_mark_as_visited = true;

  #if SIMULATION
  set_local_planner_max_vel(0.5, 0.6);
  #else
  set_local_planner_max_vel(0.5, 0.6);
  #endif

  ROS_WARN("CollectAllObjectsOnTheFloor %d objects", RemainingObjectsToPick());

  while (ros::ok())
  {
    global_counter++;
    if(global_counter >= 1000001)
      global_counter = 1;
    
    // Cleans the planning scene.
    planning_scene_interface.removeCollisionObjects(planning_scene_interface.getKnownObjectNames());
    ros::Duration(0.25).sleep();
    ros::spinOnce();
    planning_scene_interface.removeCollisionObjects(planning_scene_interface.getKnownObjectNames());
    ros::Duration(0.25).sleep();
    ros::spinOnce();

    AddObjectsAndTablesReceviedToSemanticMap();
    PubSemanticMapVisualise();
    RemoveObjectsAndTablesReceviedToSemanticMap();

    ros::Duration(0.25).sleep();
    ros::spinOnce();

    if (RemainingObjectsToPick() == 0)
    { // All objects collected.
      break;
    }

    int attempts = 0;

    LiftTorsoJoint(0.1);
    do
    {
      SendAndSaveGoalBase(robot_pose);
      SendMotionArm("look_down_to_walk");
      LiftTorsoJoint(0.1);
      std::tie(pose, object) = GoalNearestObjectOnFloor();
      if (object.id.compare(string("no_object")) == 0)
      {
        ROS_ERROR("-----------NO ACCESSIBLE OBJECT ON THE FLOOR 2");
        no_acessible_objects = true;
        return;
      }
      
      StartObjectDetection();
      #if SIMULATION
      StartTableDetection();
      #endif

      ros::Duration(0.25).sleep();
      ros::spinOnce();
    } while (!SendAndSaveGoalBase(pose) && ros::ok()); // Goes near an object.
    poses_to_revisit.push_back(pose);

    attempts = 0;
    do
    { 
      // Lowers the torso.
      LiftTorsoJoint(0.04);
      WaitToContinueExecution();
      ros::Duration(0.25).sleep();
      ros::spinOnce();

      // Looks around to be aware of the manipulation environment.
      UnblockPlanningScene();
      StartObjectDetection();
      #if SIMULATION
      StartTableDetection();
      #endif
      StartInspectSurroundings();
      ros::Duration(5.00).sleep();
      ros::spinOnce();
      WaitToPlanningSceneReady();
      ros::Duration(0.25).sleep();
      ros::spinOnce();

      attempts++;
      ROS_ERROR("ATTEMPT %d", attempts);
      success = DetectObjectAgainAndPick();

    } while (!success && attempts < 2);

    int object_index;

    for (int it = 0; it < objects_on_the_floor_recevied.size(); it++)
    {
      if (objects_on_the_floor_recevied[it].state == StateHolding)
      {
        ROS_WARN("Caught %s", objects_on_the_floor_recevied[it].object.id.c_str());
        object_to_place = objects_on_the_floor_recevied[it].object;
        object_index = it;
      }
    }

    // Possible results after DetectObjectAgainAndPick returns for the manipulated object:
    // NO OBJECT IN OCTOMAP : delete object from list (mark as invalid) and try with other objects in range
    // FAILED TO CALCULATE GRASPS : (mark as invalid grasp) and try with the base in a different pose
    // FAILURE WHEN MOVING ARM : problem in planning: (mark as invalid plan) and try with the base in a different pose
    //                           problem in execution: try again one more time and (mark as invalid)
    // FAILED TO GRAB OBJECT (GRABBED NOTHING) : probably dropped or didn't catch, (mark as not gripped) delete from list
    // CATCHED THE OBJECT correctly::::::::::: (mark as holding) and then the object is sent to the DetectTable

    AddObjectsAndTablesReceviedToSemanticMap();
    PubSemanticMapVisualise();
    RemoveObjectsAndTablesReceviedToSemanticMap();
    ros::Duration(0.25).sleep();
    ros::spinOnce();

    StopObjectDetection();
    StopTableDetection();

    int table_index;
    if (success)
    { // Goes near a table.
      attempts = 0;
      do
      { 
        do
        {
          SendAndSaveGoalBase(robot_pose);
          SendMotionArm("look_down_to_walk");
          LiftTorsoJoint(0.1);
          std::tie(pose, table, table_index) = GoalNearestTable();
          ROS_WARN("Goal on x: %f e y: %f", pose.position.x, pose.position.y);
          ros::Duration(0.25).sleep();
          ros::spinOnce();
          if (table_index == -1)
          {
            ROS_WARN("DIDN'T FIND A POSE NEAR ANY TABLE");
            pose = robot_pose;
            WaitToContinueExecution();
          }
          
        } while (!SendAndSaveGoalBase(pose) && ros::ok());
      
        LiftTorsoJoint(0.34);
        UnblockPlanningScene();
        // Looks around to be aware of the manipulation environment.
        StartInspectSurroundings();
        attempts++;
        success = DetectTableAgainAndPlace(table, object_to_place, object_index, table_index);      

      if(!success){
        ROS_WARN("failed to place %s on table %s, it will try again with the base in another pose", object_to_place.id.c_str(), table.id.c_str());
      }
      } while (!success && attempts < MAX_TRYS);

      if(!success){
        ROS_WARN("failed to place %s on any table", object_to_place.id.c_str());
      }
    } else
    {
      ROS_WARN("unsuccessful in picking up %s", object_to_place.id.c_str());
    }
    
    ROS_WARN("CollectAllObjectsOnTheFloor %d", global_counter);
  }

  SendMotionArm("look_down_to_walk");
  LiftTorsoJoint(0.1);
  ROS_WARN("CollectAllObjectsOnTheFloor ok, %d objects to pick", RemainingObjectsToPick());

  block_mark_as_visited = false;
  PrintStateAllObjects();
  PrintStateAllTables();
}

// Focuses on the robot front to detect the objects again more precisely and picks a reachable object.
bool AutonomousTidyUp::DetectObjectAgainAndPick()
{
  bool valid = false;
  holding_object = false;

  // Focuses on the robot front.
  bool result;
  do
  {
    result = SendMotionArm("look_down_detect_again");
    ros::Duration(1.0).sleep();
    ros::spinOnce();      
  } while (result == false && ros::ok());
  
  // Detects the objects again more precisely because the camera is static.
  StartObjectDetection();
  #if SIMULATION
  StartTableDetection();
  #endif
  ros::spinOnce();
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  StopObjectDetection();
  StopTableDetection();
  ros::Duration(1.0).sleep();
  ros::spinOnce();

  AddObjectsAndTablesReceviedToSemanticMap();
  PubSemanticMapVisualise();
  RemoveObjectsAndTablesReceviedToSemanticMap();
  ros::Duration(1.0).sleep();
  ros::spinOnce();

  // Adds nearby tables to help with arm movement, avoiding hitting table legs.
  float table_max_dist = 3.0, dist_table_robot;
  moveit_msgs::CollisionObject table, safety_box_on_floor;
  for (int i = 0; i < tables_recevied.size(); i++)
  {
    tables_recevied[i].object.id.resize(7);
    if (i< 10)
    {
      tables_recevied[i].object.id[5] = '0';
    } else
    {
      tables_recevied[i].object.id[5] = '0' + int(i/10.0);
    }        
    tables_recevied[i].object.id[6] = '0' + int(i%10);        

    table = tables_recevied[i].object;
    dist_table_robot = sqrt(pow(table.primitive_poses[0].position.x - robot_pose.position.x, 2.0)
              + pow(table.primitive_poses[0].position.y - robot_pose.position.y, 2.0));
    if (dist_table_robot < table_max_dist)
    {
      table.operation = table.ADD;
      planning_scene_interface.applyCollisionObject(table);
      ros::Duration(0.25).sleep();
      ros::spinOnce();
      planning_scene_interface.applyCollisionObject(table);
      ros::Duration(0.25).sleep();
      ros::spinOnce();
      planning_scene_interface.applyCollisionObject(table);
      printSceneObjects();
      ros::Duration(0.5).sleep();
      ros::spinOnce();
    }
  }

  // Safety box to avoid hitting the floor.
  safety_box_on_floor.header.frame_id = "map";
  safety_box_on_floor.header.stamp = ros::Time::now();
  safety_box_on_floor.primitive_poses.resize(1);
  safety_box_on_floor.primitive_poses[0].orientation.w = 1.000000;
  safety_box_on_floor.primitive_poses[0].orientation.x = 0.000000;
  safety_box_on_floor.primitive_poses[0].orientation.y = 0.000000;
  safety_box_on_floor.primitive_poses[0].orientation.z = -0.000000;
  safety_box_on_floor.primitive_poses[0].position = robot_pose.position;
  safety_box_on_floor.primitive_poses[0].position.z = -0.05;
  safety_box_on_floor.primitives.resize(1);
  safety_box_on_floor.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  safety_box_on_floor.primitives[0].dimensions.resize(3);
  safety_box_on_floor.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 2.0;
  safety_box_on_floor.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 2.0;
  safety_box_on_floor.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.13;
  safety_box_on_floor.operation = safety_box_on_floor.ADD;
  safety_box_on_floor.id.clear();
  safety_box_on_floor.id = string("safety_box");
  planning_scene_interface.applyCollisionObject(safety_box_on_floor);
  ros::Duration(0.25).sleep();
  ros::spinOnce();
  planning_scene_interface.applyCollisionObject(safety_box_on_floor);
  ros::Duration(0.25).sleep();
  ros::spinOnce();
  planning_scene_interface.applyCollisionObject(safety_box_on_floor);
  printSceneObjects();
  ros::Duration(0.5).sleep();
  ros::spinOnce();

  holding_object = false;

  std::map<float, moveit_msgs::CollisionObject> objects_on_reach;
  std::map<float, int> index_objects_on_reach;
  moveit_msgs::CollisionObject object, object_to_pick;
  std::vector<float> dist_array;
  float max_ang = M_PI / 3.0, max_reach = 0.85, dist, ang, robot_ang;
  robot_ang = ComputeYaw(robot_pose.orientation);

  // Selects objects on reach in front of the robot.
  for (int i = 0; i < objects_on_the_floor_recevied.size(); i++)
  {
    if (!ValidObjectOnFloor(i))
    {
      continue;
    }
    object = objects_on_the_floor_recevied[i].object;
    dist = sqrt(pow(object.primitive_poses[0].position.x - robot_pose.position.x, 2.0)
              + pow(object.primitive_poses[0].position.y - robot_pose.position.y, 2.0));
    if (dist > max_reach)
    {
      continue;
    }
    
    ang = atan2(object.primitive_poses[0].position.y - robot_pose.position.y,
                object.primitive_poses[0].position.x - robot_pose.position.x);
    if (abs(constrainAngle(ang - robot_ang)) < max_ang)
    {
      dist_array.push_back(dist);
      objects_on_reach.insert({dist, object});
      index_objects_on_reach.insert({dist, i});
    } else
    {
      ROS_WARN("Out of the angle: %f degrees", RAD2DEG(constrainAngle(ang - robot_ang)));
    }
    
  }

  if (objects_on_reach.size() == 0)
  {
    ROS_WARN("NO OBJECTS ON REACH");
    return false;
  } 

  std::sort(dist_array.begin(), dist_array.end());

  int index;

  // Selects the closest one and add it to the planning scene.
  for(int i = 0; i < dist_array.size() && !valid; i++)
  {
    object_to_pick = objects_on_reach[dist_array[i]];
    index = index_objects_on_reach[dist_array[i]];

    object_to_pick.operation = object_to_pick.ADD;

    object_to_pick.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] += OBJECT_EXPANSION;
    object_to_pick.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] += OBJECT_EXPANSION;
    object_to_pick.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] += OBJECT_EXPANSION;
    object_to_pick.primitive_poses[0].position.z += 0.01 + OBJECT_EXPANSION/2.0;

    object_to_pick.id.clear();
    object_to_pick.id = string("object1");

    ROS_WARN("OBJECT on frame %s", object_to_pick.header.frame_id.c_str());

    planning_scene_interface.applyCollisionObject(object_to_pick);
    ros::Duration(0.25).sleep();
    ros::spinOnce();
    planning_scene_interface.applyCollisionObject(object_to_pick);
    ros::Duration(0.25).sleep();
    ros::spinOnce();
    planning_scene_interface.applyCollisionObject(object_to_pick);
    printSceneObjects();
    ros::Duration(0.5).sleep();
    ros::spinOnce();
    UnblockPlanningScene();
    ros::Duration(5.0).sleep();
    ros::spinOnce();

    // Validates the object on the octomap.
    StartOnlyPubOctomap();
    WaitToPlanningSceneReady();

    if (!object_on_octomap)
    { // There is not any substance in the object's place, it was a wrong detection.
      BlockPlanningScene();
      object_to_pick.operation = object_to_pick.REMOVE;
      planning_scene_interface.applyCollisionObject(object_to_pick);
      ros::Duration(0.25).sleep();
      ros::spinOnce();
      planning_scene_interface.applyCollisionObject(object_to_pick);
      ros::Duration(0.25).sleep();
      ros::spinOnce();
      planning_scene_interface.applyCollisionObject(object_to_pick);
      printSceneObjects();
      ros::Duration(0.5).sleep();
      ros::spinOnce();
      UnblockPlanningScene();

      ROS_WARN("OBJECT REMOVED FROM PLANNING SCENE");
      WaitToContinueExecution();

      ChangeObjectState(index, StateInvalid);

      valid = false;
      continue; // Goes to the next object.
    }    

    BlockPlanningScene();

    // Picks up the validated object.
    MovementOpenEndEffector();
    WaitToContinueExecutionCritical();
    valid = PickObject(object_to_pick.id, index);
    ObjectState final_state;
    final_state = objects_on_the_floor_recevied[index].state;
    if (!valid)
    {
      if (final_state == StateInvalidGrasp or final_state == StateInvalidPlan)
      {
        // Marks the robot's current position as an expansion in the semantic map to force the base to be placed in another position in the next attempt.
        ROS_WARN("%s is it invalid plan or grasp<<<<", objects_on_the_floor_recevied[index].object.id.c_str());
        MarkAsInvalidBasePose(robot_pose.position.x, robot_pose.position.y);
        ChangeObjectState(index, StateOnFloor);
      }
    }
  }  

  // Removes nearby tables to help with arm movement. The holding object is not removed.
  planning_scene_interface.removeCollisionObjects(planning_scene_interface.getKnownObjectNames());
  ros::Duration(0.25).sleep();
  ros::spinOnce();
  planning_scene_interface.removeCollisionObjects(planning_scene_interface.getKnownObjectNames());
  ros::Duration(0.25).sleep();
  ros::spinOnce();  

  return valid;
}

// Places the object on the table top after detecting the reachable environment.
bool AutonomousTidyUp::DetectTableAgainAndPlace(moveit_msgs::CollisionObject table_near, moveit_msgs::CollisionObject object_to_place, int n_object, int n_table)
{  
  ros::Duration(1.0).sleep();
  ros::spinOnce();

  std::vector<double> previus_group_variable_values;
  group_arm_torso->getCurrentState()->copyJointGroupPositions(group_arm_torso->getCurrentState()->getRobotModel()->getJointModelGroup(group_arm_torso->getName()), previus_group_variable_values);
   
  bool success = false;
  moveit_msgs::CollisionObject table_to_place, safety_box_near_floor;
  std::vector<std::string> chose_object;
  std::map<std::string,moveit_msgs::CollisionObject> map_collision_objects;
  chose_object.resize(1);

  table_to_place = table_near;

  // Adds table and safety box to planning scene.

  table_to_place.operation = table_to_place.ADD;

  safety_box_near_floor.header.frame_id = "map";
  safety_box_near_floor.header.stamp = ros::Time::now();
  safety_box_near_floor.primitive_poses.resize(1);
  safety_box_near_floor.primitive_poses[0].orientation.w = 1.000000;
  safety_box_near_floor.primitive_poses[0].orientation.x = 0.000000;
  safety_box_near_floor.primitive_poses[0].orientation.y = 0.000000;
  safety_box_near_floor.primitive_poses[0].orientation.z = -0.000000;
  safety_box_near_floor.primitive_poses[0].position = robot_pose.position;
  safety_box_near_floor.primitive_poses[0].position.z = 0.25 / 2.0;
  safety_box_near_floor.primitives.resize(1);
  safety_box_near_floor.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  safety_box_near_floor.primitives[0].dimensions.resize(3);
  safety_box_near_floor.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 2.0;
  safety_box_near_floor.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 2.0;
  safety_box_near_floor.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.25;

  safety_box_near_floor.operation = safety_box_near_floor.ADD;
  safety_box_near_floor.id.clear();
  safety_box_near_floor.id = string("safety_box");

  table_to_place.id.clear();
  table_to_place.id = string("table1");
  planning_scene_interface.applyCollisionObject(table_to_place);
  ros::Duration(0.25).sleep();
  ros::spinOnce();
  planning_scene_interface.applyCollisionObject(table_to_place);
  ros::Duration(0.25).sleep();
  ros::spinOnce();
  planning_scene_interface.applyCollisionObject(table_to_place);
  printSceneObjects();
  ros::Duration(0.25).sleep();
  ros::spinOnce();

  planning_scene_interface.applyCollisionObject(safety_box_near_floor);
  ros::Duration(0.25).sleep();
  ros::spinOnce();
  planning_scene_interface.applyCollisionObject(safety_box_near_floor);
  ros::Duration(0.25).sleep();
  ros::spinOnce();
  planning_scene_interface.applyCollisionObject(safety_box_near_floor);
  printSceneObjects();
  ros::Duration(0.25).sleep();
  ros::spinOnce();

  ros::Duration(5.0).sleep();
  ros::spinOnce();

  // Waits for the planning scene octomap be ready.
  planning_scene_ready = false;
  WaitToPlanningSceneReady();

  BlockPlanningScene();
  holding_object = false;
  ros::spinOnce();
  ros::Duration(2.0).sleep();// to make sure there are no holdings left in the queue
  ros::spinOnce();
  holding_object = false;
  ros::spinOnce();
  ros::Duration(2.0).sleep();
  ros::spinOnce();
  holding_object = false;
  ros::spinOnce();
  ros::Duration(2.0).sleep();
  ros::spinOnce();
  holding_object = false;
  ros::spinOnce();
  ros::Duration(5.0).sleep();
  ros::spinOnce();
  ROS_ERROR("holding_object: %s", (holding_object? "true" : "false"));

  WaitToContinueExecutionCritical();

  // Places the object on table top.
  int attempts = 0;
  moveit::core::MoveItErrorCode place_result;
  object_to_place.id = string("object1");
  chose_object[0] = object_to_place.id;
  do
  {
    place_result = placeOnTheTableTop(object_to_place.id, table_to_place.id, object_to_place, n_object);
    attempts++;
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    map_collision_objects = planning_scene_interface.getObjects(chose_object);
  } while (!(place_result == moveit_msgs::MoveItErrorCodes::SUCCESS && map_collision_objects.size() != 0)
          && attempts < MAX_TRYS && ros::ok());

  bool result;
  int trys = 0;

  // Treatment of the possible outcomes.
  switch (place_result.val)
  {
    case place_result.SUCCESS :
      {
        ROS_ERROR("Place success");
        MarkAsPlacedObject(object_to_place, n_object);
        ChangeObjectState(n_object, StatePlaced);
        #if SIMULATION
        DetachGazebo(); // Detaches from the fingers.
        DetachGazebo(false);
        ros::Duration(2.0).sleep();
        #endif
        success = true;

        MovementOpenEndEffector();
        WaitToContinueExecution();
    
        do
        { // Goes back to default arm/torso pose.
          result = MovementHome();
          ros::Duration(1.0).sleep();
          ros::spinOnce();
          trys++;
        } while (trys < MAX_TRYS && result == false && ros::ok());
      }
      break;
    case place_result.INVALID_MOTION_PLAN :
    case place_result.PLANNING_FAILED :
    case place_result.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE :
    case place_result.START_STATE_IN_COLLISION :
      ROS_ERROR("Place fail %d", place_result.val);
      ChangeObjectState(n_object, StatePlaceInvalid);
      ChangeObjectState(n_object, StateHolding);
      success = false;
      result = false;
      WaitToContinueExecution();
      break;
    case place_result.TIMED_OUT :
    case place_result.PREEMPTED :
    case place_result.FAILURE :
    case place_result.CONTROL_FAILED :
      ROS_ERROR("Place TOTAL fail %d", place_result.val);
      ChangeObjectState(n_object, StatePlaceFail);
      success = false;
      result = false;
      WaitToContinueExecution();
      break;

    case autonomous_tidy_up::AutonomousTidyUp::NO_PLACE_POSITION:
      ROS_ERROR("NO place position %d", place_result.val);
      ChangeTableState(n_table, StateTableTooSmall); // Table too small.
      success = false;
      result = false;
      WaitToContinueExecution();
      break;

    default:
      ROS_ERROR("Place %d", place_result.val);
      result = false;
      WaitToContinueExecution();
      break;
  }

  if (!result)
  {
    bool go_to_previus_pose;
    trys = 0;
    do
    {
      ROS_ERROR("Going to the previus pose");
      group_arm_torso->setJointValueTarget(previus_group_variable_values);
      go_to_previus_pose = MoveGroupArmTorso();
      ros::Duration(1.0).sleep();
      ros::spinOnce();
      trys++;
    } while (trys < MAX_TRYS && go_to_previus_pose == false && ros::ok());

    if (!go_to_previus_pose)
    {
      ROS_ERROR("Going to the previus pose with problem");
      WaitToContinueExecution();
    }
  }

  UnblockPlanningScene();
  return success;
}

bool AutonomousTidyUp::ObjectAndTableAvailable()
{
  int n_tables = 0;
  for (int i = 0; i < tables_recevied.size(); i++)
  {
    if (ValidTable(i) && tables_recevied[i].object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] > 0.7///pra ele usar a fixa//0.5///0.3
                      && tables_recevied[i].object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] > 0.7)///pra ele usar a fixa//0.5)//0.3   0.5 eh melhor para o lab
    {
      n_tables++;
      ROS_WARN("Table %f x %f Available<<<<<<", tables_recevied[i].object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X], tables_recevied[i].object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y]);
    }
  }
  int n_objects = RemainingObjectsToPick();
  ROS_WARN("%d Object And %d Table Available<<<<<<", n_objects, n_tables);
  
  return ((n_objects >= 1) && (n_tables >= 1));
}    

int AutonomousTidyUp::RemainingObjectsToPick()
{
  int n = 0;
  for (int i = 0; i < objects_on_the_floor_recevied.size(); i++)
  {
    if (ValidObjectOnFloor(i))
      n++;
  }
  
  return n;
}

///////////////////////////////////////
//                                   //
//      Prints and report file       //
//                                   //
///////////////////////////////////////

// Prints planning scene objects.
void AutonomousTidyUp::printSceneObjects()
{
  bool success = false;
  while (ros::ok() && !success){
    success = psm_->requestPlanningSceneState("/get_planning_scene");
    ROS_WARN_STREAM("Request planning scene " << (success ? "succeeded." : "failed."));
  }
  psm_->getPlanningScene()->printKnownObjects();
}

// Prints the information of objects_on_the_floor_recevied and objects_wrong_size lists.
void AutonomousTidyUp::PrintStateAllObjects()
{
  ROS_WARN("------------PrintStateAllObjects %d------------", global_counter);
  for (int i = 0; i < objects_on_the_floor_recevied.size(); i++)
  {

    ROS_WARN("Object %s --- ", objects_on_the_floor_recevied[i].object.id.c_str());

    switch (objects_on_the_floor_recevied[i].state)
    {
    case StateOnFloor:
      ROS_WARN(" on floor");
      break;
    case StateHolding:
      ROS_WARN(" holding");
      break;
    case StateInvalid:
      ROS_WARN(" invalid");
      break;
    case StateInvalidGrasp:
      ROS_WARN(" invalid grasp");
      break;
    case StateInvalidPlan:
      ROS_WARN(" invalid plan");
      break;
    case StateNotGripped:
      ROS_WARN(" not gripped");
      break;
    case StatePlaceInvalid:
      ROS_WARN(" place invalid");
      break;
    case StatePlaceFail:
      ROS_WARN(" place fail");
      break;
    case StatePlaced:
      ROS_WARN(" PLACED");
      break;
    
    default:
      ROS_WARN(" invalid state %d", objects_on_the_floor_recevied[i].state);
      break;
    }
    ROS_WARN("... manipulation_tries %d ... Original pose x: %f y: %f", objects_on_the_floor_recevied[i].manipulation_tries,
                 objects_on_the_floor_recevied[i].object.primitive_poses[0].position.x, objects_on_the_floor_recevied[i].object.primitive_poses[0].position.y);

  }

  ROS_WARN("------------Objects wrong size: %d", objects_wrong_size.size());
  for (int i = 0; i < objects_wrong_size.size(); i++)
  {
    ROS_WARN("Object %s --- ", objects_wrong_size[i].object.id.c_str());
    ROS_WARN("... Original pose x: %f y: %f", objects_wrong_size[i].object.primitive_poses[0].position.x, objects_wrong_size[i].object.primitive_poses[0].position.y);
  }
}

// Prints the information of objects_on_the_floor_recevied and objects_wrong_size lists on the semantic report file.
void AutonomousTidyUp::PrintStateAllObjectsOnFile()
{
  semantic_report_file << "------------PrintStateAllObjects " << global_counter << " ------------" << std::endl;
  for (int i = 0; i < objects_on_the_floor_recevied.size(); i++)
  {
    semantic_report_file << "Object " << objects_on_the_floor_recevied[i].object.id << " --- ";

    switch (objects_on_the_floor_recevied[i].state)
    {
    case StateOnFloor:
      semantic_report_file << " on floor";
      break;
    case StateHolding:
      semantic_report_file << " holding";
      break;
    case StateInvalid:
      semantic_report_file << " invalid";
      break;
    case StateInvalidGrasp:
      semantic_report_file << " invalid grasp";
      break;
    case StateInvalidPlan:
      semantic_report_file << " invalid plan";
      break;
    case StateNotGripped:
      semantic_report_file << " not gripped";
      break;
    case StatePlaceInvalid:
      semantic_report_file << " place invalid";
      break;
    case StatePlaceFail:
      semantic_report_file << " place fail";
      break;
    case StatePlaced:
      semantic_report_file << " PLACED";
      break;
    
    default:
      semantic_report_file << " invalid state " << int(objects_on_the_floor_recevied[i].state) << " " << objects_on_the_floor_recevied[i].state;
      break;
    }
    semantic_report_file << "... manipulation_tries " << objects_on_the_floor_recevied[i].manipulation_tries
    << " ... Original pose x: " << objects_on_the_floor_recevied[i].object.primitive_poses[0].position.x
    << " y: " << objects_on_the_floor_recevied[i].object.primitive_poses[0].position.y;
    semantic_report_file << "... Size x: " << objects_on_the_floor_recevied[i].object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X]
    << " y: " << objects_on_the_floor_recevied[i].object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y]
    << " z: " << objects_on_the_floor_recevied[i].object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z]
    << " yaw: " << ComputeYaw(objects_on_the_floor_recevied[i].object.primitive_poses[0].orientation) << std::endl;

  }

  semantic_report_file << std::endl << "------------Objects wrong size: " << objects_wrong_size.size() << std::endl;
  for (int i = 0; i < objects_wrong_size.size(); i++)
  {
    semantic_report_file << "Object " << objects_wrong_size[i].object.id.c_str() << " --- "
    << "... Original pose x: " << objects_wrong_size[i].object.primitive_poses[0].position.x
    << " y: " << objects_wrong_size[i].object.primitive_poses[0].position.y;
    semantic_report_file << "... Size x: " << objects_wrong_size[i].object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X]
    << " y: " << objects_wrong_size[i].object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y]
    << " z: " << objects_wrong_size[i].object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z]
    << " yaw: " << ComputeYaw(objects_wrong_size[i].object.primitive_poses[0].orientation) << std::endl;
  }
  semantic_report_file << std::endl;
}

// Prints the information of tables_recevied list.
void AutonomousTidyUp::PrintStateAllTables()
{
  ROS_WARN("------------PrintStateAllTables %d------------", global_counter);
  for (int i = 0; i < tables_recevied.size(); i++)
  {

    ROS_WARN("Table %s --- ", tables_recevied[i].object.id.c_str());

    switch (tables_recevied[i].state)
    {
    case StateTableTooSmall:
      ROS_WARN(" too small");
      break;
    default:
      ROS_WARN(" OK");
      break;
    }
    ROS_WARN("... Pose x: %f y: %f", tables_recevied[i].object.primitive_poses[0].position.x, tables_recevied[i].object.primitive_poses[0].position.y);
  }
}

// Prints the information of tables_recevied list on the semantic report file.
void AutonomousTidyUp::PrintStateAllTablesOnFile()
{
  semantic_report_file << "------------PrintStateAllTables " << global_counter << " ------------" << std::endl;
  for (int i = 0; i < tables_recevied.size(); i++)
  {

    semantic_report_file << "Table " << tables_recevied[i].object.id << " --- ";

    switch (tables_recevied[i].state)
    {
    case StateTableTooSmall:
      semantic_report_file << " too small";
      break;
    default:
      semantic_report_file << " OK";
      break;
    }
    semantic_report_file << "... Pose x: " << tables_recevied[i].object.primitive_poses[0].position.x
    << " y: " << tables_recevied[i].object.primitive_poses[0].position.y << std::endl;

  }
  semantic_report_file << std::endl;
}

// Prints the detailed information of tables_recevied list.
void AutonomousTidyUp::PrintInfoAllTables(){
  for (int i = 0; i < tables_recevied.size(); i++)
  {
    ROS_WARN(".........................................................");
    ROS_WARN("tables_recevied[%d].object.id = %s", i, tables_recevied[i].object.id.c_str());
    ROS_WARN("tables_recevied[%d].object.header.frame_id = %s", i, tables_recevied[i].object.header.frame_id.c_str());
    ROS_WARN("tables_recevied[%d].object.header.seq = %d", i, tables_recevied[i].object.header.seq);
    ROS_WARN("tables_recevied[%d].object.header.stamp = %d", i, tables_recevied[i].object.header.stamp.sec);
    ROS_WARN("tables_recevied[%d].object.type.db = %s", i, tables_recevied[i].object.type.db.c_str());
    ROS_WARN("tables_recevied[%d].object.type.key = %s", i, tables_recevied[i].object.type.key.c_str());
    ROS_WARN("tables_recevied[%d].object.primitive_poses[0].orientation.w = %f", i, tables_recevied[i].object.primitive_poses[0].orientation.w);
    ROS_WARN("tables_recevied[%d].object.primitive_poses[0].orientation.x = %f", i, tables_recevied[i].object.primitive_poses[0].orientation.x);
    ROS_WARN("tables_recevied[%d].object.primitive_poses[0].orientation.y = %f", i, tables_recevied[i].object.primitive_poses[0].orientation.y);
    ROS_WARN("tables_recevied[%d].object.primitive_poses[0].orientation.z = %f", i, tables_recevied[i].object.primitive_poses[0].orientation.z);
    ROS_WARN("tables_recevied[%d].object.primitive_poses[0].position.x = %f", i, tables_recevied[i].object.primitive_poses[0].position.x);
    ROS_WARN("tables_recevied[%d].object.primitive_poses[0].position.y = %f", i, tables_recevied[i].object.primitive_poses[0].position.y);
    ROS_WARN("tables_recevied[%d].object.primitive_poses[0].position.z = %f", i, tables_recevied[i].object.primitive_poses[0].position.z);
    ROS_WARN("tables_recevied[%d].object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = %f", i, tables_recevied[i].object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X]);
    ROS_WARN("tables_recevied[%d].object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = %f", i, tables_recevied[i].object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y]);
    ROS_WARN("tables_recevied[%d].object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = %f", i, tables_recevied[i].object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z]);

    ROS_WARN("tables_recevied[%d].manipulation_tries = %d", i, tables_recevied[i].manipulation_tries);
    ROS_WARN("tables_recevied[%d].see_many_times = %s", i, (tables_recevied[i].see_many_times? "true" : "false"));
    ROS_WARN("tables_recevied[%d].state = %d", i, tables_recevied[i].state);    

  }
}

// Opens the semantic report file with the current time and date on the name.
bool AutonomousTidyUp::OpenReportFile(){

  string file_name;
  file_name.append(dir_arq);
  file_name.append("/semantic_report");
  file_name.append(return_current_time_and_date());

  ROS_WARN("Semantic report file at %s", file_name.c_str());

  semantic_report_file.open(file_name + ".txt", std::ofstream::out);

  return true;
}

bool AutonomousTidyUp::CloseReportFile(){

  semantic_report_file.close();
  ROS_WARN("File closed");

  return true;
}

// Saves the navigation map with the strategy, current time, and date on the name.
void AutonomousTidyUp::SaveGmappinMap(){
  pal_navigation_msgs::SaveMap save_map_msg;

  string map_name;
  map_name.append("map_");
  #if STRATEGY_BVP_CLASSIC
  map_name.append("BVP_");
  #endif
  #if STRATEGY_BVP_PLUS_VORONOI
  map_name.append("plus_voronoi");
  #endif
  #if STRATEGY_VORONOI
  map_name.append("VORONOI_");
  #endif
  #if STRATEGY_GRID
  map_name.append("GRID_");
  #endif
  map_name.append(return_current_time_and_date());
  save_map_msg.request.directory = map_name;

  save_map_client.call(save_map_msg);
  
  if (save_map_msg.response.success)
  {
    semantic_report_file << "Gmapping mapping saved as " << map_name << " at "<< save_map_msg.response.full_path << std::endl;
    
  } else
  {
    ROS_ERROR("Problem to save map %s at %s", map_name.c_str(), save_map_msg.response.full_path.c_str());
    semantic_report_file << "Problem to save map " << map_name << " at "<< save_map_msg.response.full_path << std::endl;
  }
}

///////////////////////////////////////
//                                   //
//       Real robot on the lab       //
//                                   //
///////////////////////////////////////

// Adds the real lab table to tables_recevied list.
void AutonomousTidyUp::AddFixedLabTable()
{
  autonomous_tidy_up::ObjectDetected new_table;

  new_table.object.id = std::string("table0");
  new_table.object.header.frame_id = std::string("map");
  new_table.object.primitive_poses.resize(1);
  new_table.object.primitive_poses[0].orientation.w = 1.000000;
  new_table.object.primitive_poses[0].orientation.x = 0.000000;
  new_table.object.primitive_poses[0].orientation.y = 0.000000;
  new_table.object.primitive_poses[0].orientation.z = -0.000000;
  new_table.object.primitive_poses[0].position.x = -4.746595;
  new_table.object.primitive_poses[0].position.y = -1.378744;
  new_table.object.primitive_poses[0].position.z = 0.378657;
  new_table.object.primitives.resize(1);
  new_table.object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  new_table.object.primitives[0].dimensions.resize(3);
  new_table.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.949253;
  new_table.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.655227;
  new_table.object.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.757314;
  new_table.see_many_times = true;

  tables_recevied.push_back(new_table);

  ROS_WARN("AddFixedLabTable........map_half_half_lab17jul.......OK");
}

}

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  /* Add both finger joints of robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gripper_left_finger_joint";
  posture.joint_names[1] = "gripper_right_finger_joint";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

//////// COMUNICATION
bool start;
void start_callback(const std_msgs::Bool& msg){
  start = msg.data;
}

int main(int argc, char* argv[])
{

  ros::init(argc, argv, "autonomous_tidy_up");
  ros::NodeHandle n("~");

  ros::spinOnce();

  // Run demo
  autonomous_tidy_up::AutonomousTidyUp demo;

  ros::Subscriber start_sub;

  start_sub = n.subscribe("/start_demo", 1, start_callback);
  start = false;

  while (!start && ros::ok())
  {
    ROS_WARN("Waiting start_demo...");
    ros::Duration(1.0).sleep();
    ros::spinOnce();
  }

  demo.ChangeStage(autonomous_tidy_up::StageStart);
  demo.set_local_planner_xy_goal_tolerance(0.1);
  #if not SIMULATION
  demo.AddFixedLabTable();
  #endif

  do
  {
    demo.RemoveObjectsAndTablesReceviedToSemanticMap();
    #if not SIMULATION
    demo.WaitToContinueExecution();
    #endif
    demo.ExploreToFindObjects(); // EXPLORATION
    #if not SIMULATION
    demo.WaitToContinueExecution();
    #endif
    if (demo.RemainingObjectsToPick() == 0
        #if STRATEGY_BVP_CLASSIC
        && (demo.CoverageAcessibleArea() >= 1.0 or demo.flat_potential_field >= 5)
        #elif STRATEGY_VORONOI
        && demo.visited_all_voronoi_goals
        #elif STRATEGY_GRID
        && demo.visited_all_grid_goals
        #endif
    )
    {
      break;
    }
    demo.CollectAllObjectsOnTheFloor(); // MANIPULATION
    #if not SIMULATION
    demo.WaitToContinueExecution();
    #endif
    demo.AddObjectsAndTablesReceviedToSemanticMap();
  }while (demo.CoverageAcessibleArea() < 1.0);

  demo.ChangeStage(autonomous_tidy_up::StageEnd);
  ROS_WARN("OVER.................. with %d objects and %d remaining", demo.objects_on_the_floor_recevied.size(), demo.RemainingObjectsToPick());
  #if GMAPPING_EXPLORATION
  demo.SaveGmappinMap();
  #endif
  demo.PrintStateAllObjects();
  demo.PrintStateAllObjectsOnFile();
  demo.PrintStateAllTables();
  demo.PrintStateAllTablesOnFile();
  demo.CloseReportFile();

  demo.PrintInfoAllTables();

  ros::waitForShutdown();
  return 0;
}

int mainTEST_PRINT(int argc, char **argv) {
  ros::init(argc, argv, "autonomous_tidy_up");

  ros::NodeHandle n;

  while (ros::ok()) {
    ROS_INFO("Running...");
    ros::spinOnce();
  }
  return 0;
};

