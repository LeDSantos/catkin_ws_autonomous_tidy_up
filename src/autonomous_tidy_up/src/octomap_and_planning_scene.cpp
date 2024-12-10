#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <moveit/planning_scene/planning_scene.h>
// #include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/BoundingBoxQuery.h>

#include <octomap/OcTree.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

// FOR COMUNICATION
#include <std_msgs/Bool.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <octomap/OcTree.h>
#include <octomap/AbstractOcTree.h>
#include "tf/transform_datatypes.h"
#include <octomap_msgs/conversions.h>
#include <shape_msgs/SolidPrimitive.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <geometry_msgs/PoseStamped.h>

// C++ standard headers
#include <exception>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <play_motion_msgs/PlayMotionAction.h>

#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_srvs/Empty.h>

#define MIN_OBJECT_HEIGHT 0.02
#define MAX_OBJECT_HEIGHT 0.25
#define OBJECT_EXPANSION  0.02
#define TABLE_EXPANSION   0.10
#define EXPANSION_TO_CLEAN_OBJECT_OCTOMAP 0.03

#define N_MAX_OBJECTS 5

#define SIMULATION false

#if SIMULATION
  #define EXTRA_HEIGHT 0.0
  #define EXTRA_X 0.0
#else
  #define EXTRA_HEIGHT 0.03 /////adjust for the real robot
  #define EXTRA_X -0.01
#endif

typedef moveit::planning_interface::PlanningSceneInterface PlanningScene;
typedef actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction> PlayMotionClient;
typedef moveit::planning_interface::MoveGroupInterface MoveitInterface;

octomap_msgs::Octomap octomap_full;
moveit_msgs::PlanningScene planning_scene__input;

bool only_pub_octomap;
void only_pub_octomap_callback(const std_msgs::Bool& msg){
  only_pub_octomap = msg.data;
}

bool inspect_surroundings;
void inspect_surroundings_callback(const std_msgs::Bool& msg){
  inspect_surroundings = msg.data;
}

bool not_read_octomap;
void octomap_full_callback(const octomap_msgs::Octomap& msg){
  if(not_read_octomap) return;
  octomap_full = msg;
}

void planning_scene__callback(const moveit_msgs::PlanningScene& msg){
  planning_scene__input = msg;
}

bool stop_pub_octomap;
void stop_pub_octomap_callback(const std_msgs::Bool& msg){
  stop_pub_octomap = msg.data;
}

using std::string;
#include <iostream>

namespace octomap_manipulation
{

class OctomapManipulation
{
private:
  // A shared node handle
  ros::NodeHandle nh_;
  //publishers for the planning scene
  ros::Publisher planning_scene__pub, planning_scene_ready_pub, object_on_octomap_pub;
  std_msgs::Bool planning_scene_ready_msg, object_on_octomap_msg;
  moveit_msgs::PlanningScene planning_scene__msg;
  moveit_msgs::PlanningScene& planning_scene_input;
  octomap_msgs::Octomap& octomap_input;
  bool& stop_pub;
  bool& stop_read_octomap;
  PlanningScene planning_scene_interface;
  MoveitInterface *group_arm_torso;
  PlayMotionClient *client_motion;
  geometry_msgs::Pose pose_inicial_obj;
  std::string planning_group_name_ = "arm_torso";
  ros::ServiceClient delete_box_client;
  ros::Timer timerCleanObjectFromOctomap;
  bool cleaning_object_space, old_cleaning_object_space;

  octomap_msgs::BoundingBoxQuery srv[10];
  octomap_msgs::Octomap octomap_clean_msg;

  ros::AsyncSpinner *spinner;

  inline double getYaw(const geometry_msgs::Quaternion& q);
  inline double ComputeYaw(const geometry_msgs::Quaternion& q_gm);

public:
  ros::ServiceClient reset_octomap;
  std_srvs::Empty empty_msg;

  bool object_detected, outside_objects_detected;

  /** @brief Sets the enviroment by advertising the publishers; connecting with play_motion, MoveIt interface, and octomap_server to use clear_bbx and reset.
   * Also sets important local variables.*/
  OctomapManipulation(moveit_msgs::PlanningScene& planning_scene, octomap_msgs::Octomap& octomap, bool& stop, bool& stop_read_octomap_input);
  ~OctomapManipulation();

  /** @brief Publishes the updated planning_scene for MoveIt. */
  void PubPlanningScene();

  /** @brief Publishes the first planning_scene without the objects' obstacles on octomap for MoveIt.
   * @return true when a clean octomap is found.
   * @return false when there are obstacles in the objects' locations.
  */
  bool PubPlanningSceneWithoutObjects();

  /** @brief Checks octomap obstacles on the object location.
   * @param obj object
   * @return if some obstacle has been found.
   */
  bool CheckObjectOctomap(moveit_msgs::CollisionObject obj);

  /** @brief Checks if the object1 is on the planning_scene.
   * @return if some obstacle has been found on object1 location.
   */
  bool CheckPlanningSceneObject();

  /** @brief Sends the motion BIGHeadLookAround for the play_motion.
   * @return success of the movement execution.
   */
  bool SendMotionBIGHeadLookAround();

  /** @brief Cleans the place where objects are on octomap periodically. 
   * @param event timer event that triggers the function.
  */
  void CleanObjectFromOctomap(const ros::TimerEvent& event);

  /** @brief Checks if the octomap is free on the objects' locations.
  * @return if the spaces are without obstacles.
  */
  bool ObjectsSpaceClean();

  /** @brief Publishes to notify the other nodes that the planning_scene is ready. */
  void PubPlanningSceneReady();

  bool block_clean_object = true;

};

OctomapManipulation::OctomapManipulation(moveit_msgs::PlanningScene& planning_scene, octomap_msgs::Octomap& octomap, bool& stop, bool& stop_read_octomap_input) : nh_("~"), planning_scene_input(planning_scene), octomap_input(octomap), stop_pub(stop), stop_read_octomap(stop_read_octomap_input)
{
  spinner = new ros::AsyncSpinner(4);
  spinner->start();

  // Publisher for the planning scene
  planning_scene__pub = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

  planning_scene_ready_pub = nh_.advertise<std_msgs::Bool>("planning_scene_ready", 1);
  planning_scene_ready_msg.data = false;

  object_on_octomap_pub = nh_.advertise<std_msgs::Bool>("object_on_octomap", 1);
  object_on_octomap_msg.data = false;

  // PlayMotionClient 
  client_motion = (new PlayMotionClient("/play_motion", true));
  while (!client_motion->waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the play_motion action server to come up");
  }
  #if DEBUG
    ROS_WARN("play_motion action server OK");
  #endif

  group_arm_torso = (new MoveitInterface(planning_group_name_));
  group_arm_torso->setPlanningTime(60.0);

  object_detected = false;
  outside_objects_detected = false;
  cleaning_object_space = false;
  old_cleaning_object_space = false;
  delete_box_client = nh_.serviceClient<octomap_msgs::BoundingBoxQuery>("/octomap_server/clear_bbx");
  delete_box_client.waitForExistence();
  ROS_WARN("delete_box_client READY");

  reset_octomap = nh_.serviceClient<std_srvs::Empty>("/octomap_server/reset");
  reset_octomap.waitForExistence();
  ROS_WARN("reset_octomap READY");

}

OctomapManipulation::~OctomapManipulation()
{
}


//// Source: http://docs.ros.org/en/jade/api/tf2/html/impl_2utils_8h_source.html
inline
double OctomapManipulation::getYaw(const geometry_msgs::Quaternion& q)
{
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

inline double OctomapManipulation::ComputeYaw(const geometry_msgs::Quaternion& q_gm) {
    return getYaw(q_gm);
}

bool OctomapManipulation::ObjectsSpaceClean()
{
  geometry_msgs::Pose object_pose;
  for (int i = 0; i < N_MAX_OBJECTS; i++)
  {  
    std::string object_name = string("object") + std::to_string(i);
    std::vector<std::string> chose_object;
    chose_object.resize(1);
    chose_object[0] = object_name;

    std::map<std::string,moveit_msgs::CollisionObject> map_collision_objects;

    map_collision_objects = planning_scene_interface.getObjects(chose_object);

    if (map_collision_objects.size() == 0)
    {
      continue;
    }

    // Obtaining the actual OctoMap tree.
    octomap::AbstractOcTree* new_my_abstract_map = octomap_msgs::msgToMap(octomap_input);
    octomap::OcTree* new_my_map = (octomap::OcTree*)new_my_abstract_map;
    octomap::OcTree new_tree = *new_my_map;

    // Gets the space cleaned by CleanObjectFromOctomap.
    float pos_x_min, pos_x_max, pos_y_min, pos_y_max;
    pos_x_min = srv[i].request.min.x + OBJECT_EXPANSION;
    pos_x_max = srv[i].request.max.x - OBJECT_EXPANSION;
    pos_y_min = srv[i].request.min.y + OBJECT_EXPANSION;
    pos_y_max = srv[i].request.max.y - OBJECT_EXPANSION;
    ROS_WARN("check ObjectsSpaceClean for object %d", i);

    // Verifies points above the surface.
    octomap::point3d point_below;
    octomap::point3d vector_down(0.0, 0.0, -1.0);

    float abs_step = 0.01;

    // Gets the octomap limits.
    double octomap_min_x, octomap_min_y, octomap_min_z;
    double octomap_max_x, octomap_max_y, octomap_max_z;
    new_tree.getMetricMin(octomap_min_x, octomap_min_y, octomap_min_z);
    new_tree.getMetricMax(octomap_max_x, octomap_max_y, octomap_max_z);

    float surface_height = 0.0;

    float z = srv[i].request.max.z - OBJECT_EXPANSION;
    for (float x = pos_x_min; x <= pos_x_max; x = x + abs_step)
    {
      for (float y = pos_y_min; y <= pos_y_max; y = y + abs_step)
      {
        octomap::point3d point_above(x, y, z);
        if ((x > octomap_max_x or x < octomap_min_x) or (y > octomap_max_y or y < octomap_min_y) or (z > octomap_max_z or z < octomap_min_z))
        {
          /* Out of the octomap */
          continue;
        }
        bool hit = new_tree.castRay(point_above, vector_down, point_below, true, srv[i].request.min.z + OBJECT_EXPANSION);
        if (hit && point_below.z() > surface_height + MIN_OBJECT_HEIGHT)
        {
          return false;
        }else
        {
          /* Hit the surface */
        }
        
      }
    }
    // Saves the clean octomap
    octomap_msgs::readTree(new_my_map, octomap_clean_msg);
  }

  return true;
}

bool OctomapManipulation::PubPlanningSceneWithoutObjects()
{
  ros::Duration(0.5).sleep();
  ros::spinOnce();

  if (outside_objects_detected)
  {
    old_cleaning_object_space = cleaning_object_space;      
    stop_read_octomap = true;
    if(!ObjectsSpaceClean())
    {
      cleaning_object_space=true;

    }else
    {
      cleaning_object_space=false;        
    }
    if(cleaning_object_space or (old_cleaning_object_space == true && cleaning_object_space == false)
       or (old_cleaning_object_space == false && cleaning_object_space == false)){
      if ((old_cleaning_object_space == true && cleaning_object_space == false) or ((old_cleaning_object_space == false && cleaning_object_space == false)))
      {
        ROS_WARN("FIRST OCTOMAP CLEAN------");
        PubPlanningScene();
        stop_read_octomap = false;
        return true;
      }else
      {
        ROS_WARN("OCTOMAP PUB in planning_scene NOT CLEAN");
        stop_read_octomap = false;
        return false;  
      }      
    }
  }else
  {
    PubPlanningScene();
  }
  stop_read_octomap = false;
  return false;
}

void OctomapManipulation::PubPlanningScene()
{
  planning_scene__msg = planning_scene_input;
  planning_scene__msg.world.octomap.header.stamp = ros::Time::now();
  planning_scene__msg.world.octomap.header.frame_id="base_footprint";

  if (object_detected)
  {
    std::map<std::string,moveit_msgs::CollisionObject> map_collision_objects;
    map_collision_objects = planning_scene_interface.getObjects(std::vector<std::string>{"object1"});
    if (map_collision_objects["object1"].id.size() > 2)
    {
      planning_scene__msg.world.collision_objects.push_back(map_collision_objects["object1"]);
      ROS_WARN("OBJECT %s pushed to planning_scene", map_collision_objects["object1"].id.c_str());
    }
  }  

  // Planning_scene message receives the external octomap.
  planning_scene__msg.world.octomap.octomap = octomap_input;
  planning_scene__msg.is_diff = true;
  ros::Duration(0.25).sleep();
  if(!stop_pub){
    planning_scene__pub.publish(planning_scene__msg);
    ROS_WARN("OCTOMAP PUB in planning_scene");
  }else
  {
    ROS_WARN("without OCTOMAP PUB in planning_scene");
  }
  ros::spinOnce();
}

bool OctomapManipulation::CheckPlanningSceneObject() {

  std::map<std::string,moveit_msgs::CollisionObject> map_collision_objects;
  
  do
  {  
    map_collision_objects = planning_scene_interface.getObjects(std::vector<std::string>{"object1"});

    if (map_collision_objects["object1"].id.size() == 0)
    {
      ROS_WARN("without object1 in planning scene");
      ros::spinOnce();
      ros::Duration(0.25).sleep();
    }
  } while (map_collision_objects["object1"].id.size() == 0);

  moveit_msgs::CollisionObject object_on_map = map_collision_objects["object1"];
  object_on_octomap_msg.data = CheckObjectOctomap(object_on_map);

  ROS_WARN("PUB object_on_octomap");
  object_on_octomap_pub.publish(object_on_octomap_msg);
  ros::spinOnce();
  ros::Duration(0.25).sleep();
  object_on_octomap_pub.publish(object_on_octomap_msg);
  ros::spinOnce();
  ros::Duration(0.25).sleep();
  object_on_octomap_pub.publish(object_on_octomap_msg);
  ros::spinOnce();
  ros::Duration(0.25).sleep();

  return object_on_octomap_msg.data;
}

bool OctomapManipulation::CheckObjectOctomap(moveit_msgs::CollisionObject obj){

  ROS_WARN("object on frame %s", obj.header.frame_id.c_str());

  float abs_step = 0.01;
  float height, pos_x_min, pos_y_min, pos_x_max, pos_y_max;

  // Object info.
  geometry_msgs::Pose object_pose = obj.primitive_poses[0];
  double object_x_depth = obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X];
  double object_y_width = obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
  double object_z_height = obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z];

  float yaw = ComputeYaw(obj.primitive_poses[0].orientation);
  float expansion = OBJECT_EXPANSION;
  float ang_diag = atan2(object_y_width, object_x_depth);
  float mod_diag = sqrt(object_x_depth*object_x_depth + object_y_width*object_y_width) / 2.0;

  // Calculates the vertices of the top or bottom face of the cuboid without considering height.
  double vertices[4][2];
  double max_x, max_y, min_x, min_y;

  vertices[0][0] = object_pose.position.x + ((expansion + mod_diag) * cos(ang_diag + yaw));
  vertices[0][1] = object_pose.position.y + ((expansion + mod_diag) * sin(ang_diag + yaw));
  vertices[1][0] = object_pose.position.x + ((expansion + mod_diag) * cos(-ang_diag + yaw));
  vertices[1][1] = object_pose.position.y + ((expansion + mod_diag) * sin(-ang_diag + yaw));
  vertices[2][0] = object_pose.position.x - ((expansion + mod_diag) * cos(ang_diag + yaw));
  vertices[2][1] = object_pose.position.y - ((expansion + mod_diag) * sin(ang_diag + yaw));
  vertices[3][0] = object_pose.position.x - ((expansion + mod_diag) * cos(-ang_diag + yaw));
  vertices[3][1] = object_pose.position.y - ((expansion + mod_diag) * sin(-ang_diag + yaw));

  // Gets the extreme points to check between them.
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

  height = 0.03;
  pos_x_min = min_x;
  pos_y_min = min_y;
  pos_x_max = max_x;
  pos_y_max = max_y;
  
  std::vector<octomap::point3d> points_occupied;
  octomap::point3d vector_down(0.0, 0.0, -1.0);

  ros::spinOnce();
  octomap::AbstractOcTree* new_my_abstract_map = octomap_msgs::msgToMap(octomap_input);

  // Obtaining the actual OctoMap tree.
  octomap::OcTree* new_my_map = (octomap::OcTree*)new_my_abstract_map;
  octomap::OcTree new_tree = *new_my_map;

  ROS_WARN("CHECK OBJECT x from %f until %f and y from %f until %f", pos_x_min, pos_x_max, pos_y_min, pos_y_max);

  octomap::point3d point_below;

  for (float x = pos_x_min; x <= pos_x_max; x = x + abs_step)
  {
    for (float y = pos_y_min; y <= pos_y_max; y = y + abs_step)
    {
      octomap::point3d point_above(x, y, height + MAX_OBJECT_HEIGHT);
      bool hit = new_tree.castRay(point_above, vector_down, point_below, true, height + MAX_OBJECT_HEIGHT);
      // If it finds occupied cells, save it
      if (hit && point_below.z() > height + MIN_OBJECT_HEIGHT)
      {
        points_occupied.push_back(point_below);
      }else
      {
        /* hit the surface */
      }        
    }
  }

  ROS_ERROR("FOUND %d OCCUPIED POINTS ON THE OBJECT PLACE", points_occupied.size());

  if (points_occupied.size() > 0)
  {
    return true;
  } else
  {
    return false;
  }
}

bool OctomapManipulation::SendMotionBIGHeadLookAround(){

  actionlib::SimpleClientGoalState state = client_motion->getState();
  ROS_WARN_STREAM("Motion client state: " << state.toString());
  
  if (!state.isDone())
  {
    return false;
  }

  play_motion_msgs::PlayMotionGoal motion;

  motion.motion_name = "BIGhead_look_aroundHIGHER";
  motion.skip_planning = false;
  motion.priority = 0;

  ROS_WARN_STREAM("Sending goal with motion: " << motion);
  client_motion->sendGoalAndWait(motion);

  ROS_WARN("Waiting for result ...");
  bool actionOk = client_motion->waitForResult(ros::Duration(30.0));

  state = client_motion->getState();
  
  if (state.state_ != state.SUCCEEDED)
  {
    ROS_ERROR_STREAM("Action ERROR with state: " << state.toString());
    return false;
  }  

  if (actionOk) {
    ROS_ERROR_STREAM("Action finished successfully with state: " << state.toString());
  } else {
    ROS_ERROR_STREAM("Action failed with state: " << state.toString());
  }

  return actionOk;
}

void OctomapManipulation::PubPlanningSceneReady()
{
  ROS_WARN("PUBLISHING planning_scene_ready == true");
  planning_scene_ready_msg.data = true;
  planning_scene_ready_pub.publish(planning_scene_ready_msg);
  ros::spinOnce();
  planning_scene_ready_msg.data = false;
}

void OctomapManipulation::CleanObjectFromOctomap(const ros::TimerEvent& event)
{

  if (block_clean_object)
  {
    return;
  }
  
  // Object info.
  geometry_msgs::Pose object_pose;
  double object_x_depth;
  double object_y_width;
  double object_z_height;

  for (int n = 0; n < N_MAX_OBJECTS; n++)
  {
    // Gets the object.
    std::string object_name;
    object_name = string("object")  + std::to_string(n);
      
    std::vector<std::string> chose_object;
    chose_object.resize(1);
    chose_object[0] = object_name;

    std::map<std::string,moveit_msgs::CollisionObject> map_collision_objects;

    map_collision_objects = planning_scene_interface.getObjects(chose_object);

    if (map_collision_objects.size() == 0)
    {
      // Without collision object.
      srv[n].request.min.x = 0.0;
      srv[n].request.min.y = 0.0;
      srv[n].request.min.z = 0.0;
      srv[n].request.max.x = 0.0;
      srv[n].request.max.y = 0.0;
      srv[n].request.max.z = 0.0;
      continue;
    }else
    {
      if (object_detected == false)
        outside_objects_detected = true;
    }

    // Object info.
    object_pose = map_collision_objects[object_name].primitive_poses[0];
    object_x_depth = map_collision_objects[object_name].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X];
    object_y_width = map_collision_objects[object_name].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
    object_z_height = map_collision_objects[object_name].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z];
    
    float yaw = ComputeYaw(map_collision_objects[object_name].primitive_poses[0].orientation);
    float expansion = OBJECT_EXPANSION;    
    geometry_msgs::Point min, max;
    float ang_diag = atan2(object_y_width, object_x_depth);
    float mod_diag = sqrt(object_x_depth*object_x_depth + object_y_width*object_y_width) / 2.0;

    // The object does not have pitch or roll, so, calculating the minimum and maximum height is straightforward.
    max.z = object_pose.position.z + (expansion + object_z_height / 2.0);
    min.z = object_pose.position.z - (expansion + object_z_height / 2.0);

    // Calculates the vertices of the top or bottom face of the cuboid without considering height.
    double vertices[4][2];
    double max_x, max_y, min_x, min_y;

    vertices[0][0] = object_pose.position.x + ((expansion + mod_diag) * cos(ang_diag + yaw));
    vertices[0][1] = object_pose.position.y + ((expansion + mod_diag) * sin(ang_diag + yaw));
    vertices[1][0] = object_pose.position.x + ((expansion + mod_diag) * cos(-ang_diag + yaw));
    vertices[1][1] = object_pose.position.y + ((expansion + mod_diag) * sin(-ang_diag + yaw));
    vertices[2][0] = object_pose.position.x - ((expansion + mod_diag) * cos(ang_diag + yaw));
    vertices[2][1] = object_pose.position.y - ((expansion + mod_diag) * sin(ang_diag + yaw));
    vertices[3][0] = object_pose.position.x - ((expansion + mod_diag) * cos(-ang_diag + yaw));
    vertices[3][1] = object_pose.position.y - ((expansion + mod_diag) * sin(-ang_diag + yaw));

    // Gets the extreme points to clean between them.
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
    
    max.x = max_x;
    max.y = max_y;
    min.x = min_x;
    min.y = min_y;

    srv[n].request.min = min;
    srv[n].request.max = max;

    bool result = delete_box_client.call(srv[n]);
    if (!result) {
      ROS_ERROR("Failed to call service clear_bbx");
    }else
    {
      ROS_WARN("service clear_bbx OK for %s....", object_name.c_str());
    }  
  }
}

}

/* Responsible for manipulating the octomap and sending the updated planning_scene for MoveIt when it is necessary.*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "octomap_and_planning_scene");
  ros::NodeHandle n("~");

  only_pub_octomap = false;
  not_read_octomap = false;
  inspect_surroundings = false;
  stop_pub_octomap = false;

  ros::Subscriber only_pub_octomap_sub = n.subscribe("/only_pub_octomap", 1, only_pub_octomap_callback);
  ros::Subscriber octomap_full_sub = n.subscribe("/octomap_full", 1, octomap_full_callback);
  ros::Subscriber planning_scene__sub = n.subscribe("/planning_scene", 1, planning_scene__callback);
  ros::Subscriber inspect_surroundings_sub = n.subscribe("/inspect_surroundings", 1, inspect_surroundings_callback);
  ros::Subscriber stop_pub_octomap_sub = n.subscribe("/block_planning_scene", 1, stop_pub_octomap_callback);
  
  octomap_manipulation::OctomapManipulation manipulate_octomap(planning_scene__input, octomap_full, stop_pub_octomap, not_read_octomap);
  
  ros::Timer timerCleanObjectFromOctomap = n.createTimer(ros::Duration(0.005), &octomap_manipulation::OctomapManipulation::CleanObjectFromOctomap,
     &manipulate_octomap);
  ROS_WARN("TIMER READY----------");

  while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration(1.0).sleep();

    // Only cleans the object place on octomap and sends the updated planning_scene with the updated octomap.
    if(only_pub_octomap)
    {
      manipulate_octomap.block_clean_object = true;
      ros::Duration(0.25).sleep();
      if (!manipulate_octomap.CheckPlanningSceneObject())
      {
        ROS_WARN("no object");
        manipulate_octomap.PubPlanningSceneReady();
        only_pub_octomap = false;
        continue;
      }
      manipulate_octomap.block_clean_object = false;
      ros::Duration(0.25).sleep();
      ros::spinOnce();

      while (ros::ok() && manipulate_octomap.PubPlanningSceneWithoutObjects() == false && only_pub_octomap)
      {
        ros::Duration(0.25).sleep();
        ros::spinOnce();
      }
      manipulate_octomap.PubPlanningSceneReady();
      only_pub_octomap = false;
    }

    // Resets the octomap and moves the head to create a new one.
    // Then, cleans the object place on octomap and sends the updated planning_scene with the updated octomap.
    if(inspect_surroundings)
    {
      manipulate_octomap.block_clean_object = true;
      // clean octomap
      if(manipulate_octomap.reset_octomap.call(manipulate_octomap.empty_msg))
        ROS_WARN("service reset_octomap OK.......");

      ros::Duration(0.25).sleep();
      ros::spinOnce();
      manipulate_octomap.object_detected = true;
      manipulate_octomap.outside_objects_detected = true;

      bool result;
      do
      {
        result = manipulate_octomap.SendMotionBIGHeadLookAround();
        ros::Duration(0.25).sleep();
        ros::spinOnce();      
      } while (result == false);
      
      while (ros::ok() && manipulate_octomap.PubPlanningSceneWithoutObjects() == false)
      {
        ros::Duration(0.25).sleep();
      }
      manipulate_octomap.PubPlanningScene();
      ros::spinOnce();
      ros::Duration(1.0).sleep();
      manipulate_octomap.PubPlanningScene();
      ros::spinOnce();
      ros::Duration(1.0).sleep();
      manipulate_octomap.PubPlanningScene();
      ros::spinOnce();
      ros::Duration(5.0).sleep();
      manipulate_octomap.PubPlanningSceneReady();
      inspect_surroundings = false;
      continue;
    }
  }

  ros::waitForShutdown();
  return 0;
}