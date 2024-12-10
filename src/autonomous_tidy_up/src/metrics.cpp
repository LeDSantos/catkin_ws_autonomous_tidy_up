#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

// C++ standard headers
#include <exception>
#include <string>
#include <vector>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/transform_broadcaster.h>

#include "tf/transform_datatypes.h"

// FOR COMUNICATION
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>

#include <shape_msgs/SolidPrimitive.h>

#include <moveit_msgs/MoveItErrorCodes.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <control_msgs/JointTrajectoryControllerState.h>
#include <math.h>

using std::cout;
using std::cin;
using std::endl;

#include <sstream>

using std::string;
using std::vector;

#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>

#include <ctime>   // localtime
#include <iomanip> // put_time

#define TIME_STEP 0.5 ////seconds

#include <cstdio>
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"

#define REAL_ROBOT false///////true

std::ofstream log_file;

ros::AsyncSpinner *spinner;

enum ExecutionStage{StageWaitingStart, StageStart, StageExploration, StageManipulation, StageEnd};

geometry_msgs::PoseWithCovarianceStamped robot_pose, last_robot_pose;
float walking_distance;
ExecutionStage old_stage, stage;
std::chrono::_V2::system_clock::time_point start_time, end_time, start_exploration_time, start_manipulation_time;
std::vector<float> exploration_times_sec, exploration_walking_distance, manipulation_walking_distance, manipulation_times_sec, exploration_coverage, manipulation_coverage;
std::vector<int> exploration_n_visited_cells, exploration_n_acessible_cells, manipulation_n_visited_cells, manipulation_n_acessible_cells;
float exploration_walking_distance_start, manipulation_walking_distance_start, coverage_acessible_area;
int n_visited_cells, n_acessible_cells;

ros::Subscriber execution_stage_sub, robot_pose_sub, coverage_acessible_area_sub, semantic_map_sub, navigation_map_sub, semantic_map_visualise_sub, coverage_visualise_sub, n_acessible_cells_sub, n_visited_cells_sub;
std::chrono::_V2::system_clock::time_point robot_pose_time;
// std::chrono::_V2::system_clock::time_point start_chrono, time_now_chrono;

int stage_n;

string file_name;

nav_msgs::OccupancyGrid semantic_map, coverage_visualize;

bool first_semantic_map_received, first_coverage_visualize_received;

// ADAPTED FROM https://github.com/ros-planning/navigation/blob/melodic-devel/map_server/src/map_saver.cpp
bool save_map_to_image(nav_msgs::OccupancyGrid map, string mapname_, bool fisrt_map_received)
{
  if (!fisrt_map_received)
  {
    ROS_ERROR("Without map for %s", mapname_.c_str());
    return false;
  }  

  ROS_INFO("Received a %d X %d map @ %.3f m/pix", map.info.width, map.info.height, map.info.resolution);

  std::string mapdatafile = mapname_ + ".pgm";
  ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
  FILE* out = fopen(mapdatafile.c_str(), "w");
  if (!out)
  {
    ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
    return false;
  }

  fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n", map.info.resolution, map.info.width, map.info.height);
  for(unsigned int y = 0; y < map.info.height; y++) {
    for(unsigned int x = 0; x < map.info.width; x++) {
      unsigned int i = x + (map.info.height - y - 1) * map.info.width;
      fputc(map.data[i], out);
    }
  }

  fclose(out);

  ROS_INFO("Done\n");
  return true;
}


//// Source: http://docs.ros.org/en/jade/api/tf2/html/impl_2utils_8h_source.html
inline double getYaw(const geometry_msgs::Quaternion& q)
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

inline double ComputeYaw(const geometry_msgs::Quaternion& q_gm) {
    return getYaw(q_gm);
}


void execution_stage_callback(const std_msgs::Int32& msg){
  if (ExecutionStage(msg.data) == StageWaitingStart)
  {
    ROS_ERROR("StageWaitingStart");
    return;
  }
  stage_n++;
  old_stage = stage;
  stage = ExecutionStage(msg.data);
  
  auto time_now = std::chrono::high_resolution_clock::now(); //this gives us the current time

  string local_map_name;

  if (old_stage == StageExploration)
  {
    // exploration_times_sec.push_back(ros::Duration(time_now - start_exploration_time).toSec());
    exploration_times_sec.push_back(std::chrono::duration<float>(time_now - start_exploration_time).count());
    exploration_walking_distance.push_back(walking_distance - exploration_walking_distance_start);
    exploration_coverage.push_back(coverage_acessible_area);
    exploration_n_visited_cells.push_back(n_visited_cells);
    exploration_n_acessible_cells.push_back(n_acessible_cells);

    ROS_ERROR("EXPLORATION TIME OF %f secs of %f m, coverage %f percent, %d visited and %d acessible cells", exploration_times_sec.back(), exploration_walking_distance.back(), exploration_coverage.back(), exploration_n_visited_cells.back(), exploration_n_acessible_cells.back());
  }

  if (old_stage == StageManipulation)
  {
    // manipulation_times_sec.push_back(ros::Duration(time_now - start_manipulation_time).toSec());
    manipulation_times_sec.push_back(std::chrono::duration<float>(time_now - start_manipulation_time).count());
    manipulation_walking_distance.push_back(walking_distance - manipulation_walking_distance_start);
    manipulation_coverage.push_back(coverage_acessible_area);
    manipulation_n_visited_cells.push_back(n_visited_cells);
    manipulation_n_acessible_cells.push_back(n_acessible_cells);

    ROS_ERROR("MANIPULATION TIME OF %f secs of %f m, coverage %f percent", manipulation_times_sec.back(), manipulation_walking_distance.back(), manipulation_coverage.back(), manipulation_n_visited_cells.back(), manipulation_n_acessible_cells.back());
  }

  log_file << "NEW STAGE, COUNTER, " << stage_n << ", STAGE, " << stage << ", time, " << std::chrono::duration<float>(time_now - start_time).count() << std::endl;

  save_map_to_image(semantic_map, file_name + "_semantic_" + std::to_string(stage_n), first_semantic_map_received);
  save_map_to_image(coverage_visualize, file_name + "_coverage_" + std::to_string(stage_n) , first_coverage_visualize_received);

  switch (stage)
  {
  case StageStart:
    start_time = time_now;
    break;
  case StageExploration:
    start_exploration_time = time_now;
    exploration_walking_distance_start = walking_distance;
    first_coverage_visualize_received = false;
    first_semantic_map_received = false;

    break;
  case StageManipulation:
    start_manipulation_time = time_now;
    manipulation_walking_distance_start = walking_distance;

    break;
  case StageEnd:
    end_time = time_now;

  default:
    break;
  }
}

void robot_pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  last_robot_pose = robot_pose;
  robot_pose = msg;
  robot_pose_time = std::chrono::high_resolution_clock::now();
  walking_distance += sqrt(pow(robot_pose.pose.pose.position.x - last_robot_pose.pose.pose.position.x, 2) + pow(robot_pose.pose.pose.position.y - last_robot_pose.pose.pose.position.y, 2));
}

void coverage_acessible_area_callback(const std_msgs::Float32& msg){
  coverage_acessible_area = msg.data;
}

#if not REAL_ROBOT
void n_acessible_cells_callback(const std_msgs::Int32& msg){
  n_acessible_cells = msg.data;
}

void n_visited_cells_callback(const std_msgs::Int32& msg){
  n_visited_cells = msg.data;
}
#endif

std::string return_current_time_and_date(){
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%X");
    return ss.str();
}

void semantic_map_callback(const nav_msgs::OccupancyGrid& msg){
  semantic_map = msg;
  if (!first_semantic_map_received)
  {
    first_semantic_map_received = true;
    save_map_to_image(semantic_map, file_name + "_semantic_" + std::to_string(stage_n) + "_FIRST", first_semantic_map_received);
  }  
}

void coverage_visualize_callback(const nav_msgs::OccupancyGrid& msg){
  coverage_visualize = msg;
  if (!first_coverage_visualize_received)
  {
    first_coverage_visualize_received = true;
    save_map_to_image(coverage_visualize, file_name + "_coverage_" + std::to_string(stage_n) + "_FIRST", first_coverage_visualize_received);
  }

  #if REAL_ROBOT
  int n_acessible_cells_on_coverage_map = 0;
  for (int i = 0; i < coverage_visualize.data.size(); i++)
  {
    if (coverage_visualize.data[i] < 100 && coverage_visualize.data[i] != -1)
    {
      n_acessible_cells_on_coverage_map++;
    }    
  }
  n_acessible_cells = n_acessible_cells_on_coverage_map;
  n_visited_cells = float(n_acessible_cells_on_coverage_map) * coverage_acessible_area;
    
  #endif
}

int main(int argc, char **argv) {

  start_time = end_time = std::chrono::high_resolution_clock::now();

  stage_n = 0;
  first_coverage_visualize_received = false;
  first_semantic_map_received = false;

  robot_pose.pose.pose.orientation.w = 1.0;
  robot_pose.pose.pose.orientation.x = 0.0;
  robot_pose.pose.pose.orientation.y = 0.0;
  robot_pose.pose.pose.orientation.z = 0.0;
  robot_pose.pose.pose.position.x = 0.0;
  robot_pose.pose.pose.position.y = 0.0;
  robot_pose.pose.pose.position.z = 0.0;
  last_robot_pose = robot_pose;
  walking_distance = 0.0;
  stage = StageWaitingStart;
  old_stage = StageWaitingStart;
  coverage_acessible_area = 0.0;
  n_acessible_cells = 0;
  n_visited_cells = 0;

  ros::init(argc, argv, "metrics");

  ros::NodeHandle n("~");

  std::string param_dir, dir;
  if (n.searchParam("dir_arq", param_dir)){
    n.getParam(param_dir, dir);
  }else{
    ROS_WARN("No param 'dir_arq' found in an upward search");
    return 0;
  }

  file_name.append(dir);
  file_name.append("/test_");
  file_name.append(return_current_time_and_date());

  ROS_WARN("log_file at %s", file_name.c_str());

  log_file.open(file_name + ".csv", std::ofstream::out);

  log_file << "TIME_STEP, " << TIME_STEP << ", seconds interval between records\n";
  log_file << "time, x, y, z, yaw, walking_distance, coverage\n";

  spinner = new ros::AsyncSpinner(4);
  spinner->start();  

  execution_stage_sub = n.subscribe("/execution_stage", 10, execution_stage_callback);
  robot_pose_sub = n.subscribe("/robot_pose" , 1, robot_pose_callback);
  coverage_acessible_area_sub = n.subscribe("/coverage_acessible_area", 1, coverage_acessible_area_callback);
  #if REAL_ROBOT
  n_acessible_cells = 0;
  n_visited_cells = 0;
  #else
  n_acessible_cells_sub = n.subscribe("/n_acessible_cells", 1, n_acessible_cells_callback);
  n_visited_cells_sub = n.subscribe("/n_visited_cells", 1, n_visited_cells_callback);
  #endif
  semantic_map_visualise_sub = n.subscribe("/semantic_map_visualise", 1, semantic_map_callback);///nav_msgs::OccupancyGrid
  coverage_visualise_sub = n.subscribe("/potencial_visualise", 1, coverage_visualize_callback);


  do{
    ROS_INFO("NOT STARTED");
    ros::Duration(TIME_STEP).sleep();
    ros::spinOnce();
  }while(stage == StageWaitingStart && ros::ok());

  ROS_WARN("STARTED");

  geometry_msgs::Pose pose;
  do{
    ros::Duration(TIME_STEP).sleep();
    ros::spinOnce();
    pose = robot_pose.pose.pose;    
    log_file << std::chrono::duration<float>(robot_pose_time - start_time).count();
    log_file << " , " << pose.position.x << " , " << pose.position.y << " , " << pose.position.z ;
    log_file << " , " << ComputeYaw(pose.orientation);
    log_file << " , " << walking_distance;
    log_file << " , " << coverage_acessible_area;
    log_file << " , " << n_visited_cells;
    log_file << " , " << n_acessible_cells << std::endl;
  }while(stage != StageEnd && ros::ok());


  ROS_ERROR("EXECUTION TIME OF %f secs = %f min", std::chrono::duration<float>(end_time - start_time).count(), std::chrono::duration<float>(end_time - start_time).count()/60.0);
  ROS_ERROR("The robot walked %f m", walking_distance);
  log_file << "EXECUTION TIME OF , " << std::chrono::duration<float>(end_time - start_time).count() << " , secs" << std::endl;
  log_file << "The robot walked " << walking_distance << " m" << std::endl;

  float sum = 0.0, sum_m = 0.0;
  
  sum = 0.0;
  sum_m = 0.0;
  ROS_ERROR("EXPLORATION TIMES......................");
  log_file << "EXPLORATION TIMES......................" << std::endl;
  for (int i = 0; i < exploration_times_sec.size(); i++)
  {
    ROS_ERROR("%f seg , %f m, coverage %f percent, %d visited cells, %d acessible cells", exploration_times_sec[i], exploration_walking_distance[i], exploration_coverage[i], exploration_n_visited_cells[i], exploration_n_acessible_cells[i]);
    log_file << exploration_times_sec[i]<< " seg , " << exploration_walking_distance[i] << " m, coverage " << exploration_coverage[i] << " percent, " << exploration_n_visited_cells[i] << " visited cells, " << exploration_n_acessible_cells[i] << " acessible cells" << std::endl;
    sum += exploration_times_sec[i];
    sum_m += exploration_walking_distance[i];
  }
  ROS_ERROR("Total %f seg, %f m", sum, sum_m);
  log_file << "Total " << sum << " seg, " << sum_m << " m";

  if (exploration_coverage.size() > 0)
  {
    ROS_ERROR(", final coverage %f percent, %d visited cells, %d acessible cells", exploration_coverage.back(), exploration_n_visited_cells.back(), exploration_n_acessible_cells.back());
    log_file << ", final coverage " << exploration_coverage.back() << " percent, " << exploration_n_visited_cells.back() << " visited cells, " << exploration_n_acessible_cells.back() << " acessible cells" << std::endl;
  }

  sum = 0.0;
  sum_m = 0.0;
  ROS_ERROR("MANIPULATION TIMES......................");
  log_file << "MANIPULATION TIMES......................" << std::endl;
  for (int i = 0; i < manipulation_times_sec.size(); i++)
  {
    ROS_ERROR("%f seg , %f m, coverage %f percent, %d visited cells, %d acessible cells", manipulation_times_sec[i], manipulation_walking_distance[i], manipulation_coverage[i], manipulation_n_visited_cells[i], manipulation_n_acessible_cells[i]);
    log_file << manipulation_times_sec[i]<< " seg , " << manipulation_walking_distance[i] << " m, coverage " << manipulation_coverage[i] << " percent, " << manipulation_n_visited_cells[i] << " visited cells, " << manipulation_n_acessible_cells[i] << " acessible cells" << std::endl;
    sum += manipulation_times_sec[i];
    sum_m += manipulation_walking_distance[i];
  }
  ROS_ERROR("Total %f seg, %f m", sum, sum_m);
  log_file << "Total " << sum << " seg, " << sum_m << " m";

  if (manipulation_coverage.size() > 0)
  {
    ROS_ERROR(", final coverage %f percent, %d visited cells, %d acessible cells", manipulation_coverage.back(), manipulation_n_visited_cells.back(), manipulation_n_acessible_cells.back());
    log_file << ", final coverage " << manipulation_coverage.back() << " percent, " << manipulation_n_visited_cells.back() << " visited cells, " << manipulation_n_acessible_cells.back() << " acessible cells" << std::endl;
  }  

  ROS_WARN("ENDED");

	auto end = std::chrono::high_resolution_clock::now();

	std::chrono::duration<float> duration = end - start_time; //we are computing the time difference
	log_file << "TOTAL TIME, " << duration.count() << ", seconds " << std::endl;
  ROS_ERROR("TOTAL TIME, %f", duration.count());

  log_file.close();
  ROS_WARN("File closed");

  spinner->stop();
  ros::waitForShutdown();
  return 0;
};

int mainTEST_PRINT(int argc, char **argv) {

  ros::init(argc, argv, "metrics");

  ros::NodeHandle n;

  while (ros::ok()) {
    ROS_INFO("Metrics...");
    ros::spinOnce();
  }
  return 0;
};