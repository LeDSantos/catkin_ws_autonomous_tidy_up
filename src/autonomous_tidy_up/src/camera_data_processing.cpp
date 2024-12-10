
/* FONTE: https://github.com/ros-planning/moveit_tutorials/blob/master/doc/perception_pipeline/src/detect_and_add_cylinder_collision_object_demo.cpp
https://ros-planning.github.io/moveit_tutorials/doc/perception_pipeline/perception_pipeline_tutorial.html

*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <math.h>

#include <pcl/filters/voxel_grid.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

#include <std_msgs/Bool.h>

#include <pcl_ros/transforms.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <jsk_recognition_msgs/LabelArray.h>
#include <vector>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <nav_msgs/Odometry.h>

#include <pcl/filters/passthrough.h>

#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <std_msgs/Empty.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define SIMULATION true

typedef moveit::planning_interface::PlanningSceneInterface PlanningScene;

#define MIN_OBJECT_HEIGHT 0.02
#define ONE_EIGHTH_OF_PI (M_PI/8.0)
#define MAX_DIST_ROBOT 3.0

namespace get_detected_objects{
  
  class GetDetectedObjects
  {
  private:
    ros::NodeHandle nh_;
    ros::Publisher table_history_cloud_pub, only_pub_octomap_pub, table_on_map_pub, object_on_map_pub, stop_movement_pub;  
    sensor_msgs::PointCloud2 table_history_cloud_msg;
    std_msgs::Bool only_pub_octomap_msg;

    std_msgs::Bool stop_movement_msg;

    ros::Subscriber cluster_indices_sub, cloud_sub, labels_sub, table_cloud_sub, odom_sub, robot_pose_sub;
    pcl::PointCloud<pcl::PointXYZ> input_cloud, table_input_cloud, table_cloud_history;
    geometry_msgs::Pose robot_pose;

    ros::Subscriber image_raw_slow_sub;
    ros::Time time_last_image_raw_slow;
    int seq_last_image_raw_slow;
    pcl::PointCloud<pcl::PointXYZ> input_cloud_for_object;

    PlanningScene planning_scene_interface;
    const std::string ROBOT_DESCRIPTION{"robot_description"};
    planning_scene_monitor::PlanningSceneMonitorPtr psm_;

    tf::StampedTransform& transform, tf_frame_input_cloud_to_map, tf_map_to_planning_scene;
    tf::StampedTransform transform_frame_input_cloud_to_map;
    tf::TransformListener listener_frame_input_cloud_to_map;
    bool& object_detection_on;
    bool& table_detection_on;
    bool& block_planning_scene;
    std::vector<std::string> labels;
    std::vector<std::string> labels_to_ignore;
    std::vector<int> index_labels_to_ignore;
    geometry_msgs::TransformStamped& transformStamped;
    inline double getYaw(const geometry_msgs::Quaternion& q);
    inline double ComputeYaw(const geometry_msgs::Quaternion& q_gm);

    boost::shared_ptr<ros::AsyncSpinner> g_spinner;
    ros::CallbackQueue queue;
    ros::Subscriber hb;
    // and this one uses custom queue
    ros::NodeHandle hb_n;
    bool got_input_cloud_for_object;

    //For sync: https://gist.github.com/tdenewiler/e2172f628e49ab633ef2786207793336
    message_filters::Subscriber<sensor_msgs::Image> sub_1_color_image;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_2_depth_points;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy_camera_data;
    typedef message_filters::Synchronizer<MySyncPolicy_camera_data> Sync_camera_data;
    boost::shared_ptr<Sync_camera_data> sync_camera_data_;

    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_1_cluster_indices;
    message_filters::Subscriber<jsk_recognition_msgs::LabelArray> sub_2_labels;
    typedef message_filters::sync_policies::ApproximateTime<jsk_recognition_msgs::ClusterPointIndices, jsk_recognition_msgs::LabelArray> MySyncPolicy_mask_rcnn_output;
    typedef message_filters::Synchronizer<MySyncPolicy_mask_rcnn_output> Sync_mask_rcnn_output;
    boost::shared_ptr<Sync_mask_rcnn_output> sync_mask_rcnn_output_;

  public:
    /** @brief Sets the environment by advertising the publishers; setting the synchronized subscribers; and starting the spinner.
     * Also sets important local variables, like the labels to ignore. */
    GetDetectedObjects(tf::StampedTransform& transform_input, tf::StampedTransform& tf_frame_input_cloud_to_map_input, tf::StampedTransform& tf_map_to_planning_scene_input, bool& object_detection_on_input, bool& table_detection_on_input, geometry_msgs::TransformStamped& transformStamped_input, bool& block_planning_scene_input);
    ~GetDetectedObjects();

    /** @brief Connects the node with MoveIt planning_scene. */
    bool SetupPlanningScene();

    /** @brief Uses the cloud to find tables. A cloud history is used.
     * @param input cloud message. */
    void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input);
    
    /** @brief Used in communication with octomap_and_planning_scene_node.
     * @param value value to be published. */
    void OnlyPubOctomap(bool value);
    std::string frame_input_cloud = "base_footprint";
    std::string planning_scene_frame = "base_footprint";

    /** @brief Publishes the object, which can be a tidy-up object or a table, in the respective topic with collision object format. 
     * @param name object name: table... for tables; and label name for tidy-up objects.
     * @param size_x size of the x dimension.
     * @param size_y size of the y dimension.
     * @param size_z size of the z dimension.
     * @param center object center.
     * @param frame reference frame. */
    void PubAsCollisionObject(std::string name, float size_x, float size_y, float size_z, geometry_msgs::Pose center, std::string frame);
    
    /** @brief Finds if the object's label is in ignore list. Adds its index to a list if it is.
     * @param object object label.
     * @param index index of the cluster that corresponds to the object.
     * @return true 
     * @return false */
    bool IgnoreObject(std::string object, int index);

    /** @brief Finds if the object index is in indexes to ignore list.
     * @param index index of the cluster that corresponds to the object.
     * @return true 
     * @return false */
    bool IgnoreObject(int index);

    /** @brief Calculates the cuboid that better fits the point cloud.
     * @param cloud point cloud.
     * @param new_center cuboid's center on the map.
     * @param size_x size of the x dimension.
     * @param size_y size of the y dimension.
     * @param size_z size of the z dimension. */
    void FindBoundingCube(pcl::PointCloud<pcl::PointXYZ> cloud, geometry_msgs::Pose *new_center, float *size_x, float *size_y, float *size_z);
    void odom_callback(const nav_msgs::Odometry& msg);
    void robot_pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg);

    /** @brief Synchronizes camera output and collects the cloud for object identification. 
     * @param img image message.
     * @param input cloud message. */
    void image_raw_slow_AND_cloudCB_SYNC_callback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::PointCloud2ConstPtr& input);
 
    /** @brief Synchronizes mask_rcnn node output and identifies objects. 
     * @param msg_cluster clusters message.
     * @param msg_label labels message. */    
    void cluster_indices_AND_labels_callback_SYNC_callback(const jsk_recognition_msgs::ClusterPointIndicesConstPtr& msg_cluster, const jsk_recognition_msgs::LabelArrayConstPtr& msg_label);
  
    /** @brief When running on the real robot, used to inform the autonomous_tidy_up_run node that the robot saw some object. So, stop any movement to observe.
     *  @param value value to be published. true is the default.*/
    void PubStopMovement(bool value = true);
    ros::Time last_PubStopMovement;
  };
  
  void GetDetectedObjects::odom_callback(const nav_msgs::Odometry& msg)
  {
    robot_pose = msg.pose.pose;
  }

  void GetDetectedObjects::robot_pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
  {
    robot_pose = msg.pose.pose;
  }

  //// Source: http://docs.ros.org/en/jade/api/tf2/html/impl_2utils_8h_source.html
  inline
  double GetDetectedObjects::getYaw(const geometry_msgs::Quaternion& q)
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

  inline double GetDetectedObjects::ComputeYaw(const geometry_msgs::Quaternion& q_gm) {
      return getYaw(q_gm);
  }

  
  GetDetectedObjects::GetDetectedObjects(tf::StampedTransform& transform_input, tf::StampedTransform& tf_frame_input_cloud_to_map_input, tf::StampedTransform& tf_map_to_planning_scene_input, bool& object_detection_on_input, bool& table_detection_on_input, geometry_msgs::TransformStamped& transformStamped_input, bool& block_planning_scene_input) :
    nh_("~"), transform(transform_input), tf_frame_input_cloud_to_map(tf_frame_input_cloud_to_map_input), tf_map_to_planning_scene(tf_map_to_planning_scene_input), object_detection_on(object_detection_on_input), table_detection_on(table_detection_on_input), transformStamped(transformStamped_input), block_planning_scene(block_planning_scene_input)
  {
    ROS_WARN("PlanningScene %s", (SetupPlanningScene()? "OK" : "fail"));

    table_history_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("/table_history_cloud", 5);
    only_pub_octomap_pub = nh_.advertise<std_msgs::Bool>("/only_pub_octomap", 5);

    cloud_sub = nh_.subscribe("/xtion/depth_registered/points", 1, &GetDetectedObjects::cloudCB, this);
    
    #if SIMULATION
    odom_sub = nh_.subscribe("/mobile_base_controller/odom" , 1, &GetDetectedObjects::odom_callback, this);
    #else
    robot_pose_sub = nh_.subscribe("/robot_pose" , 1, &GetDetectedObjects::robot_pose_callback, this);
    #endif

    // Detected objects to ignore. These objects are too big or too small to grasp by the robot.
    labels_to_ignore.push_back(std::string("marble"));
    labels_to_ignore.push_back(std::string("refrigerator"));
    labels_to_ignore.push_back(std::string("microwave"));
    labels_to_ignore.push_back(std::string("tv/monitor"));
    labels_to_ignore.push_back(std::string("fork"));
    labels_to_ignore.push_back(std::string("spoon"));
    labels_to_ignore.push_back(std::string("knife"));
    labels_to_ignore.push_back(std::string("alfort"));
    labels_to_ignore.push_back(std::string("table"));
    labels_to_ignore.push_back(std::string("shelf_flont"));
    labels_to_ignore.push_back(std::string("takenoko"));

    // Table or object representation of the detection after all the camera data processing.
    table_on_map_pub =  nh_.advertise<moveit_msgs::CollisionObject>("/table_on_map", 5);
    object_on_map_pub = nh_.advertise<moveit_msgs::CollisionObject>("/object_on_map", 5);
    
    stop_movement_pub = nh_.advertise<std_msgs::Bool>("/stop_movement", 1);
    last_PubStopMovement = ros::Time::now();

    got_input_cloud_for_object = false;

    // Set custom callback queue
    hb_n.setCallbackQueue(&queue);

    // The synchronization is important especially for the real robot because of delayed communication.

    // Synchronization of camera raw data.
    sub_1_color_image.subscribe(nh_, "/xtion/rgb/image_raw_slow", 1);
    sub_2_depth_points.subscribe(nh_, "/xtion/depth_registered/points", 1);
    sync_camera_data_.reset(new Sync_camera_data(MySyncPolicy_camera_data(100), sub_1_color_image, sub_2_depth_points));
    sync_camera_data_->registerCallback(boost::bind(&GetDetectedObjects::image_raw_slow_AND_cloudCB_SYNC_callback, this, _1, _2));

    // Synchronization of mask_rcnn raw data.
    sub_1_cluster_indices.subscribe(nh_, "/mask_rcnn_73b2_kitchen/output/cluster_indices", 1);
    sub_2_labels.subscribe(nh_, "/mask_rcnn_73b2_kitchen/output/labels", 1);
    sync_mask_rcnn_output_.reset(new Sync_mask_rcnn_output(MySyncPolicy_mask_rcnn_output(100), sub_1_cluster_indices, sub_2_labels));//talvez 10 no lugar de 100
    sync_mask_rcnn_output_->registerCallback(boost::bind(&GetDetectedObjects::cluster_indices_AND_labels_callback_SYNC_callback, this, _1, _2));

    // Create AsyncSpinner, run it on all available cores and make it process custom callback queue.
    g_spinner.reset(new ros::AsyncSpinner(0, &queue));

    // Clear old callback from the queue.
    queue.clear();
    // Start the spinner.
    g_spinner->start();
    ROS_INFO("Spinner enabled");
  }
  
  GetDetectedObjects::~GetDetectedObjects()
  {
  }

  void GetDetectedObjects::image_raw_slow_AND_cloudCB_SYNC_callback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::PointCloud2ConstPtr& input)
  {
    ROS_ERROR("Synchronization successful");
    time_last_image_raw_slow = img->header.stamp;
    seq_last_image_raw_slow = img->header.seq;

    ros::Duration diff = input->header.stamp - time_last_image_raw_slow;

    if (abs(diff.toSec()) < 0.3) //acceptable time difference
    {
      got_input_cloud_for_object = true;
      pcl::fromROSMsg(*input, input_cloud_for_object);    
    } else
    {
      got_input_cloud_for_object = false;
    }    
  }

  void GetDetectedObjects::cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    frame_input_cloud = input->header.frame_id;
    pcl::fromROSMsg(*input, input_cloud);

    if (!table_detection_on)
    {
      return;
    }

    // Transforms the cloud to map reference.
    try{
      listener_frame_input_cloud_to_map.waitForTransform("map", frame_input_cloud, input->header.stamp, ros::Duration(2.0));
      listener_frame_input_cloud_to_map.lookupTransform("map", frame_input_cloud, input->header.stamp, transform_frame_input_cloud_to_map);
    } catch (tf::TransformException ex){
      ROS_ERROR("cloudCB %s",ex.what());
      return;
    }
    pcl::PointCloud<pcl::PointXYZ> cloud_z_filtered, input_cloud_on_map;
    pcl_ros::transformPointCloud(input_cloud, input_cloud_on_map, transform_frame_input_cloud_to_map);

    // Filter to limit the height of the points that can be a table.
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input_cloud_on_map.makeShared());
    pass.setFilterFieldName("z");
    #if SIMULATION
    pass.setFilterLimits(0.6, 1.6);
    #else
    pass.setFilterLimits(0.7, 1.6);
    #endif
    pass.filter(cloud_z_filtered);

    pcl::PointCloud<pcl::PointXYZ> table_input_cloud_map;
    pcl::PointCloud<pcl::PointXYZ> table_cloud_history_raw, cloud_filtered;

    // Only consideres the points near the robot for better precision.
    for (int i = 0; i < cloud_z_filtered.points.size(); i++)
    {
      pcl::PointXYZ point_ = cloud_z_filtered.points[i];
      float dist = sqrt(pow(point_.x - robot_pose.position.x, 2.0) + pow(point_.y - robot_pose.position.y, 2.0));
      if (dist < MAX_DIST_ROBOT){
        table_cloud_history.push_back(point_);
      }
    }

    // Filters the history to avoid duplicate points.
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (table_cloud_history.makeShared());
    sor.setLeafSize (0.02f, 0.02f, 0.02f);
    sor.filter (cloud_filtered);

    ROS_WARN("cloud_filtered with %d points", cloud_filtered.size());
    
    pcl::PointIndices result;
    pcl::ModelCoefficients model_coefficients;

    // Segments the filtered cloud to find perpendicular planes.
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
    seg.setEpsAngle(DEG2RAD(10.0));
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud(cloud_filtered.makeShared());
    seg.segment(result, model_coefficients);

    // Adds the perpendicular points to the history.
    table_cloud_history.clear();
    for (const auto& idx : result.indices)
    {
      table_cloud_history.push_back(cloud_filtered.points[idx]);
    }

    // Publishes the history.
    pcl::toROSMsg(table_cloud_history, table_history_cloud_msg);
    table_history_cloud_msg.header.frame_id = "map";
    table_history_cloud_msg.header.stamp = ros::Time::now();
    table_history_cloud_pub.publish(table_history_cloud_msg);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    std::vector<pcl::PointIndices> cluster_indices;
    
    // Creates the KdTree object for the search method of the extraction.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    if (table_cloud_history.size() == 0)
    {
      ROS_WARN("Empty table cloud history");
      return;
    } 

    // Finds the clusters that correspond to tables.
    tree->setInputCloud (table_cloud_history.makeShared());
    ec.setClusterTolerance (0.05);
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (table_cloud_history.makeShared());
    ec.extract (cluster_indices);

    ROS_WARN("FOUND %d tables", cluster_indices.size());

    // Process each cluster to find the collision object that represents a table.
    for (int n = 0; n < cluster_indices.size(); n++)
    {
      auto cluster = cluster_indices[n];
      pcl::PointCloud<pcl::PointXYZ> cloud_on_map, cloud_raw;

      // Calculates the average points height.
      pcl::PointXYZ point;
      float height_sum = 0.0, height_avg, tolerance = 0.025;
      for (const auto& idx : cluster.indices) {
        cloud_raw.points.push_back(table_cloud_history.points[idx]);
        height_sum += table_cloud_history.points[idx].z;
      }
      height_avg = height_sum / float(cloud_raw.size());
      
      // Ignores the points outside a height tolerance. This guarantees a smooth surface.
      for (int it = 0; it < cloud_raw.size(); it++)
      {
        if (abs(cloud_raw.points[it].z - height_avg) < tolerance)
        {
          cloud_on_map.points.push_back(cloud_raw.points[it]);
        }      
      }
      if(cloud_on_map.size() == 0) continue;

      // Finds the bounding cuboid that better encapsulates the points.
      geometry_msgs::Pose new_center;
      float size_x, size_y, size_z;
      FindBoundingCube(cloud_on_map, &new_center, &size_x, &size_y, &size_z);

      if (size_x < 0.25 or size_y < 0.25)
      {
        // ROS_WARN("Too small for a table.");
        continue;
      }    

      // Ignores table with low point density, that can be a result of many objects on the table top that cause holes on the cluster.
      std::string object_name = "table";
      float dense = float(cloud_on_map.size()) / (size_x*size_y);
      if (dense < 1500.0)
      {
        // ROS_ERROR("Low density.");
        continue;
      }

      // Calculates table with solid base, to prevent the arm or object from getting caught in the legs not detected by octomap.
      size_x = size_x + 0.05; // Expansion added
      size_y = size_y + 0.05;
      float new_size_z = size_z/2.0 + new_center.position.z;
      new_center.position.z = new_size_z/2.0;      
      PubAsCollisionObject(object_name + std::to_string(n), size_x, size_y, new_size_z, new_center, "map");
    }  
  }

  bool GetDetectedObjects::SetupPlanningScene()
  {
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(ROBOT_DESCRIPTION);
    ros::spinOnce();

    // Update the planning scene monitor with the current state.
    bool success = false;
    while (ros::ok() && !success){
      success = psm_->requestPlanningSceneState("/get_planning_scene");
      ROS_WARN_STREAM("Request planning scene " << (success ? "succeeded." : "failed."));
    }

    // Keep up to date with new changes.
    psm_->startSceneMonitor("/move_group/monitored_planning_scene");
    // #if DEBUG
      psm_->getPlanningScene()->printKnownObjects();
    // #endif

    return success;
  }

  void GetDetectedObjects::PubAsCollisionObject(std::string name, float size_x, float size_y, float size_z, geometry_msgs::Pose center, std::string frame)
  {
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    collision_objects[0].header.frame_id = frame;
    collision_objects[0].header.stamp = ros::Time::now();
    collision_objects[0].id = name;

    collision_objects[0].primitives.resize(1);

    collision_objects[0].primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = size_x;
    collision_objects[0].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = size_y;
    collision_objects[0].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = size_z;

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0] = center;

    float rotation_angle = ComputeYaw(collision_objects[0].primitive_poses[0].orientation);

    collision_objects[0].operation = collision_objects[0].ADD;

    // Publishes it in the respective topic.
    if (frame.compare("map") == 0)
    {
      if (name[0] == 't' && name[1] == 'a') //table
      {
        table_on_map_pub.publish(collision_objects[0]);
      } else //object
      {
        ROS_WARN("object_on_map_pub...");
        object_on_map_pub.publish(collision_objects[0]);
      }
    }

    if (block_planning_scene)
    {
      ROS_WARN("planning_scene blocked");
      return;
    }
  }

  bool GetDetectedObjects::IgnoreObject(std::string object, int index)
  {
    bool result;
    result = (std::find(labels_to_ignore.begin(), labels_to_ignore.end(), object) != labels_to_ignore.end());
    if (result)
    {
        ROS_WARN("FOUND object to ignore");
        index_labels_to_ignore.push_back(index);
    }
    return result;
  } 

  bool GetDetectedObjects::IgnoreObject(int index)
  {
    return (std::find(index_labels_to_ignore.begin(), index_labels_to_ignore.end(), index) != index_labels_to_ignore.end());
  }

  void GetDetectedObjects::cluster_indices_AND_labels_callback_SYNC_callback(const jsk_recognition_msgs::ClusterPointIndicesConstPtr& msg_cluster, const jsk_recognition_msgs::LabelArrayConstPtr& msg_label){
    ROS_ERROR("RECEIVED CLUSTER_INDICES AND LABELS");

    if (!object_detection_on)
    {
      return;
    }

    if (input_cloud_for_object.size() <= 0)
    {
      return;
    } else
    {
      // ROS_ERROR("input_cloud_for_object size: %d", input_cloud_for_object.size());
    }

    if (!got_input_cloud_for_object)
    {
      // ROS_ERROR("don't got input_cloud_for_object");
      return;
    }

    // Gets the number of clusters.
    int num_objects = msg_cluster->cluster_indices.size();
    ROS_WARN("--- FOUND %d objects", num_objects);
    if (num_objects == 0){
      return;
    }

    // Gets the number of labels.
    labels.resize(msg_label->labels.size());
    for (int i = 0; i < msg_label->labels.size(); i++)
    {
      labels[i] = msg_label->labels[i].name;
    } 

    if (num_objects != labels.size()){
      ROS_WARN("--- FOUND %d objects, but %d labels", num_objects, labels.size());
      return;
    }

    // Transforms the cloud to map reference.
    try{
      listener_frame_input_cloud_to_map.waitForTransform("map", frame_input_cloud, time_last_image_raw_slow, ros::Duration(2.0));    
      listener_frame_input_cloud_to_map.lookupTransform("map", frame_input_cloud, time_last_image_raw_slow, transform_frame_input_cloud_to_map);
    } catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    index_labels_to_ignore.resize(0);
    std::vector<std::vector<int>> indices_object;

    // Ignores clusters with some labels.
    for (int i = 0; i < num_objects; i++)
    {
      std::vector<int> indices;
      if (IgnoreObject(labels[i], i))
      { //Ignored object has a empty indices_object vector.
        indices_object.push_back(indices);
        continue;
      }

      for (int index = 0; index < msg_cluster->cluster_indices[i].indices.size(); index++)
      {
        indices.push_back(msg_cluster->cluster_indices[i].indices[index]);
      }
      indices_object.push_back(indices);
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>> cloud_object_raw, cloud_object, cloud_without_distant_points;
    pcl::PointCloud<pcl::PointXYZ> cloud, cloud_transformed;
    cloud_object.resize(num_objects);
    cloud_without_distant_points.resize(num_objects);

    int n_valid_object_found = 0;
    // Process each cluster to find the cloud that represents a valid object.
    for (int i = 0; i < num_objects; i++)
    {
      ROS_WARN("--- object %d %s....", i, labels[i].c_str());
      if (IgnoreObject(i))
      {
        ROS_WARN("--- ignoring");
        cloud_object_raw.push_back(cloud_transformed);
        continue;
      }
      
      for (int index = 0; index < indices_object[i].size(); index++)
      {
        if(indices_object[i][index] >= 0 && indices_object[i][index] < input_cloud_for_object.points.size())
          cloud.points.push_back(input_cloud_for_object.points[indices_object[i][index]]);
      }

      pcl_ros::transformPointCloud(cloud, cloud_transformed, transform_frame_input_cloud_to_map);
      cloud_object_raw.push_back(cloud_transformed);
      n_valid_object_found++;
    }

    // Used only in real experiments to stop any robot movement and allow better observation.
    if (n_valid_object_found > 0)
    {
      PubStopMovement();
    }
    
    float object_min_z = 0.0;

    // Process each cloud to find the collision object that represents a object.
    for (int n = 0; n < num_objects; n++)
    {
      if (IgnoreObject(n) or cloud_object_raw[n].points.size() == 0)
      {
        ROS_WARN("--- ignoring or empty cloud");
        continue;
      }

      // Calculates the minimum points height.
      object_min_z = cloud_object_raw[n].points[0].z;
      for (int i = 1; i < cloud_object_raw[n].points.size(); i++)
      {
        pcl::PointXYZ point_ = cloud_object_raw[n].points[i];
        if (point_.z < object_min_z){
          object_min_z = point_.z;
        }
      }
      ROS_WARN("--- 1: cloud of %d points", cloud_object_raw[n].points.size());

      // Disregards the points under a minimum height, because the cluster can outline some points outside the object and add a shadow projected at the same level as the object bottom. 
      for (int i = 0; i < cloud_object_raw[n].points.size(); i++)
      {
        pcl::PointXYZ point_ = cloud_object_raw[n].points[i];
        if (point_.z > object_min_z + MIN_OBJECT_HEIGHT){
          cloud_object[n].points.push_back(point_);
        }
      }

      ROS_WARN("--- 2: cloud of %d points", cloud_object[n].points.size());
      if (cloud_object[n].size() == 0)
      {
        continue;
      }

      // Creating the KdTree object for the search method of the extraction.
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

      // Finds the clusters on a object cloud.
      tree->setInputCloud (cloud_object[n].makeShared());
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      std::vector<pcl::PointIndices> cluster_indices;

      ec.setClusterTolerance (0.05);
      ec.setMinClusterSize (50);
      ec.setMaxClusterSize (5000);
      ec.setSearchMethod (tree);
      ec.setInputCloud (cloud_object[n].makeShared());
      ec.extract (cluster_indices);

      // Uses the bigger cluster to filter outliers (shadow and residues that do not belong to the object).
      pcl::PointIndices bigger_cluster;
      bigger_cluster.indices.resize(0);
      for (int i = 0; i < cluster_indices.size(); i++)
      {
        if(cluster_indices[i].indices.size() > bigger_cluster.indices.size())
        {
          bigger_cluster = cluster_indices[i];
        }
      }
      for (int i = 0; i < bigger_cluster.indices.size(); i++)
      {
        pcl::PointXYZ point_ = cloud_object[n].points[bigger_cluster.indices[i]];
        cloud_without_distant_points[n].points.push_back(point_);
      }

      ROS_WARN("--- 3: cloud of %d points", cloud_without_distant_points[n].points.size());
      if (cloud_without_distant_points[n].size() == 0)
      {
        continue;
      }

      // Finds the bounding cuboid that better encapsulates the points.
      geometry_msgs::Pose new_center;
      float size_x, size_y, size_z;
      FindBoundingCube(cloud_without_distant_points[n], &new_center, &size_x, &size_y, &size_z);
      if (size_x > 2.0 or size_y > 2.0 or size_z > 2.0) // Objects too big are ignored.
      {
        continue;
      }
      PubAsCollisionObject(labels[n], size_x, size_y, size_z, new_center, "map");
    }
  }
    

  void GetDetectedObjects::FindBoundingCube(pcl::PointCloud<pcl::PointXYZ> cloud, geometry_msgs::Pose *new_center, float *size_x, float *size_y, float *size_z){

    float object_min_x = cloud.points[0].x;
    float object_max_x = cloud.points[0].x;
    float object_min_y = cloud.points[0].y;
    float object_max_y = cloud.points[0].y;
    float object_min_z = cloud.points[0].z;
    float object_max_z = cloud.points[0].z;

    // Finds the limits.
    for (int i = 1; i < cloud.points.size(); i++)
    {
      pcl::PointXYZ point_ = cloud.points[i];
      // ROS_WARN("found %d x: %f ... y: %f ... z: %f", i, point_.x(), point_.y(), point_.z());
      if (point_.x < object_min_x){
        object_min_x = point_.x;
      }
      if (point_.x > object_max_x){
        object_max_x = point_.x;
      }
      if (point_.y < object_min_y){
        object_min_y = point_.y;
      }
      if (point_.y > object_max_y){
        object_max_y = point_.y;
      }
      if (point_.z < object_min_z){
        object_min_z = point_.z;
      }
      if (point_.z > object_max_z){
        object_max_z = point_.z;
      }
    }

    // ROS_WARN("Object -- FOUND limits-- x: %f to %f;  y: %f to %f; z: %f to %f;", object_min_x, object_max_x, object_min_y, object_max_y, object_min_z, object_max_z);

    // Calculates the center of the original cloud.
    pcl::PointXYZ original_center;
    original_center.x = object_min_x + ( (object_max_x - object_min_x) / 2.0);
    original_center.y = object_min_y + ( (object_max_y - object_min_y) / 2.0);
    original_center.z = object_min_z + ( (object_max_z - object_min_z) / 2.0);


    tf::Vector3 origin(-original_center.x, -original_center.y, -original_center.z);

    tf::Quaternion new_quat, rot_quat;
    new_quat.setEuler(0.0, 0.0, 0.0);
    rot_quat.setEuler(0.0, 0.0, 3.14/2.0);
    
    // Centers the cloud on the center of the original cloud.
    tf::Transform transform_center_object(new_quat, origin), final_transform;
    
    pcl::PointCloud<pcl::PointXYZ> cloud_transformed_center_object, cloud_rot;
    pcl_ros::transformPointCloud(cloud, cloud_transformed_center_object, transform_center_object);
    std::vector<float> volumes, object_min_x_many_angles, object_max_x_many_angles, object_min_y_many_angles, object_max_y_many_angles;

    // Rotates the cloud around the origin.
    for (int i = 0; i <= 8; i++)/// 0 degrees to 90 degrees
    {
      rot_quat.setEuler(0.0, 0.0, float(i)*ONE_EIGHTH_OF_PI/2.0);
      tf::Transform transform_rot(rot_quat);
      pcl_ros::transformPointCloud(cloud_transformed_center_object, cloud_rot, transform_rot);

      object_min_x_many_angles.push_back(cloud_rot.points[0].x);
      object_max_x_many_angles.push_back(cloud_rot.points[0].x);
      object_min_y_many_angles.push_back(cloud_rot.points[0].y);
      object_max_y_many_angles.push_back(cloud_rot.points[0].y);
      // object_min_z = cloud_rot.points[0].z;
      // object_max_z = cloud_rot.points[0].z;

      // Finds the limits.
      for (int i = 1; i < cloud_rot.points.size(); i++)
      {
        pcl::PointXYZ point_ = cloud_rot.points[i];
        // ROS_WARN("found %d x: %f ... y: %f ... z: %f", i, point_.x(), point_.y(), point_.z());
        if (point_.x < object_min_x_many_angles.back()){
          object_min_x_many_angles.back() = point_.x;
        }
        if (point_.x > object_max_x_many_angles.back()){
          object_max_x_many_angles.back() = point_.x;
        }
        if (point_.y < object_min_y_many_angles.back()){
          object_min_y_many_angles.back() = point_.y;
        }
        if (point_.y > object_max_y_many_angles.back()){
          object_max_y_many_angles.back() = point_.y;
        }
      }

      // Calculates the volume.
      float volume;
      volume = (object_max_x_many_angles.back() - object_min_x_many_angles.back()) * (object_max_y_many_angles.back() - object_min_y_many_angles.back()) * (object_max_z - object_min_z);
      // ROS_WARN("Angle %d*pi/16 -- FOUND volume of %f", i, volume);
      volumes.push_back(volume);
    }
    
    // Finds the angle that results in the smallest volume.
    float min_vol = volumes[0];
    int min_vol_index = 0;
    for (int i = 1; i < volumes.size(); i++)
    {
      if (volumes[i] < min_vol)
      {
        min_vol = volumes[i];
        min_vol_index = i;
      }
    }
    rot_quat.setEuler(0.0, 0.0, float(min_vol_index)*ONE_EIGHTH_OF_PI/2.0);
    tf::Transform transform_rot(rot_quat);
    pcl_ros::transformPointCloud(cloud_transformed_center_object, cloud_rot, transform_rot);
      
    // ROS_WARN("Angle %d*pi/16 -- FOUND the SMALLEST volume of %f", min_vol_index, min_vol);

    // float size_x, size_y, size_z;

    // Calculates the new size.
    float expansion = 0.0;

    *size_x = expansion + object_max_x_many_angles[min_vol_index] - object_min_x_many_angles[min_vol_index];
    *size_y = expansion + object_max_y_many_angles[min_vol_index] - object_min_y_many_angles[min_vol_index];
    *size_z = expansion + object_max_z - object_min_z;

    float mag_center_shift, x, y, ang_center_shift;

    x = object_min_x_many_angles[min_vol_index] + (object_max_x_many_angles[min_vol_index] - object_min_x_many_angles[min_vol_index])/2.0;
    y = object_min_y_many_angles[min_vol_index] + (object_max_y_many_angles[min_vol_index] - object_min_y_many_angles[min_vol_index])/2.0;

    mag_center_shift = sqrt(x*x + y*y);
    ang_center_shift = atan2( y, x);
      
    // ROS_WARN("Center shift of %f with angle %f", mag_center_shift, ang_center_shift);

    float rotation_angle = float(min_vol_index)*ONE_EIGHTH_OF_PI/2.0;

    // Calculates the new center.
    tf::Quaternion new_quat2;
    new_quat2.setEuler(0.0, 0.0, -rotation_angle);
    geometry_msgs::Quaternion quat_msg;

    quat_msg.x = new_quat2.x();
    quat_msg.y = new_quat2.y();
    quat_msg.z = new_quat2.z();
    quat_msg.w = new_quat2.w();

    new_center->orientation = quat_msg;
    new_center->position.x = original_center.x + mag_center_shift * cos(ang_center_shift - rotation_angle);
    new_center->position.y = original_center.y + mag_center_shift * sin(ang_center_shift - rotation_angle);
    new_center->position.z = original_center.z;
  }

  void GetDetectedObjects::OnlyPubOctomap(bool value){
    only_pub_octomap_msg.data = value;
    only_pub_octomap_pub.publish(only_pub_octomap_msg);
  }

  void GetDetectedObjects::PubStopMovement(bool value){
    ros::Time now_PubStopMovement = ros::Time::now();
    ros::Duration time = (now_PubStopMovement - last_PubStopMovement);

    // 10 seconds tolerance between each observation.
    if (time.toSec() < 10.0)
    {
      return;
    }
    
    stop_movement_msg.data = value;
    stop_movement_pub.publish(stop_movement_msg);
    last_PubStopMovement = now_PubStopMovement;
  }

}

bool start, start_object, start_table, block_planning_scene;

void start_callback(const std_msgs::Bool& msg){
  start = msg.data;
}

void start_object_callback(const std_msgs::Bool& msg){
  start_object = msg.data;
}

void start_table_callback(const std_msgs::Bool& msg){
  start_table = msg.data;
}

void block_planning_scene_callback(const std_msgs::Bool& msg){
  block_planning_scene = msg.data;  
}

/* Responsible for interpretation of the data provided by the camera and the instance segmentation to find objects and tables.*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_data_processing");

  ros::NodeHandle n;

  // Subscribers for communication.
  ros::Subscriber block_planning_scene_sub, start_object_sub, start_table_sub;
  start_object_sub = n.subscribe("/start_object_detection", 1, start_object_callback);
  start_table_sub = n.subscribe("/start_table_detection", 1, start_table_callback);
  block_planning_scene_sub = n.subscribe("/block_planning_scene", 1, block_planning_scene_callback);

  tf::StampedTransform transform, transform_frame_input_cloud_to_map, transform_map_to_planning_scene;

  start_object = false;
  start_table = false;
  block_planning_scene = false;
  tf::TransformListener listener, listener_frame_input_cloud_to_map, listener_map_to_planning_scene;
  
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  ros::Duration(4.0).sleep();

  get_detected_objects::GetDetectedObjects manipulate_detected_objects(transform, transform_frame_input_cloud_to_map, transform_map_to_planning_scene, start_object, start_table, transformStamped, block_planning_scene);

  // Loop for transforms and ros::spinOnce().
  while (ros::ok())
  {
    ros::Duration(0.5).sleep();
    ros::spinOnce();

    try{
      listener.lookupTransform(manipulate_detected_objects.planning_scene_frame, manipulate_detected_objects.frame_input_cloud, ros::Time(), transform);
    } catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    try{
      listener_frame_input_cloud_to_map.lookupTransform(std::string("map"), manipulate_detected_objects.frame_input_cloud, ros::Time(), transform_frame_input_cloud_to_map);
    } catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    try{
      listener_map_to_planning_scene.lookupTransform(manipulate_detected_objects.planning_scene_frame, std::string("map"), ros::Time(), transform_map_to_planning_scene);
    } catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    try{
      ros::Time past = ros::Time::now() - ros::Duration(2.0);
      transformStamped = tfBuffer.lookupTransform("map", "base_footprint", past, ros::Duration(1.0));
    }
    catch (tf2::TransformException ex) {
      ROS_WARN("%s",ex.what());
    }
  }
}
