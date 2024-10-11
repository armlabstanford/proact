#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
// #include <moveit_msgs/Constraints.h>
// #include <moveit_msgs/OrientationConstraint.h>

// #include <moveit_visual_tools/moveit_visual_tools.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

//cpp
#include <iostream>

// ROS
#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <sensor_msgs/Joy.h>
#include "trajectory_msgs/JointTrajectory.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <mpl_control/graspAction.h>

#include <gazebo_grasp_plugin/GazeboGraspFix.h>
#include <gazebo_grasp_plugin_ros/GazeboGraspEvent.h>

#include <tf2_ros/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>


// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;
const double deg2rad = tau / 360.0;
static const int QUEUE_LENGTH = 1;


class pickplace
{

public: 
  pickplace(ros::NodeHandle &nh, std::string group_name);
  ~pickplace();
  void add_table(void);
  void add_block(std::string block_name);
  void remove_block(std::string block_name);
  // void attach_block(void);
  bool go_to_pose(geometry_msgs::Pose target_pose);
  void set_pick_pose(void);
  void set_place_pose(void);
  void go_to_place(void);
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan_;
  void emgCallback(const std_msgs::String &msg);
  void joyCallback(const sensor_msgs::Joy &msg);
  void blockIDCallback(const std_msgs::String &msg);
  void placeLocationCallback(const geometry_msgs::Vector3Stamped &msg);
  void graspStatusCallback(const gazebo_grasp_plugin_ros::GazeboGraspEventConstPtr &msg);
  void print_tf_for_debug(void);
  void add_all_blocks(void);
  void remove_all_blocks(void);
  void attach_block(std::string block_name);
  void detach_block(std::string block_name);
  void set_neutral_pose(void);
  geometry_msgs::Pose try_closer(geometry_msgs::Pose target, float extent);
  geometry_msgs::Pose try_above(geometry_msgs::Pose target, float margin);
  bool fresh_attempt(void);
  void open_grasp(void);
  void close_grasp(void);
  void select_pick_block(void); 
  void select_place_location(void);
  bool try_repeatedly_closer(geometry_msgs::Pose target, float n_times, float extent);
  bool try_repeatedly_above(geometry_msgs::Pose target, float n_times, float increment);

private:
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> tfListener;
  moveit::planning_interface::MoveGroupInterface move_group_interface;
  const moveit::core::JointModelGroup* joint_model_group;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ros::Subscriber emg_sub_, joy_sub_, block_id_sub_, grasp_status_sub_, place_location_sub_; //joy_sub_, 
  ros::Publisher block_selection_pub_, place_selection_pub_, marker_pub_, success_pub_, select_frozen_pub_; // grasp_status_pub_,
  bool joy_touch_;
	bool joy_ever_touched_;
	bool joy_touch_prev_; 
	bool joy_cont_touched_; //is the joystick being continuously operated?
  double delta_trig_;
  bool worked_first_time_ = false;
  Eigen::VectorXd joint_goal_prev_;
  int waypt;
  std::string subframe_name_;
  int trying_closer_limit_ = 5;
  int trying_closer_count_ = 0;
  bool actual_success_ = false;

  moveit::core::RobotStatePtr current_state_;
  std::vector<double> current_joint_values_;
  geometry_msgs::Pose target_pose_;

  float table_thickness = 0.03;
  float table_height = 1;
  float block_height = 0.05;
  float block_radius = 0.025;
  float partition_thickness = 0.005;
  float partition_height = 0.152;
  float wall_height = 0.085;
  float wall_thickness = partition_thickness;
  float table_width = 0.3 + 2*wall_thickness;
  float table_length = 0.6 + 2*wall_thickness;
  float side_wall_length = table_width;
  float front_wall_length = table_length - 2*wall_thickness;
  float clearance_ = block_height/2;

  float collision_padding_ = 0.005;

  actionlib::SimpleActionClient<mpl_control::graspAction> ac; 
	bool prev_grasp_button_state_;
	bool now_closed_;
  bool hand_full_;
  bool pick_selection_frozen_;
  bool place_selection_frozen_;

  bool prev_pick_select_button_state_;
  bool prev_place_select_button_state_;

  std::string block_id_selected_ = "redcylinder1"; //, block_id_selected__;
  std::string block_id_received_;
  geometry_msgs::Vector3 place_location_received_;
  geometry_msgs::Vector3 place_location_selected_;

  // pre-pick/place quaternion
  Eigen::Quaterniond pre_pick_quat_;
  Eigen::Quaterniond pre_place_quat_;
  // pre-pick/place position offset
  Eigen::Vector3d pre_pick_pos_offset_;
  Eigen::Vector3d pre_place_pos_offset_;

  std::string current_command_;
  std::string prev_command_;

  // list of all blocks
  std::vector<std::string> block_list_ = {"redcylinder1", "bluecylinder1", "redcylinder2", "bluecylinder2"};

};


//constructor
pickplace::pickplace(ros::NodeHandle &nh, std::string group_name) : 
            pnh_(nh), 
            move_group_interface(group_name), 
            joint_model_group((*move_group_interface.getCurrentState()).getJointModelGroup(group_name)), 
            joint_goal_prev_(Eigen::VectorXd::Zero(7)), 
            waypt(0), 
            delta_trig_(0.001), 
            joy_touch_prev_(false), 
            ac("grasp", true), 
            now_closed_(false), 
            hand_full_(false),
            pick_selection_frozen_(false),
            place_selection_frozen_(true)
{
  tfListener.reset(new tf2_ros::TransformListener(tfBuffer));

  // Print info messages
  ROS_INFO_NAMED("pickplace", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  ROS_INFO_NAMED("pickplace", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
  ROS_INFO_NAMED("pickplace", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));


	joy_sub_ = pnh_.subscribe("/joy", QUEUE_LENGTH, &pickplace::joyCallback, this);
	emg_sub_ = pnh_.subscribe("/emg", QUEUE_LENGTH, &pickplace::emgCallback, this);

  block_id_sub_ = pnh_.subscribe("/block_id", QUEUE_LENGTH, &pickplace::blockIDCallback, this);
  grasp_status_sub_ = pnh_.subscribe("/grasp_event_republisher/grasp_events", QUEUE_LENGTH, &pickplace::graspStatusCallback, this);
  place_location_sub_ = pnh_.subscribe("/gaze_table_hit_location", QUEUE_LENGTH, &pickplace::placeLocationCallback, this);
  // we will republish the grasp status as a boolean value
  // grasp_status_pub_ = pnh_.advertise<std_msgs::Bool>("/grasp_status", QUEUE_LENGTH);
  block_selection_pub_ = pnh_.advertise<std_msgs::String>("/current_block", QUEUE_LENGTH);
  place_selection_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>("/place_location_selected", QUEUE_LENGTH);
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  success_pub_ = pnh_.advertise<std_msgs::Bool>("/success", QUEUE_LENGTH);
  select_frozen_pub_ = pnh_.advertise<std_msgs::Bool>("/selection_frozen", QUEUE_LENGTH);

	prev_grasp_button_state_ = 0;
  prev_pick_select_button_state_ = 0;
  prev_place_select_button_state_ = 0;

  current_state_ = move_group_interface.getCurrentState();

  // pre_pick_quat_ = Eigen::Quaterniond(0.539746, 5.06562e-07, -4.9206e-06, 0.841828); // w, x, y, z
  pre_pick_quat_ = Eigen::Quaterniond(0.524, 0.127, 0.182, 0.822); // w, x, y, z
  pre_place_quat_ = pre_pick_quat_; // Eigen::Quaterniond(0.210, 0.076, 0.227, 0.948); // w, x, y, z

  pre_pick_pos_offset_ = Eigen::Vector3d(- 0.030, - 0.030, 0.130); //Eigen::Vector3d(- 0.030, - 0.030, 0.125);
  pre_place_pos_offset_ = Eigen::Vector3d(- 0.030, - 0.030, 0.170); //Eigen::Vector3d(- 0.030, - 0.030, 0.175);
  move_group_interface.setMaxVelocityScalingFactor(0.95);
  move_group_interface.setMaxAccelerationScalingFactor(0.95);

  // remove_all_blocks();

  mpl_control::graspGoal hand_action;
	hand_action.instruct = 2; // command open
  ac.waitForServer();
  // mpl_control::graspGoal goal;
  // goal.instruct = 1;
  // ac.sendGoal(goal); 
  ac.sendGoal(hand_action);

  move_group_interface.setPlanningTime(0.5);
  // Specifically request verbose logging
  // move_group_interface.setPlannerId("RRTConnectkConfigDefault");

}

//destructor
pickplace::~pickplace()
{
}

void pickplace::blockIDCallback(const std_msgs::String &msg)
{
  block_id_received_ = msg.data.c_str();
  std_msgs::String block_id_selected_msg;
  if (pick_selection_frozen_){
    block_id_selected_msg.data = block_id_selected_;
  }
  else{
    block_id_selected_msg.data = block_id_received_;
  }
  if (!hand_full_){
    block_selection_pub_.publish(block_id_selected_msg);
  }

}

void pickplace::set_pick_pose()
{
  move_group_interface.clearPoseTargets();
  move_group_interface.setEndEffectorLink("mpl_right_arm__wristz_link");
  
  bool tf_success = false;
  geometry_msgs::TransformStamped base_to_world_transform;
  // find the specified block

  while(tf_success == false)
  {
    try
    {
      base_to_world_transform = tfBuffer.lookupTransform("world", "mpl_right_arm__humerus_link",
                                                        ros::Time(0));  

      tf_success = true;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  tf_success = false;
  geometry_msgs::TransformStamped cylinder_to_world_transform;
  // find the specified block


  while(tf_success == false)
  {
    try
    {
      cylinder_to_world_transform = tfBuffer.lookupTransform("world", block_id_selected_,
                                                        ros::Time(0));

      tf_success = true;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  float l1 = 0.5; //0.29447;
  float l2 = 0.45; //0.23307;
  float x0 = -(cylinder_to_world_transform.transform.translation.y - base_to_world_transform.transform.translation.y);// - 0.121;
  float y0 = (cylinder_to_world_transform.transform.translation.x - base_to_world_transform.transform.translation.x);
  float z0 = -(cylinder_to_world_transform.transform.translation.z - base_to_world_transform.transform.translation.z);
  float l = std::sqrt(x0*x0 + y0*y0);
  float theta = std::atan2(y0, x0);
  float alpha = std::atan2(z0, l); // assumption: alpha = beta
  float beta = std::asin(0.202/0.500); //alpha;
  float gamma = std::asin((z0 - l1*sin(beta))/l2);
  float m1 = l1*cos(beta);
  float m2 = l2*cos(gamma);
  float cos_phi = (l*l + m2*m2 - m1*m1)/(2*l*m2);
  float phi = std::acos(cos_phi);
  float yaw_desired = phi + theta - M_PI/2; //M_PI/6 - M_PI/12;

  // get target orientation in rpy
  Eigen::Quaterniond target_quat(pre_pick_quat_.w(), pre_pick_quat_.x(), pre_pick_quat_.y(), pre_pick_quat_.z());
  Eigen::Vector3d target_rpy = target_quat.toRotationMatrix().eulerAngles(2, 1, 0);
  
  // reset yaw angle
  target_rpy[0] = yaw_desired;

  // convert back to quaternion
  Eigen::Quaterniond target_quat_new = Eigen::AngleAxisd(target_rpy[0], Eigen::Vector3d::UnitZ())
                                      * Eigen::AngleAxisd(target_rpy[1], Eigen::Vector3d::UnitY())
                                      * Eigen::AngleAxisd(target_rpy[2], Eigen::Vector3d::UnitX());


  target_pose_.orientation.x = target_quat_new.x();
  target_pose_.orientation.y = target_quat_new.y();
  target_pose_.orientation.z = target_quat_new.z();
  target_pose_.orientation.w = target_quat_new.w();
  float hand_l = 0.03; //std::sqrt(pre_pick_pos_offset_.x()*pre_pick_pos_offset_.x() + pre_pick_pos_offset_.y()*pre_pick_pos_offset_.y());

  target_pose_.position.x = cylinder_to_world_transform.transform.translation.x - hand_l*cos(yaw_desired-M_PI/2);
  target_pose_.position.y = cylinder_to_world_transform.transform.translation.y - hand_l*sin(yaw_desired-M_PI/2);
  target_pose_.position.z = cylinder_to_world_transform.transform.translation.z + pre_pick_pos_offset_.z();

  move_group_interface.setPoseTarget(target_pose_);

  remove_all_blocks();
  add_all_blocks();

  // broadcast a tf to show goal
  static tf2_ros::StaticTransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "pick_goal";
  transformStamped.transform.translation.x = target_pose_.position.x;
  transformStamped.transform.translation.y = target_pose_.position.y;
  transformStamped.transform.translation.z = target_pose_.position.z;
  transformStamped.transform.rotation.x = target_pose_.orientation.x;
  transformStamped.transform.rotation.y = target_pose_.orientation.y;
  transformStamped.transform.rotation.z = target_pose_.orientation.z;
  transformStamped.transform.rotation.w = target_pose_.orientation.w;

  br.sendTransform(transformStamped);

}

void pickplace::placeLocationCallback(const geometry_msgs::Vector3Stamped &msg)
{
  place_location_received_ = msg.vector;

}

void pickplace::graspStatusCallback(const gazebo_grasp_plugin_ros::GazeboGraspEventConstPtr &msg)
{
  if(msg->attached && !hand_full_) // toggle it once per grasp
  {
    hand_full_ = true; // just picked
    size_t firstDoubleColon = msg->object.find("::");
    if (firstDoubleColon != std::string::npos) {
        // Extract the substring before the first '::'
        std::string result = msg->object.substr(0, firstDoubleColon);
        attach_block(result); 
        // add all other blocks back
        for (int i = 0; i < block_list_.size(); i++){
          if (block_list_[i] != result){
            add_block(block_list_[i]);
          }
        }

        bool tf_success = false;

        geometry_msgs::TransformStamped picked_block_transform;

        while(tf_success == false)
        {
        try
        {
        picked_block_transform = tfBuffer.lookupTransform("world", result, //move_group_interface.getEndEffectorLink(), 
                                                      ros::Time(0));

        tf_success = true;
        }
        catch (tf2::TransformException &ex)
        {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        }
        }
        
        tf_success = false;
        geometry_msgs::TransformStamped wrist_transform;

        while(tf_success == false)
        {
        try
        {
        wrist_transform = tfBuffer.lookupTransform("world", "mpl_right_arm__wristz_link", //move_group_interface.getEndEffectorLink(), 
                                                      ros::Time(0));

        tf_success = true;
        }
        catch (tf2::TransformException &ex)
        {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        }
        }

        pre_place_pos_offset_.x() = wrist_transform.transform.translation.x - picked_block_transform.transform.translation.x;
        pre_place_pos_offset_.y() = wrist_transform.transform.translation.y - picked_block_transform.transform.translation.y;

    }

  // set_neutral_pose(); // instead of returning to a joint-space neutral pose, only go up from wherever currently and come a bit closer
  // target_pose_.position.z = partition_height + block_height/2 + pre_pick_pos_offset_.z() + clearance_;
  // target_pose_.position.x = -width/4;

  }
  else if (!(msg->attached) && hand_full_) // toggle it once per drop
  {
    hand_full_ = false; // just placed
    // re-add the one that was just dropped
    // Find the position of the first '::' because this is the gazebo name, which looks like redcylinder1::block_cylinder::block_cylinder_collision
    size_t firstDoubleColon = msg->object.find("::");
    if (firstDoubleColon != std::string::npos) {
        // Extract the substring before the first '::'
        std::string result = msg->object.substr(0, firstDoubleColon);
        //std::cout << result << std::endl;
        // add_block(result); // this is just to be safe, but redundant
        detach_block(result);
        remove_block(result);
    }

  // set_neutral_pose();
  // target_pose_.position.z = partition_height + block_height/2 + pre_pick_pos_offset_.z() + clearance_;
  // target_pose_.position.x = -width/4;
  open_grasp(); // since it's going to be necessary anyway

  }
}

void pickplace::open_grasp(void)
{
  mpl_control::graspGoal hand_action;
  hand_action.instruct = 2; // command open
  ac.sendGoal(hand_action);

  // wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(3.0));

	if (finished_before_timeout)
	{
	    actionlib::SimpleClientGoalState state = ac.getState();
	    ROS_INFO("Action finished: %s",state.toString().c_str());

      // if action successful, then now open
      now_closed_ = !(ac.getResult()->done);
      
	}
	else
	{
	    ROS_INFO("Action did not finish before the time out.");
	    ac.cancelGoal();
	}

  std_msgs::Bool grasp_command_unity;
  grasp_command_unity.data = false;
  success_pub_.publish(grasp_command_unity);
  
}

void pickplace::close_grasp(void)
{
  mpl_control::graspGoal hand_action;
  hand_action.instruct = 1; // command close
  ac.sendGoal(hand_action);

  // wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(3.0));

	if (finished_before_timeout)
	{
	    actionlib::SimpleClientGoalState state = ac.getState();
	    ROS_INFO("Action finished: %s",state.toString().c_str());

      // if action successful, then now closed
      now_closed_ = ac.getResult()->done;
	}
	else
	{
	    ROS_INFO("Action did not finish before the time out.");
	    ac.cancelGoal();
	}

  std_msgs::Bool grasp_command_unity;
  grasp_command_unity.data = true;
  success_pub_.publish(grasp_command_unity);
  
}

void pickplace::add_table(void) 
{
  moveit_msgs::CollisionObject table_object;
  table_object.header.frame_id = "world"; //move_group_interface.getPoseReferenceFrame();
  // The id of the object is used to identify it.
  table_object.id = "table";

  bool tf_success = false;
  
  geometry_msgs::TransformStamped table_transform;
  
  while(tf_success == false)
  {
    try
    {
      table_transform = tfBuffer.lookupTransform("world", "table",
                                                        ros::Time(0));

      tf_success = true;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  shape_msgs::SolidPrimitive table_primitive;
  table_primitive.type = table_primitive.BOX;
  table_primitive.dimensions.resize(3);
  table_primitive.dimensions[table_primitive.BOX_X] = table_length - 2*collision_padding_;
  table_primitive.dimensions[table_primitive.BOX_Y] = table_width - 2*collision_padding_;
  table_primitive.dimensions[table_primitive.BOX_Z] = table_thickness; // + collision_padding_; // let the gap from table surface be half of the collision padding

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose table_pose;
  
  table_pose.orientation.x = table_transform.transform.rotation.x;
  table_pose.orientation.y = table_transform.transform.rotation.y;
  table_pose.orientation.z = table_transform.transform.rotation.z;
  table_pose.orientation.w = table_transform.transform.rotation.w;

  table_pose.position.x = table_transform.transform.translation.x;
  table_pose.position.y = table_transform.transform.translation.y;
  table_pose.position.z = table_transform.transform.translation.z + table_height + table_thickness/2;

  table_object.primitives.push_back(table_primitive);
  table_object.primitive_poses.push_back(table_pose);

  shape_msgs::SolidPrimitive partition_primitive;
  geometry_msgs::Pose partition_pose;
  partition_primitive.type = partition_primitive.BOX;
  partition_primitive.dimensions.resize(3);
  partition_primitive.dimensions[partition_primitive.BOX_X] = partition_thickness + 2*collision_padding_;
  partition_primitive.dimensions[partition_primitive.BOX_Y] = table_width + 2*collision_padding_;
  partition_primitive.dimensions[partition_primitive.BOX_Z] = partition_height + 2*collision_padding_;

  partition_pose.orientation.x = table_transform.transform.rotation.x;
  partition_pose.orientation.y = table_transform.transform.rotation.y;
  partition_pose.orientation.z = table_transform.transform.rotation.z;
  partition_pose.orientation.w = table_transform.transform.rotation.w;

  partition_pose.position.x = table_transform.transform.translation.x;
  partition_pose.position.y = table_transform.transform.translation.y;
  partition_pose.position.z = table_transform.transform.translation.z + table_height + partition_height/2 + table_thickness/2;

  table_object.primitives.push_back(partition_primitive);
  table_object.primitive_poses.push_back(partition_pose);


  //

  shape_msgs::SolidPrimitive left_side_wall_primitive;
  shape_msgs::SolidPrimitive right_side_wall_primitive;
  geometry_msgs::Pose left_side_wall_pose;
  geometry_msgs::Pose right_side_wall_pose;
  left_side_wall_primitive.type = left_side_wall_primitive.BOX;
  left_side_wall_primitive.dimensions.resize(3);
  left_side_wall_primitive.dimensions[left_side_wall_primitive.BOX_X] = wall_thickness + 2*collision_padding_;
  left_side_wall_primitive.dimensions[left_side_wall_primitive.BOX_Y] = side_wall_length + 2*collision_padding_;
  left_side_wall_primitive.dimensions[left_side_wall_primitive.BOX_Z] = wall_height + 2*collision_padding_;
  right_side_wall_primitive.type = right_side_wall_primitive.BOX;
  right_side_wall_primitive.dimensions.resize(3);
  right_side_wall_primitive.dimensions[right_side_wall_primitive.BOX_X] = wall_thickness + 2*collision_padding_;
  right_side_wall_primitive.dimensions[right_side_wall_primitive.BOX_Y] = side_wall_length + 2*collision_padding_;
  right_side_wall_primitive.dimensions[right_side_wall_primitive.BOX_Z] = wall_height + 2*collision_padding_;

  left_side_wall_pose.orientation.x = table_transform.transform.rotation.x;
  left_side_wall_pose.orientation.y = table_transform.transform.rotation.y;
  left_side_wall_pose.orientation.z = table_transform.transform.rotation.z;
  left_side_wall_pose.orientation.w = table_transform.transform.rotation.w;
  right_side_wall_pose.orientation.x = table_transform.transform.rotation.x;
  right_side_wall_pose.orientation.y = table_transform.transform.rotation.y;
  right_side_wall_pose.orientation.z = table_transform.transform.rotation.z;
  right_side_wall_pose.orientation.w = table_transform.transform.rotation.w;

  left_side_wall_pose.position.x = table_transform.transform.translation.y;
  left_side_wall_pose.position.y = table_transform.transform.translation.x + table_length/2 - wall_thickness/2;
  left_side_wall_pose.position.z = table_transform.transform.translation.z + table_height + wall_height/2 + table_thickness;
  right_side_wall_pose.position.x = table_transform.transform.translation.y;
  right_side_wall_pose.position.y = table_transform.transform.translation.x-table_length/2 + wall_thickness/2;
  right_side_wall_pose.position.z = table_transform.transform.translation.z + table_height + wall_height/2 + table_thickness;

  table_object.primitives.push_back(left_side_wall_primitive);
  table_object.primitive_poses.push_back(left_side_wall_pose);

  table_object.primitives.push_back(right_side_wall_primitive);
  table_object.primitive_poses.push_back(right_side_wall_pose);

  //

  shape_msgs::SolidPrimitive front_wall_primitive;
  shape_msgs::SolidPrimitive back_wall_primitive;
  geometry_msgs::Pose front_wall_pose;
  geometry_msgs::Pose back_wall_pose;
  front_wall_primitive.type = front_wall_primitive.BOX;
  front_wall_primitive.dimensions.resize(3);
  front_wall_primitive.dimensions[front_wall_primitive.BOX_X] = front_wall_length - 2*collision_padding_;
  front_wall_primitive.dimensions[front_wall_primitive.BOX_Y] = wall_thickness + 2*collision_padding_;
  front_wall_primitive.dimensions[front_wall_primitive.BOX_Z] = wall_height + 2*collision_padding_;

  back_wall_primitive.type = back_wall_primitive.BOX;
  back_wall_primitive.dimensions.resize(3);
  back_wall_primitive.dimensions[back_wall_primitive.BOX_X] = front_wall_length - 2*collision_padding_;
  back_wall_primitive.dimensions[back_wall_primitive.BOX_Y] = wall_thickness + 2*collision_padding_;
  back_wall_primitive.dimensions[back_wall_primitive.BOX_Z] = wall_height + 2*collision_padding_;

  front_wall_pose.orientation.x = table_transform.transform.rotation.x;
  front_wall_pose.orientation.y = table_transform.transform.rotation.y;
  front_wall_pose.orientation.z = table_transform.transform.rotation.z;
  front_wall_pose.orientation.w = table_transform.transform.rotation.w;
  back_wall_pose.orientation.x = table_transform.transform.rotation.x;
  back_wall_pose.orientation.y = table_transform.transform.rotation.y;
  back_wall_pose.orientation.z = table_transform.transform.rotation.z;
  back_wall_pose.orientation.w = table_transform.transform.rotation.w;

  front_wall_pose.position.x = table_transform.transform.translation.y - table_width/2 - wall_thickness/2;
  front_wall_pose.position.y = table_transform.transform.translation.x;
  front_wall_pose.position.z = table_transform.transform.translation.z + table_height + wall_height/2 + table_thickness;
  back_wall_pose.position.x = table_transform.transform.translation.y + table_width/2 + wall_thickness/2;
  back_wall_pose.position.y = table_transform.transform.translation.x;
  back_wall_pose.position.z = table_transform.transform.translation.z + table_height + wall_height/2 + table_thickness;

  table_object.primitives.push_back(front_wall_primitive);
  table_object.primitive_poses.push_back(front_wall_pose);

  table_object.primitives.push_back(back_wall_primitive);
  table_object.primitive_poses.push_back(back_wall_pose);

  //

  table_object.operation = table_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(table_object);

  // Now, let's add the collision object into the world
  // (using a vector that could contain additional objects)
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);
  // planning_scene_interface.applyCollisionObjects(collision_objects);

  std::map< std::string,moveit_msgs::CollisionObject > objects =  planning_scene_interface.getObjects();
  for (std::map< std::string,moveit_msgs::CollisionObject >::iterator it=objects.begin(); it!=objects.end(); ++it)
  {
    ROS_INFO("Detected object: %s", it->first.c_str());
  }



}


void pickplace::add_block(std::string block_name)
{
  moveit_msgs::CollisionObject block_object;
  block_object.header.frame_id = "world"; //move_group_interface.getPoseReferenceFrame();
  // The id of the object is used to identify it.
  block_object.id = block_name;

  bool tf_success = false;
  
  geometry_msgs::TransformStamped block_transform;
  
  while(tf_success == false)
  {
    try
    {
      block_transform = tfBuffer.lookupTransform("world", block_name,
                                                        ros::Time(0));

      tf_success = true;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  shape_msgs::SolidPrimitive block_primitive;
  block_primitive.type = block_primitive.CYLINDER;
  block_primitive.dimensions.resize(3);
  block_primitive.dimensions[block_primitive.CYLINDER_HEIGHT] = block_height + 2*collision_padding_;
  block_primitive.dimensions[block_primitive.CYLINDER_RADIUS] = block_radius + collision_padding_;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose block_pose;
  
  block_pose.orientation.x = block_transform.transform.rotation.x;
  block_pose.orientation.y = block_transform.transform.rotation.y;
  block_pose.orientation.z = block_transform.transform.rotation.z;
  block_pose.orientation.w = block_transform.transform.rotation.w;

  block_pose.position.x = block_transform.transform.translation.x;
  block_pose.position.y = block_transform.transform.translation.y;
  block_pose.position.z = block_transform.transform.translation.z;

  block_object.primitives.push_back(block_primitive);
  block_object.primitive_poses.push_back(block_pose);

  block_object.operation = block_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(block_object);

  // Now, let's add the collision object into the world
  // (using a vector that could contain additional objects)
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);
  // planning_scene_interface.applyCollisionObjects(collision_objects);

}

void pickplace::attach_block(std::string block_name)
{
  moveit_msgs::CollisionObject block_object;
  block_object.header.frame_id = "mpl_right_arm__wristz_link"; //move_group_interface.getEndEffectorLink();
  // The id of the object is used to identify it.
  block_object.id = block_name;

  bool tf_success = false;
  
  geometry_msgs::TransformStamped block_transform;
  
  while(tf_success == false)
  {
    try
    {
      block_transform = tfBuffer.lookupTransform("mpl_right_arm__wristz_link", block_name,
                                                        ros::Time(0));

      tf_success = true;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  shape_msgs::SolidPrimitive block_primitive;
  block_primitive.type = block_primitive.CYLINDER;
  block_primitive.dimensions.resize(3);
  block_primitive.dimensions[block_primitive.CYLINDER_HEIGHT] = block_height;
  block_primitive.dimensions[block_primitive.CYLINDER_RADIUS] = block_radius;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose block_pose;
  
  block_pose.orientation.x = block_transform.transform.rotation.x;
  block_pose.orientation.y = block_transform.transform.rotation.y;
  block_pose.orientation.z = block_transform.transform.rotation.z;
  block_pose.orientation.w = block_transform.transform.rotation.w;

  block_pose.position.x = block_transform.transform.translation.x;
  block_pose.position.y = block_transform.transform.translation.y;
  block_pose.position.z = block_transform.transform.translation.z;

  block_object.primitives.push_back(block_primitive);
  block_object.primitive_poses.push_back(block_pose);

   // define a subframe positioned at the origin of the object with orientation same as wrist
  block_object.subframe_names.resize(3);
  block_object.subframe_poses.resize(3);

  block_object.subframe_names[0] = "placing_frame";
  // everything is described in the block frame
  block_object.subframe_poses[0].position.x = 0.0; //block_transform.transform.translation.x;
  block_object.subframe_poses[0].position.y = 0.0; //block_transform.transform.translation.y;
  block_object.subframe_poses[0].position.z = 0.0; //block_transform.transform.translation.z;
  // subframe has same position as object

  tf2::Quaternion orientation;
  // subframe has same orientation as wrist link, so invert the quaternion 
  block_object.subframe_poses[0].orientation.x = block_transform.transform.rotation.x;
  block_object.subframe_poses[0].orientation.y = block_transform.transform.rotation.y;
  block_object.subframe_poses[0].orientation.z = block_transform.transform.rotation.z;
  block_object.subframe_poses[0].orientation.w = -block_transform.transform.rotation.w;

  block_object.operation = block_object.ADD;

  planning_scene_interface.applyCollisionObject(block_object);

  move_group_interface.attachObject(block_object.id, "mpl_right_arm__wristz_link", { "mpl_right_arm__index3_link", "mpl_right_arm__middle3_link", "mpl_right_arm__ring3_link", "mpl_right_arm__pinky3_link", "mpl_right_arm__thumb3_link" });

  // make this the end-effector frame now
  // first concatenate the subframe name to the object name 
  subframe_name_ = block_object.id + "/" + block_object.subframe_names[0];
  // then set the end-effector link to this subframe
  move_group_interface.clearPoseTargets();
  move_group_interface.setEndEffectorLink(subframe_name_); // redundant; this will be set in set_place_pose

  // broadcast a tf to show subframe
  static tf2_ros::StaticTransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = block_name;
  transformStamped.child_frame_id = subframe_name_;
  transformStamped.transform.translation.x = block_object.subframe_poses[0].position.x;
  transformStamped.transform.translation.y = block_object.subframe_poses[0].position.y;
  transformStamped.transform.translation.z = block_object.subframe_poses[0].position.z;
  transformStamped.transform.rotation.x = block_object.subframe_poses[0].orientation.x;
  transformStamped.transform.rotation.y = block_object.subframe_poses[0].orientation.y;
  transformStamped.transform.rotation.z = block_object.subframe_poses[0].orientation.z;
  transformStamped.transform.rotation.w = block_object.subframe_poses[0].orientation.w;

  br.sendTransform(transformStamped);

}

void pickplace::detach_block(std::string block_name)
{
  move_group_interface.detachObject(block_name);
  move_group_interface.clearPoseTargets();
  move_group_interface.setEndEffectorLink("mpl_right_arm__wristz_link"); 
  // above is redundant; this will be set in set_pick_pose and set_neutral_pose
}


void pickplace::add_all_blocks(void){
  add_block("redcylinder1");
  add_block("redcylinder2");
  add_block("bluecylinder1");
  add_block("bluecylinder2");
  // add_table(); 
}

void pickplace::remove_all_blocks(void){
  remove_block("redcylinder1");
  remove_block("redcylinder2");
  remove_block("bluecylinder1");
  remove_block("bluecylinder2");
  // remove_block("table");
}

void pickplace::remove_block(std::string block_name)
{
  std::vector<std::string> collision_objects;
  collision_objects.push_back(block_name);
  // ROS_INFO_NAMED("tutorial", "Remove an object from the world");
  planning_scene_interface.removeCollisionObjects(collision_objects);
}


bool pickplace::go_to_pose(geometry_msgs::Pose target_pose)
{
  // broadcast a tf to show goal
  static tf2_ros::StaticTransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "trying_goal";
  transformStamped.transform.translation.x = target_pose.position.x;
  transformStamped.transform.translation.y = target_pose.position.y;
  transformStamped.transform.translation.z = target_pose.position.z;
  transformStamped.transform.rotation.x = target_pose.orientation.x;
  transformStamped.transform.rotation.y = target_pose.orientation.y;
  transformStamped.transform.rotation.z = target_pose.orientation.z;
  transformStamped.transform.rotation.w = target_pose.orientation.w;

  br.sendTransform(transformStamped);
  // if hand is full, set place pose just before calling this, setting EE to object subframe and orientation tol to default
  // if hand is empty, set pick pose just before calling this, setting EE to wrist and orientation tol to default
  move_group_interface.setPoseTarget(target_pose); 
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  std_msgs::Bool success_msg;
  success_msg.data = success;
  // success_pub_.publish(success_msg);
  // if (!success){
  //   set_neutral_pose(); // changes EE to wrist and increases orientation tol
  //   bool neutral_pose_success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  //   move_group_interface.setGoalOrientationTolerance(0.0001);
  // }
  if(success){
    move_group_interface.asyncExecute(my_plan);
  }
  return success;

}

void pickplace::set_place_pose(void)
{
  move_group_interface.clearPoseTargets();
  move_group_interface.setEndEffectorLink(subframe_name_);

bool tf_success = false;
  geometry_msgs::TransformStamped base_to_world_transform;
  // find the specified block

  while(tf_success == false)
  {
    try
    {
      base_to_world_transform = tfBuffer.lookupTransform("world", "mpl_right_arm__humerus_link",
                                                        ros::Time(0));  

      tf_success = true;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  bool out_of_reach = false;
  float l1 = 0.5; //0.29447;
  float l2 = 0.45; //0.23307;
  float x0 = -(place_location_selected_.y - base_to_world_transform.transform.translation.y);// - 0.121; // the base is offset from the frame origin
  float y0 = (place_location_selected_.x - base_to_world_transform.transform.translation.x);
  float z0 = -(place_location_selected_.z - base_to_world_transform.transform.translation.z);
  float l = std::sqrt(x0*x0 + y0*y0);
  float theta = std::atan2(y0, x0);
  // float phi = 0;
  float alpha = std::atan2(z0, l); // assumption: beta found visually from rviz
  float beta = std::asin(0.202/0.500); // elbow was 20.2 cm below shoulder height, 35.8 cm above the table in this picture
  float gamma = std::asin((z0 - l1*sin(beta))/l2);
  float m1 = l1*cos(beta);
  float m2 = l2*cos(gamma);
  float cos_phi = (l*l + m2*m2 - m1*m1)/(2*l*m2);
  float phi = std::acos(cos_phi);
  float yaw_desired = phi + theta - M_PI/2; // - M_PI/6 - M_PI/12;

  // get target orientation in rpy
  Eigen::Quaterniond target_quat(pre_place_quat_.w(), pre_place_quat_.x(), pre_place_quat_.y(), pre_place_quat_.z());
  Eigen::Vector3d target_rpy = target_quat.toRotationMatrix().eulerAngles(2, 1, 0);
  
  // reset yaw angle
  target_rpy[0] = yaw_desired;

  // convert back to quaternion
  Eigen::Quaterniond target_quat_new = Eigen::AngleAxisd(target_rpy[0], Eigen::Vector3d::UnitZ())
                                      * Eigen::AngleAxisd(target_rpy[1], Eigen::Vector3d::UnitY())
                                      * Eigen::AngleAxisd(target_rpy[2], Eigen::Vector3d::UnitX());


  target_pose_.orientation.x = target_quat_new.x();
  target_pose_.orientation.y = target_quat_new.y();
  target_pose_.orientation.z = target_quat_new.z();
  target_pose_.orientation.w = target_quat_new.w();
  float hand_l = std::sqrt(pre_place_pos_offset_.x()*pre_place_pos_offset_.x() + pre_place_pos_offset_.y()*pre_place_pos_offset_.y());
  // we are planning for the subframe on the object 
  target_pose_.position.x = place_location_selected_.x; // - hand_l*cos(yaw_desired - M_PI/2);
  target_pose_.position.y = place_location_selected_.y; // - hand_l*sin(yaw_desired - M_PI/2);
  target_pose_.position.z = place_location_selected_.z + pre_place_pos_offset_.z(); // + block_height/2;

  // if (out_of_reach){
  //   // float gap = std::sqrt(x0*x0 + y0*y0 + z0*z0);
  //   float l_reduced = l2;
  //   //std::cout << "l_reduced: " << l_reduced << std::endl;
  //   target_pose_.position.x = base_to_world_transform.transform.translation.x + l_reduced*sin(theta);
  //   target_pose_.position.y = base_to_world_transform.transform.translation.y -l_reduced*cos(theta);
  //   target_pose_.position.z = place_location_.z + partition_height; // just hover above
  // }

  move_group_interface.setStartStateToCurrentState();
  move_group_interface.setPoseTarget(target_pose_);

  // broadcast the target pose
  static tf2_ros::StaticTransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "target_pose";
  transformStamped.transform.translation.x = target_pose_.position.x;
  transformStamped.transform.translation.y = target_pose_.position.y;
  transformStamped.transform.translation.z = target_pose_.position.z;
  transformStamped.transform.rotation.x = target_pose_.orientation.x;
  transformStamped.transform.rotation.y = target_pose_.orientation.y;
  transformStamped.transform.rotation.z = target_pose_.orientation.z;
  transformStamped.transform.rotation.w = target_pose_.orientation.w;

  br.sendTransform(transformStamped);
  

}

void pickplace::select_pick_block(void){
  std_msgs::Bool frozen_msg;
  if (pick_selection_frozen_){ // unfreeze
    pick_selection_frozen_ = false;
    frozen_msg.data = false;
  }
  else{ // freeze
    block_id_selected_ = block_id_received_;
    pick_selection_frozen_ = true;
    frozen_msg.data = true;
  }
  select_frozen_pub_.publish(frozen_msg);
}

void pickplace::select_place_location(void){
  std_msgs::Bool frozen_msg;
  if (place_selection_frozen_){
    place_selection_frozen_ = false;
    frozen_msg.data = false;
  }
  else{
  place_location_selected_ = place_location_received_;
  place_selection_frozen_ = true;
  frozen_msg.data = true;
  }
  select_frozen_pub_.publish(frozen_msg);
}

geometry_msgs::Pose pickplace::try_closer(geometry_msgs::Pose target_pose, float extent)
{
  geometry_msgs::Pose local_target = target_pose_;

  bool tf_success = false;
  
  geometry_msgs::TransformStamped wrist_to_base_transform;
  
  while(tf_success == false)
  {
    try
    {
      wrist_to_base_transform = tfBuffer.lookupTransform(move_group_interface.getPlanningFrame(), "mpl_right_arm__wristz_link",
                                                        ros::Time(0));

      tf_success = true;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  local_target.position.x = wrist_to_base_transform.transform.translation.x + extent*(target_pose.position.x - wrist_to_base_transform.transform.translation.x);
  local_target.position.y = wrist_to_base_transform.transform.translation.y + extent*(target_pose.position.y - wrist_to_base_transform.transform.translation.y);
  local_target.position.z = wrist_to_base_transform.transform.translation.z + extent*(target_pose.position.z - wrist_to_base_transform.transform.translation.z);
  // do slerp later
  // closer_target.orientation.x = target_pose_.orientation.x;
  // closer_target.orientation.y = target_pose_.orientation.y;
  // closer_target.orientation.z = target_pose_.orientation.z;
  // closer_target.orientation.w = target_pose_.orientation.w;

  // move_group_interface.setStartStateToCurrentState();
  // move_group_interface.setPoseTarget(closer_target);

  // trying_closer_count_++;
  return local_target;

}

geometry_msgs::Pose pickplace::try_above(geometry_msgs::Pose target, float margin)
{
  geometry_msgs::Pose local_target = target;
  local_target.position.z += margin; 

  return local_target;

}

bool pickplace::try_repeatedly_closer(geometry_msgs::Pose target, float n_times, float extent)
{
    geometry_msgs::Pose local_target = target;
    bool local_success = false;
    int trying_count = 0; // reset for this touch of the joystick
    move_group_interface.setGoalOrientationTolerance(0.1);  
    while(trying_count < n_times){ // try only a certain number of times before giving up 
      local_target = try_closer(local_target, extent); // recursively bring the goal closer to the wrist
      local_success = go_to_pose(local_target); // try going there
      if(local_success){
        break; // if successful, stop trying
      }
      trying_count++;
    }
    move_group_interface.setGoalOrientationTolerance(0.0001); // return orientation tol to default regardless of success or failure
    return local_success;
}

bool pickplace::try_repeatedly_above(geometry_msgs::Pose target, float n_times, float increment)
{
    geometry_msgs::Pose local_target = target;
    bool local_success = false;
    int trying_count = 0; // reset for this touch of the joystick
    float margin = 0;
    move_group_interface.setGoalOrientationTolerance(0.1); 
    while(trying_count < n_times){ // try only a certain number of times before giving up 
      margin += increment;
      local_target = try_above(local_target, margin); // recursively increase the height of the goal 
      local_success = go_to_pose(local_target);   // try going there
      if(local_success){
        break; // if successful, stop trying
      }
      trying_count++;
    }
    move_group_interface.setGoalOrientationTolerance(0.0001); // return orientation tol to default regardless of success or failure
    return local_success;
}

bool pickplace::fresh_attempt(void){
  // std::cout << "fresh_attempt" << std::endl;
  bool local_success = false;
        // set a course for either pick or place, depending on the state (holding block/not holding block)
      // if (hand_full_)
      // {
      //   set_place_pose();
      //           //std::cout << "Setting place pose" << std::endl;

      // } 
      // else
      // {
      //   set_pick_pose();
      //   //std::cout << "Setting pick pose" << std::endl;
      // }    
    // now the target pose is set
    // }
    if(!(actual_success_ = go_to_pose(target_pose_))){ // tried once planning to actual goal and failed
      if(!(local_success = try_repeatedly_closer(target_pose_, 2, 0.75))){ // tried few times and couldn't plan along the line
        if(!(local_success =try_repeatedly_above(target_pose_, 5, 0.015))){ // tried few times and couldn't plan above
          // if(!(local_success = try_repeatedly_closer(try_above(target_pose_, 0.1), 3, 0.5))); // try planning to point far above, try a few times
            // set_neutral_pose(); // if all else fails, return to neutral posefres
            // std::cout << "fresh_attempt failed" << std::endl;
        // float margin = 0;
        // while(margin < 0.1){
        //   margin += 0.015;
        //   if(try_repeatedly_closer(try_above(target_pose_, margin), 5)); // try planning to point above, try a few times 
        //   // if successful, async execute will happen so exit
        //   // if not, this loop will run again with higher margin -- until margin reaches 10 cm which is twice the block height
        //   {
        //     // return orientation tol to default
        //     move_group_interface.setGoalOrientationTolerance(0.0001);
        //     return;
        //   }
        // }
      }
    }
  }
  return local_success;
}

void pickplace::joyCallback(const sensor_msgs::Joy &msg)
{

  int grasp_button = 4;
  int pick_select_button = 10;
  int place_select_button = 11;

  // for Unity 
  // std_msgs::Bool hand_full_msg;
  // hand_full_msg.data = hand_full_;

  //std::cout << "joyCallback" << std::endl;

  double joy_input = msg.axes[1];

  if (std::abs(joy_input)>=delta_trig_){
    joy_touch_ = true;
    joy_ever_touched_ = true;
    if (joy_touch_prev_){
      joy_cont_touched_ = true;
    } else{
      joy_cont_touched_ = false;
    }
  }
  else{
    joy_touch_ = false;
  }
  joy_touch_prev_ = joy_touch_;

  // GRASP ACTION
	if(msg.buttons[grasp_button] && !prev_grasp_button_state_){
	    prev_grasp_button_state_ = msg.buttons[grasp_button];		
		if (now_closed_){
			open_grasp();
			now_closed_ = false;
		}
		else{
      remove_block(block_id_selected_);
			close_grasp();
			now_closed_ = true;
		}
	} 
	// register button release
	if(!msg.buttons[grasp_button] && prev_grasp_button_state_){
	    prev_grasp_button_state_ = msg.buttons[grasp_button];
	} 
	// wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	{
	    actionlib::SimpleClientGoalState state = ac.getState();
	    ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
	{
	    ROS_INFO("Action did not finish before the time out.");
	    ac.cancelGoal();
	}

  // select currently highlighted block for pick
  if(msg.buttons[pick_select_button] && !prev_pick_select_button_state_){

    select_pick_block();
    prev_pick_select_button_state_ = msg.buttons[pick_select_button];

	} 
	// register button release
	if(!msg.buttons[pick_select_button] && prev_pick_select_button_state_){
	    prev_pick_select_button_state_ = msg.buttons[pick_select_button];
	} 

  // select place location
  if(msg.buttons[place_select_button] && !prev_place_select_button_state_){

    select_place_location();
    prev_place_select_button_state_ = msg.buttons[place_select_button];

	} 
	// register button release
	if(!msg.buttons[place_select_button] && prev_place_select_button_state_){
	    prev_place_select_button_state_ = msg.buttons[place_select_button];
	} 

  geometry_msgs::TransformStamped world_to_holoworld_transform_;
  try{
    world_to_holoworld_transform_ = tfBuffer.lookupTransform("hololens_world_test", "world",
                              ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }


  // convert to holoworld frame
  // get rotation from world to holoworld from world_to_holo_transform_
  Eigen::Quaterniond world_to_holoworld_quat(world_to_holoworld_transform_.transform.rotation.w, world_to_holoworld_transform_.transform.rotation.x, world_to_holoworld_transform_.transform.rotation.y, world_to_holoworld_transform_.transform.rotation.z);
  Eigen::Matrix3d world_to_holoworld_rot = world_to_holoworld_quat.toRotationMatrix();

  Eigen::Vector3d place_location_holoworld;
  // convert this to switch case
  if (place_selection_frozen_){
    place_location_holoworld = world_to_holoworld_rot*Eigen::Vector3d(place_location_selected_.x, place_location_selected_.y, place_location_selected_.z) + Eigen::Vector3d(world_to_holoworld_transform_.transform.translation.x, world_to_holoworld_transform_.transform.translation.y, world_to_holoworld_transform_.transform.translation.z);
  }
  else {
    place_location_holoworld = world_to_holoworld_rot*Eigen::Vector3d(place_location_received_.x, place_location_received_.y, place_location_received_.z) + Eigen::Vector3d(world_to_holoworld_transform_.transform.translation.x, world_to_holoworld_transform_.transform.translation.y, world_to_holoworld_transform_.transform.translation.z);
  }

  geometry_msgs::PoseStamped place_location_pose;
  place_location_pose.header.stamp = ros::Time::now();
  place_location_pose.header.frame_id = "ROSworld";

  place_location_pose.pose.position.x = place_location_holoworld.x();
  place_location_pose.pose.position.y = place_location_holoworld.y();
  place_location_pose.pose.position.z = place_location_holoworld.z();
  place_location_pose.pose.orientation.x = 0;
  place_location_pose.pose.orientation.y = 0;
  place_location_pose.pose.orientation.z = 0;
  place_location_pose.pose.orientation.w = 1;
  place_selection_pub_.publish(place_location_pose);

  bool any_plan_at_all = false;

  if (joy_touch_ && !joy_cont_touched_){ //just touched freshly
    //std::cout << "Freshly touched" << std::endl;
    if (hand_full_)
    {
      set_place_pose();
              //std::cout << "Setting place pose" << std::endl;

    } 
    else
    {
      set_pick_pose();
      //std::cout << "Setting pick pose" << std::endl;
    }    
    if(!(actual_success_ = go_to_pose(target_pose_))){
      any_plan_at_all = fresh_attempt();
    }
  }
  // if joystick is being continuously operated then don't really have to say any more for now, later we'll scale velocity
  else if (!joy_touch_){ //it doesn't catch} && joy_touch_prev_){ // it was just released (we only want to send stop ONCE! it shouldn't block future button presses etc)
    //std::cout << "Stop moving!" << std::endl;
    //stop moving!
    move_group_interface.stop();    
  }
  else if (joy_touch_ && joy_cont_touched_){
    // if(!actual_success_ and !any_plan_at_all){ // if not yet succeeded but not currently executing sub-plan
      // fresh_attempt(); // fresh attempt contains functions that will not execute anything if unable to plan
    // }
    //std::cout << "You should be executing the same plan as before" << std::endl;
    // scale velocity
    // move_group_interface.setMaxVelocityScalingFactor(0.95);
    // move_group_interface.setMaxAccelerationScalingFactor(0.95);

    // move_group_interface.execute(my_plan_);
    // if(!worked_first_time_){
    //   set_neutral_pose();
    // }
  }

}

void pickplace::emgCallback(const std_msgs::String &msg)
{
  /* this is different from joystick because
   inputs are classes, not floats

   Mapping:
    1. no_motion = nothing
    2. hand_close = grasp
    3. hand_open = release
    4. wrist_flexion or wrist_extension = move
    5. hand_open immediately followed by hand_close = select (pick if hand empty, place if full)
  */

  current_command_ = msg.data;
  
  // handle spurious signals
  // if(prev_command_ == "hand_close" && current_command_ == "hand_open"){
  //   current_command_ = "no_motion";
  // }

  if(current_command_!=prev_command_){ // only if something changed
    // input might come multiple times in a row
    // and it shouldn't keep toggling

    // std::cout << "Received command: " << current_command_ << std::endl;
    // std::cout << "Previous command: " << prev_command_ << std::endl;
    // std::cout << "now_closed_: " << now_closed_ << std::endl;

    if(current_command_ == "hand_close" && !now_closed_){
      // remove_block(block_id_selected_);
      remove_all_blocks();
      remove_block("table");
      close_grasp();
      add_table();
      // now_closed_ = true;
      // std::cout << "Closed grasp" << std::endl;
    }

    
    if(current_command_ == "hand_open" && now_closed_){
      remove_all_blocks();
      remove_block("table");
      open_grasp();
      add_table();
      // now_closed_ = false;
      // std::cout << "Opened grasp" << std::endl;
    }

    if(current_command_ == "wrist_extension"){
      if (hand_full_)
      {
        set_place_pose();
      } 
      else
      {
        set_pick_pose();
      }    
      // if(!(actual_success_ = go_to_pose(target_pose_))){
      fresh_attempt();
      // }
      
    }

    if(current_command_ == "no_motion"){
      move_group_interface.stop();
    }

    if(current_command_ == "wrist_flexion"){ // this doesn't really happen by accident much
      if(!hand_full_){
        select_pick_block();
        // std::cout << "Selected block for pick" << block_id_selected_ << std::endl;
      }
      else{
        select_place_location();
        // std::cout << "Selected place location" << place_location_selected_ << std::endl;
      }
    }
  }

  prev_command_ = current_command_; // update the previous command

  // show place location live
  geometry_msgs::TransformStamped world_to_holoworld_transform_;
  try{
    world_to_holoworld_transform_ = tfBuffer.lookupTransform("hololens_world_test", "world",
                              ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }


  // convert to holoworld frame
  // get rotation from world to holoworld from world_to_holo_transform_
  Eigen::Quaterniond world_to_holoworld_quat(world_to_holoworld_transform_.transform.rotation.w, world_to_holoworld_transform_.transform.rotation.x, world_to_holoworld_transform_.transform.rotation.y, world_to_holoworld_transform_.transform.rotation.z);
  Eigen::Matrix3d world_to_holoworld_rot = world_to_holoworld_quat.toRotationMatrix();

  Eigen::Vector3d place_location_holoworld;
  // convert this to switch case
  if (place_selection_frozen_){
    place_location_holoworld = world_to_holoworld_rot*Eigen::Vector3d(place_location_selected_.x, place_location_selected_.y, place_location_selected_.z) + Eigen::Vector3d(world_to_holoworld_transform_.transform.translation.x, world_to_holoworld_transform_.transform.translation.y, world_to_holoworld_transform_.transform.translation.z);
  }
  else {
    place_location_holoworld = world_to_holoworld_rot*Eigen::Vector3d(place_location_received_.x, place_location_received_.y, place_location_received_.z) + Eigen::Vector3d(world_to_holoworld_transform_.transform.translation.x, world_to_holoworld_transform_.transform.translation.y, world_to_holoworld_transform_.transform.translation.z);
  }

  geometry_msgs::PoseStamped place_location_pose;
  place_location_pose.header.stamp = ros::Time::now();
  place_location_pose.header.frame_id = "ROSworld";

  place_location_pose.pose.position.x = place_location_holoworld.x();
  place_location_pose.pose.position.y = place_location_holoworld.y();
  place_location_pose.pose.position.z = place_location_holoworld.z();
  place_location_pose.pose.orientation.x = 0;
  place_location_pose.pose.orientation.y = 0;
  place_location_pose.pose.orientation.z = 0;
  place_location_pose.pose.orientation.w = 1;
  place_selection_pub_.publish(place_location_pose);

}

void pickplace::print_tf_for_debug(void){
  
  bool tf_success = false;
  
  geometry_msgs::TransformStamped wrist_to_base_transform;
  
  while(tf_success == false)
  {
    try
    {
      wrist_to_base_transform = tfBuffer.lookupTransform(move_group_interface.getPlanningFrame(), "mpl_right_arm__wristz_link",
                                                        ros::Time(0));

      tf_success = true;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  // print the transform as euler angles XYZ(0, 1, 2)
  // x, y, z, w sequence
  tf2::Quaternion quat(wrist_to_base_transform.transform.rotation.x, wrist_to_base_transform.transform.rotation.y, wrist_to_base_transform.transform.rotation.z, wrist_to_base_transform.transform.rotation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  //std::cout << "Euler angles calculated with getRPY: " << "roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << std::endl;

  // calculate rpy using toRotationMatrix to XYZ
  // w, x, y, z, sequence
  Eigen::Quaterniond eig_quat(wrist_to_base_transform.transform.rotation.w, wrist_to_base_transform.transform.rotation.x, wrist_to_base_transform.transform.rotation.y, wrist_to_base_transform.transform.rotation.z);
  Eigen::Vector3d rpy = eig_quat.toRotationMatrix().eulerAngles(0, 1, 2);
  //std::cout << "Euler angles calculated with toRotationMatrix (XYZ body fixed): " << "roll: " << rpy(0) << ", pitch: " << rpy(1) << ", yaw: " << rpy(2) << std::endl;

  // now in ZYX convention (i.e., X then Y then Z in space)
  Eigen::Vector3d ypr = eig_quat.toRotationMatrix().eulerAngles(2, 1, 0);
  //std::cout << "Euler angles calculated with toRotationMatrix (ZYX body fixed): " << "yaw: " << ypr(0) << ", pitch: " << ypr(1) << ", roll: " << ypr(2) << std::endl;
}

void pickplace::set_neutral_pose(void)
{
  // define neutral pose with joint values
  std::vector<double> neutral_joint_values;
  neutral_joint_values.resize(7);
  // neutral_joint_values[0] = 19.0*deg2rad;
  // neutral_joint_values[1] = -29.0*deg2rad;
  // neutral_joint_values[2] = 8.0*deg2rad;
  // neutral_joint_values[3] = 103.0*deg2rad;
  // neutral_joint_values[4] = 61.0*deg2rad;
  // neutral_joint_values[5] = 21.0*deg2rad;
  // neutral_joint_values[6] = 55.0*deg2rad;
  neutral_joint_values[0] = 26.0*deg2rad;
  neutral_joint_values[1] = -56.0*deg2rad;
  neutral_joint_values[2] = -5.0*deg2rad;
  neutral_joint_values[3] = 120.0*deg2rad;
  neutral_joint_values[4] = 54.0*deg2rad;
  neutral_joint_values[5] = 34.0*deg2rad;
  neutral_joint_values[6] = 59.0*deg2rad;
  move_group_interface.setJointValueTarget(neutral_joint_values);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group_interface.asyncExecute(my_plan);
  // // move_group_interface.setGoalOrientationTolerance(0.01);
  // move_group_interface.move();
  //std::cout << "Sent move command to neutral pose" << std::endl;


  // move_group_interface.clearPoseTargets();
  // move_group_interface.setEndEffectorLink("mpl_right_arm__wristz_link");

  // // define neutral pose with ee pose 
  // geometry_msgs::Pose neutral_pose;

  // // get current pose
  // neutral_pose = move_group_interface.getCurrentPose().pose;
  // neutral_pose.position.z = table_height + thickness + partition_height + block_height/2 + pre_pick_pos_offset_.z();
  // neutral_pose.orientation.x = 0;
  // neutral_pose.orientation.y = 0;
  // neutral_pose.orientation.z = 1/sqrt(2);
  // neutral_pose.orientation.w = 1/sqrt(2);

  // // set larger tolerance for orientation
  // move_group_interface.setGoalOrientationTolerance(0.1);
  // target_pose_ = neutral_pose;

  // move_group_interface.setPoseTarget(neutral_pose);
  // // target_pose_ = neutral_pose;

  // move_group_interface.move();
  // move_group_interface.setGoalOrientationTolerance(0.001);

  // static tf2_ros::StaticTransformBroadcaster br;
  // geometry_msgs::TransformStamped transformStamped;
  
  // transformStamped.header.stamp = ros::Time::now();
  // transformStamped.header.frame_id = "world";
  // transformStamped.child_frame_id = "neutral_goal";
  // transformStamped.transform.translation.x = neutral_pose.position.x;
  // transformStamped.transform.translation.y = neutral_pose.position.y;
  // transformStamped.transform.translation.z = neutral_pose.position.z;
  // transformStamped.transform.rotation.x = neutral_pose.orientation.x;
  // transformStamped.transform.rotation.y = neutral_pose.orientation.y;
  // transformStamped.transform.rotation.z = neutral_pose.orientation.z;
  // transformStamped.transform.rotation.w = neutral_pose.orientation.w;

  // br.sendTransform(transformStamped);
  // return to default
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpl_pick_place");
  ros::NodeHandle nh("~");

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(0);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group_interface("mpl_arm");

  pickplace pickplace_arm(nh, "mpl_arm");
  ros::WallDuration(1.0).sleep(); //allow time for things to get set up 
  // pickplace pickplace_hand(nh, "mpl_hand");
  // ros::WallDuration(1.0).sleep();

  // pickplace_arm.remove_all_blocks(); // just in case there are any blocks left over from previous runs

  pickplace_arm.add_table(); // this should be done only once
  pickplace_arm.add_all_blocks();

  ros::WallDuration(5.0).sleep(); //wait for fingers to stop shaking
  // pickplace_arm.neutral();


  while (ros::ok()) {
    // Add any additional tasks or logic here

    // Sleep for a short duration to avoid excessive CPU usage
    ros::Duration(0.1).sleep();

  }

  return 0;
}
