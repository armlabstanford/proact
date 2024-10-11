#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <mpl_control/graspAction.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// #include <moveit_visual_tools/moveit_visual_tools.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;
const double deg2rad = tau / 360.0;


class MyClass 
{
public:
  MyClass() {};
  ~MyClass() { };
};


class movehand
{

public: 
  movehand();
  ~movehand();
//   void add_table(void);
//   // void attach_block(void);
//   void go_to_wrist_pose(void);
//   void tilt_wrist(void);
  moveit::planning_interface::MoveGroupInterface move_group_interface;
  bool close_grasp(void);
  bool open_grasp(void);
//   void go_to_place(void);
//   void release(void);
  
private:
//   ros::NodeHandle pnh_;
  tf2_ros::Buffer tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> tfListener;
  const moveit::core::JointModelGroup* joint_model_group;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  // float block_radius = 0.025;
  // float block_height = 0.05;
  float width = 0.8;
  float length = 1.5;
  float thickness = 0.03;
  float table_height = 1;
  float partition_thickness = 0.005;
  float partition_height = 0.15;
};


//constructor
movehand::movehand() :  move_group_interface("mpl_hand")
{ 
  std::cout << "0" << std::endl;
  std::cout << move_group_interface.getPlanningFrame() << std::endl;
  std::cout << move_group_interface.getEndEffectorLink() << std::endl;
  std::cout << move_group_interface.getJointModelGroupNames()[1] << std::endl;
  std::cout << &move_group_interface << std::endl;
  // std::cout << move_group_interface.getCurrentState() << std::endl;
  // joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("mpl_hand");
  std::cout << "1" << std::endl;
  tfListener.reset(new tf2_ros::TransformListener(tfBuffer));
  std::cout << "2" << std::endl;

  // Print info messages
  ROS_INFO_NAMED("movehand", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  ROS_INFO_NAMED("movehand", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
  ROS_INFO_NAMED("movehand", "Available Planning Groups:");
  std::cout << "3" << std::endl;
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
}

//destructor
movehand::~movehand()
{
}


bool movehand::close_grasp(void)
{
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("mpl_hand");

  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  joint_group_positions[0] = -12.0*deg2rad;
  joint_group_positions[1] = 65.0*deg2rad;
  joint_group_positions[2] = 13.0*deg2rad;
  joint_group_positions[3] = 14.0*deg2rad;
  joint_group_positions[4] = 0.0*deg2rad;
  joint_group_positions[5] = 64.0*deg2rad;
  joint_group_positions[6] = 22.0*deg2rad;
  joint_group_positions[7] = 0.0*deg2rad;
  joint_group_positions[8] = 12.0*deg2rad;
  joint_group_positions[9] = 55.0*deg2rad;
  joint_group_positions[10] = 7.0*deg2rad;
  joint_group_positions[11] = 2.0*deg2rad;
  joint_group_positions[12] = 12.0*deg2rad;
  joint_group_positions[13] = 69.0*deg2rad;
  joint_group_positions[14] = 5.0*deg2rad;
  joint_group_positions[15] = 7.0*deg2rad;
  joint_group_positions[16] = 98.0*deg2rad;
  joint_group_positions[17] = 21.0*deg2rad;
  joint_group_positions[18] = 13.0*deg2rad;
  joint_group_positions[19] = 18.0*deg2rad;

  move_group_interface.setJointValueTarget(joint_group_positions);
  move_group_interface.setMaxVelocityScalingFactor(1.00);
  move_group_interface.setMaxAccelerationScalingFactor(1.00);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success){
    move_group_interface.execute(my_plan);
  }
  return success;

}

bool movehand::open_grasp(void)
{
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("mpl_hand");

  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  // joint_group_positions[0] = 0.0*deg2rad;
  // joint_group_positions[1] = 0.0*deg2rad;
  // joint_group_positions[2] = 0.0*deg2rad;
  // joint_group_positions[3] = 0.0*deg2rad;
  // joint_group_positions[4] = 0.0*deg2rad;
  // joint_group_positions[5] = 0.0*deg2rad;
  // joint_group_positions[6] = 0.0*deg2rad;
  // joint_group_positions[7] = 0.0*deg2rad;
  // joint_group_positions[8] = 0.0*deg2rad;
  // joint_group_positions[9] = 0.0*deg2rad;
  // joint_group_positions[10] = 0.0*deg2rad;
  // joint_group_positions[11] = 0.0*deg2rad;
  // joint_group_positions[12] = 0.0*deg2rad;
  // joint_group_positions[13] = 0.0*deg2rad;
  // joint_group_positions[14] = 0.0*deg2rad;
  // joint_group_positions[15] = 0.0*deg2rad;
  // joint_group_positions[16] = 0.0*deg2rad;
  // joint_group_positions[17] = 0.0*deg2rad;
  // joint_group_positions[18] = 0.0*deg2rad;
  // joint_group_positions[19] = 0.0*deg2rad;

  joint_group_positions[0] = -12.0*deg2rad;
  joint_group_positions[1] = 40.0*deg2rad; // index1 // 57
  joint_group_positions[2] = 13.0*deg2rad;
  joint_group_positions[3] = 14.0*deg2rad;
  joint_group_positions[4] = 0.0*deg2rad;
  joint_group_positions[5] = 31.0*deg2rad; // middle1 // 53
  joint_group_positions[6] = 22.0*deg2rad; // middle2
  joint_group_positions[7] = 0.0*deg2rad;
  joint_group_positions[8] = 12.0*deg2rad;
  joint_group_positions[9] = 36.0*deg2rad; // pinky1 // 55
  joint_group_positions[10] = 7.0*deg2rad;
  joint_group_positions[11] = 2.0*deg2rad;
  joint_group_positions[12] = 12.0*deg2rad;
  joint_group_positions[13] = 32.0*deg2rad; // 56
  joint_group_positions[14] = 2.0*deg2rad; // ring2
  joint_group_positions[15] = 7.0*deg2rad;
  joint_group_positions[16] = 97.0*deg2rad;
  joint_group_positions[17] = 12.0*deg2rad;
  joint_group_positions[18] = 13.0*deg2rad;
  joint_group_positions[19] = 18.0*deg2rad; // thumb 0

  move_group_interface.setJointValueTarget(joint_group_positions);
  move_group_interface.setMaxVelocityScalingFactor(1.00);
  move_group_interface.setMaxAccelerationScalingFactor(1.00);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;  
  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success){
    move_group_interface.execute(my_plan);
  }
  return success;

}


class graspAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<mpl_control::graspAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  mpl_control::graspFeedback feedback_;
  mpl_control::graspResult result_;
  movehand movehand_obj;
  // MyClass myclass_obj;
  // bool done_once_;

public:

  graspAction(std::string name) :
    as_(nh_, name, boost::bind(&graspAction::executeCB, this, _1), false),
    action_name_(name),//, done_once_(false)
    movehand_obj()
  {
    as_.start();
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // // moveit::planning_interface::MoveGroupInterface move_group_interface("mpl_arm");
    // ros::WallDuration(1.0).sleep();
  }

  ~graspAction(void)
  {
  }

  void executeCB(const mpl_control::graspGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = false;
    // give feedback that it's not done yet
    feedback_.doing = 0;

    // if (!done_once_)
    // {
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // // moveit::planning_interface::MoveGroupInterface move_group_interface("mpl_arm");
    // movehand movehand;
    // movehandptr = &movehand;
    // ros::WallDuration(1.0).sleep();
    // done_once_ = true;
    // std::cout << "done once" << std::endl;
    // }

    if (goal->instruct == 1) 
    {
      // std::cout << "hello" << std::endl;
      // std::cout << "pointer's address" << &movehandptr << std::endl;
      success = movehand_obj.close_grasp(); //close fingers around the block with a pre-designed grasp
      // std::cout << "bye" << std::endl;
    }
    else if (goal->instruct == 2)
    {
      success = movehand_obj.open_grasp(); //open fingers
    }
    // publish info to the console for the user
    // ROS_INFO("%s: Executing", action_name_.c_str());

    // // start executing the action
    // for(int i=1; i<=goal->order; i++)
    // {
    //   // check that preempt has not been requested by the client
    //   if (as_.isPreemptRequested() || !ros::ok())
    //   {
    //     ROS_INFO("%s: Preempted", action_name_.c_str());
    //     // set the action state to preempted
    //     as_.setPreempted();
    //     success = false;
    //     break;
    //   }
    //   feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
    //   // publish the feedback
    //   as_.publishFeedback(feedback_);
    //   // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
    //   r.sleep();
    // }

    //execute the action all at once for now 

    feedback_.doing = 1; 
    as_.publishFeedback(feedback_);

    result_.done = success;
    as_.setSucceeded(result_);

    // if(success)
    // {
    //   result_.done= feedback_.doing;
    //   // ROS_INFO("%s: Succeeded", action_name_.c_str());
    //   // set the action state to succeeded
    //   as_.setSucceeded(result_);
    // }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "grasp");


  graspAction grasp("grasp");
  ros::spin();

  return 0;
}