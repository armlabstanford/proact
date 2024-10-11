/*
Author: Shivani Guptasarma
Listen to user input (joystick or emg) and send end-effector command along one or two task-space dof
*/
#include "geometry_msgs/TwistStamped.h"
#include "control_msgs/JointJog.h"
#include "gazebo_msgs/LinkStates.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <mpl_control/graspAction.h>
#include <iostream>
#include <kdl_parser/kdl_parser.hpp>
#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <algorithm>
#include <unordered_map>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"

static const int QUEUE_LENGTH = 1;
const double tau = 2 * M_PI;
const double deg2rad = tau / 360.0;

class ServoCommander
{
public:
  ServoCommander(ros::NodeHandle &nh); 
  ~ServoCommander();
	void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
	void emgCallback(const std_msgs::String &msg);
	void jointsCallback(const sensor_msgs::JointState::ConstPtr& msg); 
	void close_grasp(void);
    void open_grasp(void);
    void mode_switch(void);
  
private:
	ros::NodeHandle pnh_;
	ros::Subscriber joy_sub_, emg_sub_;
	ros::Subscriber joints_sub_;
	ros::Subscriber eepose_sub_;
	ros::Publisher joint_delta_pub_, grasp_pub_, mode_pub_, grasp_pub_unity_; //twist_pub_, 
	KDL::Tree kdl_tree_;
	KDL::Chain kdl_chain_;
	boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;                                                                                                                            
	KDL::JntArray  q_;            // Joint positions                                                                                                                                                                                                                                                             
	KDL::Jacobian  J_;            // Jacobian  
	Eigen::VectorXd joint_values_;
	Eigen::VectorXd joint_goal_prev_;
	std::vector<std::string> joint_names_;
	Eigen::VectorXd joint_deltas_values_;
	Eigen::VectorXd twist_values_;
	Eigen::VectorXd twist_values_compensation_;
	double delta_trig_;
	bool joy_touch_;
	bool joy_ever_touched_;
	bool joy_touch_prev_; 
	bool joy_cont_touched_; //is the joystick being continuously operated?
	double dt;
	double compensation_scale_factor_;
	double input_scale_factor_;
	double omega_scale_ = 5;
	int mode_;
	int max_mode_;
	actionlib::SimpleActionClient<mpl_control::graspAction> ac; 
	bool prev_grasp_button_state_;
	bool prev_mode_button_state_;
	bool now_closed_;
    bool publish_traj_ = true;
	std::string current_command_;
	std::string prev_command_;
	std::string prev_prev_command_;
};

//constructor
ServoCommander::ServoCommander(ros::NodeHandle &nh) :  pnh_(nh), delta_trig_(0.001), twist_values_(Eigen::VectorXd::Zero(6)), twist_values_compensation_(Eigen::VectorXd::Zero(6)), joy_touch_(false), joy_touch_prev_(false), joy_ever_touched_(false), joy_cont_touched_(false), joint_goal_prev_(Eigen::VectorXd::Zero(7)), joint_values_(Eigen::VectorXd::Zero(7)), joint_names_({"mpl_right_arm__shoulder", "mpl_right_arm__humerus", "mpl_right_arm__elbow", "mpl_right_arm__forearm", "mpl_right_arm__wristy", "mpl_right_arm__wristx", "mpl_right_arm__wristz"}), joint_deltas_values_(Eigen::VectorXd::Zero(7)), ac("grasp", true), now_closed_(false) // test_string_(std_msgs.msg.String("hello world")) //joint_deltas_(control_msgs.msg.JointJog(std_msgs.msg.Header(0, ros::Time::now(), "frame"), joint_names_, Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7), 0)) // ac("grasp", true), 
{
	dt = 0.1;
	mode_ = 0; //vx, vy, vz, wx, wy, wz
	max_mode_ = 5;
	prev_mode_button_state_ = 0;
	prev_grasp_button_state_ = 0;
	input_scale_factor_ = 0.04;

	// communication
	joy_sub_ = pnh_.subscribe("/joy", QUEUE_LENGTH, &ServoCommander::joyCallback, this);
	emg_sub_ = pnh_.subscribe("/emg", QUEUE_LENGTH, &ServoCommander::emgCallback, this);
	joints_sub_ = pnh_.subscribe("/mpl_right_arm/wrapped_joint_states", QUEUE_LENGTH, &ServoCommander::jointsCallback, this);
	joint_delta_pub_ = pnh_.advertise<trajectory_msgs::JointTrajectory>("/mpl_right_arm/arm_controller/command", QUEUE_LENGTH);
	grasp_pub_ = pnh_.advertise<mpl_control::graspActionGoal>("/grasp/goal", QUEUE_LENGTH);
	mode_pub_ = pnh_.advertise<std_msgs::Int8>("/mode", QUEUE_LENGTH);
	grasp_pub_unity_ = pnh_.advertise<std_msgs::Bool>("/success", QUEUE_LENGTH);

	// set up KDL chain
	std::cout << "\n" << std::endl;
	std::string robot_desc_string;
	ros::NodeHandle node;
	node.param("robot_description", robot_desc_string, std::string());
	if (!kdl_parser::treeFromString(robot_desc_string, kdl_tree_)){
			ROS_ERROR("Failed to construct kdl tree");
	}
	else std::cout << "kdl tree constructed" << std::endl;
	bool tree2chainDone = kdl_tree_.getChain("base_link", "mpl_right_arm__wristz_link", kdl_chain_);
	jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
	q_.resize(joint_names_.size()) ; //kdl_chain_.getNrOfJoints());
	J_.resize(joint_names_.size()); //kdl_chain_.getNrOfJoints());
	for (int i = 0; i < joint_names_.size(); i++){
			std::cout << "joint name: " << joint_names_[i] << std::endl;
	}

	sleep(3);

	std_msgs::Int8 mode_msg;
    mode_msg.data = mode_ + 7;
    mode_pub_.publish(mode_msg);
    
	ac.waitForServer();
	// close and then open the hand so that status is visible
	mpl_control::graspGoal hand_action;
	hand_action.instruct = 1; // close
	ac.sendGoal(hand_action);
	hand_action.instruct = 2; // open
	ac.sendGoal(hand_action);
};


//destructor
ServoCommander::~ServoCommander()
{
}

void ServoCommander::open_grasp(void)
{
  mpl_control::graspGoal hand_action;
  hand_action.instruct = 2; // command open
  ac.sendGoal(hand_action);

  // wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	{
	    actionlib::SimpleClientGoalState state = ac.getState();
	    ROS_INFO("Action finished: %s",state.toString().c_str());
		now_closed_ = !(ac.getResult()->done);
	}
	else
	{
	    ROS_INFO("Action did not finish before the time out.");
	    ac.cancelGoal();
	}

	std_msgs::Bool grasp_msg;
    grasp_msg.data = false;
    grasp_pub_unity_.publish(grasp_msg);
  
}

void ServoCommander::close_grasp(void)
{
  mpl_control::graspGoal hand_action;
  hand_action.instruct = 1; // command close
  ac.sendGoal(hand_action);

  // wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	{
	    actionlib::SimpleClientGoalState state = ac.getState();
	    ROS_INFO("Action finished: %s",state.toString().c_str());
		now_closed_ = ac.getResult()->done;
	}
	else
	{
	    ROS_INFO("Action did not finish before the time out.");
	    ac.cancelGoal();
	}

	std_msgs::Bool grasp_msg;
    grasp_msg.data = true;
    grasp_pub_unity_.publish(grasp_msg);
  
}

void ServoCommander::mode_switch(void){
    if(mode_ < max_mode_){
            mode_++;
        }
        else{
            mode_ = 0; //cycle back from last to first mode
        }

    // send mode update to unity 
    std_msgs::Int8 mode_msg;
    mode_msg.data = mode_ + 7;
    mode_pub_.publish(mode_msg);
    joy_ever_touched_ = false;
}

void ServoCommander::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	int grasp_button = 4;
	int mode_button = 11;

	geometry_msgs::TwistStamped twist;
	twist.header.stamp = ros::Time::now();
	twist.twist.linear.x = 0;
	twist.twist.linear.y = 0;
	twist.twist.linear.z = 0;
	twist.twist.angular.x = 0;
	twist.twist.angular.y = 0;
	twist.twist.angular.z = 0;
	if(msg->buttons[mode_button] && !prev_mode_button_state_){
		if(mode_ < max_mode_){
			mode_++;
		}
		else{
			mode_ = 0; //cycle back from last to first mode
		}

		// publish mode
		std_msgs::Int8 mode_msg;
		mode_msg.data = mode_ + 3;
		mode_pub_.publish(mode_msg);

		joy_touch_ = false; //reset touched status on changing mode
		prev_mode_button_state_ = msg->buttons[mode_button];
	} 
	//register button release
	if(!msg->buttons[mode_button] && prev_mode_button_state_){
		prev_mode_button_state_ = msg->buttons[mode_button];
	} 
	
	switch(mode_){
		// if moving 1 dof at a time
		// case 0: 
		// 	twist.twist.linear.x = msg->axes[1];
		// 	break;
		// case 1: 
		// 	twist.twist.linear.y = msg->axes[1];
		// 	break;
		// case 2: 
		// 	twist.twist.linear.z = msg->axes[1];
		// 	break;
		// case 3: 
		// 	twist.twist.angular.x = msg->axes[1];
		// 	break;
		// case 4: 
		// 	twist.twist.angular.y = msg->axes[1];
		// 	break;
		// case 5: 
		// 	twist.twist.angular.z = msg->axes[1];
		// 	break;

		// if moving 2 dofs at a time
		case 0: 
			twist.twist.linear.y = msg->axes[0];
			twist.twist.linear.x = msg->axes[1];
			break;
		case 1: 
			twist.twist.angular.z = msg->axes[0];
			twist.twist.linear.z = msg->axes[1];
			break;
		case 2: 
			twist.twist.angular.x = -msg->axes[0];
			twist.twist.angular.y = -msg->axes[1];
			break;
	}
	if(std::abs(twist.twist.linear.x)>=delta_trig_ || std::abs(twist.twist.linear.y)>=delta_trig_ || std::abs(twist.twist.linear.z)>=delta_trig_ || std::abs(twist.twist.angular.x)>=delta_trig_ || std::abs(twist.twist.angular.y)>=delta_trig_ || std::abs(twist.twist.angular.z)>=delta_trig_ ){
		joy_touch_ = true;
		joy_ever_touched_ = true;
		if (joy_touch_prev_){
			joy_cont_touched_ = true;
		} else{
			joy_cont_touched_ = false;
		}
	}
	else {
		joy_touch_ = false;
	}
	joy_touch_prev_ = joy_touch_;
	twist_values_ << twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z, omega_scale_*twist.twist.angular.x, omega_scale_*twist.twist.angular.y, omega_scale_*twist.twist.angular.z;

	// GRASP ACTION
	if(msg->buttons[grasp_button] && !prev_grasp_button_state_){
	    prev_grasp_button_state_ = msg->buttons[grasp_button];
	    std::cout << "button pressed" << std::endl;
		if (now_closed_){
			open_grasp();
		}
		else{
			close_grasp();
		}
	    std::cout << msg->buttons[grasp_button] << std::endl;    
	} 
	// register button release
	if(!msg->buttons[grasp_button] && prev_grasp_button_state_){
	    prev_grasp_button_state_ = msg->buttons[grasp_button];
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
	for (int i = 0; i < joint_names_.size(); i++){
			q_(i) = joint_values_[i]; 
	}
	// get Jacobian
	jnt_to_jac_solver_->JntToJac(q_, J_);
	Eigen::MatrixXd jacobian(6,7); // Don't use current_state_->getJacobian(joint_model_group_); //this jacobian is not reliable!
	// convert to Eigen matrix
	for (int i=0; i< J_.rows(); i ++ ){
		for (int j=0; j< J_.columns(); j++){
				jacobian(i,j) = J_.getColumn(j)[i];
		}
	}
	// get joint deltas
	Eigen::JacobiSVD<Eigen::MatrixXd> svd = Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::MatrixXd matrix_s = svd.singularValues().asDiagonal();
	std::cout << "Singular values of Jacobian: " << matrix_s << std::endl;
	Eigen::MatrixXd pseudo_inverse = svd.matrixV() * matrix_s.inverse() * svd.matrixU().transpose();
	joint_deltas_values_ = pseudo_inverse * (input_scale_factor_*twist_values_); 
	trajectory_msgs::JointTrajectory joint_deltas_;
	joint_deltas_.header.frame_id = "base_link";
	joint_deltas_.header.stamp = ros::Time::now();
	joint_deltas_.joint_names.clear();
	ros::Duration time_step(dt);
	trajectory_msgs::JointTrajectoryPoint point;
	for (int i=0; i< joint_deltas_values_.size(); i ++ )
	{
		joint_deltas_.joint_names.push_back(joint_names_[i]);
		if(joy_touch_){ //if joystick is being touched right now 
			if(!joy_cont_touched_){ //if it wasn't being touched before, then update the joint goal from the current joint values
				joint_goal_prev_[i] = joint_values_[i] + dt*joint_deltas_values_(i);
				point.positions.push_back(joint_goal_prev_[i]); //and command a motion to that goal
			} else { //if it has been touched already and is still being touched, then don't update from current joint values, just add the current increment to the previous joint goal! We don't want goals to be affected by gravity
				point.positions.push_back(joint_goal_prev_[i] + dt*joint_deltas_values_(i));
				joint_goal_prev_[i] = joint_goal_prev_[i] + dt*joint_deltas_values_(i); // increment joint_goal_previous for the next iteration 
			}
		} else if (joy_ever_touched_){ // if joystick has been released, just keep publishing whatever was the last goal 
			point.positions.push_back(joint_goal_prev_[i]);
		}
	}
	point.time_from_start = time_step; //
	joint_deltas_.points.push_back(point);
	joint_delta_pub_.publish(joint_deltas_);
}

void ServoCommander::emgCallback(const std_msgs::String &msg)
{
	publish_traj_ = true;
    current_command_ = msg.data;
	if(current_command_ == "hand_close" || current_command_ == "hand_open" || current_command_ == "no_motion"){
		publish_traj_ = false;
	}
	int direction = 1;
	if(current_command_ == "wrist_extension"){
		direction = 1;
	}
	else if(current_command_ == "wrist_flexion"){
		direction = -1;
	}
	if(publish_traj_){
		geometry_msgs::TwistStamped twist_msg;
		twist_msg.header.stamp = ros::Time::now();
		twist_msg.twist.linear.x = 0;
		twist_msg.twist.linear.y = 0;
		twist_msg.twist.linear.z = 0;
		twist_msg.twist.angular.x = 0;
		twist_msg.twist.angular.y = 0;
		twist_msg.twist.angular.z = 0;

		switch(mode_){
			case 0: 
				twist_msg.twist.linear.x = direction;
				break;
			case 1: 
				twist_msg.twist.linear.y = -direction;
				break;
			case 2: 
				twist_msg.twist.linear.z = direction;
				break;
			case 3:
				twist_msg.twist.angular.x = direction;
				break;
			case 4:
				twist_msg.twist.angular.y = -direction;
				break;
			case 5:
				twist_msg.twist.angular.z = -direction;
				break;
		}

		twist_values_ << twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z, omega_scale_*twist_msg.twist.angular.x, omega_scale_*twist_msg.twist.angular.y, omega_scale_*twist_msg.twist.angular.z;

		for (int i = 0; i < joint_names_.size(); i++){
			q_(i) = joint_values_[i]; 
		}

		// get Jacobian
		jnt_to_jac_solver_->JntToJac(q_, J_);
		Eigen::MatrixXd jacobian(6,7); // Don't use current_state_->getJacobian(joint_model_group_); //this jacobian is not reliable!
		// convert to Eigen matrix
		for (int i=0; i< J_.rows(); i ++ ){
			for (int j=0; j< J_.columns(); j++){
					jacobian(i,j) = J_.getColumn(j)[i];
			}
		}
		// get joint deltas
		Eigen::JacobiSVD<Eigen::MatrixXd> svd = Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::MatrixXd matrix_s = svd.singularValues().asDiagonal();
		Eigen::MatrixXd pseudo_inverse = svd.matrixV() * matrix_s.inverse() * svd.matrixU().transpose();
		double condition_number = matrix_s(0, 0) / matrix_s(5, 5);
		std::cout << "Jacobian condition number: " << condition_number << std::endl;
		joint_deltas_values_ = pseudo_inverse * (input_scale_factor_*twist_values_); 
		trajectory_msgs::JointTrajectory joint_deltas_;
		joint_deltas_.header.frame_id = "base_link";
		joint_deltas_.header.stamp = ros::Time::now();
		joint_deltas_.joint_names.clear();
		ros::Duration time_step(dt);
		trajectory_msgs::JointTrajectoryPoint point;
		for (int i=0; i< joint_deltas_values_.size(); i ++ )
		{
			joint_deltas_.joint_names.push_back(joint_names_[i]);
			if (condition_number > 25.0 && joint_deltas_values_(3) < 0){
				point.positions.push_back(joint_goal_prev_[i]);
			}
			else if(current_command_ != prev_command_){ 
				joint_goal_prev_[i] = joint_values_[i] + dt*joint_deltas_values_(i);
				point.positions.push_back(joint_goal_prev_[i]); //and command a motion to that goal
			} else { //if continuous move command, then don't update from current joint values, just add the current increment to the previous joint goal! We don't want goals to be affected by gravity
				point.positions.push_back(joint_goal_prev_[i] + dt*joint_deltas_values_(i));
				joint_goal_prev_[i] = joint_goal_prev_[i] + dt*joint_deltas_values_(i); // increment joint_goal_previous for the next iteration 
			}
			// it's not publishing anything when input is not coming - test this
		}
		point.time_from_start = time_step; //
		joint_deltas_.points.push_back(point);
		joint_delta_pub_.publish(joint_deltas_);
	}
  
    // handle spurious signals
    if(prev_command_ == "hand_close" && current_command_ == "hand_open"){
        current_command_ = "no_motion";
    }

    // discrete commands
    if(current_command_ == "hand_close" && current_command_ != prev_command_){
        if(!now_closed_){
            close_grasp();
            std::cout << "Closed grasp" << std::endl;
        }
        else{
            open_grasp();
            std::cout << "Opened grasp" << std::endl;
        }
    }
    else if(current_command_ == "hand_open" && prev_prev_command_ != "hand_open" && current_command_ == prev_command_){ // occured twice in a row
        mode_switch();
    }

    prev_prev_command_ = prev_command_;
    prev_command_ = current_command_;
}

void ServoCommander::jointsCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	std::unordered_map<std::string, float> stringFloatMap;
	for (int i=0; i< msg->name.size(); i++){
			stringFloatMap.insert(std::make_pair(msg->name[i], msg->position[i]));
	};
	for(int i=0; i< joint_names_.size(); i++){
			auto it = stringFloatMap.find(joint_names_[i]);
			if (it != stringFloatMap.end()) {
					joint_values_[i] = it->second;
			} else {
					std::cout << "String not found." << std::endl;
			}
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "emg2servo");
	ros::NodeHandle nh("~");
	ServoCommander ServoCommanderObj(nh); 
	ros::spin();
	return 0;
}