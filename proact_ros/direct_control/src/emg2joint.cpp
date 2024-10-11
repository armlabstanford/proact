/*
Author: Shivani Guptasarma
Listen to user input (joystick or emg) and send joint-level trajectory command
*/
#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <mpl_control/graspAction.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

static const int QUEUE_LENGTH = 1;

class JointCommander
{
public:
    JointCommander();
    void joyCallback(const sensor_msgs::Joy &msg);
    void emgCallback(const std_msgs::String &msg);
    void poseCallback(const sensor_msgs::JointState &msg);
    void close_grasp(void);
    void open_grasp(void);
    void mode_switch(void);

private:
    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_;
    ros::Subscriber emg_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher traj_pub_, grasp_pub_, mode_pub_, grasp_pub_unity_;
    double joy_input_;
    double joint_angle_current_;
    double joint_goal_prev_;
    double joint_stay_at_;
    bool staying_angle_updated_;
    double flex_angle_;
    double adduct_angle_;
    double delta_trig_;
	bool joy_touch_;
	bool joy_ever_touched_;
	bool joy_touch_prev_; 
	bool joy_cont_touched_; //is the joystick being continuously operated?
    int mode_;
    int max_mode_;
    bool prev_button_state_;
   	bool prev_grasp_button_state_;
    bool now_closed_;
    actionlib::SimpleActionClient<mpl_control::graspAction> ac; 
    std::string prev_command_ = "no_motion";
    std::string joint_name_;
    bool pose_updated_for_new_joint_ = true;
    bool publish_traj_ = true;
    int joint_direction_;
    std::string current_emg_state_ = "no_motion";
    std::string current_command_ = "no_motion";
    std::string prev_prev_command_ = "no_motion";

};

JointCommander::JointCommander() : ac("grasp", true), now_closed_(false), joy_touch_(false), joy_touch_prev_(false), joy_ever_touched_(false), joy_cont_touched_(false), joint_goal_prev_(0), joy_input_(0), delta_trig_(0), mode_(0), max_mode_(0), prev_button_state_(false), prev_grasp_button_state_(false), joint_stay_at_(0)
{
    nh_ = ros::NodeHandle();//("~");                                                                           
    traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/mpl_right_arm/j0_controller/command", 1);
    mode_pub_ = nh_.advertise<std_msgs::Int8>("/mode", 1);
    // joy_sub_ = nh_.subscribe("/joy", 1, &JointCommander::joyCallback, this); // keeping this on messes up the published command stream if the joystick is not being used
    emg_sub_ = nh_.subscribe("/emg", 1, &JointCommander::emgCallback, this);
    pose_sub_ = nh_.subscribe("/mpl_right_arm/wrapped_joint_states", 1, &JointCommander::poseCallback, this);
    grasp_pub_unity_ = nh_.advertise<std_msgs::Bool>("/success", QUEUE_LENGTH);
    delta_trig_ = 0.001; // threshold for joystick input
    mode_ = 0; 
    /* Modes:
    0: shoulder flx/ext
    1: shoulder ab/d
    2: shoulder int/ext rot
    3: elbow flx/ext
    4: wrist y
    5: wrist x
    6: wrist z
    */
    max_mode_ = 6;

    sleep(3);

    std_msgs::Int8 mode_msg;
    mode_msg.data = mode_;
    mode_pub_.publish(mode_msg);

    std::cout << "Published mode" << std::endl;
    
    prev_button_state_ = 0;
    grasp_pub_ = nh_.advertise<mpl_control::graspActionGoal>("/grasp/goal", QUEUE_LENGTH);
	prev_grasp_button_state_ = 0;

    ac.waitForServer();
    // close and then open the hand so that status is clearly visible
    mpl_control::graspGoal hand_action;
    hand_action.instruct = 2; // close
    ac.sendGoal(hand_action); 
    hand_action.instruct = 1; // open
    ac.sendGoal(hand_action); 

}

void JointCommander::open_grasp(void)
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

void JointCommander::close_grasp(void)
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

void JointCommander::mode_switch(void){
    if(mode_ < max_mode_){
            mode_++;
        }
        else{
            mode_ = 0; //cycle back from last to first mode
        }
    // send mode update to unity 
    std_msgs::Int8 mode_msg;
    mode_msg.data = mode_;
    mode_pub_.publish(mode_msg);
    joy_ever_touched_ = false;
    staying_angle_updated_ = false;
    switch(mode_){
            case 0: 
                traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/mpl_right_arm/j0_controller/command", 1);
                joint_name_ = "mpl_right_arm__shoulder";
                joint_direction_ = 1; // wrist extension emg -> shoulder flexion
                break;
            case 1:
                traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/mpl_right_arm/j1_controller/command", 1);
                joint_name_ = "mpl_right_arm__humerus";
                joint_direction_ = -1; // wrist extension emg -> shoulder abduction
                break;
            case 2:
                traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/mpl_right_arm/j2_controller/command", 1);
                joint_name_ = "mpl_right_arm__elbow";
                joint_direction_ = -1; // wrist extension emg -> external rotation
                break;
            case 3:
                traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/mpl_right_arm/j3_controller/command", 1);
                joint_name_ = "mpl_right_arm__forearm";
                joint_direction_ = 1; // wrist extension emg -> elbow flexion
                break;
            case 4:
                traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/mpl_right_arm/j4_controller/command", 1);
                joint_name_ = "mpl_right_arm__wristy";
                joint_direction_ = 1; // wrist extension emg -> wrist pronation
                break;
            case 5:
                traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/mpl_right_arm/j5_controller/command", 1);
                joint_name_ = "mpl_right_arm__wristx";
                joint_direction_ = 1; // wrist extension emg -> ulnar deviation
                break;
            case 6: 
                traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/mpl_right_arm/j6_controller/command", 1);
                joint_name_ = "mpl_right_arm__wristz";
                joint_direction_ = -1; // wrist extension emg -> wrist extension
                break;
    }
    pose_updated_for_new_joint_ = false;
}

void JointCommander::joyCallback(const sensor_msgs::Joy &msg)
{
    int grasp_button = 4;
    int mode_button = 11;
    double dt = 0.005;
    joy_input_ = msg.axes[0];
    // Note that flexion of the wrist and ulnar deviation of the forearm are positive by convention
    if(std::abs(joy_input_)>=delta_trig_){
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

    //register button press
    if(msg.buttons[mode_button] && !prev_button_state_){
        mode_switch();
        prev_button_state_ = msg.buttons[mode_button];
    } 
    //register button release
    if(!msg.buttons[mode_button] && prev_button_state_){
        prev_button_state_ = msg.buttons[mode_button];
    } 

    // GRASP ACTION
	if(msg.buttons[grasp_button] && !prev_grasp_button_state_){
	    prev_grasp_button_state_ = msg.buttons[grasp_button];
	    std::cout << "button pressed" << std::endl;
		// std::cout << "prev_grasp_button_state_" << prev_grasp_button_state_ << "\n" << std::endl;
	    mpl_control::graspGoal hand_action;
		if (now_closed_){
			open_grasp();
		}
		else{
			close_grasp();
		}
    }
	// register button release
	if(!msg.buttons[grasp_button] && prev_grasp_button_state_){
	    prev_grasp_button_state_ = msg.buttons[grasp_button];
	} 
	
    trajectory_msgs::JointTrajectory cmd_traj;
    cmd_traj.header.stamp = ros::Time::now();

    cmd_traj.joint_names.clear();
    cmd_traj.joint_names.push_back(joint_name_); 

    trajectory_msgs::JointTrajectoryPoint traj_pt;
    if(joy_touch_){ //if joystick is being touched right now 
            staying_angle_updated_ = false;
			if(!joy_cont_touched_){ //if it wasn't being touched before, then update the joint goal from the current joint values
                joint_goal_prev_ = joint_angle_current_  + joy_input_*dt;
                traj_pt.positions.push_back(joint_goal_prev_);//and command a motion to that goal
			} else { //if it has been touched already and is still being touched, then don't update from current joint values, just add the current increment to the previous joint goal! We don't want goals to be affected by gravity
                joint_goal_prev_ += joy_input_*dt;// increment joint_goal_previous for the next iteration 
                traj_pt.positions.push_back(joint_goal_prev_);
			}
		} else if (joy_ever_touched_){ // if joystick has been released, keep commanding the last joint goal
            if (!staying_angle_updated_){
                joint_stay_at_ = joint_goal_prev_;
                staying_angle_updated_ = true;
            }
            traj_pt.positions.push_back(joint_stay_at_);
		}


    traj_pt.velocities.push_back(0.0);
    traj_pt.accelerations.push_back(0.0);
    traj_pt.effort.push_back(0.0);
    ros::Duration time_step(dt/2);
    traj_pt.time_from_start = time_step;
    cmd_traj.points.push_back(traj_pt);
    traj_pub_.publish(cmd_traj);

}

void JointCommander::emgCallback(const std_msgs::String &msg)
{
    if (!pose_updated_for_new_joint_){
        std::cout << "No pose update for new joint yet" << std::endl;
        return;
    }

    double dt = 0.005; // take care of max speed later
    double k = 4*joint_direction_;

    trajectory_msgs::JointTrajectory cmd_traj;
    cmd_traj.header.stamp = ros::Time::now();
    cmd_traj.joint_names.clear();
    cmd_traj.joint_names.push_back(joint_name_); 
    trajectory_msgs::JointTrajectoryPoint traj_pt;
    publish_traj_ = true;

    current_command_ = msg.data;

    // continuous trj
    if(current_command_!=prev_command_){
        if(current_command_ == "wrist_extension"){
            staying_angle_updated_ = false; // reset the staying angle
            joint_goal_prev_ = joint_angle_current_  + k*dt;
            traj_pt.positions.push_back(joint_goal_prev_);
        } 
        else if(current_command_ == "wrist_flexion"){
            staying_angle_updated_ = false; // reset the staying angle
            joint_goal_prev_ = joint_angle_current_  - k*dt;
            traj_pt.positions.push_back(joint_goal_prev_);
        } 
        else{ // publish traj, even if some mode switch/grasping going on
            if (!staying_angle_updated_){
            joint_stay_at_ = joint_goal_prev_;
            staying_angle_updated_ = true;
            }
            publish_traj_ = false;
        }
    }
    else if(current_command_ == prev_command_){
        if(current_command_ == "wrist_extension"){
            joint_goal_prev_ += k*dt;
            traj_pt.positions.push_back(joint_goal_prev_);
        } 
        else if(current_command_ == "wrist_flexion"){
            joint_goal_prev_ += -k*dt;
            traj_pt.positions.push_back(joint_goal_prev_);
        }
        else{ // publish traj, even if some mode switch/grasping going on
            publish_traj_ = false;
        }
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
        traj_pt.positions.push_back(joint_angle_current_); // keep the new joint where it was until something is done 
    }

    if(publish_traj_){
        traj_pt.velocities.push_back(0.0);
        traj_pt.accelerations.push_back(0.0);
        traj_pt.effort.push_back(0.0);
        ros::Duration time_step(3*dt); // emg stream is slower than joystick
        traj_pt.time_from_start = time_step;
        cmd_traj.points.push_back(traj_pt);
        traj_pub_.publish(cmd_traj);
    }

    prev_prev_command_ = prev_command_;
    prev_command_ = current_command_;

}

void JointCommander::poseCallback(const sensor_msgs::JointState &msg)
{
    /*list of joints in message
    name: 
    - mpl_right_arm__elbow
    - mpl_right_arm__forearm
    - mpl_right_arm__humerus
    - mpl_right_arm__index0
    - mpl_right_arm__index1
    - mpl_right_arm__index2
    - mpl_right_arm__index3
    - mpl_right_arm__middle0
    - mpl_right_arm__middle1
    - mpl_right_arm__middle2
    - mpl_right_arm__middle3
    - mpl_right_arm__pinky0
    - mpl_right_arm__pinky1
    - mpl_right_arm__pinky2
    - mpl_right_arm__pinky3
    - mpl_right_arm__ring0
    - mpl_right_arm__ring1
    - mpl_right_arm__ring2
    - mpl_right_arm__ring3
    - mpl_right_arm__shoulder
    - mpl_right_arm__thumb0
    - mpl_right_arm__thumb1
    - mpl_right_arm__thumb2
    - mpl_right_arm__thumb3
    - mpl_right_arm__wristx
    - mpl_right_arm__wristy
    - mpl_right_arm__wristz
    */

    switch(mode_){
        case 0: //shoulder flexion 
            joint_angle_current_ = msg.position[19]; //mpl_right_arm__shoulder
            break;
        case 1: //shoulder adduction
            joint_angle_current_ = msg.position[2]; //mpl_right_arm__humerus
            break;
        case 2: //internal rotation
            joint_angle_current_ = msg.position[0]; //mpl_right_arm__elbow
            break;
        case 3: //elbow flexion
            joint_angle_current_ = msg.position[1]; //mpl_right_arm__forearm
            break;
        case 4: //wrist pronation
            joint_angle_current_ = msg.position[25]; //mpl_right_arm__wristy
            break;
        case 5: //ulnar deviation
            joint_angle_current_ = msg.position[24]; //mpl_right_arm__wristx
            break;
        case 6: //wrist flexion
            joint_angle_current_ = msg.position[26]; //mpl_right_arm__wristz
            break;     
    }


    if(!pose_updated_for_new_joint_){ // mode was just changed, fill in for emg callback
        pose_updated_for_new_joint_ = true;
        joint_goal_prev_ = joint_angle_current_;
        std::cout << "Pose updated for joint: " << joint_name_ << ", value: " << joint_angle_current_ << std::endl;

    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "emg2joint");
    JointCommander JointCommanderObj;
    ros::spin();
    return 0;
}