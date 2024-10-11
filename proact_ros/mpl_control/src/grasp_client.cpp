#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <mpl_control/graspAction.h>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "grasp_client");

    actionlib::SimpleActionClient<mpl_control::graspAction> ac("grasp", true);

    // ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    // ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    mpl_control::graspGoal goal;
    goal.instruct = 1;
    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      // ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
    {
      // ROS_INFO("Action did not finish before the time out.");
      ac.cancelGoal();
    }

    return 0;
}