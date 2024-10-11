/*
Author: Shivani Guptasarma
Receive gaze origin position and direction vectors from Unity and rank blocks based on distance from gaze
Give arg 0 for gaze alone, 1 for gaze and color
*/
#include <iostream>
#include <iomanip>
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <message_filters/subscriber.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

// #include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <gazebo_grasp_plugin/GazeboGraspFix.h>
#include <gazebo_grasp_plugin_ros/GazeboGraspEvent.h>

// bool PUBLISHING = false;

using namespace message_filters;

void callback(const geometry_msgs::Vector3StampedConstPtr& gaze_pos, const geometry_msgs::Vector3StampedConstPtr& gaze_dir, tf2_ros::Buffer* tfBuffer, ros::Publisher red1_dist_pub, ros::Publisher red2_dist_pub, ros::Publisher blue1_dist_pub, ros::Publisher blue2_dist_pub, ros::Publisher gaze_pos_pub, ros::Publisher gaze_dir_pub)
{
    bool tf_success = false;

    geometry_msgs::TransformStamped holo2ros;
    
    while(tf_success == false)
    {
        try
        {
        holo2ros = tfBuffer->lookupTransform("world", "hololens_world_test",
                                                            ros::Time(0));

        tf_success = true;
        }
        catch (tf2::TransformException &ex)
        {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        }
    }


    // convert gaze_global transform rotation to a rotation matrix
    Eigen::Quaterniond gaze_global_quat(holo2ros.transform.rotation.w, holo2ros.transform.rotation.x, holo2ros.transform.rotation.y, holo2ros.transform.rotation.z);
    Eigen::Matrix3d gaze_global_mat = gaze_global_quat.toRotationMatrix();
    // rotate the gaze direction vector by the rotation matrix
    Eigen::Vector3d gaze_global_dir = gaze_global_mat * Eigen::Vector3d(gaze_dir->vector.x, gaze_dir->vector.y, gaze_dir->vector.z);
    // get the position of the gaze frame
    // first, multiply the gaze position by the rotation matrix
    Eigen::Vector3d gaze_global_pos_rot = gaze_global_mat * Eigen::Vector3d(gaze_pos->vector.x, gaze_pos->vector.y, gaze_pos->vector.z);
    // then add the translation vector to get the gaze position in the world frame
    Eigen::Vector3d gaze_global_pos = Eigen::Vector3d(holo2ros.transform.translation.x, holo2ros.transform.translation.y, holo2ros.transform.translation.z) + gaze_global_pos_rot;

    // gaze_pos_->x() = gaze_global_pos(0);
    // gaze_pos_->y() = gaze_global_pos(1);
    // gaze_pos_->z() = gaze_global_pos(2);

    // gaze_dir_->x() = gaze_global_dir(0);
    // gaze_dir_->y() = gaze_global_dir(1);
    // gaze_dir_->z() = gaze_global_dir(2);

    // std::cout << "gaze_global_pos: " << gaze_global_pos << std::endl;
    // std::cout << "gaze_global_dir: " << gaze_global_dir << std::endl;

    // ROS_ERROR("gaze_global_pos: %f, %f, %f", gaze_global_pos(0), gaze_global_pos(1), gaze_global_pos(2));


    // send the point of intersection of gaze ray with the table
    float table_thickness = 0.03;
    float table_height = 1;
    float block_radius = 0.025;
    float partition_thickness = 0.005;
    float wall_thickness = partition_thickness;
    double table_surface_height = table_height + table_thickness/2;
    double table_length = 0.61;
    double table_width = 0.31;
    double table_x_min = -table_width/2 + wall_thickness + block_radius;
    double table_x_max = table_width/2 - wall_thickness - block_radius;
    double table_y_max = table_length/2 - wall_thickness - block_radius;
    double table_y_min = block_radius + partition_thickness/2;
    if (std::abs(gaze_global_dir(2)) > 1e-4){
        double lambda = (table_surface_height - gaze_global_pos(2))/gaze_global_dir(2);
        // std::cout << "lambda: " << lambda << std::endl;
        double table_hit_x = gaze_global_pos(0) + lambda*gaze_global_dir(0);
        double table_hit_y = gaze_global_pos(1) + lambda*gaze_global_dir(1);
        double table_hit_z = table_surface_height;
        // std::cout << "Table plane hit at: " << table_hit_x << ", " << table_hit_y << ", " << table_hit_z << std::endl;
        geometry_msgs::Vector3Stamped table_hit;
        table_hit.header.stamp = ros::Time::now();
        table_hit.vector.x = table_hit_x;
        table_hit.vector.y = table_hit_y;
        table_hit.vector.z = table_hit_z;
    }
    // set number of blocks
    int n = 4;

    // set peakiness of Gaussian distribution
    double sigma = 0.1;

    // create array of block names to iterate through
    std::string block_names[n] = {"redcylinder1", "redcylinder2", "bluecylinder1", "bluecylinder2"};
    // create array of distances in image plane
    double distances[n] = {0, 0, 0, 0};
    double scores_gaze[n] = {0, 0, 0, 0};
    double scores_gaze_color[n] = {0, 0, 0, 0};
    double prob_gaze[n] = {0, 0, 0, 0};
    double prob_gaze_color[n] = {0, 0, 0, 0};
    double prob_red[n] = {0.4, 0.4, 0.2, 0.2};
    double prob_blue[n] = {0.2, 0.2, 0.4, 0.4};
    Eigen::MatrixXd prob_color_matrix(2, n);
    prob_color_matrix << prob_red[0], prob_red[1], prob_red[2], prob_red[3],
                        prob_blue[0], prob_blue[1], prob_blue[2], prob_blue[3];

    //std::cout << "prob_color_matrix: " << prob_color_matrix << std::endl;

    for (int i = 0; i < n; i++)
    {
        bool tf_success = false;

        geometry_msgs::TransformStamped block_transform;
        
        while(tf_success == false)
        {
            // std::cout << "going to try..." << std::flush;
            try
            {
            block_transform = tfBuffer->lookupTransform("world", block_names[i],
                                                                ros::Time(0));

            tf_success = true;
            // std::cout << block_transform << std::flush;
            }
            catch (tf2::TransformException &ex)
            {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            // std::cout << "trying" << std::flush;
            }
        }

        // get the vector from the gaze origin to the block
        Eigen::Vector3d gaze_to_block(block_transform.transform.translation.x - gaze_global_pos(0),
                                    block_transform.transform.translation.y - gaze_global_pos(1),
                                    block_transform.transform.translation.z - gaze_global_pos(2));
        // normalize the vector
        gaze_to_block.normalize();
        // get the dot product of the gaze direction and the gaze to block vector
        double dot_prod = gaze_to_block.dot(gaze_global_dir);
        // get the distance to the block in the image plane sqrt(1-dot_prod^2)/dot_prod
        distances[i] = sqrt(1 - dot_prod*dot_prod)/dot_prod;
        // note that if the person is looking away from the blocks, this will be negative
        scores_gaze[i] = (1/sigma/sqrt(2*M_PI))*exp(-distances[i]*distances[i]/2/sigma/sigma); //these don't add up to 1
    }
    // red1_dist[0] = distances[0];
    // red2_dist[0] = distances[1];
    // blue1_dist[0] = distances[2];
    // blue2_dist[0] = distances[3];

    geometry_msgs::Vector3Stamped gaze_pos_ros;
    gaze_pos_ros.header.stamp = ros::Time::now();
    gaze_pos_ros.vector.x = gaze_global_pos(0);
    gaze_pos_ros.vector.y = gaze_global_pos(1);
    gaze_pos_ros.vector.z = gaze_global_pos(2);
    gaze_pos_pub.publish(gaze_pos_ros);

    geometry_msgs::Vector3Stamped gaze_dir_ros;
    gaze_dir_ros.header.stamp = ros::Time::now();
    gaze_dir_ros.vector.x = gaze_global_dir(0);
    gaze_dir_ros.vector.y = gaze_global_dir(1);
    gaze_dir_ros.vector.z = gaze_global_dir(2);
    gaze_dir_pub.publish(gaze_dir_ros);

    std_msgs::Float64 temp_msg;
    temp_msg.data = distances[0];
    red1_dist_pub.publish(temp_msg);
    temp_msg.data = distances[1];
    red2_dist_pub.publish(temp_msg);
    temp_msg.data = distances[2];
    blue1_dist_pub.publish(temp_msg);
    temp_msg.data = distances[3];
    blue2_dist_pub.publish(temp_msg);

}

// class loggerClass{
//     public:
//         loggerClass(ros::NodeHandle &nh);
//         ~loggerClass();
//         float red1_dist_ = 0;
//         float red2_dist_ = 0;
//         float blue1_dist_ = 0;
//         float blue2_dist_ = 0;
//         Eigen::Vector3d gaze_pos_;
//         Eigen::Vector3d gaze_dir_;
//         void publish_data();
//     private:
//         ros::NodeHandle pnh_;
//         ros::Publisher red1_dist_pub;
//         ros::Publisher red2_dist_pub;
//         ros::Publisher blue1_dist_pub;
//         ros::Publisher blue2_dist_pub;
//         ros::Publisher gaze_pos_pub;
//         ros::Publisher gaze_dir_pub;
// };

// loggerClass::loggerClass(ros::NodeHandle &nh):pnh_(nh)
// {
//         red1_dist_pub = pnh_.advertise<std_msgs::Float64>("/red1_dist", 1);
//         red2_dist_pub = pnh_.advertise<std_msgs::Float64>("/red2_dist", 1);
//         blue1_dist_pub = pnh_.advertise<std_msgs::Float64>("/blue1_dist", 1);
//         blue2_dist_pub = pnh_.advertise<std_msgs::Float64>("/blue2_dist", 1);

//         gaze_pos_pub = pnh_.advertise<geometry_msgs::Vector3Stamped>("/gaze_position_ros", 1);
//         gaze_dir_pub = pnh_.advertise<geometry_msgs::Vector3Stamped>("/gaze_direction_ros", 1);

//         while (ros::ok()){
//             publish_data();
//         }

// }

// loggerClass::~loggerClass()
// {
// }

// void loggerClass::publish_data(){
//     geometry_msgs::Vector3Stamped gaze_pos_ros;
//     gaze_pos_ros.header.stamp = ros::Time::now();
//     gaze_pos_ros.vector.x = gaze_pos_(0);
//     gaze_pos_ros.vector.y = gaze_pos_(1);
//     gaze_pos_ros.vector.z = gaze_pos_(2);
//     gaze_pos_pub.publish(gaze_pos_ros);

//     geometry_msgs::Vector3Stamped gaze_dir_ros;
//     gaze_dir_ros.header.stamp = ros::Time::now();
//     gaze_dir_ros.vector.x = gaze_dir_(0);
//     gaze_dir_ros.vector.y = gaze_dir_(1);
//     gaze_dir_ros.vector.z = gaze_dir_(2);
//     gaze_dir_pub.publish(gaze_dir_ros);

//     std_msgs::Float64 temp_msg;
//     temp_msg.data = red1_dist_;
//     red1_dist_pub.publish(temp_msg);
//     temp_msg.data = red2_dist_;
//     red2_dist_pub.publish(temp_msg);
//     temp_msg.data = blue1_dist_;
//     blue1_dist_pub.publish(temp_msg);
//     temp_msg.data = blue2_dist_;
//     blue2_dist_pub.publish(temp_msg);

//     // PUBLISHING = false;
    
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gaze_log_node");

    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    message_filters::Subscriber<geometry_msgs::Vector3Stamped> gaze_pos_sub(nh, "/gaze_position", 1);
    // ros::Subscriber joy_sub_ = nh.subscribe("/gaze_position", 1, callback);
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> gaze_dir_sub(nh, "/gaze_direction", 1);

    // loggerClass loggerClass(nh);

    typedef sync_policies::ApproximateTime<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), gaze_pos_sub, gaze_dir_sub);
    // publisher for highest ranked block name
    ros::Publisher red1_dist_pub = nh.advertise<std_msgs::Float64>("/red1_dist", 1);
    ros::Publisher red2_dist_pub = nh.advertise<std_msgs::Float64>("/red2_dist", 1);
    ros::Publisher blue1_dist_pub = nh.advertise<std_msgs::Float64>("/blue1_dist", 1);
    ros::Publisher blue2_dist_pub = nh.advertise<std_msgs::Float64>("/blue2_dist", 1);

    ros::Publisher gaze_pos_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/gaze_position_ros", 1);
    ros::Publisher gaze_dir_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/gaze_direction_ros", 1);
    sync.registerCallback(boost::bind(&callback, _1, _2, &tfBuffer, red1_dist_pub, red2_dist_pub, blue1_dist_pub, blue2_dist_pub, gaze_pos_pub, gaze_dir_pub));


    ros::spin();

    return 0;
}
