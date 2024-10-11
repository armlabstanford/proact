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
#include <gazebo_grasp_plugin/GazeboGraspFix.h>
#include <gazebo_grasp_plugin_ros/GazeboGraspEvent.h>

using namespace message_filters;

void callback(const geometry_msgs::Vector3StampedConstPtr& gaze_pos, const geometry_msgs::Vector3StampedConstPtr& gaze_dir, const ros::Publisher& blockname_pub, const ros::Publisher& tablehit_pub, const ros::Publisher& tablehit_pub_logging, tf2_ros::Buffer* tfBuffer, int* color, int mode)
{
    // std::cout << "Received gaze position: " << gaze_pos->vector.x << ", " << gaze_pos->vector.y << ", " << gaze_pos->vector.z << std::endl;
    // std::cout << "Received gaze direction: " << gaze_dir->vector.x << ", " << gaze_dir->vector.y << ", " << gaze_dir->vector.z << std::endl;

    // publish tf from gaze origin to world

    // static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    // transformStamped.header.stamp = ros::Time::now();
    // transformStamped.header.frame_id = "hololens_world_test";
    // transformStamped.child_frame_id = "gaze_frame";
    // transformStamped.transform.translation.x = gaze_pos->vector.x;
    // transformStamped.transform.translation.y = gaze_pos->vector.y;
    // transformStamped.transform.translation.z = gaze_pos->vector.z;
    // Eigen::Vector3d gaze_frame_z(gaze_dir->vector.x, gaze_dir->vector.y, gaze_dir->vector.z);
    // Eigen::Vector3d original_y(0, 1, 0);
    // Eigen::Vector3d gaze_frame_x = original_y.cross(gaze_frame_z);
    // Eigen::Vector3d gaze_frame_y = gaze_frame_z.cross(gaze_frame_x);

    // // if gaze_frame_z is nearly parallel to original_y, then use the original x vector instead
    // if (gaze_frame_x.norm() < 0.0001)
    // {
    //     Eigen::Vector3d original_x(1, 0, 0);
    //     gaze_frame_x = original_x.cross(gaze_frame_z);
    //     gaze_frame_y = gaze_frame_z.cross(gaze_frame_x);
    //     std::cout << "gaze direction close to world y" << std::endl;
    // }

    // // form a matrix from the vectors
    // Eigen::Matrix3d gaze_frame_mat;
    // gaze_frame_mat.col(0) = gaze_frame_x;
    // gaze_frame_mat.col(1) = gaze_frame_y;
    // gaze_frame_mat.col(2) = gaze_frame_z;

    // // form a quaternion from the matrix
    // Eigen::Quaterniond gaze_frame_quat(gaze_frame_mat);
    // // normalize the quaternion
    // gaze_frame_quat.normalize();

    // // convert to tf quaternion
    // tf2::Quaternion q(gaze_frame_quat.x(), gaze_frame_quat.y(), gaze_frame_quat.z(), gaze_frame_quat.w());
    // transformStamped.transform.rotation.x = q.x();
    // transformStamped.transform.rotation.y = q.y();
    // transformStamped.transform.rotation.z = q.z();
    // transformStamped.transform.rotation.w = q.w();

    // br.sendTransform(transformStamped);

    bool tf_success = false;

    geometry_msgs::TransformStamped holo2ros;
    
    while(tf_success == false)
    {
        // std::cout << "going to try..." << std::flush;
        try
        {
        holo2ros = tfBuffer->lookupTransform("world", "hololens_world_test",
                                                            ros::Time(0));

        tf_success = true;
        // std::cout << holo2ros << std::flush;
        }
        catch (tf2::TransformException &ex)
        {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        // std::cout << "trying" << std::flush;
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

    // std::cout << "gaze_global_pos: " << gaze_global_pos << std::endl;
    // std::cout << "gaze_global_dir: " << gaze_global_dir << std::endl;


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
        tablehit_pub_logging.publish(table_hit);
        if (table_hit_x > table_x_min && table_hit_x < table_x_max && table_hit_y > table_y_min && table_hit_y < table_y_max){ //point is in allowed region
        tablehit_pub.publish(table_hit);
        }
        // else {
        //     std::cout << "Not looking at place section" << std::endl;
        // }
    }
    // else {
    //     std::cout << "Gaze direction is parallel to table" << std::endl;
    // }

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
    double prob_red[n] = {0.45, 0.45, 0.15, 0.15};
    double prob_blue[n] = {0.15, 0.15, 0.45, 0.45};
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
    double score_sum = 0;
    for (int i = 0; i < n; i++)
    {
        score_sum += scores_gaze[i]; // std::cout << block_names[i] << " distance: " << distances[i] << std::endl;
    }
    // make them add up to 1
    for (int i = 0; i < n; i++)
    {
        prob_gaze[i] = scores_gaze[i]/score_sum;
    }
    // p(gaze| color, particular block) = p(gaze | particular block) = p(particular block|gaze)
    for (int i = 0; i < n; i++)
    {
        scores_gaze_color[i] = prob_gaze[i] * prob_color_matrix(*color, i);
        // std::cout << "color used: " << *color << std::endl;
        //std::cout << "scores_gaze_color: " << scores_gaze_color[i] << std::endl;
    }
    double score_sum_color = 0;
    for (int i = 0; i < n; i++)
    {
        score_sum_color += scores_gaze_color[i];
        // std::cout << "probs redcylinder1,redcylinder2,bluecylinder1,bluecylinder2" << prob_color_matrix(*color, i) << std::endl;
    }
    for (int i = 0; i < n; i++)
    {
        prob_gaze_color[i] = scores_gaze_color[i]/score_sum_color;
    }

    std_msgs::String block_name_msg;
    if (!mode){
        block_name_msg.data = block_names[std::distance(scores_gaze, std::max_element(scores_gaze, scores_gaze + n))];
    }
    else {
        block_name_msg.data = block_names[std::distance(scores_gaze_color, std::max_element(scores_gaze_color, scores_gaze_color + n))];
    }
    // block_name_msg.data = block_names[std::distance(prob_gaze_color, std::max_element(prob_gaze_color, prob_gaze_color + n))];
    blockname_pub.publish(block_name_msg);

}

class colorClass{
    public: 
        colorClass(ros::NodeHandle &nh);
        ~colorClass();
        int color_ = 0; //start with red 
        int mode_ = 0;
        void colorCallback(const gazebo_grasp_plugin_ros::GazeboGraspEvent::ConstPtr& msg);
    private:      
        ros::NodeHandle pnh_;
        ros::Subscriber color_sub_;
};

colorClass::colorClass(ros::NodeHandle &nh):pnh_(nh)
{
        color_sub_ = pnh_.subscribe("/grasp_event_republisher/grasp_events", 1, &colorClass::colorCallback, this);

}

colorClass::~colorClass()
{
}

void colorClass::colorCallback(const gazebo_grasp_plugin_ros::GazeboGraspEvent::ConstPtr& msg){
    std::string block_name = msg->object;
    // if the string starts with "red", set color to 1, i.e. next block should be blue
    if (block_name.compare(0, 3, "red") == 0){
        color_ = 1;
    }
    // if the string starts with "blue", set color to 0, i.e., next block should be red
    else if (block_name.compare(0, 4, "blue") == 0){
        color_ = 0;
    }
    else {
        std::cout << "block name not recognized" << std::endl;
    }
    // if the block is dropped and you need to re-pick, just pick through the difficulty of a biased selection
    // because this way the system will support the next pick, at least, instead of having a global plan that gets offset
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gaze_sync_node");

    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    int color;

    message_filters::Subscriber<geometry_msgs::Vector3Stamped> gaze_pos_sub(nh, "/gaze_position", 1);
    // ros::Subscriber joy_sub_ = nh.subscribe("/gaze_position", 1, callback);
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> gaze_dir_sub(nh, "/gaze_direction", 1);

    // subscribe to color message
    colorClass colorClass(nh);

    typedef sync_policies::ApproximateTime<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), gaze_pos_sub, gaze_dir_sub);
    // publisher for highest ranked block name
    ros::Publisher blockname_pub = nh.advertise<std_msgs::String>("/block_id", 1);
    // publisher for the location where gaze hits the table
    ros::Publisher tablehit_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/gaze_table_hit_location", 1);
    ros::Publisher tablehit_pub_logging = nh.advertise<geometry_msgs::Vector3Stamped>("/gaze_table_hit_location_logging", 1);

    colorClass.mode_ = std::atoi(argv[1]); // mode 0 for gaze, 1 for color
    sync.registerCallback(boost::bind(&callback, _1, _2, blockname_pub, tablehit_pub, tablehit_pub_logging, &tfBuffer, &colorClass.color_, colorClass.mode_));


    ros::spin();

    return 0;
}
