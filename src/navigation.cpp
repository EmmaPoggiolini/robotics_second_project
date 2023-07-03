#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <stdio.h>
#include <tf/transform_broadcaster.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/Quaternion.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

class Navigation {
   private:
    ros::NodeHandle n;
    ros::Subscriber goal_sub;
    // csv file
    std::fstream file;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;

    bool read_line() {
        std::string line;
        if (std::getline(Navigation::file, line)) {
            std::istringstream iss(line);
            std::string token;
            std::vector<double> values;

            while (std::getline(iss, token, ',')) {
                double value;
                std::istringstream token_ss(token);
                token_ss >> value;
                values.push_back(value);
            }

            move_base_msgs::MoveBaseGoal goal;

            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.pose.position.x = values[0];
            goal.target_pose.pose.position.y = values[1];
            goal.target_pose.pose.position.z = 0.0;
            goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(values[2]);
            ac.sendGoal(goal);
            ROS_INFO("New goal published: (%f, %f, %f)", values[0], values[1], values[2]);
            return true;
        }
        return false;
    }

    void goal_callback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg) {
        if (msg->status.status == 3) {
            ROS_INFO("Goal reached!");
            if (!read_line()) {
                ROS_INFO("No more goals!");
                file.close();
            }
        } else {
            ROS_INFO("Goal not reached!");
        }
    }

   public:
    Navigation() {
        file = std::fstream("/root/robotics/src/robotics_second_project/waypoints.csv", std::ios::in);
        if (!file.is_open()) {
            ROS_ERROR_STREAM("Failed to open CSV file.");
        }

        ac = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true);

        ROS_INFO("Initializing navigation node...");
        goal_sub = n.subscribe("/move_base/result", 1000, &Navigation::goal_callback, this);
        read_line();
        ROS_INFO("Navigation node initialized!");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation");
    Navigation navigation;
    ros::spin();
    return 0;
}