#include <tf/transform_broadcaster.h>

#include <sstream>

#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

class TFPublisher {
   private:
    ros::NodeHandle n;

    tf::TransformBroadcaster br;
    ros::Subscriber odom_sub;

    void tf_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        // TF
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
    }

   public:
    TFPublisher() {
        odom_sub = n.subscribe("/t265/odom", 1000, &TFPublisher::tf_callback, this);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_publisher");
    TFPublisher tf_publisher;
    ros::spin();
    return 0;
}
