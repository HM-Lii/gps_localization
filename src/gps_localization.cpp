#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <mqtt_com/mqtt_com.h>

using namespace GeographicLib;

class GpsLocalization {
public:
 GpsLocalization()
     : is_first_fix(true),
       gps_heading(0),
       enu_heading(0),
       body_dx(0),
       body_dy(0),
       earth_model(Constants::WGS84_a(), Constants::WGS84_f())  
       {
   MqttCom_ = std::make_unique<MqttCom>(nh);
   nh.param("gps_localization/body_dx", body_dx, 0.0);
   nh.param("/gps_localization/body_dy", body_dy, 0.0);

   fix_subscriber =
       nh.subscribe("/fix", 10, &GpsLocalization::fixCallback, this);
   heading_subscriber =
       nh.subscribe("/heading", 10, &GpsLocalization::headingCallback, this);
   vel_subscriber =
       nh.subscribe("/vel", 10, &GpsLocalization::velCallback, this);
   work_publisher = nh.advertise<std_msgs::Int8>("/cmd_work", 10);
   odometry_publisher = nh.advertise<nav_msgs::Odometry>("/Odometry", 10);
   path_publisher = nh.advertise<nav_msgs::Path>("/path", 10);

   path.header.frame_id = "camera_init";
 }

    void spin() {
        ros::AsyncSpinner spinner(2);  
        spinner.start(); 
        ros::Rate rate(1);
        while(ros::ok())
        {  
            path_publisher.publish(path);
            // MqttCom_->publish("gecao/upload",latitude,longitude,vel,100);
            rate.sleep();
        }
        spinner.stop();
    }

private:
    bool is_first_fix;
    int wait_count=0;
    double gps_heading, enu_heading ,latitude,longitude,vel,body_dx, body_dy;
    Geocentric earth_model;  
    LocalCartesian local_projector; 
    nav_msgs::Path path; 
    ros::Publisher odometry_publisher, path_publisher,work_publisher;  
    ros::Subscriber fix_subscriber, heading_subscriber,vel_subscriber;  
    std::unique_ptr<MqttCom> MqttCom_;
    ros::NodeHandle nh;
    void headingCallback(const geometry_msgs::Pose2DPtr &msg) {
        gps_heading = msg->theta;
        enu_heading = 90 - gps_heading;
        if (enu_heading < 0) {
            enu_heading += 360;
        }
    }

    void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
      if (is_first_fix) {
        if (wait_count != 10) {
          wait_count++;
          return;
        }
        local_projector.Reset(msg->latitude, msg->longitude, msg->altitude);
        is_first_fix = false;
      }
        updateMqttMsg(msg);
        double x, y, z;
        local_projector.Forward(msg->latitude, msg->longitude, msg->altitude, x, y, z);
        tf2::Transform transform = getTransform();
        tf2::Vector3 enu_offset = getEnuOffset(transform);
        x += enu_offset.getX();
        y += enu_offset.getY();
        publishOdometry(x, y, z);
    }

    void velCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        updateVelocity(msg);
    }

    void updateMqttMsg(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        static int j=10;
        if(j==10){
            j=0;
            latitude=msg->latitude;
            longitude=msg->longitude;  
        }
        j++;
    }

    tf2::Transform getTransform() {
        tf2::Transform transform;
        transform.setOrigin(tf2::Vector3(0, 0, 0));
        tf2::Quaternion quaternion_;
        quaternion_.setRPY(0, 0, enu_heading * M_PI / 180.0);
        transform.setRotation(quaternion_);
        return transform;
    }

    tf2::Vector3 getEnuOffset(tf2::Transform &transform) {
        tf2::Vector3 body_offset(body_dx, body_dy, 0.0);
        return transform * body_offset;
    }

    void publishOdometry(double x, double y, double z) {
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, enu_heading * M_PI / 180.0);
        pose.orientation.x = quaternion.x();
        pose.orientation.y = quaternion.y();
        pose.orientation.z = quaternion.z();
        pose.orientation.w = quaternion.w();
        nav_msgs::Odometry odometry;    
        odometry.pose.pose = pose;
        odometry.header.stamp = ros::Time::now();
        odometry.header.frame_id = "camera_init";
        odometry_publisher.publish(odometry);
        publishPath(pose);
    }

    void publishPath(geometry_msgs::Pose &pose) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose = pose;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "camera_init";
        path.poses.push_back(pose_stamped);
    }

    void updateVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        static int i=10;
        if(i==10){
            i=0;
            vel = sqrt(pow(msg->twist.linear.x,2)+pow(msg->twist.linear.y,2));  
        }
        i++;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_localization");
    GpsLocalization gpsLocalization;
    gpsLocalization.spin();
    return 0;
}
