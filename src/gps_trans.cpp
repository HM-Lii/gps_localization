#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Path.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <tf2/LinearMath/Quaternion.h>
using namespace std;
using namespace GeographicLib;
struct fixPoint
{
    double latitude;
    double longitude;
    double altitude;/* data */
    fixPoint(double latitude_,double longitude_,double altitude_):latitude(latitude_),longitude(longitude_),altitude(altitude_){};
};

Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());
bool first = true;
double heading;
nav_msgs::Path path;
geometry_msgs::Pose gps_trans(fixPoint fix_,double heading){
    LocalCartesian proj;  
    if (first) {
        proj.Reset(fix_.latitude, fix_.longitude, fix_.altitude);
        first = false;
    }  
    // 转换为当地坐标系中的坐标
    double x, y, z;
    proj.Forward(fix_.latitude, fix_.latitude, 0, x, y,z);
    geometry_msgs::Pose pose_;
    pose_.position.x=x;
    pose_.position.y=y;
    pose_.position.z=z;
    double headingEnu = 90 - heading;
    if (headingEnu < 0) {
        headingEnu += 360;
    }
    tf2::Quaternion q;
    q.setRPY(0, 0, headingEnu * M_PI / 180.0);
    pose_.orientation.x = q.x();
    pose_.orientation.y = q.y();
    pose_.orientation.z = q.z();
    pose_.orientation.w = q.w();

    return pose_;
}
void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    fixPoint fix_(msg->latitude,msg->longitude,msg->altitude);
    geometry_msgs::PoseStamped pose_;
    pose_.pose=gps_trans(fix_,heading);
    path.poses.push_back(pose_);
}

void headingCallback(const geometry_msgs::Pose2D &msg) {
    heading=msg.theta;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_localization");
    ros::NodeHandle nh;
    ros::Subscriber fix_sub = nh.subscribe("/fix", 10, fixCallback);
    ros::Subscriber heading_sub = nh.subscribe("/heading", 10, headingCallback);
    path.header.frame_id="camera_init";
    path.header.stamp=ros::Time::now();
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/gps_path", 10);
while(ros::ok()){
    path_pub.publish(path);
    ros::spinOnce();
}
    return 0;
}
class GpsLocalization {
public:
  GpsLocalization() {
    // 订阅GPS和航向信息
    fix_sub = nh.subscribe("/gps", 1, &GpsLocalization::fixCallback, this);
    heading_sub = nh.subscribe("/heading", 1, &GpsLocalization::headingCallback, this);

    // 初始化GPS转换器
    geocentric = Geocentric(Constants::WGS84_a(), Constants::WGS84_f());
    
    local_cartesian = GeographicLib::LocalCartesian(lat_origin, lon_origin, alt_origin, geocentric);
    
    // 初始化路径发布器
    path_pub = nh.advertise<nav_msgs::Path>("/gps_path", 1);
  }

  void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    // 获取GPS信息
    if(first){

    }
    fixPoint fix = {msg->latitude, msg->longitude, msg->altitude};

    // 将GPS点转换为本地坐标系中的点
    Eigen::Vector3d local_point;
    geodeticToEnu(fix, local_point);

    // 将点和航向信息组合成位姿信息
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "camera_init";
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.pose = gpsTrans(local_point, heading);

    // 将位姿信息加入路径中
    path.push_back(pose_stamped);
  }

  void headingCallback(const geometry_msgs::Pose2D &msg) {
    // 获取航向信息
    heading = msg.theta;
  }

  void publishPath() {
    // 发布路径信息
    path_msg.header.frame_id = "camera_init";
    path_msg.header.stamp = ros::Time::now();
    path_msg.poses = path;
    path_pub.publish(path_msg);
  }

private:
  // 坐标系转换器
  void geodeticToEnu(fixPoint fix, Eigen::Vector3d &enu_point) {
    double x, y, z;
    geocentric.Forward(fix.lat, fix.lon, fix.alt, x, y, z);
    local_cartesian.Forward(fix.lat, fix.lon, fix.alt, enu_point.x(), enu_point.y(), enu_point.z());
    enu_point -= Eigen::Vector3d(x, y, z);
  }

  // GPS点转换为位姿信息
  geometry_msgs::Pose gpsTrans(Eigen::Vector3d point, double heading) {
    geometry_msgs::Pose pose;

    // 设置位置
    pose.position.x = point.x();
    pose.position.y = point.y();
    pose.position.z = point.z();

    // 设置方向
    Eigen::Quaterniond q = Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ());
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();

    return pose;
  }

  // ROS节点和话题
  ros::NodeHandle nh;
  ros::Subscriber fix_sub;
  ros::Subscriber heading_sub;
  ros::Publisher path_pub;

  // GPS转换器
  GeographicLib::Geocentric geocentric;
  GeographicLib::LocalCartesian local_cartesian;

  // 路径和位姿信息
  nav_msgs::Path path_msg;
  std::vector<geometry_msgs::PoseStamped> path;
  double heading = 0.0;
  double lat_origin = 0.0, lon_origin = 0.0, alt_origin = 0.0;
  bool first=true;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gps_localization");
  GpsLocalization gpsLocalization;

  while (ros::ok()) {
    gpsLocalization.publishPath();
    ros::spinOnce();
  }
  
  return 0;
}