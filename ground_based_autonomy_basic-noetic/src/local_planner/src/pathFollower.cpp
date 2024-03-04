#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <geometry_msgs/msg/twist_stamped.hpp> // Updated
#include <nav_msgs/msg/odometry.hpp>           // Updated
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp> // Updated
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp> // Updated
#include <std_msgs/msg/int8.hpp>

#include <tf2/LinearMath/Matrix3x3.h>      // Potential replacement
#include <tf2/LinearMath/Quaternion.h>     // Potential replacement
#include <tf2/LinearMath/Transform.h>      // Potential replacement
#include <tf2_ros/transform_broadcaster.h> // Potential replacement

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

const double PI = 3.1415926;

rclcpp::Clock::SharedPtr rclcpp_clock;

double sensorOffsetX = 0;
double sensorOffsetY = 0;
int pubSkipNum = 1;
int pubSkipCount = 0;
bool twoWayDrive = true;
double lookAheadDis = 0.5;
double yawRateGain = 7.5;
double stopYawRateGain = 7.5;
double maxYawRate = 45.0;
double maxSpeed = 1.0;
double maxAccel = 1.0;
double switchTimeThre = 1.0;
double dirDiffThre = 0.1;
double stopDisThre = 0.2;
double slowDwnDisThre = 1.0;
bool useInclRateToSlow = false;
double inclRateThre = 120.0;
double slowRate1 = 0.25;
double slowRate2 = 0.5;
double slowTime1 = 2.0;
double slowTime2 = 2.0;
bool useInclToStop = false;
double inclThre = 45.0;
double stopTime = 5.0;
bool noRotAtStop = false;
bool noRotAtGoal = true;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyYaw = 0;
int safetyStop = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleZRec = 0;
float vehicleRollRec = 0;
float vehiclePitchRec = 0;
float vehicleYawRec = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

double odomTime = 0;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = false;
int pathPointID = 0;
bool pathInit = false;
bool navFwd = true;
double switchTime = 0;

nav_msgs::msg::Path path;

#define stampToSec(stamp) (stamp.sec + stamp.nanosec / 1000000000.0)

void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odomIn) {
  odomTime = stampToSec(odomIn->header.stamp);

  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
      .getRPY(roll, pitch, yaw);
  // tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
  //     .getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX +
             sin(yaw) * sensorOffsetY;
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX -
             cos(yaw) * sensorOffsetY;
  vehicleZ = odomIn->pose.pose.position.z;

  if ((fabs(roll) > inclThre * PI / 180.0 ||
       fabs(pitch) > inclThre * PI / 180.0) &&
      useInclToStop) {
    stopInitTime = stampToSec(odomIn->header.stamp);
  }

  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 ||
       fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) &&
      useInclRateToSlow) {
    slowInitTime = stampToSec(odomIn->header.stamp);
  }
}

void pathHandler(const nav_msgs::msg::Path::ConstSharedPtr pathIn) {
  int pathSize = pathIn->poses.size();
  path.poses.resize(pathSize);
  for (int i = 0; i < pathSize; i++) {
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
  }

  vehicleXRec = vehicleX;
  vehicleYRec = vehicleY;
  vehicleZRec = vehicleZ;
  vehicleRollRec = vehicleRoll;
  vehiclePitchRec = vehiclePitch;
  vehicleYawRec = vehicleYaw;

  pathPointID = 0;
  pathInit = true;
}

void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy) {
  joyTime = rclcpp_clock->now().seconds();

  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0)
    joySpeed = 1.0;
  if (joy->axes[4] == 0)
    joySpeed = 0;
  joyYaw = joy->axes[3];
  if (joySpeed == 0 && noRotAtStop)
    joyYaw = 0;

  if (joy->axes[4] < 0 && !twoWayDrive) {
    joySpeed = 0;
    joyYaw = 0;
  }

  if (joy->axes[2] > -0.1) {
    autonomyMode = false;
  } else {
    autonomyMode = true;
  }
}

void speedHandler(const std_msgs::msg::Float32::ConstSharedPtr speed) {
  double speedTime = rclcpp_clock->now().seconds();

  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay &&
      joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0)
      joySpeed = 0;
    else if (joySpeed > 1.0)
      joySpeed = 1.0;
  }
}

void stopHandler(const std_msgs::msg::Int8::ConstSharedPtr stop) {
  safetyStop = stop->data;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("pathFollower");
  rclcpp_clock = nh->get_clock();

  nh->declare_parameter("sensorOffsetX", 0.0);
  nh->declare_parameter("sensorOffsetY", 0.0);
  nh->declare_parameter("pubSkipNum", 1);
  nh->declare_parameter("twoWayDrive", true);
  nh->declare_parameter("lookAheadDis", 0.5);
  nh->declare_parameter("yawRateGain", 7.5);
  nh->declare_parameter("stopYawRateGain", 7.5);
  nh->declare_parameter("maxYawRate", 45.0);
  nh->declare_parameter("maxSpeed", 1.0);
  nh->declare_parameter("maxAccel", 1.0);
  nh->declare_parameter("switchTimeThre", 1.0);
  nh->declare_parameter("dirDiffThre", 0.1);
  nh->declare_parameter("stopDisThre", 0.2);
  nh->declare_parameter("slowDwnDisThre", 1.0);
  nh->declare_parameter("useInclRateToSlow", false);
  nh->declare_parameter("inclRateThre", 120.0);
  nh->declare_parameter("slowRate1", 0.25);
  nh->declare_parameter("slowRate2", 0.5);
  nh->declare_parameter("slowTime1", 2.0);
  nh->declare_parameter("slowTime2", 2.0);
  nh->declare_parameter("useInclToStop", false);
  nh->declare_parameter("inclThre", 45.0);
  nh->declare_parameter("stopTime", 5.0);
  nh->declare_parameter("noRotAtStop", false);
  nh->declare_parameter("noRotAtGoal", true);
  nh->declare_parameter("autonomyMode", false);
  nh->declare_parameter("autonomySpeed", 1.0);
  nh->declare_parameter("joyToSpeedDelay", 2.0);

  nh->get_parameter("sensorOffsetX", sensorOffsetX);
  nh->get_parameter("sensorOffsetY", sensorOffsetY);
  nh->get_parameter("pubSkipNum", pubSkipNum);
  nh->get_parameter("twoWayDrive", twoWayDrive);
  nh->get_parameter("lookAheadDis", lookAheadDis);
  nh->get_parameter("yawRateGain", yawRateGain);
  nh->get_parameter("stopYawRateGain", stopYawRateGain);
  nh->get_parameter("maxYawRate", maxYawRate);
  nh->get_parameter("maxSpeed", maxSpeed);
  nh->get_parameter("maxAccel", maxAccel);
  nh->get_parameter("switchTimeThre", switchTimeThre);
  nh->get_parameter("dirDiffThre", dirDiffThre);
  nh->get_parameter("stopDisThre", stopDisThre);
  nh->get_parameter("slowDwnDisThre", slowDwnDisThre);
  nh->get_parameter("useInclRateToSlow", useInclRateToSlow);
  nh->get_parameter("inclRateThre", inclRateThre);
  nh->get_parameter("slowRate1", slowRate1);
  nh->get_parameter("slowRate2", slowRate2);
  nh->get_parameter("slowTime1", slowTime1);
  nh->get_parameter("slowTime2", slowTime2);
  nh->get_parameter("useInclToStop", useInclToStop);
  nh->get_parameter("inclThre", inclThre);
  nh->get_parameter("stopTime", stopTime);
  nh->get_parameter("noRotAtStop", noRotAtStop);
  nh->get_parameter("noRotAtGoal", noRotAtGoal);
  nh->get_parameter("autonomyMode", autonomyMode);
  nh->get_parameter("autonomySpeed", autonomySpeed);
  nh->get_parameter("joyToSpeedDelay", joyToSpeedDelay);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(5));

  auto subOdom = nh->create_subscription<nav_msgs::msg::Odometry>(
      "/state_estimation", qos, odomHandler);

  auto subPath =
      nh->create_subscription<nav_msgs::msg::Path>("/path", qos, pathHandler);

  auto subJoystick = nh->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", qos, joystickHandler);

  auto subSpeed = nh->create_subscription<std_msgs::msg::Float32>("/speed", qos,
                                                                  speedHandler);

  auto subStop =
      nh->create_subscription<std_msgs::msg::Int8>("/stop", qos, stopHandler);

  auto pubSpeed =
      nh->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", qos);
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "vehicle";

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0)
      joySpeed = 0;
    else if (joySpeed > 1.0)
      joySpeed = 1.0;
  }

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    if (pathInit) {
      float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) +
                          sin(vehicleYawRec) * (vehicleY - vehicleYRec);
      float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) +
                          cos(vehicleYawRec) * (vehicleY - vehicleYRec);

      int pathSize = path.poses.size();
      float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
      float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
      float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

      float disX, disY, dis;
      while (pathPointID < pathSize - 1) {
        disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
        disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
        dis = sqrt(disX * disX + disY * disY);
        if (dis < lookAheadDis) {
          pathPointID++;
        } else {
          break;
        }
      }

      disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
      disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
      dis = sqrt(disX * disX + disY * disY);
      float pathDir = atan2(disY, disX);

      float dirDiff = vehicleYaw - vehicleYawRec - pathDir;
      if (dirDiff > PI)
        dirDiff -= 2 * PI;
      else if (dirDiff < -PI)
        dirDiff += 2 * PI;
      if (dirDiff > PI)
        dirDiff -= 2 * PI;
      else if (dirDiff < -PI)
        dirDiff += 2 * PI;

      if (twoWayDrive) {
        double time = rclcpp_clock->now().seconds();
        if (fabs(dirDiff) > PI / 2 && navFwd &&
            time - switchTime > switchTimeThre) {
          navFwd = false;
          switchTime = time;
        } else if (fabs(dirDiff) < PI / 2 && !navFwd &&
                   time - switchTime > switchTimeThre) {
          navFwd = true;
          switchTime = time;
        }
      }

      float joySpeed2 = maxSpeed * joySpeed;
      if (!navFwd) {
        dirDiff += PI;
        if (dirDiff > PI)
          dirDiff -= 2 * PI;
        joySpeed2 *= -1;
      }

      if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0)
        vehicleYawRate = -stopYawRateGain * dirDiff;
      else
        vehicleYawRate = -yawRateGain * dirDiff;

      if (vehicleYawRate > maxYawRate * PI / 180.0)
        vehicleYawRate = maxYawRate * PI / 180.0;
      else if (vehicleYawRate < -maxYawRate * PI / 180.0)
        vehicleYawRate = -maxYawRate * PI / 180.0;

      if (joySpeed2 == 0 && !autonomyMode) {
        vehicleYawRate = maxYawRate * joyYaw * PI / 180.0;
      } else if (pathSize <= 1 || (dis < stopDisThre && noRotAtGoal)) {
        vehicleYawRate = 0;
      }

      if (pathSize <= 1) {
        joySpeed2 = 0;
      } else if (endDis / slowDwnDisThre < joySpeed) {
        joySpeed2 *= endDis / slowDwnDisThre;
      }

      float joySpeed3 = joySpeed2;
      if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0)
        joySpeed3 *= slowRate1;
      else if (odomTime < slowInitTime + slowTime1 + slowTime2 &&
               slowInitTime > 0)
        joySpeed3 *= slowRate2;

      if (fabs(dirDiff) < dirDiffThre && dis > stopDisThre) {
        if (vehicleSpeed < joySpeed3)
          vehicleSpeed += maxAccel / 100.0;
        else if (vehicleSpeed > joySpeed3)
          vehicleSpeed -= maxAccel / 100.0;
      } else {
        if (vehicleSpeed > 0)
          vehicleSpeed -= maxAccel / 100.0;
        else if (vehicleSpeed < 0)
          vehicleSpeed += maxAccel / 100.0;
      }

      if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
        vehicleSpeed = 0;
        vehicleYawRate = 0;
      }

      if (safetyStop >= 1)
        vehicleSpeed = 0;
      if (safetyStop >= 2)
        vehicleYawRate = 0;

      pubSkipCount--;
      if (pubSkipCount < 0) {
        cmd_vel.header.stamp =
            rclcpp::Time(static_cast<int64_t>(odomTime * 1e9));
        if (fabs(vehicleSpeed) <= maxAccel / 100.0)
          cmd_vel.twist.linear.x = 0;
        else
          cmd_vel.twist.linear.x = vehicleSpeed;
        cmd_vel.twist.angular.z = vehicleYawRate;
        pubSpeed->publish(cmd_vel);

        pubSkipCount = pubSkipNum;
      }
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
