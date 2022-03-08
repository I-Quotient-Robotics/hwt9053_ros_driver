#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <boost/format.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf/LinearMath/Quaternion.h>

#include "modbus_rtu_master.h"

#define G 9.801

class HWT9053Driver
{
public:
  HWT9053Driver(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~HWT9053Driver();
  void loop();

private:
  ros::NodeHandle nh_, private_nh_;
  sensor_msgs::Imu imu_msg_;
  sensor_msgs::MagneticField mag_msg_;
  ros::Publisher imu_msg_pub_;
  ros::Publisher mag_msg_pub_;
  bool pub_mag_;
  int32_t baudrate_;
  std::string frame_id_;
  std::string sensor_com_;
  ModbusRTUMaster *sensor_;
};

HWT9053Driver::HWT9053Driver(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh), private_nh_(private_nh)
{

  private_nh_.param<int>("baudrate", baudrate_, 115200);
  private_nh_.param<bool>("pub_mag_msg", pub_mag_, false);
  private_nh_.param<std::string>("frame_id", frame_id_, "imu_link");
  private_nh_.param<std::string>("sensor_com", sensor_com_, "/dev/ttyUSB0");

  sensor_ = new ModbusRTUMaster(sensor_com_, baudrate_);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  imu_msg_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 10);
  if (pub_mag_)
  {
    mag_msg_pub_ = nh_.advertise<sensor_msgs::MagneticField>("imu/mag", 10);
  }
}

HWT9053Driver::~HWT9053Driver()
{
  free(sensor_);
  sensor_ = nullptr;
}

void HWT9053Driver::loop()
{
  uint16_t data[256] = {0};
  uint8_t ret = 0;

  // get Atmospheric humidity and temperature
  ret = sensor_->GetMultipleRegisters(0x50, 0x0034, 0x0010, data, 1);
  if (!ret)
    ROS_WARN("get Atmospheric humidity&temperature faild!!");
  else
  {
    //IMU
    imu_msg_.header.stamp = ros::Time::now();
    imu_msg_.header.frame_id = frame_id_;

    imu_msg_.linear_acceleration.x = int16_t(data[0]) / 32768.0 * 16.0 * G;
    imu_msg_.linear_acceleration.y = int16_t(data[1]) / 32768.0 * 16.0 * G;
    imu_msg_.linear_acceleration.z = int16_t(data[2]) / 32768.0 * 16.0 * G;
    imu_msg_.linear_acceleration_covariance[0] = 0;

    imu_msg_.angular_velocity.x = int16_t(data[3]) / 32768.0 * 2000.0;
    imu_msg_.angular_velocity.y = int16_t(data[4]) / 32768.0 * 2000.0;
    imu_msg_.angular_velocity.z = int16_t(data[5]) / 32768.0 * 2000.0;
    imu_msg_.angular_velocity_covariance[0] = 0;

    double ang_x = int32_t((data[10] << 16) | data[9]) / 1000.0;
    double ang_y = int32_t((data[12] << 16) | data[11]) / 1000.0;
    double ang_z = int32_t((data[14] << 16) | data[13]) / 1000.0;

    tf::Quaternion quate;
    quate.setRPY(ang_x, ang_y, ang_z);
    imu_msg_.orientation.x = quate.x();
    imu_msg_.orientation.y = quate.y();
    imu_msg_.orientation.z = quate.z();
    imu_msg_.orientation.w = quate.w();
    imu_msg_.orientation_covariance[0] = 0;

    imu_msg_pub_.publish(imu_msg_);

    //Magnetic Field
    if (pub_mag_)
    {
      mag_msg_.header.stamp = ros::Time::now();
      mag_msg_.header.frame_id = frame_id_;

      mag_msg_.magnetic_field.x = int16_t(data[6]);
      mag_msg_.magnetic_field.y = int16_t(data[7]);
      mag_msg_.magnetic_field.z = int16_t(data[8]);

      mag_msg_.magnetic_field_covariance[0] = 0;

      mag_msg_pub_.publish(mag_msg_);
    }
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "hwt9053_driver_node");
  ros::NodeHandle nh, private_nh("~");

  int publish_rate;
  private_nh.param<int>("publish_rate", publish_rate, 100);

  HWT9053Driver imu(nh, private_nh);

  ros::Rate r(publish_rate);

  while (ros::ok())
  {
    ros::spinOnce();
    imu.loop();
    r.sleep();
  }

  ROS_INFO("All finish");

  return 0;
}