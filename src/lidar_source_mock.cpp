// lidar_source_mock.cpp: Тестовый узел для генерации сообщений лидара

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
//#include <tf2/Transform.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "lidar_source_mock");
  ROS_INFO("Started node lidar_source_mock.");

  ros::NodeHandle n;
  tf::TransformBroadcaster br;  // для публикации системы координат
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan_0", 50);

  unsigned int num_readings = 360;
  double laser_frequency = 10;
  double ranges[num_readings];
  double intensities[num_readings];

  int count = 0;
  ros::Rate r(10.0);
  while(n.ok()){
    //generate some fake data for our laser scan
    for(unsigned int i = 0; i < num_readings; ++i){
      ranges[i] = 2.0;
      intensities[i] = 1.0; //100 + count;
    }
    ros::Time scan_time = ros::Time::now();

    //populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "world";
    scan.angle_min = - M_PI;
    scan.angle_max = + M_PI;
    scan.angle_increment = 2 * M_PI / num_readings;
    scan.time_increment = (1 / laser_frequency) / (num_readings);
    scan.range_min = 0.0;
    scan.range_max = 100.0;

    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);
    for(unsigned int i = 0; i < num_readings; ++i){
      scan.ranges[i] = ranges[i];
      scan.intensities[i] = i;
    }

    // Публикуем систему координат
    tf::Transform new_transform = tf::Transform::getIdentity();
    br.sendTransform(tf::StampedTransform(
        new_transform, ros::Time::now(), "world", "ROS"));
    // Публикуем лазер скан от лидара
    scan_pub.publish(scan);
    ++count;
    r.sleep();
  }
}



