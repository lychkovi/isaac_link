// SlamData: Класс для публикации топиков с данными от Elbrus SLAM

#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>

//#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>  
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>


class SlamData
{
private:

    tf::TransformBroadcaster br;
    ros::Publisher pose_pub;
    ros::Publisher all_point_cloud_pub;
    ros::Publisher ref_point_cloud_pub;  
    image_transport::Publisher current_frame_pub;

void GetROSPointCloud(
    std::vector<tf::Vector3>& pts, 
    ros::Time timestamp, 
    bool asRefPts,
    sensor_msgs::PointCloud2 &all_point_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_all( 
        new pcl::PointCloud<pcl::PointXYZRGBA> );
	
    size_t npts = pts.size();
    cloud_all->points.reserve(npts);   
    for(size_t i = 0; i < npts; i++)
    {
        pcl::PointXYZRGBA p;	
        p.x = pts[i].x();
        p.y = pts[i].y();
        p.z = pts[i].z();
        if (asRefPts)
        {
            p.b = 0;
            p.g = 0;
            p.r = 255;
            p.a = 255;
        }
        else
        {
            p.b = 255;
            p.g = 255;
            p.r = 255;
            p.a = 255;
        }
        cloud_all->points.push_back( p );
    }
    pcl::PCLPointCloud2 pcl_pc1;
    // 1) pcl::PointXYZRGBA -> pcl::PCLPointCloud2
    pcl::toPCLPointCloud2(*cloud_all, pcl_pc1);    
    // 2) pcl::PCLPointCloud2 -> sensor_msgs::PointCloud2
    pcl_conversions::fromPCL(pcl_pc1, all_point_cloud);  
    all_point_cloud.header.frame_id = "world";  
    all_point_cloud.header.stamp = timestamp;   
}

public:

SlamData(ros::NodeHandle *nodeHandler)
{
    pose_pub = (*nodeHandler).advertise<geometry_msgs::PoseStamped>(
        "posestamped", 1000);
    ROS_INFO("Published topic /posestamped of type Pose!");
 
    all_point_cloud_pub = (*nodeHandler).advertise<sensor_msgs::PointCloud2>(
        "point_cloud_all", 1);
    ROS_INFO("Published topic /point_cloud_all of type PointCloud2!");

    ref_point_cloud_pub = (*nodeHandler).advertise<sensor_msgs::PointCloud2>(
        "point_cloud_ref", 1);

    image_transport::ImageTransport it_((*nodeHandler));
    current_frame_pub = it_.advertise("/current_frame", 1);

    ROS_INFO("Select world frame in RViz to view the topic data!");
}

void PublishTFForROS(
    tf::Vector3 origin, 
    tf::Quaternion rotation, 
    ros::Time timestamp)
{
    // Публикуем систему координат
    tf::Transform new_transform = tf::Transform::getIdentity();
    //tf::Transform new_transform;
    //new_transform.setOrigin(origin);
    //new_transform.setRotation(rotation);
    br.sendTransform(tf::StampedTransform(
        new_transform, timestamp, "world", "ISAAC"));
}

void PublishPoseForROS(
    tf::Vector3 origin, 
    tf::Quaternion rotation, 
    ros::Time timestamp)
{
    tf::Transform new_transform;
    new_transform.setOrigin(origin);
    new_transform.setRotation(rotation);

    // Публикуем расположение камеры в пространстве
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = timestamp;
    pose.header.frame_id ="world";
    tf::poseTFToMsg(new_transform, pose.pose);
    pose_pub.publish(pose);
}

void PublishPointCloudForROS(
    std::vector<tf::Vector3>& allpts,
    std::vector<tf::Vector3>& refpts, 
    ros::Time timestamp)
{
    sensor_msgs::PointCloud2 allMapPoints;
    sensor_msgs::PointCloud2 refMapPoints;
    GetROSPointCloud(allpts, timestamp, false, allMapPoints);
    GetROSPointCloud(refpts, timestamp, true,  refMapPoints);
    all_point_cloud_pub.publish(allMapPoints);
    ref_point_cloud_pub.publish(refMapPoints);
}


}; /* SlamData */



