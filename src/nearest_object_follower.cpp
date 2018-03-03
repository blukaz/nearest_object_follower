/*

	Algorithm for nearest object follower of PCL data from Zed camera

*/



#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
// PCL specific includes
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <iterator>
#include <math.h>

#define PI 3.14159265

using namespace std;

//*** function declaration ***
void filter_pcl(float z_min, float z_max, const pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in);

void callback(const sensor_msgs::PointCloud2 &pcl);
//****************************

double min_z = 1000;		// set a great value in for minimum search
double avg_z = 0;		// average distance in z direction to the segmented object
double avg_x = 0;		// average distance in x direction to the segmented object: for ANGULAR VEL
double thres_dist = 1.0;	// threshold distance to segmented object: ADJUSTABLE

// ros publishers
ros::Publisher pub_nearest_object;
ros::Publisher cmd_vel_pub;

// point clouds
pcl::PointCloud<pcl::PointXYZ> cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);

// velocity msg
geometry_msgs::Twist turtle_cmd;


int main (int argc, char** argv)
{

	// Initialize ROS
	ros::init (argc, argv, "pcl_compare");

	ros::NodeHandle nh;

	// SUBSCRIBE here to zed camera depth points * or something similar, e.g. pointcloud
	ros::Subscriber sub = nh.subscribe ("/camera/depth/points" , 5, callback);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_nearest_object (new pcl::PointCloud<pcl::PointXYZ>);
	pcl_nearest_object->header.frame_id = "camera_depth_optical_frame";

	pcl::PointXYZ point;

	pub_nearest_object = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("pub_nearest_object", 1000000);
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);


	ROS_INFO("Start nearest object segmentation!");

	while(ros::ok()){


		ros::spinOnce();
		
		if (cloud.points.size() == 0)
			continue;		

		// find the closest point
		for ( int i = 0; i < cloud.points.size(); i++)
		{
			point.x = cloud.points[i].z;
			if (point.z < min_z)
			{
				min_z = point.z;
			}
		}
		

		// filter out the nearest points surrounding the closest point in x direction 
		filter_pcl(min_z, min_z + 0.1, in_cloud);


		// it takes a few ms at the beginning to get the topics in sync and to listen to each other, thus this has a effect as a delay 
		if (in_cloud->points.size() == 0)
			continue;
		

		// filter out outliers using ransac
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (0.01);
		// Segmentation
		seg.setInputCloud (in_cloud);
		seg.segment (*inliers, *coefficients);

		if (inliers->indices.size () == 0)
		{
			PCL_ERROR ("Could not estimate a planar model for the given dataset.");
			continue;
		}

		//std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

		for (size_t i = 0; i < inliers->indices.size (); ++i){
			
			avg_z += in_cloud->points[inliers->indices[i]].z;
			avg_x += in_cloud->points[inliers->indices[i]].x;
			pcl_nearest_object->points.push_back(in_cloud->points[inliers->indices[i]]);

		}
		   
		
		avg_z /= inliers->indices.size (); 	// calculated average in z direction
		avg_x /= inliers->indices.size (); 	// calculated average in x direction
		
		// *** CMD_VEL CALCULATION ***
		// calculate linear velocity in z direction
		turtle_cmd.linear.x = (avg_z - thres_dist);

		// calculate angular velocity in order to align with object
		if (avg_x > -0.1 and avg_x < 0.1)
		{
			turtle_cmd.angular.z = 0;
			ROS_INFO("In dead zone!");
		}
		else
		{	
			if (avg_x < 0)
				turtle_cmd.angular.z =  - PI/10;	// I set angluar velocity to 0.314 rad, but we can make it adaptable (which is always better)
			else
				turtle_cmd.angular.z =  + PI/10;
		}	
		cmd_vel_pub.publish(turtle_cmd);
		// ***************************

		// clear up the point cloud memory in order to free it up for the next iteration
		in_cloud->points.clear();
		pub_nearest_object.publish (pcl_nearest_object);
		pcl_nearest_object->points.clear();
	}


	ROS_INFO("Nearest object segmentation ended!");
	
	return 0;

}



void callback(const sensor_msgs::PointCloud2 &pcl){

	//ROS_INFO("I heard some pcl data ...");
	pcl::fromROSMsg(pcl, cloud);

}


void filter_pcl(float z_min, float z_max, const pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// Passthrough filter
	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (pcl_in);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (z_min, z_max);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filtered);

	//cout << "Number of pcl points after PASSTHROUGH filter: " << cloud_filtered->points.size() << endl;

	
	pcl_in->points.clear();

	for ( int i = 0; i < cloud_filtered->points.size(); i++)
	{
		pcl_in->points.push_back(cloud_filtered->points[i]);
	}
}

