#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <mcl_msgs/PointCloud2Array.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/filter.h>

ros::Publisher _pub_filtered;
ros::Publisher _pub_drone_pos;
ros::Subscriber _sub_cloud;


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg (*msg, *cloud);

	// remove NAN points from the cloud
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);

	// downsample cloud
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud (cloud);
	vg.setLeafSize (0.01f, 0.01f, 0.01f);
	vg.filter (*cloud_filtered);

	// create quadratic conditional to find drone
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
	Eigen::Matrix3f comparison_matrix = Eigen::Matrix3f::Zero();
	comparison_matrix(0,0) = 1;
	comparison_matrix(1,1) = 1;
	comparison_matrix(2,2) = 1;
	Eigen::Vector3f comparison_vector;
	comparison_vector(0) = 1;
	comparison_vector(0) = 0;
	comparison_vector(0) = 0;
	range_cond->addComparison (pcl::TfQuadraticXYZComparison<pcl::PointXYZ>::ConstPtr (new pcl::TfQuadraticXYZComparison<pcl::PointXYZ> (pcl::ComparisonOps::LT, comparison_matrix, comparison_vector, -5)));
	pcl::ConditionalRemoval<pcl::PointXYZ> condrem (range_cond);

	// build the filter 
	condrem.setCondition(range_cond);
	condrem.setInputCloud (cloud_filtered); 

	// apply filter 
	condrem.filter (*cloud_filtered);

	// if filtered cloud is too small probably didn't find the drone so return
	if(cloud_filtered->size()<200) return;

	// downsample drone cloud
	vg.setInputCloud (cloud_filtered);
	vg.filter (*cloud_filtered);

	// euclidean segmentation
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud_filtered);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.2); 
	ec.setMinClusterSize (200);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_filtered);
	ec.extract (cluster_indices);

	std::cout << "num clusters= " << cluster_indices.size() << "\n";

	pcl::PointCloud<pcl::PointXYZ>::Ptr drone_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	int largest_cluster_size = 0;

	// populate pointclouds with the found cluster indicies
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		{
			cloud_cluster->points.push_back (cloud_filtered->points[*pit]); 
		}
		if (cloud_cluster->size() > largest_cluster_size)
		{
			drone_cluster = cloud_cluster;
			largest_cluster_size = cloud_cluster->size();
		}
	}

	// find centroid
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid (*drone_cluster, centroid);
	std::cout << "centroid:" << centroid[0] << " " <<  centroid[1] << " " <<   centroid[2] << " " <<   centroid[3] << " \n";

	// publish largest cluster
	sensor_msgs::PointCloud2 out_cloud;
	pcl::toROSMsg( *cloud_filtered, out_cloud);
	out_cloud.header.frame_id="camera_depth_optical_frame";
	_pub_filtered.publish(out_cloud);

	// publish centroid 
	geometry_msgs::PointStamped out_pos;
	out_pos.point.x = centroid[0];
	out_pos.point.y = centroid[1];
	out_pos.point.z = centroid[2];
	out_pos.header.frame_id="camera_depth_optical_frame";
	_pub_drone_pos.publish(out_pos);


}
int main (int argc, char** argv)
{
	ros::init (argc, argv, "drone_depth_tracking");
	ros::NodeHandle nh;

	// ROS subscriber for point cloud
	_sub_cloud = nh.subscribe("/camera/depth/points", 2, cloud_cb);

	// create ROS publisher for transformed pointcloud
	_pub_filtered = nh.advertise<sensor_msgs::PointCloud2>("filterd_cloud", 1);

	// create ROS publisher for drone position (x,y,z)
	_pub_drone_pos = nh.advertise<geometry_msgs::PointStamped>("point_cmd", 1);

	// spin
	ros::spin ();
}





