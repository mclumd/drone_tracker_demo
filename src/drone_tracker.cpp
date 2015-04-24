#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

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
#include <pcl/common/eigen.h>
#include <pcl/filters/filter.h>

ros::Publisher _pub_filtered;
ros::Publisher _pub_drone_pos;
ros::Subscriber _sub_cloud;
tf::TransformListener* listener;

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
	Eigen::Matrix3f comparison_matrix0 = Eigen::Matrix3f::Zero();
	comparison_matrix0(0,0) = 1;
	comparison_matrix0(1,1) = 1;
	comparison_matrix0(2,2) = 1;
	Eigen::Vector3f comparison_vector0;
	comparison_vector0(0) = 1;
	comparison_vector0(1) = 0;
	comparison_vector0(2) = 0;
	range_cond->addComparison (pcl::TfQuadraticXYZComparison<pcl::PointXYZ>::ConstPtr (new pcl::TfQuadraticXYZComparison<pcl::PointXYZ> (pcl::ComparisonOps::LT, comparison_matrix0, comparison_vector0, -5)));
	/*
	Eigen::Matrix3f comparison_matrix1 = Eigen::Matrix3f::Zero();
	Eigen::Vector3f comparison_vector1;
	comparison_vector1(0) = 0;
	comparison_vector1(1) = 1;
	comparison_vector1(2) = 0;
	range_cond->addComparison (pcl::TfQuadraticXYZComparison<pcl::PointXYZ>::ConstPtr (new pcl::TfQuadraticXYZComparison<pcl::PointXYZ> (pcl::ComparisonOps::GT, comparison_matrix1, comparison_vector1, -100)));
	Eigen::Matrix3f comparison_matrix2 = Eigen::Matrix3f::Zero();
	Eigen::Vector3f comparison_vector2;
	comparison_vector2(0) = 0;
	comparison_vector2(1) = 1;
	comparison_vector2(2) = 0;
	range_cond->addComparison (pcl::TfQuadraticXYZComparison<pcl::PointXYZ>::ConstPtr (new pcl::TfQuadraticXYZComparison<pcl::PointXYZ> (pcl::ComparisonOps::LT, comparison_matrix2, comparison_vector2, 100)));
	*/
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

	//std::cout << "num clusters= " << cluster_indices.size() << "\n";

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
		pcl::PointXYZ min_pt, max_pt; 
		pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);
		//std::cout << "Cluster height0=" << max_pt.y - min_pt.y << "\n";
		if (max_pt.y - min_pt.y > 0.20) continue;
		//std::cout << "z dist=" << max_pt.z - min_pt.z << "\n";
		//std::cout << "y dist=" << max_pt.y - min_pt.y << "\n";
		//std::cout << "x dist=" << max_pt.x - min_pt.x << "\n";
		//std::cout << "Cluster height1=" << max_pt.y - min_pt.y << "\n";
		if (cloud_cluster->size() > largest_cluster_size)
		{
			drone_cluster = cloud_cluster;
			largest_cluster_size = cloud_cluster->size();
		}
	}

	// find centroid
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid (*drone_cluster, centroid);
	//std::cout << "centroid:" << centroid[0] << " " <<  centroid[1] << " " <<   centroid[2] << " " <<   centroid[3] << " \n";

	// publish largest cluster
	sensor_msgs::PointCloud2 out_cloud;
	pcl::toROSMsg( *drone_cluster, out_cloud);
	out_cloud.header.frame_id="camera_depth_optical_frame";
	_pub_filtered.publish(out_cloud);

	// publish centroid 
	geometry_msgs::PointStamped out_pos;
  	out_pos.header.frame_id = "camera_depth_optical_frame";
	out_pos.header.stamp = ros::Time();
	out_pos.point.x = centroid[0];
	out_pos.point.y = centroid[1];
	out_pos.point.z = centroid[2];
	geometry_msgs::PointStamped base_point;
    	listener->transformPoint("torso", out_pos, base_point);
	_pub_drone_pos.publish(base_point);


}
int main (int argc, char** argv)
{
	ros::init (argc, argv, "drone_depth_tracking");
	ros::NodeHandle nh;
	listener = new tf::TransformListener();
	// ROS subscriber for point cloud
	_sub_cloud = nh.subscribe("/camera/depth_registered/points", 2, cloud_cb);

	// create ROS publisher for transformed pointcloud
	_pub_filtered = nh.advertise<sensor_msgs::PointCloud2>("filterd_cloud", 1);

	// create ROS publisher for drone position (x,y,z)
	_pub_drone_pos = nh.advertise<geometry_msgs::PointStamped>("quad_pos", 1);

	// spin
	ros::spin ();
}





