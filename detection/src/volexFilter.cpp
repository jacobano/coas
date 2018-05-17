#include <detection/volexFilter.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
// --
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <boost/lexical_cast.hpp>

VolexFilter::VolexFilter()
{
    n = ros::NodeHandle();

    // Subscriptions
    ros::Subscriber velodyne_sub = n.subscribe("/velodyne_points", 1, &VolexFilter::cloud_cb, this);

    // Publishers
    pub = n.advertise<sensor_msgs::PointCloud2>("/volexFilterPoints", 1);

    loop();
}

VolexFilter::~VolexFilter()
{
}

void VolexFilter::cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
{
    sensor_msgs::PointCloud2 output;

    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    pcl_conversions::toPCL(*input, *cloud);

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(0.08, 0.08, 0.08);
    sor.filter(cloud_filtered);

    pcl_conversions::fromPCL(cloud_filtered, output);

    pub.publish(output);
}

void VolexFilter::loop()
{
    while (ros::ok())
    {
        ros::spinOnce();
    }
}