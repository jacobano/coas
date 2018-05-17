#include <detection/euclideanClusterer.h>
#include <detection/boundingBoxes.h>
#include "detection/vectorPointCloud.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

EuclideanClusterer::EuclideanClusterer()
{
    n = ros::NodeHandle();

    // Subscriptions
    ros::Subscriber velodyne_sub = n.subscribe("/map_filtered2", 1, &EuclideanClusterer::cloud_cb, this);

    // Publishers
    pub_pointclouds = n.advertise<detection::vectorPointCloud>("/vector_pointclouds", 1);
    loop();
}

EuclideanClusterer::~EuclideanClusterer()
{
}

void EuclideanClusterer::cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
{
    sensor_msgs::PointCloud2 input_cloud = *input;
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_XYZ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    // Change from type sensor_msgs::PointCloud2 to pcl::PointXYZ
    pcl::fromROSMsg(input_cloud, *downsampled_XYZ);

    //Create the SACSegmentation object and set the model and method type
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC); //For more info: wikipedia.org/wiki/RANSAC
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.5); //How close a point must be to the model to considered an inlier (default 0.02)

    int i = 0, nr_points = (int)downsampled_XYZ->points.size();

    //Contains the plane point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    // While 30% [90%] of the original cloud is still there
    while (downsampled_XYZ->points.size() > 0.9 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(downsampled_XYZ);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(downsampled_XYZ);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        downsampled_XYZ.swap(cloud_f);
        i++;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(downsampled_XYZ);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.8); // default = 0.02 (2cm)
    ec.setMinClusterSize(30);    // default = 100
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(downsampled_XYZ);
    ec.extract(cluster_indices);

    ros::NodeHandle nh;
    //Create a publisher for each cluster
    for (int i = 0; i < cluster_indices.size(); ++i)
    {
        std::string topicName = "/pcl_tut/cluster" + boost::lexical_cast<std::string>(i);

        ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>(topicName, 1);

        pub_vec.push_back(pub);
    }
    int j = 0;
    detection::vectorPointCloud vector_pointclouds;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
            cloud_cluster->points.push_back(downsampled_XYZ->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        //Convert the pointcloud to be used in ROS
        sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*cloud_cluster, *output);
        output->header.frame_id = input_cloud.header.frame_id;

        vector_pointclouds.clouds.push_back(*output);
        // Publish the data
        pub_vec[j].publish(output);
        ++j;
    }

    pub_pointclouds.publish(vector_pointclouds);
}

void EuclideanClusterer::loop()
{
    while (ros::ok())
    {
        ros::spinOnce();
    }
}