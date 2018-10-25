#include <detection/euclidean_clusterer.h>

EuclideanClusterer::EuclideanClusterer()
{
    n = ros::NodeHandle();

    // getParameters();

    // Subscriptions
    sub_velodyne = n.subscribe("/voxel_filter_points", 1, &EuclideanClusterer::cloudCallback, this);
    sub_phase = n.subscribe("/phase", 1, &EuclideanClusterer::phaseCallback, this);

    // Publishers
    pub_point_clouds = n.advertise<detection::VectorPointCloud>("/vector_pointclouds", 1);
}

EuclideanClusterer::~EuclideanClusterer()
{
}

void EuclideanClusterer::phaseCallback(const std_msgs::Int8 phaseMode)
{
    phase = phaseMode.data;
    switch (phase)
    {
    // Docking
    case 1:
        distance_threshold = 0.4;
        cluster_tolerance = 0.8;
        min_cluster_size = 30;
        max_cluster_size = 600;
        break;
    // Harbor
    case 2:
        distance_threshold = 0.5;
        cluster_tolerance = 0.8;
        min_cluster_size = 30;
        max_cluster_size = 25000;
        break;
    // Sea
    case 3:
        distance_threshold = 0.5;
        cluster_tolerance = 0.8;
        min_cluster_size = 2;
        max_cluster_size = 25000;
        break;
    }
}

void EuclideanClusterer::getParameters()
{
    // ros::NodeHandle nparam("~");
    // if (nparam.getParam("distance_threshold", distance_threshold))
    // {
    //     ROS_WARN("Got EuclideanClusterer param distance_threshold: %f", distance_threshold);
    // }
    // else
    // {
    //     distance_threshold = 0.02;
    //     ROS_WARN("Failed to get EuclideanClusterer param distance_threshold: %f", distance_threshold);
    // }
    // if (nparam.getParam("cluster_tolerance", cluster_tolerance))
    // {
    //     ROS_WARN("Got EuclideanClusterer param param cluster_tolerance: %f", cluster_tolerance);
    // }
    // else
    // {
    //     cluster_tolerance = 0.02;
    //     ROS_WARN("Failed to get EuclideanClusterer param cluster_tolerance: %f", cluster_tolerance);
    // }
    // if (nparam.getParam("min_cluster_size", min_cluster_size))
    // {
    //     ROS_WARN("Got EuclideanClusterer param min_cluster_size: %f", min_cluster_size);
    // }
    // else
    // {
    //     min_cluster_size = 100;
    //     ROS_WARN("Failed to get EuclideanClusterer param min_cluster_size: %f", min_cluster_size);
    // }
    // if (nparam.getParam("max_cluster_size", max_cluster_size))
    // {
    //     ROS_WARN("Got EuclideanClusterer param max_cluster_size: %f", max_cluster_size);
    // }
    // else
    // {
    //     max_cluster_size = 25000;
    //     ROS_WARN("Failed to get EuclideanClusterer param max_cluster_size: %f", max_cluster_size);
    // }
    // if (nparam.getParam("phase", phase))
    // {
    //     ROS_WARN("Got EuclideanClusterer param phase: %f", phase);
    // }
    // else
    // {
    //     ROS_WARN("Failed to get EuclideanClusterer param phase: %f", phase);
    // }
}

void EuclideanClusterer::cloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
{
    double begin = ros::Time::now().toSec();

    sensor_msgs::PointCloud2 input_cloud = *input;
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_XYZ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    // Change the variable type from sensor_msgs::PointCloud2 to pcl::PointXYZ
    pcl::fromROSMsg(input_cloud, *downsampled_XYZ);
    // Create the object SACSegmentation, define the model and the type of the method
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create the object SACSegmentation
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Necessary
    seg.setModelType(pcl::SACMODEL_PLANE);
    // More info: wikipedia.org/wiki/RANSAC
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    // How close must be a point from the model to consider it in line
    seg.setDistanceThreshold(distance_threshold); // (default 0.02)

    int i = 0, nr_points = (int)downsampled_XYZ->points.size();

    // Contains the point cloud of the plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    if (phase == 1 || phase == 2)
    {
        float percentage;
        if (phase == 1)
            percentage = 0.3;
        if (phase == 2)
            percentage = 0.9;

        // While 30% [90%] of the original point cloud still there
        while (downsampled_XYZ->points.size() > percentage * nr_points) // docking = 0.9 // default = 0.3
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
    }
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(downsampled_XYZ);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance); // default = 0.02 (2cm)
    ec.setMinClusterSize(min_cluster_size);    // Sea = 2 // Docking = 30 // default = 100
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(downsampled_XYZ);
    ec.extract(cluster_indices);

    // Create a publisher for each cluster
    for (int i = 0; i < cluster_indices.size(); ++i)
    {
        std::string topicName = "/cluster" + boost::lexical_cast<std::string>(i);
        ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>(topicName, 1);
        pub_vec_point_clouds.push_back(pub);
    }
    int j = 0;
    detection::VectorPointCloud vector_pointclouds;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
            cloud_cluster->points.push_back(downsampled_XYZ->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        // Convert the point cloud into a typed message to be used in ROS
        sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*cloud_cluster, *output);
        output->header.frame_id = input_cloud.header.frame_id;
        vector_pointclouds.clouds.push_back(*output);
        pub_vec_point_clouds[j].publish(output);
        ++j;
    }

    pub_point_clouds.publish(vector_pointclouds);
    std::cout << "[ EUCL] Time: " << ros::Time::now().toSec() - begin << std::endl;
}