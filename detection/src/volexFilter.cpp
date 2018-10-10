#include <detection/volexFilter.h>

VolexFilter::VolexFilter()
{
    n = ros::NodeHandle();

    // Subscriptions
    ros::Subscriber velodyne_sub = n.subscribe("/map_filtered2", 1, &VolexFilter::cloud_cb, this);

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
        sleep(0.1);
        ros::spinOnce();
    }
}