#include <filtering/voxel_filter.h>

VoxelFilter::VoxelFilter()
{
    n = ros::NodeHandle();

    // Subscriptions
    ros::Subscriber sub_sensor = n.subscribe("/filter_points", 1, &VoxelFilter::sensorCallback, this);

    // Publishers
    pub_voxel_filter_points = n.advertise<sensor_msgs::PointCloud2>("/voxel_filter_points", 1);

    loop();
}

VoxelFilter::~VoxelFilter()
{
}

void VoxelFilter::sensorCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
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

    pub_voxel_filter_points.publish(output);
}

void VoxelFilter::loop()
{
    while (ros::ok())
    {
        sleep(0.1);
        ros::spinOnce();
    }
}