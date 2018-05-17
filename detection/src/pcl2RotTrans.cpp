#include <detection/pcl2RotTrans.h>

Pcl2RotTrans::Pcl2RotTrans()
{
    n = ros::NodeHandle();

    // Subscriptions
    ros::Subscriber velodyne_sub = n.subscribe("/velodyne_points", 1, &Pcl2RotTrans::velodyne_cb, this);
    ros::Subscriber imu_sub = n.subscribe("/imu_euler_rad_origin", 1, &Pcl2RotTrans::imu_cb, this);
    ros::Subscriber gps_sub = n.subscribe("/fix", 1, &Pcl2RotTrans::gps_cb, this);

    // Publishers
    pub_gps_vel = n.advertise<std_msgs::Float64>("/gps_vel", 1);
    pub_map_imu = n.advertise<sensor_msgs::PointCloud2>("/map_imu", 1);

    loop();
}

Pcl2RotTrans::~Pcl2RotTrans()
{
}

void Pcl2RotTrans::gps_cb(sensor_msgs::NavSatFix msg)
{
    lat = msg.latitude;
    lon = msg.longitude;
    alt = msg.altitude;
    if (flag == false)
    {
        geo_pt_origin.latitude = lat;
        geo_pt_origin.longitude = lon;
        geodesy::UTMPoint utm_pt_origin(geo_pt_origin);
        originx = utm_pt_origin.easting;
        originy = utm_pt_origin.northing;
        flag = true;
    }
    geo_pt.latitude = lat;
    geo_pt.longitude = lon;
    geo_pt.altitude = alt;
    geodesy::UTMPoint utm_pt(geo_pt);
    despX = utm_pt.easting - originx;
    despY = utm_pt.northing - originy;
}

void Pcl2RotTrans::imu_cb(sensor_msgs::Imu msg)
{
    rotRoll = msg.orientation.x;
    rotPitch = msg.orientation.y;
    rotYaw = msg.orientation.z;
}

void Pcl2RotTrans::velodyne_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
{

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2); // Transforma de PointCloud2 a PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud); // Trabajar con la nube temporal

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << despX, despY, 0.0;
    transform.rotate(Eigen::AngleAxisf(rotRoll, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(rotPitch, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(rotYaw, Eigen::Vector3f::UnitZ()));

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Aplica la transformación a la nube temporal obteniendo la nube transformada como resultado
    pcl::transformPointCloud(*temp_cloud, *transformed_cloud, transform);

    sensor_msgs::PointCloud2 output_cloud;
    pcl::toROSMsg(*transformed_cloud, output_cloud); // Pasa de PCL a PointCloud2

    pub_map_imu.publish(output_cloud);
}

void Pcl2RotTrans::loop()
{
    while (ros::ok())
    {
        ros::spinOnce();
    }
}