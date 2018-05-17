#include <detection/boundingBoxes.h>
#include "matplotlibcpp/matplotlibcpp.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

namespace plt = matplotlibcpp;

BoundingBoxes::BoundingBoxes()
{
    n = ros::NodeHandle();

    // Subscriptions
    sub_vector_pointclouds = n.subscribe("/vector_pointclouds", 1, &BoundingBoxes::test_cb, this);

    // Publishers
    pub_boxArray = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/boundingBoxes", 1);

    loop();
}

BoundingBoxes::~BoundingBoxes()
{
}

void BoundingBoxes::test_cb(const detection::vectorPointCloud input)
{
    ros::Time begin = ros::Time::now();
    if (input.clouds.size() > 0)
    {
        j = 0;
        // Trabaja con cada uno de los clusters
        for (int i = 0; i < input.clouds.size(); i++)
        {
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(input.clouds[i], pcl_pc2);
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

            // Reinicializa variables
            resetVariables();

            // Calcula el centroide del cluster
            pcl::compute3DCentroid(*temp_cloud, centroid);

            if (temp_cloud->points.size() > 0)
            {
                // Compara todos los puntos con el resto de puntos de la nube de puntos
                for (int i = 0; i < temp_cloud->points.size(); i++)
                {
                    for (int j = 0; j < temp_cloud->points.size(); j++)
                    {
                        // Almacenar la distancia mayor en X
                        if (maxDistX < fabs(temp_cloud->points[i].x - temp_cloud->points[j].x))
                        {
                            maxDistX = fabs(temp_cloud->points[i].x - temp_cloud->points[j].x);
                            if (temp_cloud->points[i].x > temp_cloud->points[j].x)
                            {
                                xMax = temp_cloud->points[i].x;
                                xMin = temp_cloud->points[j].x;
                            }
                            else
                            {
                                xMax = temp_cloud->points[j].x;
                                xMin = temp_cloud->points[i].x;
                            }
                        }
                        // Almacenar la distancia mayor en Y
                        if (maxDistY < fabs(temp_cloud->points[i].y - temp_cloud->points[j].y))
                        {
                            maxDistY = fabs(temp_cloud->points[i].y - temp_cloud->points[j].y);
                            if (temp_cloud->points[i].y > temp_cloud->points[j].y)
                            {
                                yMax = temp_cloud->points[i].y;
                                yMin = temp_cloud->points[j].y;
                            }
                            else
                            {
                                yMax = temp_cloud->points[j].y;
                                yMin = temp_cloud->points[i].y;
                            }
                        }
                        // Almacenar la distancia mayor en Z
                        if (maxDistZ < fabs(temp_cloud->points[i].z - temp_cloud->points[j].z))
                        {
                            maxDistZ = fabs(temp_cloud->points[i].z - temp_cloud->points[j].z);
                            if (temp_cloud->points[i].z > temp_cloud->points[j].z)
                            {
                                zMax = temp_cloud->points[i].z;
                                zMin = temp_cloud->points[j].z;
                            }
                            else
                            {
                                zMax = temp_cloud->points[j].z;
                                zMin = temp_cloud->points[i].z;
                            }
                        }
                    }
                }
                calculateCenters();
                constructBoundingBoxes();
            }
            j++;
        }
        // Publica todas las boxes de una vez y limpia el vector de boxes
        pub_boxArray.publish(boxes);
        boxes.boxes.clear();
    }
    std::cout << "Time processing: " << ros::Time::now() - begin << \
    "(s) Clusters detected: " << input.clouds.size() << std::endl;
}

void BoundingBoxes::resetVariables()
{
    maxDistX = maxDistY = maxDistZ = xMax = xMin = yMax = yMin = zMax = zMin = centerX = centerY = centerZ = 0;
    box.header.frame_id = boxes.header.frame_id = "velodyne";
    centroid[0] = centroid[1] = centroid[2] = centroid[3] = 0.0;
}

void BoundingBoxes::calculateCenters()
{
    if (maxDistX > maxDistY)
    {
        centerX = xMin + (maxDistX / 2);
        centerY = yMin + (maxDistY / 2);
    }
    if (maxDistX < maxDistY)
    {
        centerX = xMin + (maxDistX / 2);
        centerY = yMin + (maxDistY / 2);
    }
}

void BoundingBoxes::constructBoundingBoxes()
{
    box.pose.position.x = centerX;
    box.pose.position.y = centerY;
    box.pose.position.z = centroid[2];
    box.dimensions.x = maxDistX;
    box.dimensions.y = maxDistY;
    box.dimensions.z = maxDistZ;
    boxes.boxes.push_back(box);
}

void BoundingBoxes::loop()
{
    while (ros::ok())
    {
        //plt::plot(cloud_size_vec);
        ros::spinOnce();
    }
    //plt::show();
}