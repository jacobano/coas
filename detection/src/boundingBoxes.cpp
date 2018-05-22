#include <detection/boundingBoxes.h>
#include "matplotlibcpp/matplotlibcpp.h"

namespace plt = matplotlibcpp;

BoundingBoxes::BoundingBoxes()
{
    n = ros::NodeHandle();

    // Subscriptions
    sub_vector_pointclouds = n.subscribe("/vector_pointclouds", 1, &BoundingBoxes::test_cb, this);

    // Publishers
    pub_boxArray = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/boundingBoxes", 1);
    pub_boxArray1 = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/boundingBoxes1", 1);

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
        postProcess();
        boxes.boxes.clear();
    }
    std::cout << "Time processing: " << ros::Time::now() - begin << "(s) Clusters detected: " << input.clouds.size() << std::endl;
}

void BoundingBoxes::resetVariables()
{
    maxDistX = maxDistY = maxDistZ = xMax = xMin = yMax = yMin = zMax = zMin = centerX = centerY = centerZ = 0;
    box.header.frame_id = boxes.header.frame_id = box1.header.frame_id = boxes1.header.frame_id = "velodyne";
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

void BoundingBoxes::postProcess()
{
    float dist;
    if (boxes.boxes.size() > 0)
    {
        for (int i = 0; i < boxes.boxes.size(); i++)
        {
            // polygon.clear();
            polygon_cloud.points.clear();
            int polygon_cont = 0;
            // pose.pose.position.x = boxes.boxes[i].pose.position.x;
            // pose.pose.position.y = boxes.boxes[i].pose.position.y;
            // pose.pose.position.z = boxes.boxes[i].pose.position.z;
            // polygon.push_back(pose);
            //- polygon_pose.x = boxes.boxes[i].pose.position.x;
            //- polygon_pose.y = boxes.boxes[i].pose.position.y;
            //- polygon_pose.z = boxes.boxes[i].pose.position.z;
            //- polygon_cloud.push_back(polygon_pose);
            for (int j = 0; j <= boxes.boxes.size(); j++)
            {
                dist = dist2Points(boxes.boxes[i].pose.position.x, boxes.boxes[i].pose.position.y, boxes.boxes[i].pose.position.z,
                                   boxes.boxes[j].pose.position.x, boxes.boxes[j].pose.position.y, boxes.boxes[j].pose.position.z);
                if (0.0 < dist && dist < 8.0)
                {
                    // pose.pose.position.x = boxes.boxes[j].pose.position.x;
                    // pose.pose.position.y = boxes.boxes[j].pose.position.y;
                    // pose.pose.position.z = boxes.boxes[j].pose.position.z;
                    // polygon.push_back(pose);
                    if (j == 1)
                    {
                        polygon_pose.x = boxes.boxes[i].pose.position.x;
                        polygon_pose.y = boxes.boxes[i].pose.position.y;
                        polygon_pose.z = boxes.boxes[i].pose.position.z;
                        polygon_cloud.push_back(polygon_pose);
                    }
                    polygon_pose.x = boxes.boxes[j].pose.position.x;
                    polygon_pose.y = boxes.boxes[j].pose.position.y;
                    polygon_pose.z = boxes.boxes[j].pose.position.z;
                    polygon_cloud.push_back(polygon_pose);
                    //ROS_WARN("dist %f", dist);
                    polygon_cont++;
                }
            }
            // Muestra poligono
            for (int i = 0; i < polygon_cont; i++)
            {
                pcl::compute3DCentroid(polygon_cloud, polygon_centroid);
                // ROS_WARN("Polygon pose [%i]: x = %f | y = %f | z = %f", i, polygon[i].pose.position.x, polygon[i].pose.position.y, polygon[i].pose.position.z);
                ROS_WARN("Centroid [%i]: x = %f | y = %f | z = %f", i + 1, polygon_centroid[0], polygon_centroid[1], polygon_centroid[2]);

                box1.pose.position.x = polygon_centroid[0];
                box1.pose.position.y = polygon_centroid[1];
                box1.pose.position.z = polygon_centroid[2];
                box1.dimensions.x = 5;
                box1.dimensions.y = 5;
                box1.dimensions.z = 3;
                boxes1.boxes.push_back(box1);
            }
            pub_boxArray1.publish(boxes1);
            boxes1.boxes.clear();
        }
    }
}

float BoundingBoxes::dist2Points(float x1, float y1, float z1, float x2, float y2, float z2)
{
    float dist;
    dist = sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)) + ((z2 - z1) * (z2 - z1)));
    return dist;
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