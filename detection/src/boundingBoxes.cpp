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
        label_box = 0;
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
        //postProcess();
        postProcess1(input);
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
    centerZ = zMin + (maxDistZ / 2);
}

void BoundingBoxes::constructBoundingBoxes()
{
    box.pose.position.x = centerX;
    box.pose.position.y = centerY;
    box.pose.position.z = centerZ; // centroid[2]
    box.dimensions.x = maxDistX;
    box.dimensions.y = maxDistY;
    box.dimensions.z = maxDistZ;
    box.label = label_box;
    boxes.boxes.push_back(box);
    label_box++;
}

/*void BoundingBoxes::postProcess()
{
    float dist;
    if (boxes.boxes.size() > 0)
    {
        for (int i = 0; i < boxes.boxes.size(); i++)
        {
            polygon_cloud.points.clear();
            int polygon_cont = 0;

            for (int j = 0; j <= boxes.boxes.size(); j++)
            {
                dist = dist2Points(boxes.boxes[i].pose.position.x, boxes.boxes[i].pose.position.y, boxes.boxes[i].pose.position.z,
                                   boxes.boxes[j].pose.position.x, boxes.boxes[j].pose.position.y, boxes.boxes[j].pose.position.z);
                if (0.0 < dist && dist < 10.0)
                {
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
                    polygon_cont++;
                }
            }
            for (int i = 0; i < polygon_cont; i++)
            {
                pcl::compute3DCentroid(polygon_cloud, polygon_centroid);
                //ROS_WARN("Centroid [%i]: x = %f | y = %f | z = %f", i + 1, polygon_centroid[0], polygon_centroid[1], polygon_centroid[2]);
                // -- Busca dimensiones para el nuevo cluster
                maxDistXpol = maxDistYpol = maxDistZpol = 0;
                for (int j = 0; j < polygon_cloud.points.size(); j++)
                {
                    for (int k = 0; k < boxes.boxes.size(); k++)
                    {
                        if (polygon_cloud.points[j].x == boxes.boxes[k].pose.position.x &&
                            polygon_cloud.points[j].y == boxes.boxes[k].pose.position.y &&
                            polygon_cloud.points[j].z == boxes.boxes[k].pose.position.z)
                        {
                            if ((fabs(boxes.boxes[k].pose.position.x - polygon_centroid[0]) + boxes.boxes[k].dimensions.x) > maxDistXpol)
                            {
                                maxDistXpol = fabs(boxes.boxes[k].pose.position.x - polygon_centroid[0]) + boxes.boxes[k].dimensions.x;
                                //ROS_WARN("maxdistxpol %f", maxDistXpol);
                            }
                            if ((fabs(boxes.boxes[k].pose.position.y - polygon_centroid[1]) + boxes.boxes[k].dimensions.y) > maxDistYpol)
                            {
                                maxDistYpol = fabs(boxes.boxes[k].pose.position.y - polygon_centroid[1]) + boxes.boxes[k].dimensions.y;
                                //ROS_WARN("maxdistypol %f", maxDistYpol);
                            }
                            if ((fabs(boxes.boxes[k].pose.position.z - polygon_centroid[2]) + boxes.boxes[k].dimensions.z) > maxDistZpol)
                            {
                                maxDistZpol = fabs(boxes.boxes[k].pose.position.z - polygon_centroid[2]) + boxes.boxes[k].dimensions.z;
                                //ROS_WARN("maxdistzpol %f", maxDistZpol);
                            }
                        }
                    }
                }
                // --
                box1.pose.position.x = polygon_centroid[0];
                box1.pose.position.y = polygon_centroid[1];
                box1.pose.position.z = polygon_centroid[2];
                box1.dimensions.x = maxDistXpol;
                box1.dimensions.y = maxDistYpol;
                box1.dimensions.z = maxDistZpol;
                boxes1.boxes.push_back(box1);
            }
            pub_boxArray1.publish(boxes1);
            boxes1.boxes.clear();
        }
    }
}*/

void BoundingBoxes::postProcess1(const detection::vectorPointCloud clusters)
{
    float dist;
    if (boxes.boxes.size() > 0)
    {
        // CALCULA POLIGONOS //
        // Compara todas las cajas con todas las demas
        for (int i = 0; i < boxes.boxes.size(); i++)
        {
            Eigen::Vector4f centroid_clusterI = pc2_centroid(clusters.clouds[i]);
            for (int j = 0; j < boxes.boxes.size(); j++)
            {
                Eigen::Vector4f centroid_clusterJ = pc2_centroid(clusters.clouds[j]);

                dist = dist2Points(centroid_clusterI[0], centroid_clusterI[1], centroid_clusterI[2],
                                   centroid_clusterJ[0], centroid_clusterJ[1], centroid_clusterJ[2]);
                if (dist < 6.0)
                {
                    if (poligono.empty())
                    {
                        pose.pose.position.x = centroid_clusterI[0];
                        pose.pose.position.y = centroid_clusterI[1];
                        pose.pose.position.z = centroid_clusterI[2];
                        poligono[boxes.boxes[i].label] = pose;
                    }
                    if (!poligono.empty())
                    {
                        pose.pose.position.x = centroid_clusterJ[0];
                        pose.pose.position.y = centroid_clusterJ[1];
                        pose.pose.position.z = centroid_clusterJ[2];
                        poligono[boxes.boxes[j].label] = pose;
                    }
                }
            }
            if (i == boxes.boxes.size() - 1)
            {
                vec_polygons.push_back(poligono);
                ROS_WARN("poligono size %i |   boxes.boxes.size  %i | vec size %i", poligono.size(), boxes.boxes.size(), vec_polygons.size());
            }
            poligono.clear();
        }
        // CALCULA POLIGONOS //
        // CREA BOUNDINGBOX PARA CADA POLIGONO //
        // Para cada poligono, crea una boundingBox
        for (int i = 0; i < vec_polygons.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZ> polygon_cloud_temp;
            poligono = vec_polygons[i];
            // Agrupo las nubes de puntos de los clusters que forman un poligono en una unica nube de puntos
            for (std::map<int, geometry_msgs::PoseStamped>::iterator it = poligono.begin(); it != poligono.end(); ++it)
            {
                // Cluster PC2 a temp_cluster PCL_PointXYZ
                pcl::PCLPointCloud2 pcl_pc2;
                pcl_conversions::toPCL(clusters.clouds[it->first], pcl_pc2);
                pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cluster(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromPCLPointCloud2(pcl_pc2, *temp_cluster);

                polygon_cloud_temp += *temp_cluster;
                //ROS_WARN("polygon cloud temp size %i", polygon_cloud_temp.size());
            }
            // Calcular centroide de esta nube de puntos unica
            Eigen::Vector4f centroid_polygon_cloud;
            pcl::compute3DCentroid(polygon_cloud_temp, centroid_polygon_cloud);
            // Caluclar dimensiones para la nueba boundingBox con las distancias maximas.
            maxDistXpol = maxDistYpol = maxDistZpol = 0;
            for (int i = 0; i < polygon_cloud_temp.size(); i++)
            {
                for (int j = 0; j < polygon_cloud_temp.size(); j++)
                {
                    if (fabs(polygon_cloud_temp.points[i].x - polygon_cloud_temp.points[j].x) > maxDistXpol)
                    {
                        maxDistXpol = fabs(polygon_cloud_temp.points[i].x - polygon_cloud_temp.points[j].x);
                    }
                    if (fabs(polygon_cloud_temp.points[i].y - polygon_cloud_temp.points[j].y) > maxDistYpol)
                    {
                        maxDistYpol = fabs(polygon_cloud_temp.points[i].y - polygon_cloud_temp.points[j].y);
                    }
                    if (fabs(polygon_cloud_temp.points[i].z - polygon_cloud_temp.points[j].z) > maxDistZpol)
                    {
                        maxDistZpol = fabs(polygon_cloud_temp.points[i].z - polygon_cloud_temp.points[j].z);
                    }
                }
            }
            // Crea la boundingBox que engloba
            box1.pose.position.x = centroid_polygon_cloud[0];
            box1.pose.position.y = centroid_polygon_cloud[1];
            box1.pose.position.z = centroid_polygon_cloud[2];
            box1.dimensions.x = maxDistXpol;
            box1.dimensions.y = maxDistYpol;
            box1.dimensions.z = maxDistZpol;
            boxes1.boxes.push_back(box1);
        }
        pub_boxArray1.publish(boxes1);
        boxes1.boxes.clear();
        vec_polygons.clear();
        // CREA BOUNDINGBOX PARA CADA POLIGONO //
    }
}

float BoundingBoxes::dist2Points(float x1, float y1, float z1, float x2, float y2, float z2)
{
    float distance;
    distance = sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)) + ((z2 - z1) * (z2 - z1)));
    return distance;
}

Eigen::Vector4f BoundingBoxes::pc2_centroid(const sensor_msgs::PointCloud2 pc2)
{
    Eigen::Vector4f centroidPC2;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(pc2, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_pcl);
    pcl::compute3DCentroid(*temp_pcl, centroidPC2);
    return centroidPC2;
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