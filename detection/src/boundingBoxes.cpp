#include <detection/boundingBoxes.h>

BoundingBoxes::BoundingBoxes()
{
    n = ros::NodeHandle();

    // Subscriptions
    sub_vector_pointclouds = n.subscribe("/vector_pointclouds", 1, &BoundingBoxes::test_cb, this);

    // Publishers
    pub_boxArray = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/boundingBoxes", 1);
    pub_mergeBoxesArray = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/mergeBoundingBoxes", 1);

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
            // Calcula distancias mácimas del cluster
            calcMaxDistancesCluster(*temp_cloud);
            // Calcula el centro del cluster
            calculateCenters();
            // Construye la boundingBox
            constructBoundingBoxes(centerX, centerY, centerZ, maxDistX, maxDistY, maxDistZ, true);
        }
        // Publica todas las boxes de una vez y limpia el vector de boxes
        pub_boxArray.publish(boxes);
        // Calcula todos los poligonos posibles entre clusters
        calcVecPolygons(input);
        // Unifica las boundingBoxes que forman el poligono en una boundingBox
        mergeBoundingBoxes(input);
    
        boxes.boxes.clear();
    }
    std::cout << "Time processing: " << ros::Time::now() - begin << " | (s) Clusters detected: " << input.clouds.size() << std::endl;
}

void BoundingBoxes::resetVariables()
{
    maxDistX = maxDistY = maxDistZ = xMax = xMin = yMax = yMin = zMax = zMin = centerX = centerY = centerZ = 0;
    box.header.frame_id = boxes.header.frame_id = mergeBoxes.header.frame_id = "velodyne";
    centroid[0] = centroid[1] = centroid[2] = centroid[3] = 0.0;
}

void BoundingBoxes::calcMaxDistancesCluster(const pcl::PointCloud<pcl::PointXYZ> cluster)
{
    // Compara todos los puntos con el resto de puntos de la nube de puntos
    for (int i = 0; i < cluster.points.size(); i++)
    {
        for (int j = 0; j < cluster.points.size(); j++)
        {
            // Almacenar la distancia mayor en X
            if (maxDistX < fabs(cluster.points[i].x - cluster.points[j].x))
            {
                maxDistX = fabs(cluster.points[i].x - cluster.points[j].x);
                if (cluster.points[i].x > cluster.points[j].x)
                {
                    xMax = cluster.points[i].x;
                    xMin = cluster.points[j].x;
                }
                else
                {
                    xMax = cluster.points[j].x;
                    xMin = cluster.points[i].x;
                }
            }
            // Almacenar la distancia mayor en Y
            if (maxDistY < fabs(cluster.points[i].y - cluster.points[j].y))
            {
                maxDistY = fabs(cluster.points[i].y - cluster.points[j].y);
                if (cluster.points[i].y > cluster.points[j].y)
                {
                    yMax = cluster.points[i].y;
                    yMin = cluster.points[j].y;
                }
                else
                {
                    yMax = cluster.points[j].y;
                    yMin = cluster.points[i].y;
                }
            }
            // Almacenar la distancia mayor en Z
            if (maxDistZ < fabs(cluster.points[i].z - cluster.points[j].z))
            {
                maxDistZ = fabs(cluster.points[i].z - cluster.points[j].z);
                if (cluster.points[i].z > cluster.points[j].z)
                {
                    zMax = cluster.points[i].z;
                    zMin = cluster.points[j].z;
                }
                else
                {
                    zMax = cluster.points[j].z;
                    zMin = cluster.points[i].z;
                }
            }
        }
    }
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

void BoundingBoxes::constructBoundingBoxes(float x, float y, float z, float dimX, float dimY, float dimZ, bool label)
{
    box.pose.position.x = x;
    box.pose.position.y = y;
    box.pose.position.z = z;
    box.dimensions.x = dimX;
    box.dimensions.y = dimY;
    box.dimensions.z = dimZ;
    if (label == true)
    {
        box.label = label_box;
        boxes.boxes.push_back(box);
        label_box++;
    }
    if (label == false)
    {
        mergeBoxes.boxes.push_back(box);
    }
}

void BoundingBoxes::calcVecPolygons(const detection::vectorPointCloud clusters)
{
    float dist;
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
            //ROS_WARN("poligono size %i |   boxes.boxes.size  %i | vec size %i", poligono.size(), boxes.boxes.size(), vec_polygons.size());
        }
        poligono.clear();
    }
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

float BoundingBoxes::dist2Points(float x1, float y1, float z1, float x2, float y2, float z2)
{
    float distance;
    distance = sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)) + ((z2 - z1) * (z2 - z1)));
    return distance;
}

void BoundingBoxes::mergeBoundingBoxes(const detection::vectorPointCloud clusters)
{
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
        // // Crea la boundingBox que engloba
        constructBoundingBoxes(centroid_polygon_cloud[0], centroid_polygon_cloud[1], centroid_polygon_cloud[2], maxDistXpol, maxDistYpol, maxDistZpol, false);
    }
    pub_mergeBoxesArray.publish(mergeBoxes);
    mergeBoxes.boxes.clear();
    vec_polygons.clear();
}

void BoundingBoxes::loop()
{
    while (ros::ok())
    {
        ros::spinOnce();
    }
}