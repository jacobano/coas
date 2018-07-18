#include <detection/boundingBoxes.h>

BoundingBoxes::BoundingBoxes()
{
    n = ros::NodeHandle();

    // params();

    // Subscriptions
    sub_vector_pointclouds = n.subscribe("/vector_pointclouds", 1, &BoundingBoxes::clusters_cb, this);
    sub_phase = n.subscribe("/phase", 1, &BoundingBoxes::phase_cb, this);

    // Publishers
    pub_boxArray = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/boundingBoxes", 1);
    pub_mergeBoxesArray = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/mergeBoundingBoxes", 1);
    pub_boxesRef = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/boxesRef", 1);
    pub_pathPoste1 = n.advertise<nav_msgs::Path>("/pathPoste1", 1);
    pub_pathPoste2 = n.advertise<nav_msgs::Path>("/pathPoste2", 1);
    pub_pathPoste3 = n.advertise<nav_msgs::Path>("/pathPoste3", 1);
    pub_pathPoste12 = n.advertise<nav_msgs::Path>("/pathPoste12", 1);
    pub_pathPoste13 = n.advertise<nav_msgs::Path>("/pathPoste13", 1);
    pub_pathPoste23 = n.advertise<nav_msgs::Path>("/pathPoste23", 1);

    loop();
}

BoundingBoxes::~BoundingBoxes()
{
}

void BoundingBoxes::phase_cb(const std_msgs::Int8 phaseMode)
{
    phase = phaseMode.data;
    switch (phase)
    {
    // Atraque
    case 1:
        close_dist = 1.0;
        xyMinPoste = 0.3;
        xyMaxPoste = 0.7;
        zMinPoste = 1.0;
        zMaxPoste = 2.0;
        minDistPoste12 = 3.5;
        maxDistPoste12 = 4.5;
        minDistPoste13 = 9.0;
        maxDistPoste13 = 10.0;
        break;
    // Puerto
    case 2:
        close_dist = 1.0;
        xyMinPoste = 0.3;
        xyMaxPoste = 1.6;
        zMinPoste = 0.5;
        zMaxPoste = 2.0;
        minDistPoste12 = 0.0;
        maxDistPoste12 = 20.0;
        minDistPoste13 = 15.0;
        maxDistPoste13 = 17.0;
        break;
    // Mar abierto
    case 3:
        close_dist = 5.0;
        xyMinPoste = 0.0;
        xyMaxPoste = 0.0;
        zMinPoste = 0.0;
        zMaxPoste = 0.0;
        minDistPoste12 = 0.0;
        maxDistPoste12 = 0.0;
        minDistPoste13 = 0.0;
        maxDistPoste13 = 0.0;
        break;
    }
}

void BoundingBoxes::clusters_cb(const detection::vectorPointCloud input)
{
    ros::Time begin = ros::Time::now();
    detection::vectorPointCloud clusters = input;
    if (!clusters.clouds.empty())
    {
        label_box = label_mergeBox = 0;
        // Trabaja con cada uno de los clusters
        for (int i = 0; i < clusters.clouds.size(); i++)
        {
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(clusters.clouds[i], pcl_pc2);
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
            // Reinicializa variables
            resetVariables();
            // Calcula el centroide del cluster
            pcl::compute3DCentroid(*temp_cloud, centroid);
            // Calcula distancias máximas del cluster
            calcMaxDistancesCluster(*temp_cloud);
            // Calcula el centro del cluster
            calcCenters();
            // Construye la boundingBox
            constructBoundingBoxes(centerX, centerY, centerZ, maxDistX, maxDistY, maxDistZ, false);
        }
        // Publica todas las boxes de una vez y limpia el vector de boxes
        pub_boxArray.publish(boxes);
        // Calcula todos los poligonos posibles entre clusters
        calcVecPolygons();
        // Unifica las boundingBoxes que forman el poligono en una boundingBox
        mergeBoundingBoxes();
    }
    std::cout << "[ BBXS] Time: " << ros::Time::now() - begin << std::endl;
    std::cout << " - - - - - - - - - - - - - - - - -" << std::endl;
    cleanVariables();
}

void BoundingBoxes::resetVariables()
{
    maxDistX = maxDistY = maxDistZ = xMax = xMin = yMax = yMin = zMax = zMin = centerX = centerY = centerZ = 0;
    box.header.frame_id = boxes.header.frame_id = boxesRef.header.frame_id = mergeBoxes.header.frame_id = "velodyne";
    centroid[0] = centroid[1] = centroid[2] = centroid[3] = 0.0;
}

void BoundingBoxes::cleanVariables()
{
    vec_polygon_labels.clear();
    polygon_labels.clear();
    boxes.boxes.clear();
    boxesRef.boxes.clear();
    mergeBoxes.boxes.clear();
    vec_polygon_labels.clear();
    pathPoste1.poses.clear();
    pathPoste2.poses.clear();
    pathPoste3.poses.clear();
    pathPoste12.poses.clear();
    pathPoste13.poses.clear();
    pathPoste23.poses.clear();
    cont_postes = 0;
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

void BoundingBoxes::calcCenters()
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

void BoundingBoxes::constructBoundingBoxes(float x, float y, float z, float dimX, float dimY, float dimZ, bool merge)
{
    box.pose.position.x = x;
    box.pose.position.y = y;
    box.pose.position.z = z;
    if (merge == false)
    {
        box.dimensions.x = dimX;
        box.dimensions.y = dimY;
        box.dimensions.z = dimZ;
        box.label = label_box;
        checkPostes(dimX, dimY, dimZ);
        boxes.boxes.push_back(box);
        label_box++;
    }
    if (merge == true)
    {
        box.dimensions.x = dimX * 2;
        box.dimensions.y = dimY * 2;
        box.dimensions.z = dimZ * 2;
        box.label = label_mergeBox;
        mergeBoxes.boxes.push_back(box);
        label_mergeBox++;
    }
}

void BoundingBoxes::checkPostes(float xDim, float yDim, float zDim)
{
    // Revisa si la boundingBox actual es un poste
    if (xyMinPoste < xDim && xDim < xyMaxPoste && xyMinPoste < yDim && yDim < xyMaxPoste && zMinPoste < zDim && zDim < zMaxPoste)
    {
        // Distancia al poste
        float dist = dist2Points(0, 0, 0, box.pose.position.x, box.pose.position.y, box.pose.position.z);
        // Prepara el path
        std::vector<float> x1, y1, z1;
        x1.push_back(0.0);
        y1.push_back(0.0);
        z1.push_back(0.0);
        x1.push_back(box.pose.position.x);
        y1.push_back(box.pose.position.y);
        z1.push_back(box.pose.position.z);
        // Construye el path y mete la box de referencia en el array
        switch (cont_postes)
        {
        case 0:
            pathPoste1 = constructPath(x1, y1, z1, 2);
            boxesRef.boxes.push_back(box);
            break;
        case 1:
            pathPoste2 = constructPath(x1, y1, z1, 2);
            boxesRef.boxes.push_back(box);
            break;
        case 2:
            pathPoste3 = constructPath(x1, y1, z1, 2);
            boxesRef.boxes.push_back(box);
            break;
        }
        // Si hay más de un poste detectado
        if (boxesRef.boxes.size() > 1)
        {
            int cont_entrePostes = 0;
            // Calcula la distancia entre todos los postes
            for (int i = 0; i < boxesRef.boxes.size(); i++)
            {
                for (int j = i + 1; j < boxesRef.boxes.size(); j++)
                {
                    float dist = dist2Points(boxesRef.boxes.at(i).pose.position.x, boxesRef.boxes.at(i).pose.position.y, boxesRef.boxes.at(i).pose.position.z,
                                             boxesRef.boxes.at(j).pose.position.x, boxesRef.boxes.at(j).pose.position.y, boxesRef.boxes.at(j).pose.position.z);
                    // Prepara el path
                    std::vector<float> x1, y1, z1;
                    x1.push_back(boxesRef.boxes.at(i).pose.position.x);
                    y1.push_back(boxesRef.boxes.at(i).pose.position.y);
                    z1.push_back(boxesRef.boxes.at(i).pose.position.z);
                    x1.push_back(boxesRef.boxes.at(j).pose.position.x);
                    y1.push_back(boxesRef.boxes.at(j).pose.position.y);
                    z1.push_back(boxesRef.boxes.at(j).pose.position.z);
                    // Si las distancias corresponden a las distancias buscadas
                    if ((minDistPoste12 < dist && dist < maxDistPoste12) || (minDistPoste13 < dist && dist < maxDistPoste13))
                    {
                        // Construye el path para representar distancias entre postes
                        switch (cont_entrePostes)
                        {
                        case 0:
                            pathPoste12 = constructPath(x1, y1, z1, 2);
                            ROS_WARN("dist12: %f", dist);
                        case 1:
                            pathPoste13 = constructPath(x1, y1, z1, 2);
                            ROS_WARN("dist13: %f", dist);
                        case 2:
                            pathPoste23 = constructPath(x1, y1, z1, 2);
                            ROS_WARN("dist23: %f", dist);
                        }
                    }
                    cont_entrePostes++;
                }
            }
        }
        cont_postes++;
        // Publicaciones
        pub_boxesRef.publish(boxesRef);
        pub_pathPoste1.publish(pathPoste1);
        pub_pathPoste2.publish(pathPoste2);
        pub_pathPoste3.publish(pathPoste3);
        pub_pathPoste12.publish(pathPoste12);
        pub_pathPoste13.publish(pathPoste13);
        pub_pathPoste23.publish(pathPoste23);
    }
}

nav_msgs::Path BoundingBoxes::constructPath(std::vector<float> x, std::vector<float> y, std::vector<float> z, int length)
{
    nav_msgs::Path msg;
    std::vector<geometry_msgs::PoseStamped> poses(length);
    msg.header.frame_id = "velodyne";
    for (int i = 0; i < length; i++)
    {
        poses.at(i).pose.position.x = x[i];
        poses.at(i).pose.position.y = y[i];
        poses.at(i).pose.position.z = z[i];
    }
    msg.poses = poses;
    return msg;
}

void BoundingBoxes::calcVecPolygons()
{
    float dist;
    bool repeat, found;
    // Compara todas las distancias entre boxes
    for (int i = 0; i < boxes.boxes.size(); i++)
    {
        found = false;
        for (int j = i + 1; j < boxes.boxes.size(); j++)
        {
            dist = dist2Points(boxes.boxes[i].pose.position.x, boxes.boxes[i].pose.position.y, boxes.boxes[i].pose.position.z,
                               boxes.boxes[j].pose.position.x, boxes.boxes[j].pose.position.y, boxes.boxes[j].pose.position.z);

            // Si el label ya está en otro polígono, no incluirlo en un nuevo polígono
            repeat = false;
            for (int k = 0; k < vec_polygon_labels.size(); k++)
            {
                // Comprueba label del iterador i
                if (std::find(vec_polygon_labels[k].begin(), vec_polygon_labels[k].end(), boxes.boxes[i].label) != vec_polygon_labels[k].end())
                {
                    repeat = true;
                }
                // Comprueba label del iterador j
                if (std::find(vec_polygon_labels[k].begin(), vec_polygon_labels[k].end(), boxes.boxes[j].label) != vec_polygon_labels[k].end())
                {
                    repeat = true;
                }
            }
            // Si están cerca
            if (0.01 < dist && dist < close_dist && repeat == false)
            {
                // Si el polígono está vacío introduce i en vez de j
                if (polygon_labels.empty())
                {
                    polygon_labels.push_back(boxes.boxes[i].label);
                }
                if (!polygon_labels.empty())
                {
                    if (std::find(polygon_labels.begin(), polygon_labels.end(), boxes.boxes[j].label) != polygon_labels.end())
                    {
                        // Repetido
                    }
                    else
                    {
                        // No repetido
                        polygon_labels.push_back(boxes.boxes[j].label);
                    }
                }
                found = true;
            }
        }
        // Si no se ha encontrado ningún label[j] cerca del label[i]
        if (found == false)
        {
            // Si el polígono no está vacío, almacena el polígono generado en el vector de polígonos
            if (!polygon_labels.empty())
            {
                vec_polygon_labels.push_back(polygon_labels);
            }
            else
            {
                // Comprueba que el label de i no está en ningún otro polígono
                for (int k = 0; k < vec_polygon_labels.size(); k++)
                {
                    // Comprueba label del iterador i
                    if (std::find(vec_polygon_labels[k].begin(), vec_polygon_labels[k].end(), boxes.boxes[i].label) != vec_polygon_labels[k].end())
                    {
                        repeat = true;
                    }
                }
                // Si no está se almacena como polígono de un único label
                if (repeat == false)
                {
                    polygon_labels.push_back(boxes.boxes[i].label);
                    vec_polygon_labels.push_back(polygon_labels);
                }
            }
            polygon_labels.clear();
        }
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

void BoundingBoxes::mergeBoundingBoxes()
{
    // Para cada poligono, crea una boundingBox
    for (int i = 0; i < vec_polygon_labels.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ> polygon_cloud_temp;
        polygon_labels.clear();
        polygon_labels = vec_polygon_labels[i];
        // Agrupo las nubes de puntos de los clusters que forman un poligono en una unica nube de puntos
        for (int j = 0; j < polygon_labels.size(); j++)
        {
            // El centroide de una boundingBox se transforma en punto de PC2
            pcl::PointXYZ point;
            point.x = boxes.boxes[polygon_labels[j]].pose.position.x;
            point.y = boxes.boxes[polygon_labels[j]].pose.position.y;
            point.z = boxes.boxes[polygon_labels[j]].pose.position.z;
            // Crea una nube de puntos con todos los centroides de las boundingBoxes cercanas
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            temp_cluster->points.push_back(point);
            polygon_cloud_temp += *temp_cluster;
        }
        // Calcular centroide de esta nube de puntos unica
        Eigen::Vector4f centroid_polygon_cloud;
        pcl::compute3DCentroid(polygon_cloud_temp, centroid_polygon_cloud);
        // Caluclar dimensiones para la nueva boundingBox con las distancias maximas.
        maxDistXpol = maxDistYpol = maxDistZpol = 0;
        for (int k = 0; k < polygon_cloud_temp.size(); k++)
        {
            if (fabs(centroid_polygon_cloud[0] - boxes.boxes[polygon_labels[k]].pose.position.x) + boxes.boxes[polygon_labels[k]].dimensions.x > maxDistXpol)
            {
                maxDistXpol = fabs(centroid_polygon_cloud[0] - boxes.boxes[polygon_labels[k]].pose.position.x) + boxes.boxes[polygon_labels[k]].dimensions.x;
            }
            if (fabs(centroid_polygon_cloud[1] - boxes.boxes[polygon_labels[k]].pose.position.y) + boxes.boxes[polygon_labels[k]].dimensions.y > maxDistYpol)
            {
                maxDistYpol = fabs(centroid_polygon_cloud[1] - boxes.boxes[polygon_labels[k]].pose.position.y) + boxes.boxes[polygon_labels[k]].dimensions.y;
            }
            if (fabs(centroid_polygon_cloud[2] - boxes.boxes[polygon_labels[k]].pose.position.z) + boxes.boxes[polygon_labels[k]].dimensions.z > maxDistZpol)
            {
                maxDistZpol = fabs(centroid_polygon_cloud[2] - boxes.boxes[polygon_labels[k]].pose.position.z) + boxes.boxes[polygon_labels[k]].dimensions.z;
            }
        }
        // Crea la boundingBox que engloba las demás
        constructBoundingBoxes(centroid_polygon_cloud[0], centroid_polygon_cloud[1], centroid_polygon_cloud[2], maxDistXpol, maxDistYpol, maxDistZpol, true);
    }
    pub_mergeBoxesArray.publish(mergeBoxes);
}

void BoundingBoxes::params()
{
    // ros::NodeHandle nparam("~");
    // if (nparam.getParam("close_dist", close_dist))
    // {
    //     ROS_WARN("Got BoundingBoxes param close_dist: %f", close_dist);
    // }
    // else
    // {
    //     close_dist = 5.0;
    //     ROS_WARN("Failed to get BoundingBoxes param close_dist: %f", close_dist);
    // }
    // if (nparam.getParam("xyMinPoste", xyMinPoste))
    // {
    //     ROS_WARN("Got BoundingBoxes param xyMinPoste: %f", xyMinPoste);
    // }
    // else
    // {
    //     xyMinPoste = 0.2;
    //     ROS_WARN("Failed to get BoundingBoxes param xyMinPoste: %f", xyMinPoste);
    // }
    // if (nparam.getParam("xyMaxPoste", xyMaxPoste))
    // {
    //     ROS_WARN("Got BoundingBoxes param xyMaxPoste: %f", xyMaxPoste);
    // }
    // else
    // {
    //     xyMaxPoste = 0.8;
    //     ROS_WARN("Failed to get BoundingBoxes param xyMaxPoste: %f", xyMaxPoste);
    // }
    // if (nparam.getParam("zMinPoste", zMinPoste))
    // {
    //     ROS_WARN("Got BoundingBoxes param zMinPoste: %f", zMinPoste);
    // }
    // else
    // {
    //     zMinPoste = 0.5;
    //     ROS_WARN("Failed to get BoundingBoxes param zMinPoste: %f", zMinPoste);
    // }
    // if (nparam.getParam("zMaxPoste", zMaxPoste))
    // {
    //     ROS_WARN("Got BoundingBoxes param zMaxPoste: %f", zMaxPoste);
    // }
    // else
    // {
    //     zMaxPoste = 1.5;
    //     ROS_WARN("Failed to get BoundingBoxes param zMaxPoste: %f", zMaxPoste);
    // }
    // if (nparam.getParam("minDistPoste12", minDistPoste12))
    // {
    //     ROS_WARN("Got BoundingBoxes param minDistPoste12: %f", minDistPoste12);
    // }
    // else
    // {
    //     minDistPoste12 = 4.5;
    //     ROS_WARN("Failed to get BoundingBoxes param minDistPoste12: %f", minDistPoste12);
    // }
    // if (nparam.getParam("maxDistPoste12", maxDistPoste12))
    // {
    //     ROS_WARN("Got BoundingBoxes param maxDistPoste12: %f", maxDistPoste12);
    // }
    // else
    // {
    //     maxDistPoste12 = 5.0;
    //     ROS_WARN("Failed to get BoundingBoxes param maxDistPoste12: %f", maxDistPoste12);
    // }
    // if (nparam.getParam("minDistPoste13", minDistPoste13))
    // {
    //     ROS_WARN("Got BoundingBoxes param minDistPoste13: %f", minDistPoste13);
    // }
    // else
    // {
    //     minDistPoste13 = 12.5;
    //     ROS_WARN("Failed to get BoundingBoxes param minDistPoste13: %f", minDistPoste13);
    // }
    // if (nparam.getParam("maxDistPoste13", maxDistPoste13))
    // {
    //     ROS_WARN("Got BoundingBoxes param maxDistPoste13: %f", maxDistPoste13);
    // }
    // else
    // {
    //     maxDistPoste13 = 13.5;
    //     ROS_WARN("Failed to get BoundingBoxes param maxDistPoste13: %f", maxDistPoste13);
    // }
}

void BoundingBoxes::loop()
{
    while (ros::ok())
    {
        sleep(0.1);
        ros::spinOnce();
    }
}