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

    char* envvar_home;
    envvar_home = std::getenv("HOME");
    std::stringstream logOutputAux;
    logOutputAux << envvar_home << "/Matlab_ws/";
    logOutput = logOutputAux.str();

    fileToPosts.open(logOutput + "distancesToPosts");
    fileBetweenPosts.open(logOutput + "distancesBetweenPosts");
    fileBetweenPostsTimes.open(logOutput + "timesBetweenPosts");
    fileToPostsTimes.open(logOutput + "timesToPosts");
    filePost1.open(logOutput + "post1");
    filePost2.open(logOutput + "post2");
    filePost3.open(logOutput + "post3");
    filePost1Time.open(logOutput + "post1time");
    filePost2Time.open(logOutput + "post2time");
    filePost3Time.open(logOutput + "post3time");

    contTest = contTestPose = 0;

    loop();
}

BoundingBoxes::~BoundingBoxes()
{
    fileToPosts.close();
    fileBetweenPosts.close();
    fileToPostsTimes.close();
    fileBetweenPostsTimes.close();
    filePost1.close();
    filePost2.close();
    filePost3.close();
    filePost1Time.close();
    filePost2Time.close();
    filePost3Time.close();
}

void BoundingBoxes::phase_cb(const std_msgs::Int8 phaseMode)
{
    phase = phaseMode.data;
    switch (phase)
    {
    // Docking
    case 1:
        close_dist = 1.0;
        xyMinPoste = 0.1;
        xyMaxPoste = 0.6;
        zMinPoste = 1;
        zMaxPoste = 3.0;
        minDistPoste12 = 3.5;
        maxDistPoste12 = 3.9;
        minDistPoste13 = 8.5;
        maxDistPoste13 = 8.8;
        break;
    // Harbor
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
    // Sea
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
    double begin = ros::Time::now().toSec();
    detection::vectorPointCloud clusters = input;
    if (!clusters.clouds.empty())
    {
        label_box = label_mergeBox = 0;
        // Work with every single cluster
        for (int i = 0; i < clusters.clouds.size(); i++)
        {
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(clusters.clouds[i], pcl_pc2);
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
            // Reinitialize variables
            resetVariables();
            // Calculate centroid of the cluster
            pcl::compute3DCentroid(*temp_cloud, centroid);
            // Calculate maximum distances of the cluster
            calcMaxDistancesCluster(*temp_cloud);
            // Calculate center of the cluster
            calcCenters();
            // Construct bounding box
            constructBoundingBoxes(centerX, centerY, centerZ, maxDistX, maxDistY, maxDistZ, false);
        }
        // Publish all boxes at one time and clean boxes vector
        pub_boxArray.publish(boxes);
        // Calculate all possible polygons formed by clusters
        calcVecPolygons();
        // Merge all bounding boxes that form one polygon into a new bounding box
        mergeBoundingBoxes();
    }
    std::cout << "[ BBXS] Time: " << ros::Time::now().toSec() - begin << std::endl;
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
    cont_posts = 0;
}

void BoundingBoxes::calcMaxDistancesCluster(const pcl::PointCloud<pcl::PointXYZ> cluster)
{
    // Compare every single point with the rest of the point cloud
    for (int i = 0; i < cluster.points.size(); i++)
    {
        for (int j = 0; j < cluster.points.size(); j++)
        {
            // Storage the largest distance in X
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
            // Storage the largest distance in Y
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
            // Storage the largest distance in Z
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
        // If it is at docking mode search and valid posts
        if (phase == 1)
        {
            checkPosts(dimX, dimY, dimZ);
        }
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

void BoundingBoxes::checkPosts(float xDim, float yDim, float zDim)
{
    // Check if the actual bounding box is a post
    if (xyMinPoste < xDim && xDim < xyMaxPoste && xyMinPoste < yDim && yDim < xyMaxPoste && zMinPoste < zDim && zDim < zMaxPoste)
    {
        // Distance to the post
        float dist = dist2Points(0, 0, 0, box.pose.position.x, box.pose.position.y, box.pose.position.z);
        // Prepare the path
        std::vector<float> x1, y1, z1;
        x1.push_back(0.0);
        y1.push_back(0.0);
        z1.push_back(0.0);
        x1.push_back(box.pose.position.x);
        y1.push_back(box.pose.position.y);
        z1.push_back(box.pose.position.z);
        // Construct the path and put the bounding box in the reference array
        switch (cont_posts)
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
        // If there is more than one post detected
        if (boxesRef.boxes.size() > 1)
        {
            int cont_entrePostes = 0;
            // Calculate the distance between all posts
            for (int i = 0; i < boxesRef.boxes.size(); i++)
            {
                for (int j = i + 1; j < boxesRef.boxes.size(); j++)
                {
                    float dist = dist2Points(boxesRef.boxes.at(i).pose.position.x, boxesRef.boxes.at(i).pose.position.y, boxesRef.boxes.at(i).pose.position.z,
                                             boxesRef.boxes.at(j).pose.position.x, boxesRef.boxes.at(j).pose.position.y, boxesRef.boxes.at(j).pose.position.z);
                    // Prepare the path
                    std::vector<float> x0, y0, z0;
                    x0.push_back(boxesRef.boxes.at(i).pose.position.x);
                    y0.push_back(boxesRef.boxes.at(i).pose.position.y);
                    z0.push_back(boxesRef.boxes.at(i).pose.position.z);
                    x0.push_back(boxesRef.boxes.at(j).pose.position.x);
                    y0.push_back(boxesRef.boxes.at(j).pose.position.y);
                    z0.push_back(boxesRef.boxes.at(j).pose.position.z);
                    // If the calculated distances match with the searched distances
                    if ((minDistPoste12 < dist && dist < maxDistPoste12) || (minDistPoste13 < dist && dist < maxDistPoste13))
                    {
                        // Construct the path to represent distances between posts
                        switch (cont_entrePostes)
                        {
                        case 0:
                            pathPoste12 = constructPath(x0, y0, z0, 2);
                            break;
                        case 1:
                            pathPoste13 = constructPath(x0, y0, z0, 2);
                            break;
                        case 2:
                            pathPoste23 = constructPath(x0, y0, z0, 2);
                            break;
                        }
                        cont_entrePostes++;
                    }
                }
            }
        }
        cont_posts++;
        // Publishers
        pub_boxesRef.publish(boxesRef);
        pub_pathPoste1.publish(pathPoste1);
        pub_pathPoste2.publish(pathPoste2);
        pub_pathPoste3.publish(pathPoste3);
        pub_pathPoste12.publish(pathPoste12);
        pub_pathPoste13.publish(pathPoste13);
        pub_pathPoste23.publish(pathPoste23);
        // Save logs
        save_distances(true, pathPoste1, pathPoste2, pathPoste3);
        save_distances(false, pathPoste12, pathPoste13, pathPoste23);
        if (contTestPose == 0)
        {
            startTimePose = ros::Time::now().toSec();
            contTestPose++;
        }
        if (pathPoste1.poses.size() > 0)
        {
            save_pose(1, pathPoste1);
        }
        if (pathPoste2.poses.size() > 0)
        {
            save_pose(2, pathPoste2);
        }
        if (pathPoste3.poses.size() > 0)
        {
            save_pose(3, pathPoste3);
        }
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
    // Compare all distances between boxes
    for (int i = 0; i < boxes.boxes.size(); i++)
    {
        found = false;
        for (int j = i + 1; j < boxes.boxes.size(); j++)
        {
            dist = dist2Points(boxes.boxes[i].pose.position.x, boxes.boxes[i].pose.position.y, boxes.boxes[i].pose.position.z,
                               boxes.boxes[j].pose.position.x, boxes.boxes[j].pose.position.y, boxes.boxes[j].pose.position.z);

            // If the label is in another polygon do not use it in a new polygon
            repeat = false;
            for (int k = 0; k < vec_polygon_labels.size(); k++)
            {
                // Check the label of the iterator i
                if (std::find(vec_polygon_labels[k].begin(), vec_polygon_labels[k].end(), boxes.boxes[i].label) != vec_polygon_labels[k].end())
                {
                    repeat = true;
                }
                // Check the label of the iterator j
                if (std::find(vec_polygon_labels[k].begin(), vec_polygon_labels[k].end(), boxes.boxes[j].label) != vec_polygon_labels[k].end())
                {
                    repeat = true;
                }
            }
            // If they are close
            if (0.01 < dist && dist < close_dist && repeat == false)
            {
                // If the polygon is empty put in i instead of j
                if (polygon_labels.empty())
                {
                    polygon_labels.push_back(boxes.boxes[i].label);
                }
                if (!polygon_labels.empty())
                {
                    if (std::find(polygon_labels.begin(), polygon_labels.end(), boxes.boxes[j].label) != polygon_labels.end())
                    {
                        // Repeated
                    }
                    else
                    {
                        // Not repeated
                        polygon_labels.push_back(boxes.boxes[j].label);
                    }
                }
                found = true;
            }
        }
        // If no label[j] has been found near label[i].
        if (found == false)
        {
            // If the polygon is not empty storage it in the polygons vector
            if (!polygon_labels.empty())
            {
                vec_polygon_labels.push_back(polygon_labels);
            }
            else
            {
                // Check that label i is not in another polygon
                for (int k = 0; k < vec_polygon_labels.size(); k++)
                {
                    // Check label of the iterator i
                    if (std::find(vec_polygon_labels[k].begin(), vec_polygon_labels[k].end(), boxes.boxes[i].label) != vec_polygon_labels[k].end())
                    {
                        repeat = true;
                    }
                }
                // If not, storage it like a polygon formed by one label
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
    // For each polygon create a bounding box
    for (int i = 0; i < vec_polygon_labels.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ> polygon_cloud_temp;
        polygon_labels.clear();
        polygon_labels = vec_polygon_labels[i];
        // Merge all point clouds (clusters) that form a polygon in a single point cloud
        for (int j = 0; j < polygon_labels.size(); j++)
        {
            // Transform the bounding box centroid into a PC2 point
            pcl::PointXYZ point;
            point.x = boxes.boxes[polygon_labels[j]].pose.position.x;
            point.y = boxes.boxes[polygon_labels[j]].pose.position.y;
            point.z = boxes.boxes[polygon_labels[j]].pose.position.z;
            // Create a point cloud with every centroid of the near bounding boxes
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            temp_cluster->points.push_back(point);
            polygon_cloud_temp += *temp_cluster;
        }
        // Calculate the centroid of this single point cloud
        Eigen::Vector4f centroid_polygon_cloud;
        pcl::compute3DCentroid(polygon_cloud_temp, centroid_polygon_cloud);
        // Calculate dimensions for the new bounding box using maximum distances
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
        // Create the bounding box that includes the others
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

void BoundingBoxes::save_pose(int nPost, nav_msgs::Path path)
{
    switch (nPost)
    {
    case 1:
        filePost1 << path.poses.at(1).pose.position.x << " " << path.poses.at(1).pose.position.y << std::endl;
        filePost1Time << ros::Time::now().toSec() - startTimePose << std::endl;
        break;
    case 2:
        filePost2 << path.poses.at(1).pose.position.x << " " << path.poses.at(1).pose.position.y << std::endl;
        filePost2Time << ros::Time::now().toSec() - startTimePose << std::endl;
        break;
    case 3:
        filePost3 << path.poses.at(1).pose.position.x << " " << path.poses.at(1).pose.position.y << std::endl;
        filePost3Time << ros::Time::now().toSec() - startTimePose << std::endl;
        break;
    }
}

void BoundingBoxes::save_distances(bool b, nav_msgs::Path path1, nav_msgs::Path path2, nav_msgs::Path path3)
{
    float dist1, dist2, dist3;

    if (path1.poses.size() > 0)
    {
        dist1 = dist2Points(path1.poses.at(0).pose.position.x, path1.poses.at(0).pose.position.y, path1.poses.at(0).pose.position.z,
                            path1.poses.at(1).pose.position.x, path1.poses.at(1).pose.position.y, path1.poses.at(1).pose.position.z);
    }
    else
    {
        dist1 = 0.0;
    }
    if (path2.poses.size() > 0)
    {
        dist2 = dist2Points(path2.poses.at(0).pose.position.x, path2.poses.at(0).pose.position.y, path2.poses.at(0).pose.position.z,
                            path2.poses.at(1).pose.position.x, path2.poses.at(1).pose.position.y, path2.poses.at(1).pose.position.z);
    }
    else
    {
        dist2 = 0.0;
    }
    if (path3.poses.size() > 0)
    {
        dist3 = dist2Points(path3.poses.at(0).pose.position.x, path3.poses.at(0).pose.position.y, path3.poses.at(0).pose.position.z,
                            path3.poses.at(1).pose.position.x, path3.poses.at(1).pose.position.y, path3.poses.at(1).pose.position.z);
    }
    else
    {
        dist3 = 0.0;
    }

    if (contTest == 0)
    {
        startTime = ros::Time::now().toSec();
        contTest++;
    }

    if (dist1 != 0 && dist2 != 0 && dist3 != 0)
    {
        if (b == true)
        {
            fileToPosts << dist1 << " " << dist2 << " " << dist3 << std::endl;
            fileToPostsTimes << ros::Time::now().toSec() - startTime << std::endl;
        }
        else
        {
            fileBetweenPosts << dist1 << " " << dist2 << " " << dist3 << std::endl;
            fileBetweenPostsTimes << ros::Time::now().toSec() - startTime << std::endl;
        }
    }
}

void BoundingBoxes::loop()
{
    while (ros::ok())
    {
        sleep(0.1);
        ros::spinOnce();
    }
    //ros::spin();
}