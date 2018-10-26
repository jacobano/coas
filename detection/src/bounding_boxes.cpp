#include <detection/bounding_boxes.h>

BoundingBoxes::BoundingBoxes()
{
    n = ros::NodeHandle();

    // params();

    // Subscriptions
    sub_velodyne = n.subscribe("/filter_points", 1, &BoundingBoxes::cloudCallback, this);
    sub_phase = n.subscribe("/phase", 1, &BoundingBoxes::phaseCallback, this);

    // Publishers
    pub_boxes = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bounding_boxes", 1);
    pub_merge_boxes = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/merge_bounding_boxes", 1);
    pub_reference_boxes = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/reference_bounding_boxes", 1);
    pub_path_post_1 = n.advertise<nav_msgs::Path>("/path_poste_1", 1);
    pub_path_post_3 = n.advertise<nav_msgs::Path>("/path_poste_2", 1);
    pub_path_post_2 = n.advertise<nav_msgs::Path>("/path_poste_3", 1);
    pub_path_post_12 = n.advertise<nav_msgs::Path>("/path_poste_12", 1);
    pub_path_post_13 = n.advertise<nav_msgs::Path>("/path_poste_13", 1);
    pub_path_post_23 = n.advertise<nav_msgs::Path>("/path_poste_23", 1);


    char* envvar_home;
    envvar_home = std::getenv("HOME");
    std::stringstream log_output_aux_stream;
    log_output_aux_stream << envvar_home << "/Matlab_ws/";
    log_output = log_output_aux_stream.str();

    file_distance_to_posts.open(log_output + "distancesToPosts");
    file_distance_between_posts.open(log_output + "distancesBetweenPosts");
    file_distance_between_posts_times.open(log_output + "timesBetweenPosts");
    file_distance_to_posts_times.open(log_output + "timesToPosts");
    file_post_1.open(log_output + "post1");
    file_post_2.open(log_output + "post2");
    file_post_3.open(log_output + "post3");
    file_post_1_time.open(log_output + "post1time");
    file_post_2_time.open(log_output + "post2time");
    file_post_3_time.open(log_output + "post3time");

    contTest = contTestPose = 0;
}

BoundingBoxes::~BoundingBoxes()
{
    file_distance_to_posts.close();
    file_distance_between_posts.close();
    file_distance_to_posts_times.close();
    file_distance_between_posts_times.close();
    file_post_1.close();
    file_post_2.close();
    file_post_3.close();
    file_post_1_time.close();
    file_post_2_time.close();
    file_post_3_time.close();
}

void BoundingBoxes::phaseCallback(const std_msgs::Int8 phaseMode)
{
    phase = phaseMode.data;
    switch (phase)
    {
        // Docking
        case 1:
            // Euclidean Clusterer Parameters
            distance_threshold = 0.4;
            cluster_tolerance = 0.8;
            min_cluster_size = 30;
            max_cluster_size = 600;
            // Bounding Boxes parameters
            close_distance = 1.0;
            xy_min_post = 0.1;
            xy_max_post = 0.6;
            z_min_post = 1;
            z_max_post = 3.0;
            min_distance_post_12 = 3.5;
            max_distance_post_12 = 3.9;
            min_distance_post_13 = 8.5;
            max_distance_post_13 = 8.8;
            break;
        // Harbor
        case 2:
            // Euclidean Clusterer Parameters
            distance_threshold = 0.5;
            cluster_tolerance = 0.8;
            min_cluster_size = 30;
            max_cluster_size = 25000;
            // Bounding Boxes parameters
            close_distance = 1.0;
            xy_min_post = 0.3;
            xy_max_post = 1.6;
            z_min_post = 0.5;
            z_max_post = 2.0;
            min_distance_post_12 = 0.0;
            max_distance_post_12 = 20.0;
            min_distance_post_13 = 15.0;
            max_distance_post_13 = 17.0;
            break;
        // Sea
        case 3:
            // Euclidean Clusterer Parameters
            distance_threshold = 0.5;
            cluster_tolerance = 0.8;
            min_cluster_size = 2;
            max_cluster_size = 25000;
            // Bounding Boxes parameters
            close_distance = 5.0;
            xy_min_post = 0.0;
            xy_max_post = 0.0;
            z_min_post = 0.0;
            z_max_post = 0.0;
            min_distance_post_12 = 0.0;
            max_distance_post_12 = 0.0;
            min_distance_post_13 = 0.0;
            max_distance_post_13 = 0.0;
            break;
    }
}

void BoundingBoxes::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input)
{
    // Euclidean Clusterer Part
    double time_start = ros::Time::now().toSec();

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
    detection::VectorPointCloud clusters_vector;
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
        clusters_vector.clouds.push_back(*output);
        pub_vec_point_clouds[j].publish(output);
        ++j;
    }

    std::cout << "[ EUCL] Time: " << ros::Time::now().toSec() - time_start << std::endl;

    //*****************************************************************************
    // Bounding Boxes part
    time_start = ros::Time::now().toSec();
    if (!clusters_vector.clouds.empty())
    {
        label_box = label_merge_box = 0;
        // Work with every single cluster
        for (int i = 0; i < clusters_vector.clouds.size(); i++)
        {
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(clusters_vector.clouds[i], pcl_pc2);
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
            // Reinitialize variables
            resetVariables();
            // Calculate centroid of the cluster
            pcl::compute3DCentroid(*temp_cloud, centroid);
            // Calculate maximum distances of the cluster
            calculateMaxDistancesCluster(*temp_cloud);
            // Calculate center of the cluster
            calculateCenters();
            // Construct bounding box
            constructBoundingBoxes(x_center, y_center, z_center, max_dist_x, max_dist_y, max_dist_z, false);
        }
        // Publish all boxes at one time and clean boxes vector
        pub_boxes.publish(boxes);
        // Calculate all possible polygons formed by clusters
        calculateVectorPolygons();
        // Merge all bounding boxes that form one polygon into a new bounding box
        mergeBoundingBoxes();
    }
    std::cout << "[ BBXS] Time: " << ros::Time::now().toSec() - time_start << std::endl;
    std::cout << " - - - - - - - - - - - - - - - - -" << std::endl;

    cleanVariables();
}

void BoundingBoxes::resetVariables()
{
    max_dist_x = max_dist_y = max_dist_z = x_max = x_min = y_max = y_min = z_max = z_min = x_center = y_center = z_center = 0;
    box.header.frame_id = boxes.header.frame_id = reference_boxes.header.frame_id = merge_boxes.header.frame_id = "velodyne";
    centroid[0] = centroid[1] = centroid[2] = centroid[3] = 0.0;
}

void BoundingBoxes::cleanVariables()
{
    vec_vec_label_polygon.clear();
    vec_label_polygon.clear();
    boxes.boxes.clear();
    reference_boxes.boxes.clear();
    merge_boxes.boxes.clear();
    vec_vec_label_polygon.clear();
    path_post_1.poses.clear();
    path_post_2.poses.clear();
    path_post_3.poses.clear();
    path_post_12.poses.clear();
    path_post_13.poses.clear();
    path_post_23.poses.clear();
    counter_posts = 0;
}

void BoundingBoxes::calculateMaxDistancesCluster(const pcl::PointCloud<pcl::PointXYZ> cluster)
{
    // Compare every single point with the rest of the point cloud
    for (int i = 0; i < cluster.points.size(); i++)
    {
        for (int j = 0; j < cluster.points.size(); j++)
        {
            // Storage the largest distance in X
            if (max_dist_x < fabs(cluster.points[i].x - cluster.points[j].x))
            {
                max_dist_x = fabs(cluster.points[i].x - cluster.points[j].x);
                if (cluster.points[i].x > cluster.points[j].x)
                {
                    x_max = cluster.points[i].x;
                    x_min = cluster.points[j].x;
                }
                else
                {
                    x_max = cluster.points[j].x;
                    x_min = cluster.points[i].x;
                }
            }
            // Storage the largest distance in Y
            if (max_dist_y < fabs(cluster.points[i].y - cluster.points[j].y))
            {
                max_dist_y = fabs(cluster.points[i].y - cluster.points[j].y);
                if (cluster.points[i].y > cluster.points[j].y)
                {
                    y_max = cluster.points[i].y;
                    y_min = cluster.points[j].y;
                }
                else
                {
                    y_max = cluster.points[j].y;
                    y_min = cluster.points[i].y;
                }
            }
            // Storage the largest distance in Z
            if (max_dist_z < fabs(cluster.points[i].z - cluster.points[j].z))
            {
                max_dist_z = fabs(cluster.points[i].z - cluster.points[j].z);
                if (cluster.points[i].z > cluster.points[j].z)
                {
                    z_max = cluster.points[i].z;
                    z_min = cluster.points[j].z;
                }
                else
                {
                    z_max = cluster.points[j].z;
                    z_min = cluster.points[i].z;
                }
            }
        }
    }
}

void BoundingBoxes::calculateCenters()
{
    if (max_dist_x > max_dist_y)
    {
        x_center = x_min + (max_dist_x / 2);
        y_center = y_min + (max_dist_y / 2);
    }
    if (max_dist_x < max_dist_y)
    {
        x_center = x_min + (max_dist_x / 2);
        y_center = y_min + (max_dist_y / 2);
    }
    z_center = z_min + (max_dist_z / 2);
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
            checkPostDimension(dimX, dimY, dimZ);
        }
        boxes.boxes.push_back(box);
        label_box++;
    }
    if (merge == true)
    {
        box.dimensions.x = dimX * 2;
        box.dimensions.y = dimY * 2;
        box.dimensions.z = dimZ * 2;
        box.label = label_merge_box;
        merge_boxes.boxes.push_back(box);
        label_merge_box++;
    }
}

void BoundingBoxes::checkPostDimension(float xDim, float yDim, float zDim)
{
    // Check if the actual bounding box is a post
    if (xy_min_post < xDim && xDim < xy_max_post && xy_min_post < yDim && yDim < xy_max_post && z_min_post < zDim && zDim < z_max_post)
    {
        // Distance to the post
        float dist = calculateDistance2Points(0, 0, 0, box.pose.position.x, box.pose.position.y, box.pose.position.z);
        // Prepare the path
        std::vector<float> x1, y1, z1;
        x1.push_back(0.0);
        y1.push_back(0.0);
        z1.push_back(0.0);
        x1.push_back(box.pose.position.x);
        y1.push_back(box.pose.position.y);
        z1.push_back(box.pose.position.z);
        // Construct the path and put the bounding box in the reference array
        switch (counter_posts)
        {
        case 0:
            path_post_1 = constructPath(x1, y1, z1, 2);
            reference_boxes.boxes.push_back(box);
            break;
        case 1:
            path_post_2 = constructPath(x1, y1, z1, 2);
            reference_boxes.boxes.push_back(box);
            break;
        case 2:
            path_post_3 = constructPath(x1, y1, z1, 2);
            reference_boxes.boxes.push_back(box);
            break;
        }
        // If there is more than one post detected
        if (reference_boxes.boxes.size() > 1)
        {
            int counter_between_posts = 0;
            // Calculate the distance between all posts
            for (int i = 0; i < reference_boxes.boxes.size(); i++)
            {
                for (int j = i + 1; j < reference_boxes.boxes.size(); j++)
                {
                    float dist = calculateDistance2Points(reference_boxes.boxes.at(i).pose.position.x, reference_boxes.boxes.at(i).pose.position.y, reference_boxes.boxes.at(i).pose.position.z,
                                                          reference_boxes.boxes.at(j).pose.position.x, reference_boxes.boxes.at(j).pose.position.y, reference_boxes.boxes.at(j).pose.position.z);
                    // Prepare the path
                    std::vector<float> x0, y0, z0;
                    x0.push_back(reference_boxes.boxes.at(i).pose.position.x);
                    y0.push_back(reference_boxes.boxes.at(i).pose.position.y);
                    z0.push_back(reference_boxes.boxes.at(i).pose.position.z);
                    x0.push_back(reference_boxes.boxes.at(j).pose.position.x);
                    y0.push_back(reference_boxes.boxes.at(j).pose.position.y);
                    z0.push_back(reference_boxes.boxes.at(j).pose.position.z);
                    // If the calculated distances match with the searched distances
                    if ((min_distance_post_12 < dist && dist < max_distance_post_12) || (min_distance_post_13 < dist && dist < max_distance_post_13))
                    {
                        // Construct the path to represent distances between posts
                        switch (counter_between_posts)
                        {
                        case 0:
                            path_post_12 = constructPath(x0, y0, z0, 2);
                            break;
                        case 1:
                            path_post_13 = constructPath(x0, y0, z0, 2);
                            break;
                        case 2:
                            path_post_23 = constructPath(x0, y0, z0, 2);
                            break;
                        }
                        counter_between_posts++;
                    }
                }
            }
        }
        counter_posts++;
        // Publishers
        pub_reference_boxes.publish(reference_boxes);
        pub_path_post_1.publish(path_post_1);
        pub_path_post_2.publish(path_post_2);
        pub_path_post_3.publish(path_post_3);
        pub_path_post_12.publish(path_post_12);
        pub_path_post_13.publish(path_post_13);
        pub_path_post_23.publish(path_post_23);
        // Save logs
        saveDistances(true, path_post_1, path_post_2, path_post_3);
        saveDistances(false, path_post_12, path_post_13, path_post_23);
        if (contTestPose == 0)
        {
            time_pose_start = ros::Time::now().toSec();
            contTestPose++;
        }
        if (path_post_1.poses.size() > 0)
        {
            savePose(1, path_post_1);
        }
        if (path_post_2.poses.size() > 0)
        {
            savePose(2, path_post_2);
        }
        if (path_post_3.poses.size() > 0)
        {
            savePose(3, path_post_3);
        }
    }
}

nav_msgs::Path BoundingBoxes::constructPath(std::vector<float> x, std::vector<float> y, std::vector<float> z, int length)
{
    nav_msgs::Path path;
    std::vector<geometry_msgs::PoseStamped> poses(length);
    path.header.frame_id = "velodyne";
    for (int i = 0; i < length; i++)
    {
        poses.at(i).pose.position.x = x[i];
        poses.at(i).pose.position.y = y[i];
        poses.at(i).pose.position.z = z[i];
    }
    path.poses = poses;
    return path;
}

void BoundingBoxes::calculateVectorPolygons()
{
    float dist;
    bool repeat, found;
    // Compare all distances between boxes
    for (int i = 0; i < boxes.boxes.size(); i++)
    {
        found = false;
        for (int j = i + 1; j < boxes.boxes.size(); j++)
        {
            dist = calculateDistance2Points(boxes.boxes[i].pose.position.x, boxes.boxes[i].pose.position.y, boxes.boxes[i].pose.position.z,
                                            boxes.boxes[j].pose.position.x, boxes.boxes[j].pose.position.y, boxes.boxes[j].pose.position.z);

            // If the label is in another polygon do not use it in a new polygon
            repeat = false;
            for (int k = 0; k < vec_vec_label_polygon.size(); k++)
            {
                // Check the label of the iterator i
                if (std::find(vec_vec_label_polygon[k].begin(), vec_vec_label_polygon[k].end(), boxes.boxes[i].label) != vec_vec_label_polygon[k].end())
                {
                    repeat = true;
                }
                // Check the label of the iterator j
                if (std::find(vec_vec_label_polygon[k].begin(), vec_vec_label_polygon[k].end(), boxes.boxes[j].label) != vec_vec_label_polygon[k].end())
                {
                    repeat = true;
                }
            }
            // If they are close
            if (0.01 < dist && dist < close_distance && repeat == false)
            {
                // If the polygon is empty put in i instead of j
                if (vec_label_polygon.empty())
                {
                    vec_label_polygon.push_back(boxes.boxes[i].label);
                }
                if (!vec_label_polygon.empty())
                {
                    if (std::find(vec_label_polygon.begin(), vec_label_polygon.end(), boxes.boxes[j].label) != vec_label_polygon.end())
                    {
                        // Repeated
                    }
                    else
                    {
                        // Not repeated
                        vec_label_polygon.push_back(boxes.boxes[j].label);
                    }
                }
                found = true;
            }
        }
        // If no label[j] has been found near label[i].
        if (found == false)
        {
            // If the polygon is not empty storage it in the polygons vector
            if (!vec_label_polygon.empty())
            {
                vec_vec_label_polygon.push_back(vec_label_polygon);
            }
            else
            {
                // Check that label i is not in another polygon
                for (int k = 0; k < vec_vec_label_polygon.size(); k++)
                {
                    // Check label of the iterator i
                    if (std::find(vec_vec_label_polygon[k].begin(), vec_vec_label_polygon[k].end(), boxes.boxes[i].label) != vec_vec_label_polygon[k].end())
                    {
                        repeat = true;
                    }
                }
                // If not, storage it like a polygon formed by one label
                if (repeat == false)
                {
                    vec_label_polygon.push_back(boxes.boxes[i].label);
                    vec_vec_label_polygon.push_back(vec_label_polygon);
                }
            }
            vec_label_polygon.clear();
        }
    }
}

float BoundingBoxes::calculateDistance2Points(float x1, float y1, float z1, float x2, float y2, float z2)
{
    float distance;
    distance = sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)) + ((z2 - z1) * (z2 - z1)));
    return distance;
}

void BoundingBoxes::mergeBoundingBoxes()
{
    // For each polygon create a bounding box
    for (int i = 0; i < vec_vec_label_polygon.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ> polygon_cloud_temp;
        vec_label_polygon.clear();
        vec_label_polygon = vec_vec_label_polygon[i];
        // Merge all point clouds (clusters) that form a polygon in a single point cloud
        for (int j = 0; j < vec_label_polygon.size(); j++)
        {
            // Transform the bounding box centroid into a PC2 point
            pcl::PointXYZ point;
            point.x = boxes.boxes[vec_label_polygon[j]].pose.position.x;
            point.y = boxes.boxes[vec_label_polygon[j]].pose.position.y;
            point.z = boxes.boxes[vec_label_polygon[j]].pose.position.z;
            // Create a point cloud with every centroid of the near bounding boxes
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            temp_cluster->points.push_back(point);
            polygon_cloud_temp += *temp_cluster;
        }
        // Calculate the centroid of this single point cloud
        Eigen::Vector4f centroid_polygon_cloud;
        pcl::compute3DCentroid(polygon_cloud_temp, centroid_polygon_cloud);
        // Calculate dimensions for the new bounding box using maximum distances
        max_dist_x_polygon = max_dist_y_polygon = max_dist_z_polygon = 0;
        for (int k = 0; k < polygon_cloud_temp.size(); k++)
        {
            if (fabs(centroid_polygon_cloud[0] - boxes.boxes[vec_label_polygon[k]].pose.position.x) + boxes.boxes[vec_label_polygon[k]].dimensions.x > max_dist_x_polygon)
            {
                max_dist_x_polygon = fabs(centroid_polygon_cloud[0] - boxes.boxes[vec_label_polygon[k]].pose.position.x) + boxes.boxes[vec_label_polygon[k]].dimensions.x;
            }
            if (fabs(centroid_polygon_cloud[1] - boxes.boxes[vec_label_polygon[k]].pose.position.y) + boxes.boxes[vec_label_polygon[k]].dimensions.y > max_dist_y_polygon)
            {
                max_dist_y_polygon = fabs(centroid_polygon_cloud[1] - boxes.boxes[vec_label_polygon[k]].pose.position.y) + boxes.boxes[vec_label_polygon[k]].dimensions.y;
            }
            if (fabs(centroid_polygon_cloud[2] - boxes.boxes[vec_label_polygon[k]].pose.position.z) + boxes.boxes[vec_label_polygon[k]].dimensions.z > max_dist_z_polygon)
            {
                max_dist_z_polygon = fabs(centroid_polygon_cloud[2] - boxes.boxes[vec_label_polygon[k]].pose.position.z) + boxes.boxes[vec_label_polygon[k]].dimensions.z;
            }
        }
        // Create the bounding box that includes the others
        constructBoundingBoxes(centroid_polygon_cloud[0], centroid_polygon_cloud[1], centroid_polygon_cloud[2], max_dist_x_polygon, max_dist_y_polygon, max_dist_z_polygon, true);
    }
    pub_merge_boxes.publish(merge_boxes);
}

void BoundingBoxes::getParameters()
{
    // ros::NodeHandle nparam("~");
    // Euclidean Clustererer Parameters
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

    // Bounding Boxes Parameters
    //***************************************************************************
    // if (nparam.getParam("close_distance", close_distance))
    // {
    //     ROS_WARN("Got BoundingBoxes param close_distance: %f", close_distance);
    // }
    // else
    // {
    //     close_distance = 5.0;
    //     ROS_WARN("Failed to get BoundingBoxes param close_distance: %f", close_distance);
    // }
    // if (nparam.getParam("xy_min_post", xy_min_post))
    // {
    //     ROS_WARN("Got BoundingBoxes param xy_min_post: %f", xy_min_post);
    // }
    // else
    // {
    //     xy_min_post = 0.2;
    //     ROS_WARN("Failed to get BoundingBoxes param xy_min_post: %f", xy_min_post);
    // }
    // if (nparam.getParam("xy_max_post", xy_max_post))
    // {
    //     ROS_WARN("Got BoundingBoxes param xy_max_post: %f", xy_max_post);
    // }
    // else
    // {
    //     xy_max_post = 0.8;
    //     ROS_WARN("Failed to get BoundingBoxes param xy_max_post: %f", xy_max_post);
    // }
    // if (nparam.getParam("z_min_post", z_min_post))
    // {
    //     ROS_WARN("Got BoundingBoxes param z_min_post: %f", z_min_post);
    // }
    // else
    // {
    //     z_min_post = 0.5;
    //     ROS_WARN("Failed to get BoundingBoxes param z_min_post: %f", z_min_post);
    // }
    // if (nparam.getParam("z_max_post", z_max_post))
    // {
    //     ROS_WARN("Got BoundingBoxes param z_max_post: %f", z_max_post);
    // }
    // else
    // {
    //     z_max_post = 1.5;
    //     ROS_WARN("Failed to get BoundingBoxes param z_max_post: %f", z_max_post);
    // }
    // if (nparam.getParam("min_distance_post_12", min_distance_post_12))
    // {
    //     ROS_WARN("Got BoundingBoxes param min_distance_post_12: %f", min_distance_post_12);
    // }
    // else
    // {
    //     min_distance_post_12 = 4.5;
    //     ROS_WARN("Failed to get BoundingBoxes param min_distance_post_12: %f", min_distance_post_12);
    // }
    // if (nparam.getParam("max_distance_post_12", max_distance_post_12))
    // {
    //     ROS_WARN("Got BoundingBoxes param max_distance_post_12: %f", max_distance_post_12);
    // }
    // else
    // {
    //     max_distance_post_12 = 5.0;
    //     ROS_WARN("Failed to get BoundingBoxes param max_distance_post_12: %f", max_distance_post_12);
    // }
    // if (nparam.getParam("min_distance_post_13", min_distance_post_13))
    // {
    //     ROS_WARN("Got BoundingBoxes param min_distance_post_13: %f", min_distance_post_13);
    // }
    // else
    // {
    //     min_distance_post_13 = 12.5;
    //     ROS_WARN("Failed to get BoundingBoxes param min_distance_post_13: %f", min_distance_post_13);
    // }
    // if (nparam.getParam("max_distance_post_13", max_distance_post_13))
    // {
    //     ROS_WARN("Got BoundingBoxes param max_distance_post_13: %f", max_distance_post_13);
    // }
    // else
    // {
    //     max_distance_post_13 = 13.5;
    //     ROS_WARN("Failed to get BoundingBoxes param max_distance_post_13: %f", max_distance_post_13);
    // }  
}

void BoundingBoxes::savePose(int nPost, nav_msgs::Path path)
{
    switch (nPost)
    {
    case 1:
        file_post_1 << path.poses.at(1).pose.position.x << " " << path.poses.at(1).pose.position.y << std::endl;
        file_post_1_time << ros::Time::now().toSec() - time_pose_start << std::endl;
        break;
    case 2:
        file_post_2 << path.poses.at(1).pose.position.x << " " << path.poses.at(1).pose.position.y << std::endl;
        file_post_2_time << ros::Time::now().toSec() - time_pose_start << std::endl;
        break;
    case 3:
        file_post_3 << path.poses.at(1).pose.position.x << " " << path.poses.at(1).pose.position.y << std::endl;
        file_post_3_time << ros::Time::now().toSec() - time_pose_start << std::endl;
        break;
    }
}

void BoundingBoxes::saveDistances(bool b, nav_msgs::Path path1, nav_msgs::Path path2, nav_msgs::Path path3)
{
    float dist_1, dist_2, dist_3;

    if (path1.poses.size() > 0)
    {
        dist_1 = calculateDistance2Points(path1.poses.at(0).pose.position.x, path1.poses.at(0).pose.position.y, path1.poses.at(0).pose.position.z,
                                          path1.poses.at(1).pose.position.x, path1.poses.at(1).pose.position.y, path1.poses.at(1).pose.position.z);
    }
    else
    {
        dist_1 = 0.0;
    }
    if (path2.poses.size() > 0)
    {
        dist_2 = calculateDistance2Points(path2.poses.at(0).pose.position.x, path2.poses.at(0).pose.position.y, path2.poses.at(0).pose.position.z,
                                          path2.poses.at(1).pose.position.x, path2.poses.at(1).pose.position.y, path2.poses.at(1).pose.position.z);
    }
    else
    {
        dist_2 = 0.0;
    }
    if (path3.poses.size() > 0)
    {
        dist_3 = calculateDistance2Points(path3.poses.at(0).pose.position.x, path3.poses.at(0).pose.position.y, path3.poses.at(0).pose.position.z,
                                          path3.poses.at(1).pose.position.x, path3.poses.at(1).pose.position.y, path3.poses.at(1).pose.position.z);
    }
    else
    {
        dist_3 = 0.0;
    }

    if (contTest == 0)
    {
        time_start = ros::Time::now().toSec();
        contTest++;
    }

    if (dist_1 != 0 && dist_2 != 0 && dist_3 != 0)
    {
        if (b == true)
        {
            file_distance_to_posts << dist_1 << " " << dist_2 << " " << dist_3 << std::endl;
            file_distance_to_posts_times << ros::Time::now().toSec() - time_start << std::endl;
        }
        else
        {
            file_distance_between_posts << dist_1 << " " << dist_2 << " " << dist_3 << std::endl;
            file_distance_between_posts_times << ros::Time::now().toSec() - time_start << std::endl;
        }
    }
}
