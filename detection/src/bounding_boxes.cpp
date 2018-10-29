#include <detection/bounding_boxes.h>

BoundingBoxes::BoundingBoxes():
nh_()
{
    // params();

    // Subscriptions
    sub_filter_points_ = nh_.subscribe("/filter_points", 1, &BoundingBoxes::cloudCallback, this);
    sub_phase_ = nh_.subscribe("/phase", 1, &BoundingBoxes::phaseCallback, this);

    // Publishers
    pub_boxes_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bounding_boxes", 1);
    pub_merge_boxes_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/merge_bounding_boxes", 1);
    pub_reference_boxes_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/reference_bounding_boxes", 1);
    pub_path_post_1_ = nh_.advertise<nav_msgs::Path>("/path_poste_1", 1);
    pub_path_post_2_ = nh_.advertise<nav_msgs::Path>("/path_poste_2", 1);
    pub_path_post_3_ = nh_.advertise<nav_msgs::Path>("/path_poste_3", 1);
    pub_path_post_12_ = nh_.advertise<nav_msgs::Path>("/path_poste_12", 1);
    pub_path_post_13_ = nh_.advertise<nav_msgs::Path>("/path_poste_13", 1);
    pub_path_post_23_ = nh_.advertise<nav_msgs::Path>("/path_poste_23", 1);


    char* envvar_home;
    envvar_home = std::getenv("HOME");
    std::stringstream log_output_aux_stream;
    log_output_aux_stream << envvar_home << "/Matlab_ws/";
    log_output_ = log_output_aux_stream.str();

    file_distance_to_posts_.open(log_output_ + "distancesToPosts");
    file_distance_between_posts_.open(log_output_ + "distancesBetweenPosts");
    file_distance_between_posts_times_.open(log_output_ + "timesBetweenPosts");
    file_distance_to_posts_times_.open(log_output_ + "timesToPosts");
    file_post_1_.open(log_output_ + "post1");
    file_post_2_.open(log_output_ + "post2");
    file_post_3_.open(log_output_ + "post3");
    file_post_1_time_.open(log_output_ + "post1time");
    file_post_2_time_.open(log_output_ + "post2time");
    file_post_3_time_.open(log_output_ + "post3time");

    contTest_ = contTestPose_ = 0;
}

BoundingBoxes::~BoundingBoxes()
{
    file_distance_to_posts_.close();
    file_distance_between_posts_.close();
    file_distance_to_posts_times_.close();
    file_distance_between_posts_times_.close();
    file_post_1_.close();
    file_post_2_.close();
    file_post_3_.close();
    file_post_1_time_.close();
    file_post_2_time_.close();
    file_post_3_time_.close();
}

void BoundingBoxes::phaseCallback(const std_msgs::Int8 phaseMode)
{
    phase_ = phaseMode.data;
    switch (phase_)
    {
        // Docking
        case 1:
            // Euclidean Clusterer Parameters
            distance_threshold_ = 0.4;
            cluster_tolerance_ = 0.8;
            min_cluster_size_ = 30;
            max_cluster_size_ = 600;
            // Bounding Boxes parameters
            close_distance_ = 1.0;
            xy_min_post_ = 0.1;
            xy_max_post_ = 0.6;
            z_min_post_ = 1;
            z_max_post_ = 3.0;
            min_distance_post_12_ = 3.5;
            max_distance_post_12_ = 3.9;
            min_distance_post_13_ = 8.5;
            max_distance_post_13_ = 8.8;
            break;
        // Harbor_
        case 2:
            // Euclidean Clusterer Parameters
            distance_threshold_ = 0.5;
            cluster_tolerance_ = 0.8;
            min_cluster_size_ = 30;
            max_cluster_size_ = 25000;
            // Bounding Boxes parameters
            close_distance_ = 1.0;
            xy_min_post_ = 0.3;
            xy_max_post_ = 1.6;
            z_min_post_ = 0.5;
            z_max_post_ = 2.0;
            min_distance_post_12_ = 0.0;
            max_distance_post_12_ = 20.0;
            min_distance_post_13_ = 15.0;
            max_distance_post_13_ = 17.0;
            break;
        // Sea
        case 3:
            // Euclidean Clusterer Parameters
            distance_threshold_ = 0.5;
            cluster_tolerance_ = 0.8;
            min_cluster_size_ = 2;
            max_cluster_size_ = 25000;
            // Bounding Boxes parameters
            close_distance_ = 5.0;
            xy_min_post_ = 0.0;
            xy_max_post_ = 0.0;
            z_min_post_ = 0.0;
            z_max_post_ = 0.0;
            min_distance_post_12_ = 0.0;
            max_distance_post_12_ = 0.0;
            min_distance_post_13_ = 0.0;
            max_distance_post_13_ = 0.0;
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
    seg.setDistanceThreshold(distance_threshold_); // (default 0.02)

    int i = 0, nr_points = (int)downsampled_XYZ->points.size();

    // Contains the point cloud of the plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    if (phase_ == 1 || phase_ == 2)
    {
        float percentage;
        if (phase_ == 1)
            percentage = 0.3;
        if (phase_ == 2)
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
    ec.setClusterTolerance(cluster_tolerance_); // default = 0.02 (2cm)
    ec.setMinClusterSize(min_cluster_size_);    // Sea = 2 // Docking = 30 // default = 100
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(downsampled_XYZ);
    ec.extract(cluster_indices);

    // Create a publisher for each cluster
    for (int i = 0; i < cluster_indices.size(); ++i)
    {
        std::string topicName = "/cluster" + boost::lexical_cast<std::string>(i);
        ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>(topicName, 1);
        pub_vec_point_clouds_.push_back(pub);
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
        pub_vec_point_clouds_[j].publish(output);
        ++j;
    }

    std::cout << "[ EUCL] Time: " << ros::Time::now().toSec() - time_start << std::endl;

    //*****************************************************************************
    // Bounding Boxes part
    time_start = ros::Time::now().toSec();
    if (!clusters_vector.clouds.empty())
    {
        label_box_ = label_merge_box_ = 0;
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
            pcl::compute3DCentroid(*temp_cloud, centroid_);
            // Calculate maximum distances of the cluster
            calculateMaxDistancesCluster(*temp_cloud);
            // Calculate center of the cluster
            calculateCenters();
            // Construct bounding box
            constructBoundingBoxes(x_center_, y_center_, z_center_, max_dist_x_, max_dist_y_, max_dist_z_, false);
        }
        // Publish all boxes at one time and clean boxes vector
        pub_boxes_.publish(boxes_);
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
    max_dist_x_ = max_dist_y_ = max_dist_z_ = 0;
    x_max_ = x_min_ = y_max_ = y_min_ = z_max_ = z_min_ = 0;
    x_center_ = y_center_ = z_center_ = 0;
    box_.header.frame_id = boxes_.header.frame_id = "velodyne";
    reference_boxes_.header.frame_id = merge_boxes_.header.frame_id = "velodyne";
    centroid_[0] = centroid_[1] = centroid_[2] = centroid_[3] = 0.0;
}

void BoundingBoxes::cleanVariables()
{
    vec_vec_label_polygon_.clear();
    vec_label_polygon_.clear();
    boxes_.boxes.clear();
    reference_boxes_.boxes.clear();
    merge_boxes_.boxes.clear();
    vec_vec_label_polygon_.clear();
    path_post_1_.poses.clear();
    path_post_2_.poses.clear();
    path_post_3_.poses.clear();
    path_post_12_.poses.clear();
    path_post_13_.poses.clear();
    path_post_23_.poses.clear();
    counter_posts_ = 0;
}

void BoundingBoxes::calculateMaxDistancesCluster(const pcl::PointCloud<pcl::PointXYZ> cluster)
{
    // Compare every single point with the rest of the point cloud
    for (int i = 0; i < cluster.points.size(); i++)
    {
        for (int j = 0; j < cluster.points.size(); j++)
        {
            // Storage the largest distance in X
            if (max_dist_x_ < fabs(cluster.points[i].x - cluster.points[j].x))
            {
                max_dist_x_ = fabs(cluster.points[i].x - cluster.points[j].x);
                if (cluster.points[i].x > cluster.points[j].x)
                {
                    x_max_ = cluster.points[i].x;
                    x_min_ = cluster.points[j].x;
                }
                else
                {
                    x_max_ = cluster.points[j].x;
                    x_min_ = cluster.points[i].x;
                }
            }
            // Storage the largest distance in Y
            if (max_dist_y_ < fabs(cluster.points[i].y - cluster.points[j].y))
            {
                max_dist_y_ = fabs(cluster.points[i].y - cluster.points[j].y);
                if (cluster.points[i].y > cluster.points[j].y)
                {
                    y_max_ = cluster.points[i].y;
                    y_min_ = cluster.points[j].y;
                }
                else
                {
                    y_max_ = cluster.points[j].y;
                    y_min_ = cluster.points[i].y;
                }
            }
            // Storage the largest distance in Z
            if (max_dist_z_ < fabs(cluster.points[i].z - cluster.points[j].z))
            {
                max_dist_z_ = fabs(cluster.points[i].z - cluster.points[j].z);
                if (cluster.points[i].z > cluster.points[j].z)
                {
                    z_max_ = cluster.points[i].z;
                    z_min_ = cluster.points[j].z;
                }
                else
                {
                    z_max_ = cluster.points[j].z;
                    z_min_ = cluster.points[i].z;
                }
            }
        }
    }
}

void BoundingBoxes::calculateCenters()
{
    if (max_dist_x_ > max_dist_y_)
    {
        x_center_ = x_min_ + (max_dist_x_ / 2);
        y_center_ = y_min_ + (max_dist_y_ / 2);
    }
    if (max_dist_x_ < max_dist_y_)
    {
        x_center_ = x_min_ + (max_dist_x_ / 2);
        y_center_ = y_min_ + (max_dist_y_ / 2);
    }
    z_center_ = z_min_ + (max_dist_z_ / 2);
}

void BoundingBoxes::constructBoundingBoxes(float x, float y, float z, float dimX, float dimY, float dimZ, bool merge)
{
    box_.pose.position.x = x;
    box_.pose.position.y = y;
    box_.pose.position.z = z;
    if (merge == false)
    {
        box_.dimensions.x = dimX;
        box_.dimensions.y = dimY;
        box_.dimensions.z = dimZ;
        box_.label = label_box_;
        // If it is at docking mode search and valid posts
        if (phase_ == 1)
        {
            checkPostDimension(dimX, dimY, dimZ);
        }
        boxes_.boxes.push_back(box_);
        label_box_++;
    }
    if (merge == true)
    {
        box_.dimensions.x = dimX * 2;
        box_.dimensions.y = dimY * 2;
        box_.dimensions.z = dimZ * 2;
        box_.label = label_merge_box_;
        merge_boxes_.boxes.push_back(box_);
        label_merge_box_++;
    }
}

void BoundingBoxes::checkPostDimension(float xDim, float yDim, float zDim)
{
    // Check if the actual bounding box is a post
    if ( (xy_min_post_ < xDim) && (xDim < xy_max_post_) && 
         (xy_min_post_ < yDim) && (yDim < xy_max_post_) && 
         (z_min_post_ < zDim) && (zDim < z_max_post_) )
    {
        // Distance to the post
        float dist = calculateDistance2Points(0, 0, 0, box_.pose.position.x, box_.pose.position.y, box_.pose.position.z);
        // Prepare the path
        std::vector<float> x1, y1, z1;
        x1.push_back(0.0);
        y1.push_back(0.0);
        z1.push_back(0.0);
        x1.push_back(box_.pose.position.x);
        y1.push_back(box_.pose.position.y);
        z1.push_back(box_.pose.position.z);
        // Construct the path and put the bounding box in the reference array
        switch (counter_posts_)
        {
            case 0:
                path_post_1_ = constructPath(x1, y1, z1, 2);
                reference_boxes_.boxes.push_back(box_);
                break;
            case 1:
                path_post_2_ = constructPath(x1, y1, z1, 2);
                reference_boxes_.boxes.push_back(box_);
                break;
            case 2:
                path_post_3_ = constructPath(x1, y1, z1, 2);
                reference_boxes_.boxes.push_back(box_);
                break;
        }
        // If there is more than one post detected
        if (reference_boxes_.boxes.size() > 1)
        {
            int counter_between_posts = 0;
            // Calculate the distance between all posts
            for (int i = 0; i < reference_boxes_.boxes.size(); i++)
            {
                for (int j = i + 1; j < reference_boxes_.boxes.size(); j++)
                {
                    jsk_recognition_msgs::BoundingBox aux_box_1 = reference_boxes_.boxes.at(i);
                    jsk_recognition_msgs::BoundingBox aux_box_2 = reference_boxes_.boxes.at(j);
                    float dist = calculateDistance2Points(aux_box_1.pose.position.x, aux_box_1.pose.position.y, aux_box_1.pose.position.z,
                                                          aux_box_2.pose.position.x, aux_box_2.pose.position.y, aux_box_2.pose.position.z);
                    // Prepare the path
                    std::vector<float> x0, y0, z0;
                    x0.push_back(reference_boxes_.boxes.at(i).pose.position.x);
                    y0.push_back(reference_boxes_.boxes.at(i).pose.position.y);
                    z0.push_back(reference_boxes_.boxes.at(i).pose.position.z);
                    x0.push_back(reference_boxes_.boxes.at(j).pose.position.x);
                    y0.push_back(reference_boxes_.boxes.at(j).pose.position.y);
                    z0.push_back(reference_boxes_.boxes.at(j).pose.position.z);
                    // If the calculated distances match with the searched distances
                    if ( ((min_distance_post_12_ < dist) && (dist < max_distance_post_12_)) || 
                         ((min_distance_post_13_ < dist) && (dist < max_distance_post_13_)) )
                    {
                        // Construct the path to represent distances between posts
                        switch (counter_between_posts)
                        {
                            case 0:
                                path_post_12_ = constructPath(x0, y0, z0, 2);
                                break;
                            case 1:
                                path_post_13_ = constructPath(x0, y0, z0, 2);
                                break;
                            case 2:
                                path_post_23_ = constructPath(x0, y0, z0, 2);
                                break;
                        }
                        counter_between_posts++;
                    }
                }
            }
        }
        counter_posts_++;
        // Publishers
        pub_reference_boxes_.publish(reference_boxes_);
        pub_path_post_1_.publish(path_post_1_);
        pub_path_post_2_.publish(path_post_2_);
        pub_path_post_3_.publish(path_post_3_);
        pub_path_post_12_.publish(path_post_12_);
        pub_path_post_13_.publish(path_post_13_);
        pub_path_post_23_.publish(path_post_23_);
        // Save logs
        saveDistances(true, path_post_1_, path_post_2_, path_post_3_);
        saveDistances(false, path_post_12_, path_post_13_, path_post_23_);
        if (contTestPose_ == 0)
        {
            time_pose_start_ = ros::Time::now().toSec();
            contTestPose_++;
        }
        if (path_post_1_.poses.size() > 0)
        {
            savePose(1, path_post_1_);
        }
        if (path_post_2_.poses.size() > 0)
        {
            savePose(2, path_post_2_);
        }
        if (path_post_3_.poses.size() > 0)
        {
            savePose(3, path_post_3_); 
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
    for (int i = 0; i < boxes_.boxes.size(); i++)
    {
        found = false;
        for (int j = i + 1; j < boxes_.boxes.size(); j++)
        {
            jsk_recognition_msgs::BoundingBox aux_box_1 = boxes_.boxes[i];
            jsk_recognition_msgs::BoundingBox aux_box_2 = boxes_.boxes[j];
            dist = calculateDistance2Points(aux_box_1.pose.position.x, aux_box_1.pose.position.y, aux_box_1.pose.position.z,
                                            aux_box_2.pose.position.x, aux_box_2.pose.position.y, aux_box_2.pose.position.z);

            // If the label is in another polygon do not use it in a new polygon
            repeat = false;
            for (int k = 0; k < vec_vec_label_polygon_.size(); k++)
            {
                // Check the label of the iterator i
                if (std::find(vec_vec_label_polygon_[k].begin(), vec_vec_label_polygon_[k].end(), boxes_.boxes[i].label) != vec_vec_label_polygon_[k].end())
                {
                    repeat = true;
                }
                // Check the label of the iterator j
                if (std::find(vec_vec_label_polygon_[k].begin(), vec_vec_label_polygon_[k].end(), boxes_.boxes[j].label) != vec_vec_label_polygon_[k].end())
                {
                    repeat = true;
                }
            }
            // If they are close
            if (0.01 < dist && dist < close_distance_ && repeat == false)
            {
                // If the polygon is empty put in i instead of j
                if (vec_label_polygon_.empty())
                {
                    vec_label_polygon_.push_back(boxes_.boxes[i].label);
                }
                if (!vec_label_polygon_.empty())
                {
                    if (std::find(vec_label_polygon_.begin(), vec_label_polygon_.end(), 
                        boxes_.boxes[j].label) != vec_label_polygon_.end() )
                    {
                        // Repeated
                    }
                    else
                    {
                        // Not repeated
                        vec_label_polygon_.push_back(boxes_.boxes[j].label);
                    }
                }
                found = true;
            }
        }
        // If no label[j] has been found near label[i].
        if (found == false)
        {
            // If the polygon is not empty storage it in the polygons vector
            if (!vec_label_polygon_.empty())
            {
                vec_vec_label_polygon_.push_back(vec_label_polygon_);
            }
            else
            {
                // Check that label i is not in another polygon
                for (int k = 0; k < vec_vec_label_polygon_.size(); k++)
                {
                    // Check label of the iterator i
                    if (std::find(vec_vec_label_polygon_[k].begin(), vec_vec_label_polygon_[k].end(), 
                        boxes_.boxes[i].label) != vec_vec_label_polygon_[k].end())
                    {
                        repeat = true;
                    }
                }
                // If not, storage it like a polygon formed by one label
                if (repeat == false)
                {
                    vec_label_polygon_.push_back(boxes_.boxes[i].label);
                    vec_vec_label_polygon_.push_back(vec_label_polygon_);
                }
            }
            vec_label_polygon_.clear();
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
    for (int i = 0; i < vec_vec_label_polygon_.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ> polygon_cloud_temp;
        vec_label_polygon_.clear();
        vec_label_polygon_ = vec_vec_label_polygon_[i];
        // Merge all point clouds (clusters) that form a polygon in a single point cloud
        for (int j = 0; j < vec_label_polygon_.size(); j++)
        {
            // Transform the bounding box centroid into a PC2 point
            pcl::PointXYZ point;
            point.x = boxes_.boxes[vec_label_polygon_[j]].pose.position.x;
            point.y = boxes_.boxes[vec_label_polygon_[j]].pose.position.y;
            point.z = boxes_.boxes[vec_label_polygon_[j]].pose.position.z;
            // Create a point cloud with every centroid of the near bounding boxes
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            temp_cluster->points.push_back(point);
            polygon_cloud_temp += *temp_cluster;
        }
        // Calculate the centroid of this single point cloud
        Eigen::Vector4f centroid_polygon_cloud;
        pcl::compute3DCentroid(polygon_cloud_temp, centroid_polygon_cloud);
        // Calculate dimensions for the new bounding box using maximum distances
        max_dist_x_polygon_ = max_dist_y_polygon_ = max_dist_z_polygon_ = 0;
        for (int k = 0; k < polygon_cloud_temp.size(); k++)
        {
            if ( (fabs(centroid_polygon_cloud[0] - boxes_.boxes[vec_label_polygon_[k]].pose.position.x) + 
                  boxes_.boxes[vec_label_polygon_[k]].dimensions.x) > max_dist_x_polygon_)
            {
                max_dist_x_polygon_ = fabs(centroid_polygon_cloud[0]
                                           - boxes_.boxes[vec_label_polygon_[k]].pose.position.x)  
                                      + boxes_.boxes[vec_label_polygon_[k]].dimensions.x;
            }
            if ( (fabs(centroid_polygon_cloud[1] - boxes_.boxes[vec_label_polygon_[k]].pose.position.y) + 
                  boxes_.boxes[vec_label_polygon_[k]].dimensions.y) > max_dist_y_polygon_)
            {
                max_dist_y_polygon_ = fabs(centroid_polygon_cloud[1] 
                                           - boxes_.boxes[vec_label_polygon_[k]].pose.position.y) 
                                      + boxes_.boxes[vec_label_polygon_[k]].dimensions.y;
            }
            if ( (fabs(centroid_polygon_cloud[2] - boxes_.boxes[vec_label_polygon_[k]].pose.position.z) + 
                  boxes_.boxes[vec_label_polygon_[k]].dimensions.z) > max_dist_z_polygon_)
            {
                max_dist_z_polygon_ = fabs(centroid_polygon_cloud[2] 
                                           - boxes_.boxes[vec_label_polygon_[k]].pose.position.z) 
                                      + boxes_.boxes[vec_label_polygon_[k]].dimensions.z;
            }
        }
        // Create the bounding box that includes the others
        constructBoundingBoxes(centroid_polygon_cloud[0], centroid_polygon_cloud[1], centroid_polygon_cloud[2], 
                               max_dist_x_polygon_, max_dist_y_polygon_, max_dist_z_polygon_, true);
    }
    pub_merge_boxes_.publish(merge_boxes_);
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
        file_post_1_ << path.poses.at(1).pose.position.x << " " << path.poses.at(1).pose.position.y << std::endl;
        file_post_1_time_ << ros::Time::now().toSec() - time_pose_start_ << std::endl;
        break;
    case 2:
        file_post_2_ << path.poses.at(1).pose.position.x << " " << path.poses.at(1).pose.position.y << std::endl;
        file_post_2_time_ << ros::Time::now().toSec() - time_pose_start_ << std::endl;
        break;
    case 3:
        file_post_3_ << path.poses.at(1).pose.position.x << " " << path.poses.at(1).pose.position.y << std::endl;
        file_post_3_time_ << ros::Time::now().toSec() - time_pose_start_ << std::endl;
        break;
    }
}

void BoundingBoxes::saveDistances(bool b, nav_msgs::Path path1, nav_msgs::Path path2, nav_msgs::Path path3)
{
    float dist_1, dist_2, dist_3;
    double time_start;
    geometry_msgs::Pose pose_1;
    geometry_msgs::Pose pose_2;
    if (path1.poses.size() > 0)
    {
        pose_1 = path1.poses.at(0).pose;
        pose_2 = path1.poses.at(1).pose;
        dist_1 = calculateDistance2Points(pose_1.position.x, pose_1.position.y, pose_1.position.z,
                                          pose_2.position.x, pose_2.position.y, pose_2.position.z);
    }
    else
    {
        dist_1 = 0.0;
    }

    if (path2.poses.size() > 0)
    {
        pose_1 = path2.poses.at(0).pose;
        pose_2 = path2.poses.at(1).pose;
        dist_2 = calculateDistance2Points(pose_1.position.x, pose_1.position.y, pose_1.position.z,
                                          pose_2.position.x, pose_2.position.y, pose_2.position.z);
    }
    else
    {
        dist_2 = 0.0;
    }

    if (path3.poses.size() > 0)
    {
        pose_1 = path3.poses.at(0).pose;
        pose_2 = path3.poses.at(1).pose;
        dist_3 = calculateDistance2Points(pose_1.position.x, pose_1.position.y, pose_1.position.z,
                                          pose_2.position.x, pose_2.position.y, pose_2.position.z);
    }
    else
    {
        dist_3 = 0.0;
    }

    if (contTest_ == 0)
    {
        time_start = ros::Time::now().toSec();
        contTest_++;
    }

    if (dist_1 != 0 && dist_2 != 0 && dist_3 != 0)
    {
        if (b == true)
        {
            file_distance_to_posts_ << dist_1 << " " << dist_2 << " " << dist_3 << std::endl;
            file_distance_to_posts_times_ << ros::Time::now().toSec() - time_start << std::endl;
        }
        else
        {
            file_distance_between_posts_ << dist_1 << " " << dist_2 << " " << dist_3 << std::endl;
            file_distance_between_posts_times_ << ros::Time::now().toSec() - time_start << std::endl;
        }
    }
}
