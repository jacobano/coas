#include <detection/euclideanClusterer.h>

EuclideanClusterer::EuclideanClusterer()
{
    n = ros::NodeHandle();

    // params();

    // Subscriptions
    velodyne_sub = n.subscribe("/volexFilterPoints", 1, &EuclideanClusterer::cloud_cb, this);
    sub_phase = n.subscribe("/phase", 1, &EuclideanClusterer::phase_cb, this);

    // Publishers
    pub_pointclouds = n.advertise<detection::vectorPointCloud>("/vector_pointclouds", 1);
    loop();
}

EuclideanClusterer::~EuclideanClusterer()
{
}

void EuclideanClusterer::phase_cb(const std_msgs::Int8 phaseMode)
{
    phase = phaseMode.data;
    switch (phase)
    {
    // Atraque
    case 1:
        distanceThreshold = 0.4;
        clusterTolerance = 0.8;
        minClusterSize = 30;
        maxClusterSize = 600;
        break;
    // Puerto
    case 2:
        distanceThreshold = 0.5;
        clusterTolerance = 0.8;
        minClusterSize = 30;
        maxClusterSize = 25000;
        break;
    // Mar abierto
    case 3:
        distanceThreshold = 0.5;
        clusterTolerance = 0.8;
        minClusterSize = 2;
        maxClusterSize = 25000;
        break;
    }
}

void EuclideanClusterer::params()
{
    // ros::NodeHandle nparam("~");
    // if (nparam.getParam("distanceThreshold", distanceThreshold))
    // {
    //     ROS_WARN("Got EuclideanClusterer param distanceThreshold: %f", distanceThreshold);
    // }
    // else
    // {
    //     distanceThreshold = 0.02;
    //     ROS_WARN("Failed to get EuclideanClusterer param distanceThreshold: %f", distanceThreshold);
    // }
    // if (nparam.getParam("clusterTolerance", clusterTolerance))
    // {
    //     ROS_WARN("Got EuclideanClusterer param param clusterTolerance: %f", clusterTolerance);
    // }
    // else
    // {
    //     clusterTolerance = 0.02;
    //     ROS_WARN("Failed to get EuclideanClusterer param clusterTolerance: %f", clusterTolerance);
    // }
    // if (nparam.getParam("minClusterSize", minClusterSize))
    // {
    //     ROS_WARN("Got EuclideanClusterer param minClusterSize: %f", minClusterSize);
    // }
    // else
    // {
    //     minClusterSize = 100;
    //     ROS_WARN("Failed to get EuclideanClusterer param minClusterSize: %f", minClusterSize);
    // }
    // if (nparam.getParam("maxClusterSize", maxClusterSize))
    // {
    //     ROS_WARN("Got EuclideanClusterer param maxClusterSize: %f", maxClusterSize);
    // }
    // else
    // {
    //     maxClusterSize = 25000;
    //     ROS_WARN("Failed to get EuclideanClusterer param maxClusterSize: %f", maxClusterSize);
    // }
    // if (nparam.getParam("phase", phase))
    // {
    //     ROS_WARN("Got EuclideanClusterer param phase: %f", phase);
    // }
    // else
    // {
    //     ROS_WARN("Failed to get EuclideanClusterer param phase: %f", phase);
    // }
}

void EuclideanClusterer::cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
{
    ros::Time begin = ros::Time::now();

    sensor_msgs::PointCloud2 input_cloud = *input;
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_XYZ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    // Cambio de tipo de variable de sensor_msgs::PointCloud2 a pcl::PointXYZ
    pcl::fromROSMsg(input_cloud, *downsampled_XYZ);
    // Crea el objeto SACSegmentation y define el model y el tipo de metodo
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Crea el objeto SACSegmentation
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Opcional
    seg.setOptimizeCoefficients(true);
    // Necesario
    seg.setModelType(pcl::SACMODEL_PLANE);
    // Más info: wikipedia.org/wiki/RANSAC
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    // Como de cerca tiene que estar un punto del modelo para considerarlo en línea
    seg.setDistanceThreshold(distanceThreshold); // (default 0.02)

    int i = 0, nr_points = (int)downsampled_XYZ->points.size();

    // Contiene la nube de puntos del plano
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    if (phase == 1 || phase == 2)
    {
        float percentage;
        if (phase == 1)
            percentage = 0.3;
        if (phase == 2)
            percentage = 0.9;

        // Mientras el 30% [90%] de la nube original siga aqui
        while (downsampled_XYZ->points.size() > percentage * nr_points) // atraque = 0.9 // default = 0.3
        {
            // Segmenta la mayor componente playa de la nube de puntos restante
            seg.setInputCloud(downsampled_XYZ);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() == 0)
            {
                std::cerr << "No se puede estimar el modelo plano del dataset dado" << std::endl;
                break;
            }
            // Extrae el plano de la nube de puntos de entrada
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(downsampled_XYZ);
            extract.setIndices(inliers);
            extract.setNegative(false);
            // Obtiene los puntos asociados con la superficie plana
            extract.filter(*cloud_plane);
            // Elimina el plano, extrae el resto
            extract.setNegative(true);
            extract.filter(*cloud_f);
            downsampled_XYZ.swap(cloud_f);
            i++;
        }
    }
    // Crea el objeto KdTree para el método de búsqueda de extracción
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(downsampled_XYZ);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(clusterTolerance); // default = 0.02 (2cm)
    ec.setMinClusterSize(minClusterSize);     // Mar = 2 // Atraque = 30 // default = 100
    ec.setMaxClusterSize(maxClusterSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(downsampled_XYZ);
    ec.extract(cluster_indices);

    // Crea un publicador para cada cluster
    for (int i = 0; i < cluster_indices.size(); ++i)
    {
        std::string topicName = "/cluster" + boost::lexical_cast<std::string>(i);
        ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>(topicName, 1);
        pub_vec.push_back(pub);
    }
    int j = 0;
    detection::vectorPointCloud vector_pointclouds;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
            cloud_cluster->points.push_back(downsampled_XYZ->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        // Convierte la nube de puntos en el tipo de mensaje para ser usado en ROS
        sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*cloud_cluster, *output);
        output->header.frame_id = input_cloud.header.frame_id;
        vector_pointclouds.clouds.push_back(*output);
        pub_vec[j].publish(output);
        ++j;
    }

    pub_pointclouds.publish(vector_pointclouds);
    std::cout << "[ EUCL] Time: " << ros::Time::now() - begin << std::endl;
}

void EuclideanClusterer::loop()
{
    while (ros::ok())
    {
        sleep(0.1);
        ros::spinOnce();
    }
}