#include <filtering/sensor_filter.h>

SensorFilter::SensorFilter()
{
    n = ros::NodeHandle();

    // // Subscriptions
    // matrix_sub = n.subscribe("/v_map", 1, &SensorFilter::matrix_cb, this);
    sub_phase = n.subscribe("/phase", 1, &SensorFilter::phaseCallback, this);
    sub_sensor = n.subscribe("/velodyne_points", 1, &SensorFilter::sensorCallback, this);

    // // Publishers
    pub_filtered_cloud_3D = n.advertise<sensor_msgs::PointCloud2>("/filter_points", 1);
    pub_matrix = n.advertise<filtering::VectorVector>("/v_map", 1);

    range_dock = 10;
    range_sea = 100;
    cell_div;
    range = range_dock * cell_div;
    rows = 2 * range + 1;
    columns = 2 * range + 1;

    loop();
}

SensorFilter::~SensorFilter()
{
}

void SensorFilter::phaseCallback(const std_msgs::Int8 phase_mode)
{
    phase = phase_mode.data;
    switch (phase)
    {
    // Docking
    case 1:
        range_dock = 20;
        cell_div = 2;
        break;
    // Harbor
    case 2:
        range_dock = 10;
        cell_div = 2;
        break;
    // Sea
    case 3:
        range_dock = 100;
        cell_div = 1; // Check this. Default = 2 but it was too slow.
        break;
    }
    range = range_dock * cell_div;
    rows = 2 * range + 1;
    columns = 2 * range + 1;
}

void SensorFilter::sensorCallback(const sensor_msgs::PointCloud2 &cloud)
{
    double begin = ros::Time::now().toSec();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_XYZ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr data_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Change from type sensor_msgs::PointCloud2 to pcl::PointXYZ
    pcl::fromROSMsg(cloud, *cloud_XYZ);

    int size_cloud_input = cloud_XYZ->points.size();
    bool empty_pattern = true;

    if (empty_pattern == true)
    {
        switch (phase)
        {
        // Docking
        case 1:
            for (int i = 0; i < size_cloud_input - 1; i++)
            {
                float x = cloud_XYZ->points[i].x;
                float y = cloud_XYZ->points[i].y;
                float z = cloud_XYZ->points[i].z;
                // Lateral
                if (x > -1 && x < 1 && abs(y) < range_dock && abs(y) > 1)
                {
                    data_cloud->points.push_back(cloud_XYZ->points[i]);
                }
                // Front
                if (x > 1 && x < range_dock && abs(y) < range_dock)
                {
                    data_cloud->points.push_back(cloud_XYZ->points[i]);
                }
                // Back
                if (x > -range_dock && x < -1 && abs(y) < range_dock)
                {
                    data_cloud->points.push_back(cloud_XYZ->points[i]);
                }
            }
            break;
        // Harbor
        case 2:
            for (int i = 0; i < size_cloud_input - 1; i++)
            {
                float x = cloud_XYZ->points[i].x;
                float y = cloud_XYZ->points[i].y;
                float z = cloud_XYZ->points[i].z;
                // Lateral
                if (x > -3.6 && x < 2.4 && abs(y) < range_dock && abs(y) > 1.2)
                {
                    data_cloud->points.push_back(cloud_XYZ->points[i]);
                }
                // Front
                if (x > 2.4 && x < range_dock && abs(y) < range_dock)
                {
                    data_cloud->points.push_back(cloud_XYZ->points[i]);
                }
                // Back
                if (x > -range_dock && x < -3.6 && abs(y) < range_dock)
                {
                    data_cloud->points.push_back(cloud_XYZ->points[i]);
                }
            }
            break;
        // Sea
        case 3:
            for (int i = 0; i < size_cloud_input - 1; i++)
            {
                float x = cloud_XYZ->points[i].x;
                float y = cloud_XYZ->points[i].y;
                float z = cloud_XYZ->points[i].z;
                //if (abs(y) < range_sea && abs(x) < range_sea )
                //if (abs(y) < range_open && abs(x) < range_open )
                //if ((y < 100 || y > -100) && (x < 100 || x > -100))
                if ((y > 50) && (x < 100 || x > -100))
                {
                    data_cloud->points.push_back(cloud_XYZ->points[i]);
                }
            }
            break;
        }
        // Convert the pointcloud to be used in ROS, using variable sensor_msgs::PointCloud2 output;
        sensor_msgs::PointCloud2 cloud_filtered;
        pcl::toROSMsg(*data_cloud, cloud_filtered);
        cloud_filtered.header.frame_id = cloud.header.frame_id;
        // BUILD THE MAP (PointCloud2D)
        exploration(cloud_filtered);
    }
    std::cout << "[ MAPP] Time: " << ros::Time::now().toSec() - begin << std::endl;
}

// BUILD THE MAP
//   V[i][j][k] is:
//   0 if empty or unexplored
//   1 if there is an obstacle
//   2 virtual obstacle
//   4 if it is the robot's cell
//   5 goal
//   >=10 if the cell surrounds an obstacle. 11 and 12 are safer than 10.
void SensorFilter::exploration(const sensor_msgs::PointCloud2 &cloud_3D_uas)
{
    sensor_msgs::PointCloud2 filtered_cloud_3D;

    VVVI V(rows, VVI(columns, VI(2)));
    VVVI V_map(rows, VVI(columns, VI(2)));
    VVVI cont_V(rows, VVI(columns, VI(2)));
    // Initialize the matrix
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < columns; j++)
        {
            for (int k = 0; k < 2; k++)
            {
                if (k == 0)
                {
                    V[i][j][k] = 0;
                    cont_V[i][j][k] = 0;
                    V_map[i][j][k] = 0;
                }
            }
        }
    }
    // Indicate the cell where the vessel is.
    V[range][range][0] = 4; //Position of the sensor (it is always the same because the map is local)

    int tam = cloud_3D_uas.width * cloud_3D_uas.height;

    pcl::PointCloud<pcl::PointXYZ>::Ptr fil_cloud_XYZ(new pcl::PointCloud<pcl::PointXYZ>);  //Data to use in the map.
    pcl::PointCloud<pcl::PointXYZ>::Ptr fil_data_cloud(new pcl::PointCloud<pcl::PointXYZ>); //Filtered point cloud to publish
    // Change from type sensor_msgs::PointCloud2 to pcl::PointXYZ
    pcl::fromROSMsg(cloud_3D_uas, *fil_cloud_XYZ);
    // Build the matrix
    int max = 0;
    double cloud_thres_distance = 0.5; // Meters
    for (int i = 0; i < tam; i++)      // nlements --> size_cloud_input
    {
        float cloud_dist = hypot(fil_cloud_XYZ->points[i].x, fil_cloud_XYZ->points[i].y);
        if (cloud_dist >= cloud_thres_distance)
        {
            int iM = round(range - cell_div * fil_cloud_XYZ->points[i].x);
            int jM = round(range - cell_div * fil_cloud_XYZ->points[i].y);
            if (cloud_dist < range && is_in_map(iM, jM) == 1 && readV(V, iM, jM) != 4)
            {
                if (V[iM][jM][0] != 1)
                {
                    V[iM][jM][0] = 1; //JAC
                    cont_V[iM][jM][0] = cont_V[iM][jM][0] + 1;
                }
                if (V[iM][jM][0] == 1 && cont_V[iM][jM][0] >= 1)
                {
                    cont_V[iM][jM][0] = cont_V[iM][jM][0] + 1;
                }
                if (cont_V[iM][jM][0] > max)
                {
                    max = cont_V[iM][jM][0];
                }
            }
        }
    }
    // Filtered map to send to the collision avoidance module --> V_map
    V_map = V;
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < columns; j++)
        {
            for (int k = 0; k < 2; k++)
            {
                if (cont_V[i][j][0] < 1 && V[i][j][0] == 1) //5
                {
                    V_map[i][j][0] = 0;
                }
            }
        }
    }
    // Publish V
    filtering::VectorInt columns_mat;
    filtering::VectorVector rows_mat;
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < columns; j++)
        {
            columns_mat.columns.push_back(V[i][j][0]);
        }
        rows_mat.rows.push_back(columns_mat);
        columns_mat.columns.clear();
    }
    pub_matrix.publish(rows_mat);
    rows_mat.rows.clear();
    // This is to confirm on RViz that waves are filtered.
    // map1_
    for (int i = 0; i < tam; i++) // nlements --> size_cloud_input
    {
        float cloud_dist = hypot(fil_cloud_XYZ->points[i].x, fil_cloud_XYZ->points[i].y);
        if (cloud_dist >= cloud_thres_distance)
        {
            int iM = round(range - cell_div * fil_cloud_XYZ->points[i].x);
            int jM = round(range - cell_div * fil_cloud_XYZ->points[i].y);
            if (cloud_dist < range && is_in_map(iM, jM) == 1 && readV(V, iM, jM) != 4)
            {
                if (V[iM][jM][0] == 1 && V_map[iM][jM][0] == 0)
                {
                }
                else
                {
                    fil_data_cloud->points.push_back(fil_cloud_XYZ->points[i]);
                }
            }
        }
    }
    //Convert the pointcloud to be used in ROS, using variable sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*fil_data_cloud, filtered_cloud_3D);
    filtered_cloud_3D.header.frame_id = "velodyne";
    filtered_cloud_3D.header.stamp = ros::Time();
    // Publish filtered cloud
    pub_filtered_cloud_3D.publish(filtered_cloud_3D);

    // GENERATE FILES TO CHECK THE OUTPUTS AND THE GENERATION OF THE MATRIX.
    char* envvar_home;
    envvar_home = std::getenv("HOME");
    std::stringstream matrix_filename;
    std::stringstream counter_filename;
    std::stringstream map_filename;
    matrix_filename << envvar_home << "/Matlab_ws/matrix.txt";
    counter_filename << envvar_home << "/Matlab_ws/counter.txt";
    map_filename << envvar_home << "/Matlab_ws/map.txt";
    save_matrix3d(matrix_filename.str().c_str(), V, false);
    save_matrix3d(counter_filename.str().c_str(), cont_V, false);
    save_matrix3d(map_filename.str().c_str(), V_map, false);  
}

int SensorFilter::isInMap(int i, int j)
{
    if (i < 0 || i > rows - 1)
        return 0;
    if (j < 0 || j > columns - 1)
        return 0;
    return 1;
}

int SensorFilter::readV(const VVVI &V, int i, int j)
{
    if (i < rows && j < columns && i >= 0 && j >= 0)
        return V[i][j][0];
    else
        return -1;
}

// FILE TO SAVE A MATRIX
void SensorFilter::saveMatrix3D(char *file_name, const VVVI &m, bool vel)
{
    std::ofstream file;
    file.open(file_name);

    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < columns; j++)
        {
            if (vel == false)
                file << m[i][j][0] << " ";
            if (vel == true)
                file << m[i][j][1] << " ";
        }
        file << std::endl;
    }

    file.close();
}

void SensorFilter::loop()
{
    while (ros::ok())
    {
        sleep(0.1);
        ros::spinOnce();
    }
}
