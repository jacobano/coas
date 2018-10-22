#include <detection/matching.h>

Matching::Matching()
{
    n = ros::NodeHandle();

    // Subscriptions
    ros::Subscriber sub_path_post_2 = n.subscribe("/path_poste2", 1, &Matching::pathPost2Callback, this);
    ros::Subscriber sub_path_post_3 = n.subscribe("/path_poste3", 1, &Matching::pathPost3Callback, this);
    ros::Subscriber sub_path_post_1 = n.subscribe("/path_poste1", 1, &Matching::pathPost1Callback, this);
    ros::Subscriber sub_path_post_12 = n.subscribe("/path_poste12", 1, &Matching::pathPost12Callback, this);
    ros::Subscriber sub_path_post_13 = n.subscribe("/path_poste13", 1, &Matching::pathPost13Callback, this);
    ros::Subscriber sub_path_post_23 = n.subscribe("/path_poste23", 1, &Matching::pathPost23Callback, this);

    // Publishers
    pub_marker_1 = n.advertise<visualization_msgs::Marker>("/marker_post_1", 1);
    pub_marker_2 = n.advertise<visualization_msgs::Marker>("/marker_post_2", 1);
    pub_marker_3 = n.advertise<visualization_msgs::Marker>("/marker_post_3", 1);
    pub_marker_4 = n.advertise<visualization_msgs::Marker>("/marker_post_4", 1);
    pub_marker_5 = n.advertise<visualization_msgs::Marker>("/marker_post_5", 1);
    pub_marker_6 = n.advertise<visualization_msgs::Marker>("/marker_post_6", 1);

    logOutput = "/home/hector/matlab_ws/COAS/";

    file_post_1.open(logOutput + "matchPost1");
    file_post_2.open(logOutput + "matchPost2");
    file_post_3.open(logOutput + "matchPost3");
    file_post_1_time.open(logOutput + "matchPost1time");
    file_post_2_time.open(logOutput + "matchPost2time");
    file_post_3_time.open(logOutput + "matchPost3time");

    counter = 0;

    toDo();

    loop();
}

Matching::~Matching()
{
    file_post_1.close();
    file_post_2.close();
    file_post_3.close();
    file_post_1_time.close();
    file_post_2_time.close();
    file_post_3_time.close();
}

void Matching::pathPost1Callback(const nav_msgs::Path path)
{
    now_path_post_1 = path;
}

void Matching::pathPost2Callback(const nav_msgs::Path path)
{
    now_path_post_2 = path;
}

void Matching::pathPost3Callback(const nav_msgs::Path path)
{
    now_path_post_3 = path;
}

void Matching::pathPost12Callback(const nav_msgs::Path path)
{
    now_path_post_12 = path;
    toDo();
}

void Matching::pathPost13Callback(const nav_msgs::Path path)
{
    now_path_post_13 = path;
    toDo();
}

void Matching::pathPost23Callback(const nav_msgs::Path path)
{

    now_path_post_23 = path;
    toDo();
}

float Matching::calculateDistance2Points(float x1, float y1, float z1, float x2, float y2, float z2)
{
    float distance;
    distance = sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)) + ((z2 - z1) * (z2 - z1)));
    return distance;
}

void Matching::drawPosts()
{
    marker_post_1.header.frame_id = "velodyne";
    marker_post_1.header.stamp = ros::Time();
    marker_post_1.ns = "my_namespace";
    marker_post_1.id = 0;
    marker_post_1.type = visualization_msgs::Marker::CYLINDER;
    marker_post_1.action = visualization_msgs::Marker::ADD;
    marker_post_1.pose.position.x = prev_path_posts.poses.at(0).pose.position.x;
    marker_post_1.pose.position.y = prev_path_posts.poses.at(0).pose.position.y;
    marker_post_1.pose.position.z = prev_path_posts.poses.at(0).pose.position.z;
    marker_post_1.pose.orientation.x = 0.0;
    marker_post_1.pose.orientation.y = 0.0;
    marker_post_1.pose.orientation.z = 0.0;
    marker_post_1.pose.orientation.w = 1.0;
    marker_post_1.scale.x = 0.6;
    marker_post_1.scale.y = 0.6;
    marker_post_1.scale.z = 0.6;
    marker_post_1.color.a = 0.8;
    marker_post_1.color.r = 1.0;
    marker_post_1.color.g = 0.0;
    marker_post_1.color.b = 0.0;

    marker_post_2.header.frame_id = "velodyne";
    marker_post_2.header.stamp = ros::Time();
    marker_post_2.ns = "my_namespace";
    marker_post_2.id = 0;
    marker_post_2.type = visualization_msgs::Marker::CYLINDER;
    marker_post_2.action = visualization_msgs::Marker::ADD;
    marker_post_2.pose.position.x = prev_path_posts.poses.at(1).pose.position.x;
    marker_post_2.pose.position.y = prev_path_posts.poses.at(1).pose.position.y;
    marker_post_2.pose.position.z = prev_path_posts.poses.at(1).pose.position.z;
    marker_post_2.pose.orientation.x = 0.0;
    marker_post_2.pose.orientation.y = 0.0;
    marker_post_2.pose.orientation.z = 0.0;
    marker_post_2.pose.orientation.w = 1.0;
    marker_post_2.scale.x = 0.6;
    marker_post_2.scale.y = 0.6;
    marker_post_2.scale.z = 0.6;
    marker_post_2.color.a = 0.8;
    marker_post_2.color.r = 0.0;
    marker_post_2.color.g = 1.0;
    marker_post_2.color.b = 0.0;

    marker_post_3.header.frame_id = "velodyne";
    marker_post_3.header.stamp = ros::Time();
    marker_post_3.ns = "my_namespace";
    marker_post_3.id = 0;
    marker_post_3.type = visualization_msgs::Marker::CYLINDER;
    marker_post_3.action = visualization_msgs::Marker::ADD;
    marker_post_3.pose.position.x = prev_path_posts.poses.at(2).pose.position.x;
    marker_post_3.pose.position.y = prev_path_posts.poses.at(2).pose.position.y;
    marker_post_3.pose.position.z = prev_path_posts.poses.at(2).pose.position.z;
    marker_post_3.pose.orientation.x = 0.0;
    marker_post_3.pose.orientation.y = 0.0;
    marker_post_3.pose.orientation.z = 0.0;
    marker_post_3.pose.orientation.w = 1.0;
    marker_post_3.scale.x = 0.6;
    marker_post_3.scale.y = 0.6;
    marker_post_3.scale.z = 0.6;
    marker_post_3.color.a = 0.8;
    marker_post_3.color.r = 0.0;
    marker_post_3.color.g = 0.0;
    marker_post_3.color.b = 1.0;

    marker_post_4.header.frame_id = "velodyne";
    marker_post_4.header.stamp = ros::Time();
    marker_post_4.ns = "my_namespace";
    marker_post_4.id = 0;
    marker_post_4.type = visualization_msgs::Marker::CYLINDER;
    marker_post_4.action = visualization_msgs::Marker::ADD;
    marker_post_4.pose.position.x = now_path_posts.poses.at(0).pose.position.x;
    marker_post_4.pose.position.y = now_path_posts.poses.at(0).pose.position.y;
    marker_post_4.pose.position.z = now_path_posts.poses.at(0).pose.position.z;
    marker_post_4.pose.orientation.x = 0.0;
    marker_post_4.pose.orientation.y = 0.0;
    marker_post_4.pose.orientation.z = 0.0;
    marker_post_4.pose.orientation.w = 1.0;
    marker_post_4.scale.x = 0.3;
    marker_post_4.scale.y = 0.3;
    marker_post_4.scale.z = 1.0;
    marker_post_4.color.a = 1.0;
    marker_post_4.color.r = 0.5;
    marker_post_4.color.g = 0.5;
    marker_post_4.color.b = 0.0;

    marker_post_5.header.frame_id = "velodyne";
    marker_post_5.header.stamp = ros::Time();
    marker_post_5.ns = "my_namespace";
    marker_post_5.id = 0;
    marker_post_5.type = visualization_msgs::Marker::CYLINDER;
    marker_post_5.action = visualization_msgs::Marker::ADD;
    marker_post_5.pose.position.x = now_path_posts.poses.at(1).pose.position.x;
    marker_post_5.pose.position.y = now_path_posts.poses.at(1).pose.position.y;
    marker_post_5.pose.position.z = now_path_posts.poses.at(1).pose.position.z;
    marker_post_5.pose.orientation.x = 0.0;
    marker_post_5.pose.orientation.y = 0.0;
    marker_post_5.pose.orientation.z = 0.0;
    marker_post_5.pose.orientation.w = 1.0;
    marker_post_5.scale.x = 0.3;
    marker_post_5.scale.y = 0.3;
    marker_post_5.scale.z = 1.0;
    marker_post_5.color.a = 1.0;
    marker_post_5.color.r = 0.5;
    marker_post_5.color.g = 0.0;
    marker_post_5.color.b = 0.5;

    marker_post_6.header.frame_id = "velodyne";
    marker_post_6.header.stamp = ros::Time();
    marker_post_6.ns = "my_namespace";
    marker_post_6.id = 0;
    marker_post_6.type = visualization_msgs::Marker::CYLINDER;
    marker_post_6.action = visualization_msgs::Marker::ADD;
    marker_post_6.pose.position.x = now_path_posts.poses.at(2).pose.position.x;
    marker_post_6.pose.position.y = now_path_posts.poses.at(2).pose.position.y;
    marker_post_6.pose.position.z = now_path_posts.poses.at(2).pose.position.z;
    marker_post_6.pose.orientation.x = 0.0;
    marker_post_6.pose.orientation.y = 0.0;
    marker_post_6.pose.orientation.z = 0.0;
    marker_post_6.pose.orientation.w = 1.0;
    marker_post_6.scale.x = 0.3;
    marker_post_6.scale.y = 0.3;
    marker_post_6.scale.z = 1.0;
    marker_post_6.color.a = 1.0;
    marker_post_6.color.r = 0.0;
    marker_post_6.color.g = 0.5;
    marker_post_6.color.b = 0.5;

    pub_marker_1.publish(marker_post_1);
    pub_marker_2.publish(marker_post_2);
    pub_marker_3.publish(marker_post_3);
    pub_marker_4.publish(marker_post_4);
    pub_marker_5.publish(marker_post_5);
    pub_marker_6.publish(marker_post_6);
}

void Matching::savePose(int nPost, geometry_msgs::PoseStamped waypoint)
{
    switch (nPost)
    {
    case 0:
        file_post_1 << waypoint.pose.position.x << " " << waypoint.pose.position.y << std::endl;
        file_post_1_time << ros::Time::now().toSec() - startTimePose << std::endl;
        break;
    case 1:
        file_post_2 << waypoint.pose.position.x << " " << waypoint.pose.position.y << std::endl;
        file_post_2_time << ros::Time::now().toSec() - startTimePose << std::endl;
        break;
    case 2:
        file_post_3 << waypoint.pose.position.x << " " << waypoint.pose.position.y << std::endl;
        file_post_3_time << ros::Time::now().toSec() - startTimePose << std::endl;
        break;
    }
}

void Matching::toDo()
{
    marker_post_1.action = visualization_msgs::Marker::DELETE;
    pub_marker_1.publish(marker_post_1);
    marker_post_2.action = visualization_msgs::Marker::DELETE;
    pub_marker_2.publish(marker_post_2);
    marker_post_3.action = visualization_msgs::Marker::DELETE;
    pub_marker_3.publish(marker_post_3);
    marker_post_4.action = visualization_msgs::Marker::DELETE;
    pub_marker_4.publish(marker_post_4);
    marker_post_5.action = visualization_msgs::Marker::DELETE;
    pub_marker_5.publish(marker_post_5);
    marker_post_6.action = visualization_msgs::Marker::DELETE;
    pub_marker_6.publish(marker_post_6);
    // At the initial instant it storage the first entry as valid labels
    switch (counter)
    {
    case 0:
        // It initializes if there are three valid posts
        if (!now_path_post_12.poses.empty() && !now_path_post_13.poses.empty() && !now_path_post_23.poses.empty())
        {
            ROS_WARN("Init Matching");
            // It storages the three relevant waypoints of the actual state into the previous state vector
            prev_path_posts.poses.push_back(now_path_post_1.poses.at(1));
            prev_path_posts.poses.push_back(now_path_post_2.poses.at(1));
            prev_path_posts.poses.push_back(now_path_post_3.poses.at(1));
            counter++;
            startTimePose = ros::Time::now().toSec();
        }
        break;
    case 1:
        // If there are three valid posts and paths did not change
        if ((!now_path_post_12.poses.empty() && (!now_path_post_13.poses.empty() || !now_path_post_23.poses.empty())) ||
            (!now_path_post_13.poses.empty() && (!now_path_post_12.poses.empty() || !now_path_post_23.poses.empty())) ||
            (!now_path_post_23.poses.empty() && (!now_path_post_12.poses.empty() || !now_path_post_13.poses.empty())))
        {
            // Storage the six relevant waypoints of the actual state in a vector
            if (!now_path_post_12.poses.empty())
            {
                now_path_posts.poses.push_back(now_path_post_12.poses.at(0));
                now_path_posts.poses.push_back(now_path_post_12.poses.at(1));
            }
            if (!now_path_post_13.poses.empty())
            {
                now_path_posts.poses.push_back(now_path_post_13.poses.at(0));
                now_path_posts.poses.push_back(now_path_post_13.poses.at(1));
            }
            if (!now_path_post_23.poses.empty())
            {
                now_path_posts.poses.push_back(now_path_post_23.poses.at(0));
                now_path_posts.poses.push_back(now_path_post_23.poses.at(1));
            }
            std::vector<int> vec_labels;
            std::vector<float> vec_check_dist;
            // Compare each relevant waypoint of the previous state with the waypoints of the actual state
            for (int i = 0; i < prev_path_posts.poses.size(); i++)
            {
                for (int j = 0; j < now_path_posts.poses.size(); j++)
                {
                    // Calculate the distance between the waypoints of the previous state and the waypoints of the actual state
                    vec_distance_and_label[j] = calculateDistance2Points(prev_path_posts.poses.at(i).pose.position.x, prev_path_posts.poses.at(i).pose.position.y, prev_path_posts.poses.at(i).pose.position.z,
                                                                         now_path_posts.poses.at(j).pose.position.x, now_path_posts.poses.at(j).pose.position.y, now_path_posts.poses.at(j).pose.position.z);

                    vec_distance.push_back(calculateDistance2Points(prev_path_posts.poses.at(i).pose.position.x, prev_path_posts.poses.at(i).pose.position.y, prev_path_posts.poses.at(i).pose.position.z,
                                                                    now_path_posts.poses.at(j).pose.position.x, now_path_posts.poses.at(j).pose.position.y, now_path_posts.poses.at(j).pose.position.z));
                }

                // Sort all distances from lowest to highest
                std::vector<pair> vec;
                std::copy(vec_distance_and_label.begin(), vec_distance_and_label.end(), std::back_inserter<std::vector<pair>>(vec));
                std::sort(vec.begin(), vec.end(),
                          [](const pair &l, const pair &r) {
                              if (l.second != r.second)
                                  return l.second < r.second;
                              return l.first < r.first;
                          });

                vec_labels.push_back(vec[0].first);
                vec_check_dist.push_back(vec[0].second);
                vec_distance.clear();
            }
            // Clean the vector of waypoints from the previous state
            prev_path_posts.poses.clear();
            prev_path_posts.header.frame_id = "velodyne";
            // [LOOP .BAG]
            // for (int i = 0; i < vec_check_dist.size(); i++)
            // {
            //     if (vec_check_dist[i] > 2.0)
            //     {
            //         counter = 0;
            //         ROS_WARN("Reset Matching");
            //         break;
            //     }
            // }
            // // [LOOP.BAG]
            // if (counter == 0)
            // {
            //     break;
            // }
            // Storage as previous state the posts with the correct label placed
            for (int i = 0; i < vec_labels.size(); i++)
            {
                prev_path_posts.poses.push_back(now_path_posts.poses.at(vec_labels[i]));
            }
            // Storage identified posts positions
            for (int i = 0; i < prev_path_posts.poses.size(); i++)
            {
                savePose(i, prev_path_posts.poses.at(i));
            }
            // Use markers to visualize on Rviz the result
            drawPosts();
            // Clean vectors
            vec_labels.clear();
            vec_check_dist.clear();
            vec_distance_and_label.clear();
            now_path_posts.poses.clear();
            now_path_post_23.poses.clear();
        }
        break;
    }
}

void Matching::loop()
{
    while (ros::ok())
    {
        sleep(0.1);
        ros::spinOnce();
    }
}