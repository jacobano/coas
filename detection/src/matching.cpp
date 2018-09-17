#include <detection/matching.h>

Matching::Matching()
{
    n = ros::NodeHandle();

    // Subscriptions
    ros::Subscriber sub_pathPoste1 = n.subscribe("/pathPoste1", 1, &Matching::cb_pathPoste1, this);
    ros::Subscriber sub_pathPoste2 = n.subscribe("/pathPoste2", 1, &Matching::cb_pathPoste2, this);
    ros::Subscriber sub_pathPoste3 = n.subscribe("/pathPoste3", 1, &Matching::cb_pathPoste3, this);
    ros::Subscriber sub_pathPoste12 = n.subscribe("/pathPoste12", 1, &Matching::cb_pathPoste12, this);
    ros::Subscriber sub_pathPoste13 = n.subscribe("/pathPoste13", 1, &Matching::cb_pathPoste13, this);
    ros::Subscriber sub_pathPoste23 = n.subscribe("/pathPoste23", 1, &Matching::cb_pathPoste23, this);

    // Publishers
    pub = n.advertise<nav_msgs::Path>("/TestPath", 1);
    pub_marker1 = n.advertise<visualization_msgs::Marker>("/marker_post1", 1);
    pub_marker2 = n.advertise<visualization_msgs::Marker>("/marker_post2", 1);
    pub_marker3 = n.advertise<visualization_msgs::Marker>("/marker_post3", 1);
    cont = 0;
    toDo();

    loop();
}

Matching::~Matching()
{
}

void Matching::cb_pathPoste1(const nav_msgs::Path path)
{
    nowPathPost1 = path;
}

void Matching::cb_pathPoste2(const nav_msgs::Path path)
{
    nowPathPost2 = path;
}

void Matching::cb_pathPoste3(const nav_msgs::Path path)
{
    nowPathPost3 = path;
}

void Matching::cb_pathPoste12(const nav_msgs::Path path)
{
    nowPathPost12 = path;
}

void Matching::cb_pathPoste13(const nav_msgs::Path path)
{
    nowPathPost13 = path;
}

void Matching::cb_pathPoste23(const nav_msgs::Path path)
{
    nowPathPost23 = path;
    toDo();
}

float Matching::dist2Points(float x1, float y1, float z1, float x2, float y2, float z2)
{
    float distance;
    distance = sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)) + ((z2 - z1) * (z2 - z1)));
    return distance;
}

void Matching::drawPosts()
{
    marker_post1.header.frame_id = "velodyne";
    marker_post1.header.stamp = ros::Time();
    marker_post1.ns = "my_namespace";
    marker_post1.id = 0;
    marker_post1.type = visualization_msgs::Marker::CYLINDER;
    marker_post1.action = visualization_msgs::Marker::ADD;
    marker_post1.pose.position.x = prevPathPosts.poses.at(0).pose.position.x;
    marker_post1.pose.position.y = prevPathPosts.poses.at(0).pose.position.y;
    marker_post1.pose.position.z = prevPathPosts.poses.at(0).pose.position.z;
    marker_post1.pose.orientation.x = 0.0;
    marker_post1.pose.orientation.y = 0.0;
    marker_post1.pose.orientation.z = 0.0;
    marker_post1.pose.orientation.w = 1.0;
    marker_post1.scale.x = 0.8;
    marker_post1.scale.y = 0.8;
    marker_post1.scale.z = 0.6;
    marker_post1.color.a = 0.8;
    marker_post1.color.r = 1.0;
    marker_post1.color.g = 0.0;
    marker_post1.color.b = 0.0;

    marker_post2.header.frame_id = "velodyne";
    marker_post2.header.stamp = ros::Time();
    marker_post2.ns = "my_namespace";
    marker_post2.id = 0;
    marker_post2.type = visualization_msgs::Marker::CYLINDER;
    marker_post2.action = visualization_msgs::Marker::ADD;
    marker_post2.pose.position.x = prevPathPosts.poses.at(1).pose.position.x;
    marker_post2.pose.position.y = prevPathPosts.poses.at(1).pose.position.y;
    marker_post2.pose.position.z = prevPathPosts.poses.at(1).pose.position.z;
    marker_post2.pose.orientation.x = 0.0;
    marker_post2.pose.orientation.y = 0.0;
    marker_post2.pose.orientation.z = 0.0;
    marker_post2.pose.orientation.w = 1.0;
    marker_post2.scale.x = 0.8;
    marker_post2.scale.y = 0.8;
    marker_post2.scale.z = 0.6;
    marker_post2.color.a = 0.8;
    marker_post2.color.r = 0.0;
    marker_post2.color.g = 1.0;
    marker_post2.color.b = 0.0;

    marker_post3.header.frame_id = "velodyne";
    marker_post3.header.stamp = ros::Time();
    marker_post3.ns = "my_namespace";
    marker_post3.id = 0;
    marker_post3.type = visualization_msgs::Marker::CYLINDER;
    marker_post3.action = visualization_msgs::Marker::ADD;
    marker_post3.pose.position.x = prevPathPosts.poses.at(2).pose.position.x;
    marker_post3.pose.position.y = prevPathPosts.poses.at(2).pose.position.y;
    marker_post3.pose.position.z = prevPathPosts.poses.at(2).pose.position.z;
    marker_post3.pose.orientation.x = 0.0;
    marker_post3.pose.orientation.y = 0.0;
    marker_post3.pose.orientation.z = 0.0;
    marker_post3.pose.orientation.w = 1.0;
    marker_post3.scale.x = 0.8;
    marker_post3.scale.y = 0.8;
    marker_post3.scale.z = 0.6;
    marker_post3.color.a = 0.8;
    marker_post3.color.r = 0.0;
    marker_post3.color.g = 0.0;
    marker_post3.color.b = 1.0;

    pub_marker1.publish(marker_post1);
    pub_marker2.publish(marker_post2);
    pub_marker3.publish(marker_post3);
}

void Matching::toDo()
{
    ROS_WARN("Matching | cont: %i", cont);
    switch (cont)
    {
    case 0:
        if (!nowPathPost12.poses.empty() && !nowPathPost13.poses.empty() && !nowPathPost23.poses.empty())
        {
            prevPathPosts.poses.push_back(nowPathPost1.poses.at(1));
            prevPathPosts.poses.push_back(nowPathPost2.poses.at(1));
            prevPathPosts.poses.push_back(nowPathPost3.poses.at(1));

            cont++;
        }
        break;
    case 1:
        // Me aseguro de hacer este paso una vez por callback.
        if (prevPathPosts.poses.at(1).pose.position.x != nowPathPost1.poses.at(1).pose.position.x && !nowPathPost12.poses.empty() && !nowPathPost13.poses.empty() && !nowPathPost23.poses.empty())
        {
            nowPathPosts.poses.push_back(nowPathPost1.poses.at(1));
            nowPathPosts.poses.push_back(nowPathPost2.poses.at(1));
            nowPathPosts.poses.push_back(nowPathPost3.poses.at(1));
            std::vector<int> vec_labels;
            for (int i = 0; i < prevPathPosts.poses.size(); i++)
            {
                for (int j = 0; j < nowPathPosts.poses.size(); j++)
                {
                    vec_distAndLabel[j] = dist2Points(prevPathPosts.poses.at(i).pose.position.x, prevPathPosts.poses.at(i).pose.position.y, prevPathPosts.poses.at(i).pose.position.z,
                                                      nowPathPosts.poses.at(j).pose.position.x, nowPathPosts.poses.at(j).pose.position.y, nowPathPosts.poses.at(j).pose.position.z);

                    vec_dist.push_back(dist2Points(prevPathPosts.poses.at(i).pose.position.x, prevPathPosts.poses.at(i).pose.position.y, prevPathPosts.poses.at(i).pose.position.z,
                                                   nowPathPosts.poses.at(j).pose.position.x, nowPathPosts.poses.at(j).pose.position.y, nowPathPosts.poses.at(j).pose.position.z));
                }

                // for (std::map<int, float>::iterator ii = vec_distAndLabel.begin(); ii != vec_distAndLabel.end(); ++ii)
                // {
                //     std::cout << (*ii).first << ": " << (*ii).second << std::endl;
                // }

                std::vector<pair> vec;
                std::copy(vec_distAndLabel.begin(), vec_distAndLabel.end(), std::back_inserter<std::vector<pair>>(vec));
                std::sort(vec.begin(), vec.end(),
                          [](const pair &l, const pair &r) {
                              if (l.second != r.second)
                                  return l.second < r.second;

                              return l.first < r.first;
                          });

                // for (auto const &pair : vec)
                // {
                //     std::cout << '{' << pair.first << "," << pair.second << "}" << std::endl;
                // }

                vec_labels.push_back(vec[0].first);
                // for (int i = 0; i < vec.size(); i++){
                //     std::cout << vec[i].first << std::endl;
                // }

                // std::cout << "sort: ";
                // std::sort(vec_dist.begin(), vec_dist.end());
                // for (int i = 0; i < vec_dist.size(); i++)
                // {
                //     std::cout << vec_dist[i] << " ";
                // }
                // std::cout << "\n";
                vec_dist.clear();
            }

            prevPathPosts.poses.clear();
            prevPathPosts.header.frame_id = "velodyne";
            std::cout << "[     ] Vec Labels: ";
            for (int i = 0; i < vec_labels.size(); i++)
            {
                std::cout << vec_labels[i] << " ";
                prevPathPosts.poses.push_back(nowPathPosts.poses.at(vec_labels[i]));
            }
            std::cout << std::endl;
            pub.publish(prevPathPosts);
            nowPathPost23.poses.clear();
            nowPathPosts.poses.clear();
            vec_labels.clear();

            drawPosts();
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