#include <detection/matching.h>
#include "rootfinding/multi_dimensional_root_finding.hpp"

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
    pub_marker1 = n.advertise<visualization_msgs::Marker>("/marker_post1", 1);
    pub_marker2 = n.advertise<visualization_msgs::Marker>("/marker_post2", 1);
    pub_marker3 = n.advertise<visualization_msgs::Marker>("/marker_post3", 1);
    pub_marker4 = n.advertise<visualization_msgs::Marker>("/marker_post4", 1);
    pub_marker5 = n.advertise<visualization_msgs::Marker>("/marker_post5", 1);
    pub_marker6 = n.advertise<visualization_msgs::Marker>("/marker_post6", 1);

    filePost1.open("/home/hector/matlab_ws/COAS/matchPost1");
    filePost2.open("/home/hector/matlab_ws/COAS/matchPost2");
    filePost3.open("/home/hector/matlab_ws/COAS/matchPost3");
    filePost1Time.open("/home/hector/matlab_ws/COAS/matchPost1time");
    filePost2Time.open("/home/hector/matlab_ws/COAS/matchPost2time");
    filePost3Time.open("/home/hector/matlab_ws/COAS/matchPost3time");

    flagInit12 = true;
    flagInit13 = true;
    flagInit23 = true;

    checkDiff12 = false;
    checkDiff13 = false;
    checkDiff23 = false;

    cont = 0;

    toDo();

    loop();
}

Matching::~Matching()
{
    filePost1.close();
    filePost2.close();
    filePost3.close();
    filePost1Time.close();
    filePost2Time.close();
    filePost3Time.close();
}

void Matching::cb_pathPoste1(const nav_msgs::Path path)
{
    nowPathPost1 = path;
    toDo();
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
    if (flagInit12 == true)
    {
        nowPathPost12 = path;
        flagInit12 = false;
    }
    else
    {
        checkDiff12 = checkDiff(1, path);
        nowPathPost12 = path;
    }
}

void Matching::cb_pathPoste13(const nav_msgs::Path path)
{
    if (flagInit13 == true)
    {
        nowPathPost13 = path;
        flagInit13 = false;
    }
    else
    {
        checkDiff13 = checkDiff(2, path);
        nowPathPost13 = path;
    }
}

void Matching::cb_pathPoste23(const nav_msgs::Path path)
{
    if (flagInit23 == true)
    {
        nowPathPost23 = path;
        flagInit23 = false;
    }
    else
    {
        checkDiff23 = checkDiff(3, path);
        nowPathPost23 = path;
    }
}

bool Matching::checkDiff(int p, const nav_msgs::Path checkPath)
{
    if (!checkPath.poses.empty())
    {

        nav_msgs::Path comparePath;
        bool result;
        float xPre, xNow;
        switch (p)
        {
        case 1:
            comparePath = nowPathPost12;
            break;
        case 2:
            comparePath = nowPathPost13;
            break;
        case 3:
            comparePath = nowPathPost23;
            break;
        }
        if (!comparePath.poses.empty())
        {
            xNow = checkPath.poses.at(0).pose.position.x;
            xPre = comparePath.poses.at(0).pose.position.x;
            if (xNow != xPre)
            {
                ROS_WARN("true");
                result = true;
            }
            else
            {
                ROS_WARN("false");
                result = false;
            }
            return result;
        }
    }
}

float Matching::dist2Points(float x1, float y1, float z1, float x2, float y2, float z2)
{
    float distance;
    distance = sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)) + ((z2 - z1) * (z2 - z1)));
    return distance;
}

void Matching::drawPosts()
{
    if (prevPathPosts.poses.size() > 0)
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
        marker_post1.scale.x = 0.6;
        marker_post1.scale.y = 0.6;
        marker_post1.scale.z = 0.6;
        marker_post1.color.a = 0.8;
        marker_post1.color.r = 1.0;
        marker_post1.color.g = 0.0;
        marker_post1.color.b = 0.0;
    }
    if (prevPathPosts.poses.size() > 1)
    {
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
        marker_post2.scale.x = 0.6;
        marker_post2.scale.y = 0.6;
        marker_post2.scale.z = 0.6;
        marker_post2.color.a = 0.8;
        marker_post2.color.r = 0.0;
        marker_post2.color.g = 1.0;
        marker_post2.color.b = 0.0;
    }
    if (prevPathPosts.poses.size() > 2)
    {
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
        marker_post3.scale.x = 0.6;
        marker_post3.scale.y = 0.6;
        marker_post3.scale.z = 0.6;
        marker_post3.color.a = 0.8;
        marker_post3.color.r = 0.0;
        marker_post3.color.g = 0.0;
        marker_post3.color.b = 1.0;
    }
    if (nowPathPosts.poses.size() > 0)
    {
        marker_post4.header.frame_id = "velodyne";
        marker_post4.header.stamp = ros::Time();
        marker_post4.ns = "my_namespace";
        marker_post4.id = 0;
        marker_post4.type = visualization_msgs::Marker::CYLINDER;
        marker_post4.action = visualization_msgs::Marker::ADD;
        marker_post4.pose.position.x = nowPathPosts.poses.at(0).pose.position.x;
        marker_post4.pose.position.y = nowPathPosts.poses.at(0).pose.position.y;
        marker_post4.pose.position.z = nowPathPosts.poses.at(0).pose.position.z;
        marker_post4.pose.orientation.x = 0.0;
        marker_post4.pose.orientation.y = 0.0;
        marker_post4.pose.orientation.z = 0.0;
        marker_post4.pose.orientation.w = 1.0;
        marker_post4.scale.x = 0.3;
        marker_post4.scale.y = 0.3;
        marker_post4.scale.z = 1.0;
        marker_post4.color.a = 1.0;
        marker_post4.color.r = 0.5;
        marker_post4.color.g = 0.5;
        marker_post4.color.b = 0.0;
    }
    if (nowPathPosts.poses.size() > 1)
    {
        marker_post5.header.frame_id = "velodyne";
        marker_post5.header.stamp = ros::Time();
        marker_post5.ns = "my_namespace";
        marker_post5.id = 0;
        marker_post5.type = visualization_msgs::Marker::CYLINDER;
        marker_post5.action = visualization_msgs::Marker::ADD;
        marker_post5.pose.position.x = nowPathPosts.poses.at(1).pose.position.x;
        marker_post5.pose.position.y = nowPathPosts.poses.at(1).pose.position.y;
        marker_post5.pose.position.z = nowPathPosts.poses.at(1).pose.position.z;
        marker_post5.pose.orientation.x = 0.0;
        marker_post5.pose.orientation.y = 0.0;
        marker_post5.pose.orientation.z = 0.0;
        marker_post5.pose.orientation.w = 1.0;
        marker_post5.scale.x = 0.3;
        marker_post5.scale.y = 0.3;
        marker_post5.scale.z = 1.0;
        marker_post5.color.a = 1.0;
        marker_post5.color.r = 0.5;
        marker_post5.color.g = 0.0;
        marker_post5.color.b = 0.5;
    }
    if (nowPathPosts.poses.size() > 2)
    {
        marker_post6.header.frame_id = "velodyne";
        marker_post6.header.stamp = ros::Time();
        marker_post6.ns = "my_namespace";
        marker_post6.id = 0;
        marker_post6.type = visualization_msgs::Marker::CYLINDER;
        marker_post6.action = visualization_msgs::Marker::ADD;
        marker_post6.pose.position.x = nowPathPosts.poses.at(2).pose.position.x;
        marker_post6.pose.position.y = nowPathPosts.poses.at(2).pose.position.y;
        marker_post6.pose.position.z = nowPathPosts.poses.at(2).pose.position.z;
        marker_post6.pose.orientation.x = 0.0;
        marker_post6.pose.orientation.y = 0.0;
        marker_post6.pose.orientation.z = 0.0;
        marker_post6.pose.orientation.w = 1.0;
        marker_post6.scale.x = 0.3;
        marker_post6.scale.y = 0.3;
        marker_post6.scale.z = 1.0;
        marker_post6.color.a = 1.0;
        marker_post6.color.r = 0.0;
        marker_post6.color.g = 0.5;
        marker_post6.color.b = 0.5;
    }

    pub_marker1.publish(marker_post1);
    pub_marker2.publish(marker_post2);
    pub_marker3.publish(marker_post3);
    pub_marker4.publish(marker_post4);
    pub_marker5.publish(marker_post5);
    pub_marker6.publish(marker_post6);
}

void Matching::save_pose(int nPost, geometry_msgs::PoseStamped waypoint)
{
    // ROS_WARN("1");
    switch (nPost)
    {
    case 0:
        // ROS_WARN("2 timepose: %d | timedistance: %d", startTimePose, startTime);
        filePost1 << waypoint.pose.position.x << " " << waypoint.pose.position.y << std::endl;
        filePost1Time << ros::Time::now().toSec() - startTimePose << std::endl;
        break;
    case 1:
        filePost2 << waypoint.pose.position.x << " " << waypoint.pose.position.y << std::endl;
        filePost2Time << ros::Time::now().toSec() - startTimePose << std::endl;
        break;
    case 2:
        filePost3 << waypoint.pose.position.x << " " << waypoint.pose.position.y << std::endl;
        filePost3Time << ros::Time::now().toSec() - startTimePose << std::endl;
        break;
    }
    // ROS_WARN("3");
}

geometry_msgs::PoseStamped Matching::findMissingPost(float x2, float y2, float x3, float y3)
{
    NonlinearSystem<2> sys;
    float l, s;
    l = 8.60;
    s = 3.70;
    // Equations
    // cerr << "===================================================" << endl;
    // cout << sqrt(pow((x2 - x3), 2) + pow((y2 - y3), 2)) << endl;
    if (5 < sqrt(pow((x2 - x3), 2) + pow((y2 - y3), 2)))
    {
        s = 3.7;
    }
    else
    {
        s = 8.6;
    }
    // cout << s << endl;
    // ROS_WARN("x2 = %f; y2 = %f; x3 = %f; y3 = %f;", x2, y2, x3, y3);
    equation_type eq1 = [x2, y2, x3, y3, s](const vector_type &x) {
        return sqrt((x2 - x[0]) * (x2 - x[0]) + (y2 - x[1]) * (y2 - x[1])) - s;
    };
    sys.assign_equation(eq1, 0);
    equation_type eq2 = [x2, y2, x3, y3, l](const vector_type &x) {
        return sqrt((x3 - x[0]) * (x3 - x[0]) + (y3 - x[1]) * (y3 - x[1])) - l;
    };
    sys.assign_equation(eq2, 1);
    // IMPORTANTE COLOCAR BIEN init EN FUNCIÓN DE LA POSICIÓN DE LOS POSTES RESPECTO DEL VELODYNE
    array<double, 2> init = {15, 15};
    sys.initialize(init);
    sys.find_roots_gnewton();
    boost::numeric::ublas::vector<double> r;
    r = sys.get_values();
    // cout << "Point: [" << r[0] << ", " << r[1] << "]" << endl;
    // cerr << "===================================================" << endl;
    geometry_msgs::PoseStamped result;
    result.pose.position.x = r[0];
    result.pose.position.y = r[1];
    return result;
}

void Matching::toDo()
{
    // Borra las marcas generadas con anterioridad.
    marker_post1.action = visualization_msgs::Marker::DELETE;
    pub_marker1.publish(marker_post1);
    marker_post2.action = visualization_msgs::Marker::DELETE;
    pub_marker2.publish(marker_post2);
    marker_post3.action = visualization_msgs::Marker::DELETE;
    pub_marker3.publish(marker_post3);
    marker_post4.action = visualization_msgs::Marker::DELETE;
    pub_marker4.publish(marker_post4);
    marker_post5.action = visualization_msgs::Marker::DELETE;
    pub_marker5.publish(marker_post5);
    marker_post6.action = visualization_msgs::Marker::DELETE;
    pub_marker6.publish(marker_post6);
    // En el instante inicial se guarda la primera entrada como etiquetas válidas.
    switch (cont)
    {
    // Init
    case 0:
        // Se inicializa si están los tres postes validados.
        if (!nowPathPost12.poses.empty() && !nowPathPost13.poses.empty() && !nowPathPost23.poses.empty())
        {
            ROS_WARN("Init Matching");
            // Se guardan los tres waypoints relevantes del estado actual en el vector del instante anterior.
            prevPathPosts.poses.push_back(nowPathPost1.poses.at(1));
            prevPathPosts.poses.push_back(nowPathPost2.poses.at(1));
            prevPathPosts.poses.push_back(nowPathPost3.poses.at(1));
            cont++;
            // sleep(1.0);
            startTimePose = ros::Time::now().toSec();
        }
        break;
    case 1:
        // Se ejecuta si están los tres postes validados y si los paths han cambiado.
        if ((!nowPathPost12.poses.empty() && checkDiff12 == true) ||
            (!nowPathPost13.poses.empty() && checkDiff13 == true) ||
            (!nowPathPost23.poses.empty() && checkDiff23 == true))

        // if (!nowPathPost12.poses.empty() || !nowPathPost13.poses.empty() || !nowPathPost23.poses.empty())

        // if ((!nowPathPost12.poses.empty() && (!nowPathPost13.poses.empty() || !nowPathPost23.poses.empty())) ||
        //     (!nowPathPost13.poses.empty() && (!nowPathPost12.poses.empty() || !nowPathPost23.poses.empty())) ||
        //     (!nowPathPost23.poses.empty() && (!nowPathPost12.poses.empty() || !nowPathPost13.poses.empty())))
        {

            int contposts = 0;
            // Se guardan los 6 waypoints relevantes del estado actual en un vector.
            if (!nowPathPost12.poses.empty())
            {
                nowPathPosts.poses.push_back(nowPathPost12.poses.at(0));
                nowPathPosts.poses.push_back(nowPathPost12.poses.at(1));
                contposts++;
                // ROS_WARN("contposts: %i, now path size: %i", contposts, nowPathPosts.poses.size());
            }
            if (!nowPathPost13.poses.empty())
            {
                nowPathPosts.poses.push_back(nowPathPost13.poses.at(0));
                nowPathPosts.poses.push_back(nowPathPost13.poses.at(1));
                contposts++;
                // ROS_WARN("contposts: %i, now path size: %i", contposts, nowPathPosts.poses.size());
            }
            if (!nowPathPost23.poses.empty())
            {
                nowPathPosts.poses.push_back(nowPathPost23.poses.at(0));
                nowPathPosts.poses.push_back(nowPathPost23.poses.at(1));
                contposts++;
                // ROS_WARN("contposts: %i, now path size: %i", contposts, nowPathPosts.poses.size());
            }

            if (contposts == 1)
            {
                if (!nowPathPost12.poses.empty())
                {
                    nowPathPosts.poses.push_back(findMissingPost(nowPathPost12.poses.at(0).pose.position.x, nowPathPost12.poses.at(0).pose.position.y,
                                                                 nowPathPost12.poses.at(1).pose.position.x, nowPathPost12.poses.at(1).pose.position.y));
                }
                if (!nowPathPost13.poses.empty())
                {
                    nowPathPosts.poses.push_back(findMissingPost(nowPathPost13.poses.at(0).pose.position.x, nowPathPost13.poses.at(0).pose.position.y,
                                                                 nowPathPost13.poses.at(1).pose.position.x, nowPathPost13.poses.at(1).pose.position.y));
                }
                if (!nowPathPost23.poses.empty())
                {
                    nowPathPosts.poses.push_back(findMissingPost(nowPathPost23.poses.at(0).pose.position.x, nowPathPost23.poses.at(0).pose.position.y,
                                                                 nowPathPost23.poses.at(1).pose.position.x, nowPathPost23.poses.at(1).pose.position.y));
                }
            }
            ROS_WARN("* now size: %i", nowPathPosts.poses.size());
            std::vector<int> vec_labels;
            std::vector<float> vec_check_dist;
            // Se compara cada waypoint relevante del estado anterior con los waypoints del estado actual.
            ROS_WARN("prev size: %i, now size: %i", prevPathPosts.poses.size(), nowPathPosts.poses.size());
            for (int i = 0; i < prevPathPosts.poses.size(); i++)
            {
                std::vector<float> vtest;
                // cout << "vec with zeros : [ ";
                cout << endl;
                for (int j = 0; j < nowPathPosts.poses.size(); j++)
                {
                    // Se calcula la distancia que hay entre los waypoints del instante actual y el instante anterior.

                    // if (0 < dist2Points(prevPathPosts.poses.at(i).pose.position.x, prevPathPosts.poses.at(i).pose.position.y, prevPathPosts.poses.at(i).pose.position.z,
                    //                     nowPathPosts.poses.at(j).pose.position.x, nowPathPosts.poses.at(j).pose.position.y, nowPathPosts.poses.at(j).pose.position.z))
                    // {
                    vec_distAndLabel[j] = dist2Points(prevPathPosts.poses.at(i).pose.position.x, prevPathPosts.poses.at(i).pose.position.y, prevPathPosts.poses.at(i).pose.position.z,
                                                      nowPathPosts.poses.at(j).pose.position.x, nowPathPosts.poses.at(j).pose.position.y, nowPathPosts.poses.at(j).pose.position.z);
                    // }

                    // vtest.push_back(dist2Points(prevPathPosts.poses.at(i).pose.position.x, prevPathPosts.poses.at(i).pose.position.y, prevPathPosts.poses.at(i).pose.position.z,
                    // nowPathPosts.poses.at(j).pose.position.x, nowPathPosts.poses.at(j).pose.position.y, nowPathPosts.poses.at(j).pose.position.z));
                    // cout << vtest[j] << " ";
                }
                // cout << "]" << endl;
                // cout << "vec without zeros : [ ";
                // vtest.erase(
                //     std::remove(vtest.begin(), vtest.end(), 0),
                //     vtest.end());
                for (int k = 0; k < vtest.size(); k++)
                {
                    // cout << vtest[k] << " ";
                    // vec_distAndLabel[k] = vtest[k];
                }
                // cout << "]" << endl;
                // Se ordenan todas las distancias resultantes de menor a mayor.
                std::vector<pair> vec;
                std::copy(vec_distAndLabel.begin(), vec_distAndLabel.end(), std::back_inserter<std::vector<pair>>(vec));
                std::sort(vec.begin(), vec.end(),
                          [](const pair &l, const pair &r) {
                              if (l.second != r.second)
                                  return l.second < r.second;
                              return l.first < r.first;
                          });
                vec.shrink_to_fit();
                for (auto const &pair : vec)
                {
                    cout << "{" << pair.first << ", " << pair.second << "}" << endl;
                }

                // if (vec_labels.size() > 0)
                // {
                //     for (int j = 0; j < vec.size(); j++)
                //     {
                //         for (int k = 0; k < vec_labels.size(); k++)
                //         {
                //             if (vec[j].first != vec_labels[k])
                //             {
                //                 vec_labels.push_back(vec[j].first);
                //                 ROS_WARN("!=");
                //                 k = vec_labels.size();
                //                 j = vec.size();
                //             }
                //             else
                //             {
                //                 ROS_WARN("%f = %f", vec[j].first, vec_labels[k]);
                //             }
                //         }
                //     }
                // }

                vec_labels.push_back(vec[0].first);
                vec_check_dist.push_back(vec[0].second);
                for (int k = 0; k < vec_labels.size(); k++)
                {
                    cout << vec_labels[k] << " ";
                }
                cout << endl;
            }
            // Se limpia el vector de waypoints del instante anterios y se le coloca el frame_id correspondiente.
            prevPathPosts.poses.clear();
            prevPathPosts.header.frame_id = "velodyne";
            // [PARA .BAG CON LOOP] Se revisan todas las distancias y si hay mucha diferencia, se asume que el estado actual es el bueno y, por tanto, se tiene que volver a inicializar.
            // for (int i = 0; i < vec_check_dist.size(); i++)
            // {
            //     if (vec_check_dist[i] > 2.0)
            //     {
            //         cont = 0;
            //         ROS_WARN("Reset Matching");
            //         break;
            //     }
            // }
            // // [PARA .BAG CON LOOP] Se sale de el estado actual y reinicia el algoritmo.
            // if (cont == 0)
            // {
            //     break;
            // }
            // Se guarda como instante anterior, los postes con el identificador correctamente colocado.
            for (int i = 0; i < vec_labels.size(); i++)
            {
                cout << "[" << vec_labels[i] << ", " << vec_check_dist[vec_labels[i]] << "]";
                prevPathPosts.poses.push_back(nowPathPosts.poses.at(vec_labels[i]));
            }
            cout << endl;
            // Se guardan las posiciones de los postes identificados para dijubarlos en Matlab.
            for (int i = 0; i < prevPathPosts.poses.size(); i++)
            {
                save_pose(i, prevPathPosts.poses.at(i));
            }
            // Se utilizan Markers para visualizar en Rviz el resultado.
            drawPosts();
            // Limpia vectores.
            vec_labels.clear();
            vec_check_dist.clear();
            vec_distAndLabel.clear();
            nowPathPosts.poses.clear();
            checkDiff12 = checkDiff13 = checkDiff23 = false;
        }
        break;
    }
}

void Matching::loop()
{
    while (ros::ok())
    {
        sleep(0.9);
        ros::spinOnce();
    }
}