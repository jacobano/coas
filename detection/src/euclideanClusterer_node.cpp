#include <detection/euclideanClusterer.h>

int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "euclideanClusterer_node");

    EuclideanClusterer clusterer;

    while (ros::ok())
    {   
        sleep(1);
    }

    return 0;
}