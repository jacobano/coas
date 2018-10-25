#include <detection/euclidean_clusterer.h>

int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "euclideanClusterer_node");

    EuclideanClusterer clusterer;

    ros::spin();
    return 0;
}