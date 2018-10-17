#include <mapping/mapFilter.h>

int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "mapFilter_node");

    MapFilter mapper;
/**
    while (ros::ok())
    {
        sleep(1);
    }
**/
    ros::spin();
    return 0;
}