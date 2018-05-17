#include <detection/volexFilter.h>

int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "volexFilter_node");

    VolexFilter filter;

    while (ros::ok())
    {
        sleep(1);
    }

    return 0;
}