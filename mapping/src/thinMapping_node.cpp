#include <mapping/thinMapping.h>

int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "thinMapping_node");

    ThinMapping mapper;

    while (ros::ok())
    {
        sleep(1);
    }

    return 0;
}