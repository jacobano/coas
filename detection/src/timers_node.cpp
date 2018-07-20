#include <detection/timers.h>

int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "timers_node");

    Timers timers;

    while (ros::ok())
    {
        sleep(1);
    }

    return 0;
}