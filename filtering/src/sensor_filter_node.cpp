#include <filtering/sensor_filter.h>

int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "sensor_filter_node");

    SensorFilter filter;
    ros::spin();
    return 0;
}