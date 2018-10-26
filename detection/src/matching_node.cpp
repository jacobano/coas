#include <detection/matching.h>

int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "matching_node");

    Matching match;

    ros::spin();

    return 0;
}