#include <detection/bounding_boxes.h>

int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "bounding_boxes_node");

    BoundingBoxes boundingBoxer;

    ros::spin();

    return 0;
}