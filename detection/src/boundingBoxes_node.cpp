#include <detection/boundingBoxes.h>

int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "boundingBoxes_node");

    BoundingBoxes ellipser;

    while (ros::ok())
    {
        sleep(1);
    }

    return 0;
}