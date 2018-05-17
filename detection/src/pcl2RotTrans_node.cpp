#include <detection/pcl2RotTrans.h>

int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "pcl2RotTrans_node");

    Pcl2RotTrans rotTrans;

    while (ros::ok())
    {
        sleep(1);
    }

    return 0;
}