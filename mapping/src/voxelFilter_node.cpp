#include <mapping/voxelFilter.h>

int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "voxelFilter_node");

    VoxelFilter filter;

    while (ros::ok())
    {
        sleep(1);
    }

    //ros::spin(); //Quizás esto sea mejor

    return 0;
}