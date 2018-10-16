#include <filtering/voxel_filter.h>

int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "voxel_filter_node");

    VoxelFilter filter;

    while (ros::ok())
    {
        sleep(1);
    }

    return 0;
}