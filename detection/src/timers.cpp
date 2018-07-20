#include <detection/timers.h>
#include "detection/timeNode.h"

Timers::Timers()
{
    n = ros::NodeHandle();

    // Subscriptions
    sub_mapp = n.subscribe("/time_mapp", 1, &Timers::mapp_cb, this);
    sub_eucl = n.subscribe("/time_eucl", 1, &Timers::eucl_cb, this);
    sub_bbxs = n.subscribe("/time_bbxs", 1, &Timers::bbxs_cb, this);

    // Publishers

    flagMapp = flagEucl = flagBbxs = false;
    loop();
}

Timers::~Timers()
{
}

void Timers::mapp_cb(const detection::timeNode _time)
{
    timeMapp = _time;
}

void Timers::eucl_cb(const detection::timeNode _time)
{
    timeEucl = _time;
}

void Timers::bbxs_cb(const detection::timeNode _time)
{
    timeBbxs = _time;
}

void Timers::loop()
{
    while (ros::ok())
    {
        timeTotal.time_node = timeMapp.time_node + timeEucl.time_node + timeBbxs.time_node;
        std::cout << "[COAS] Total time: " << timeTotal.time_node << std::endl;
        std::cout << "[COAS] Times mapp: " << timeMapp.time_node << " eucl: " << timeEucl.time_node << " bbxs: " << timeBbxs.time_node << std::endl;
        std::cout << "[COAS] -------------------------------------------------------------" << std::endl;
        sleep(0.1);
        ros::spinOnce();
    }
}