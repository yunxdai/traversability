#include <ros/ros.h>
#include <string>
#include <nav_msgs/OccupancyGrid.h>
#include "elevation_msgs/occupancyLocal.h"

void responseProcess(elevation_msgs::occupancyLocal &srv, ros::Publisher pub_local);

int main(int argc,  char** argv)
{
    // client
    ros::init(argc, argv, "service_test");
    ros::NodeHandle nh;
    ros::Publisher pub_local = nh.advertise<nav_msgs::OccupancyGrid> ("/occupancy_map_local", 5);
    std::string servicesName = "/test";
    ros::ServiceClient client = nh.serviceClient<elevation_msgs::occupancyLocal>(servicesName);
    elevation_msgs::occupancyLocal srv;
    srv.request.flag = 1;

    ros::Rate rate(1);
    while (ros::ok()) {
        if (client.call(srv)) {
            // process response
            responseProcess(srv,pub_local);
        }
        else {
            // ROS_ERROR_STREAM("Failed to call test service!");
        }
        rate.sleep();
    }
    

}
void responseProcess(elevation_msgs::occupancyLocal &srv, ros::Publisher pub_local) {
    // srv.response.occupancy.
    // ROS_INFO_STREAM("Response Data Size = " << srv.response.occupancy.data.size());
    pub_local.publish(srv.response.occupancy);
    return;
}