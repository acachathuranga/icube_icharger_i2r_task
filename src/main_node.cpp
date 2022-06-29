/**
 * Copyright 2017 by Institute for Infocomm Research, Singapore (I2R). All rights reserved.
 * @author Ng Kam Pheng (ngkp@i2r.a-star.edu.sg)
 */

#include <ros/ros.h>
#include "task_handler.h"

#include "charger_control_task/charger/charger.h"

/// name of this ROS node
const std::string kROSNodeName("task_handler_template");

/**
 * Main entry of application
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * @return always return 0
 */
int main(int argc, char *argv[])
{
    // initialise ROS and start it
    ros::init(argc, argv, kROSNodeName);
    ros::NodeHandle nh(ros::this_node::getName());

    // create the handler
    ROS_INFO_STREAM(ros::this_node::getName());
    handler::TaskHandler handler(&nh);
    handler.init();

    ros::spin();
    return 0;
}
