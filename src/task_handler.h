/**
 * Copyright 2017 by Institute for Infocomm Research, Singapore (I2R). All rights reserved.
 * @author Ng Kam Pheng (ngkp@i2r.a-star.edu.sg)
 */
#include "base.hpp"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <jsoncpp/json/json.h>

#include <ros/ros.h>
#include "charger_control_task/global_defs.h"
#include "charger_control_task/charger/charger.h"

namespace handler
{

/**
 * Sample template
 *
 */
class TaskHandler: public BaseHandler
{
public:
    TaskHandler(ros::NodeHandle *nh);
    ~TaskHandler();

private:
    void InitGeneric() override;
    void InitROSParams() override;
    void InitROSPublishers() override;
    void InitROSSubscribers() override;
    void PreInit() override;
    void runTask(const std::string &jsonstr_data) override;
    void stopTask() override;

    void TaskCallback(Charger::Status status);

    /// true if task is running, false otherwise
    bool m_is_running;
    
    ros::NodeHandle *nh_;
    Charger *charger_;

    // constants
    const uint8_t enable = K_ENABLE_CHARGER;
    const uint8_t disable = K_DISABLE_CHARGER;
    const std::string command_param = K_COMMAND_PARAM;
};

} // handler
