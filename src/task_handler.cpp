/**
 * Copyright 2017 by Institute for Infocomm Research, Singapore (I2R). All rights reserved.
 * @author Ng Kam Pheng (ngkp@i2r.a-star.edu.sg)
 */
#include "task_handler.h"

namespace handler
{

/**
 * Class constructor
 *
 */
TaskHandler::TaskHandler(ros::NodeHandle *nh)
    : BaseHandler(),
      m_is_running(false),
      nh_(nh)
{
}

/**
 * Class destructor
 * 
 */
TaskHandler::~TaskHandler()
{
    if (charger_ != nullptr)
    {
        charger_->disable_charger();
        delete charger_;
        charger_ = nullptr;
    }
}

/**
 * Perform generic initialisation
 *
 */
void TaskHandler::InitGeneric()
{
    ROS_DEBUG_STREAM(ros::this_node::getName() << " - Executing generic initialization");

    // Create new Charger
    charger_ = new WirelessCharger(nh_, boost::bind(&TaskHandler::TaskCallback, this, _1));
}

/**
 * Initialise all global and private ROS parameters
 *
 */
void TaskHandler::InitROSParams()
{
    ROS_DEBUG_STREAM(ros::this_node::getName() << " - Executing ROS params initialization");

    /// @todo retrieve your ROS params here
}

/**
 * Initialise all the publishers
 *
 */
void TaskHandler::InitROSPublishers()
{
    ROS_DEBUG_STREAM(ros::this_node::getName() << " - Executing ROS publishers initialization");

    /// @todo add your publishers here
}

/**
 * Initialise all the subscribers
 *
 */
void TaskHandler::InitROSSubscribers()
{
    ROS_DEBUG_STREAM(ros::this_node::getName() << " - Executing ROS subscribers initialization");

    /// @todo add your subscribers here
}

/**
 * Perform initialisation before init() is called
 *
 */
void TaskHandler::PreInit()
{
    ROS_DEBUG_STREAM(ros::this_node::getName() << " - Executing pre-initialization initialization");

    /// @todo add your pre-initialization here
}

/**
 * Runs the idle task
 * @param jsonstr_data JSON data from CommandPub2
 */
void TaskHandler::runTask(const std::string &jsonstr_data)
{  
    if (m_is_running)
    {
        ROS_WARN_STREAM(ros::this_node::getName() << " - The task is already running");
    }
    else
    {
        Json::Reader reader;
        Json::Value params;
        if(!reader.parse(jsonstr_data, params))
        {
            ROS_ERROR_STREAM("Error Parsing Task Parameters. Aborting task");
        }
        else
        {
            int cmd = params[command_param].asInt();
            ROS_INFO_STREAM(ros::this_node::getName() << " - Running task : cmd[" << cmd << "]" );

            // publish running status
            publishTaskStatus(task_msgs::TaskStatus::K_TASK_STATUS_RUNNING);
            m_is_running = true;

            switch (cmd)
            {
                case K_ENABLE_CHARGER:      ROS_INFO_STREAM("Enabling Charger");
                                            charger_->enable_charger();
                                            break;

                case K_DISABLE_CHARGER:     ROS_INFO_STREAM("Disabling Charger");
                                            charger_->disable_charger();
                                            break;

                default:                    ROS_INFO_STREAM("Unknown Command");
            }

            // Note that when mission data contains ROS parameters for your task, and it is
            // passed in via the commandpub2 message jsonstr_data
        }
    }
}

/**
 * Stops the idle task
 *
 */
void TaskHandler::stopTask()
{
    if (m_is_running)
    {
        ROS_INFO_STREAM(ros::this_node::getName() << " - Stopping task");
        
        //charger_->cancel();
        publishTaskStatus(task_msgs::TaskStatus::K_TASK_STATUS_IDLE);
        m_is_running = false;
    }
    else
    {
        if (getCurrentTaskStatus() != task_msgs::TaskStatus::K_TASK_STATUS_IDLE)
        {
            publishTaskStatus(task_msgs::TaskStatus::K_TASK_STATUS_IDLE);
        }
    }
}

void TaskHandler::TaskCallback(Charger::Status status)
{
    switch (status)
    {
        case Charger::Status::CHARGING: ROS_INFO_STREAM("Started Charging");
                                        publishTaskStatus(task_msgs::TaskStatus::K_TASK_STATUS_DONE);   // State = 2
                                        break;
        case Charger::Status::NOT_CHARGING: ROS_INFO_STREAM("Stopped Charging");
                                            publishTaskStatus(task_msgs::TaskStatus::K_TASK_STATUS_DONE);   // State = 2
                                            break;
        case Charger::Status::UNKNOWN: 
        case Charger::Status::ERROR:
        default:    ROS_INFO_STREAM("Task Failed");
                    publishTaskStatus(task_msgs::TaskStatus::K_TASK_STATUS_EXCEPTION);  // State = -1
    }
}

} // handler
