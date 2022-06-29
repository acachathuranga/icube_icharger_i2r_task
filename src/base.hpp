/**
 * Copyright 2017 by Institute for Infocomm Research, Singapore (I2R). All rights reserved.
 * @author Ng Kam Pheng (ngkp@i2r.a-star.edu.sg)
 */
#ifndef BASE_HANDLER_HPP
#define BASE_HANDLER_HPP

#include <ros/ros.h>
#include "mrccc_ros_messages/CommandPub2.h"
#include "task_msgs/TaskStatus.h"
#include "../../../mrccc/common/rosmsg_defs.hpp"
#include "../../../mrccc/common/utils/utils_tasks_definition.hpp"

namespace handler
{

/// our task ID is obtained from this ROS param
#define K_STR_ROS_PARAM_TASK_ID  "task_id"

/**
 * This is the abstract virtual base class. To use this class,
 * 1. sub-class this class e.g. MyHandler() : BaseHandler() {}.
 * 2. call the init() method which will in turn call the reset of the InitXXX() methods.
 *    The order of calling the methods are
 *        PreInit()
 *        InitROSParams()
 *        InitROSPublishers()
 *        InitROSSubscribers()
 *        InitGeneric();
 */
class BaseHandler
{
public:
    /**
     * Class constructor
     *
     */
    BaseHandler(): m_task_id(-1), m_task_name("")
    {
        // initialise the private node handle
        m_nh_private = ros::NodeHandle("~");
        m_nh_global  = ros::NodeHandle();
    }

    /// Class destructor
    virtual ~BaseHandler() {}

    /**
     * Returns the current task status
     * @return the current task status
     */
    int getCurrentTaskStatus() const { return m_curr_task_status; }

    /**
     * Retrieves the global ROS param value
     * @param key Key to look for
     * @param value Value will be returned here
     * @return true if retrieved successfully, false otherwise
     */
    template <typename T>
    bool getGlobalROSParamValue(const std::string &key, T &value)
    {
        bool isok = false;
        if (m_nh_global.hasParam(key))
        {
            if (m_nh_global.getParam(key, value))
            {
                isok = true;
            }
            else
            {
                ROS_ERROR_STREAM("Could not retrieve the global ROS parameter " << key);
            }
        }
        else
        {
            ROS_ERROR_STREAM("Global ROS parameter " << key << " is missing");
        }
        return isok;
    }

    /**
     * Retrieves the ROS param value using the private node handle
     * @param key Key to look for
     * @param value Value will be returned here
     * @return true if retrieved successfully, false otherwise
     */
    template <typename T>
    bool getPrivateROSParamValue(const std::string &key, T &value)
    {
        bool isok = false;
        if (m_nh_private.hasParam(key))
        {
            if (m_nh_private.getParam(key, value))
            {
                isok = true;
            }
            else
            {
                ROS_ERROR_STREAM("Could not retrieve the private ROS parameter " << key);
            }
        }
        else
        {
            ROS_ERROR_STREAM("Private ROS parameter " << key << " is missing");
        }
        return isok;
    }

    /**
     * Returns the full ROS param key name task_name/ros_param.
     * @param ros_param ROS param key name
     * @return the full ROS param key name
     */
    inline std::string GetROSParamFullKeyName(const std::string &ros_param)
    {
        return m_task_name + "/" + ros_param;
    }

    /**
     * Initialises the system
     *
     */
    void init()
    {
        // pre-initialisation if any
        PreInit();

        // read our ROS params
        InitROSParamTaskID();
        InitROSParams();

        // initialise our publishers
        InitROSPublisherTaskStatus();
        InitROSPublishers();

        // initialise our subscribers
        InitROSSubscriberCommandPub2();
        InitROSSubscribers();

        // other generic initialisation
        InitGeneric();
    }

    /**
     * Publishes the task's status
     * @param status Status to publish
     */
    void publishTaskStatus(const int &status)
    {
        m_curr_task_status = status;
        logg::ldebug() << ros::this_node::getName() << " - publishing task status " << m_curr_task_status;

        task_msgs::TaskStatus msg;
        msg.status = status;
        m_ros_pub_task_status.publish(msg);
    }

protected:
    /// MUST be declared first, global Node Handle
    ros::NodeHandle m_nh_global;

    /// private node handle
    ros::NodeHandle m_nh_private;

private:
    /// Generic initialisation method which is called last by init()
    virtual void InitGeneric() = 0;

    /// method called by init() to read all ROS params
    virtual void InitROSParams() = 0;

    /// method called by init() to initialise all the publishers
    virtual void InitROSPublishers() = 0;

    /// method called by init() to initialise all the subscribers
    virtual void InitROSSubscribers() = 0;

    /**
     * Always the first method called by init() for pre-initialisation.
     * Useful to perform checking for availability of service and actionserver
     */
    virtual void PreInit() = 0;

    /**
     * Runs the task
     * @param jsonstr_data JSON data from CommandPub2
     */
    virtual void runTask(const std::string &jsonstr_data) = 0;

    /// stops the task
    virtual void stopTask() = 0;

    /// initialises the task ID
    void InitROSParamTaskID()
    {
        // retrieve the task ID
        int task_id;
        if (getPrivateROSParamValue(K_STR_ROS_PARAM_TASK_ID, task_id))
        {
            if (task_id >= K_USER_TASK_ID_START)
            {
                m_task_id = task_id;

                // retrieve the navigation taskname as read from the task definition file
                InitTaskName();
            }
            else
            {
                ROS_ERROR_STREAM("Task ID cannot be a system task ID for node " << ros::this_node::getName());
            }
        }
        else
        {
            ROS_ERROR_STREAM("Failed to retrieve task ID for node " << ros::this_node::getName());
        }
    }

    /// Initialises the ROS publisher for the task status
    void InitROSPublisherTaskStatus()
    {
        // only run if task id is valid
        if (m_task_id >= K_USER_TASK_ID_START)
        {
            ROS_DEBUG_STREAM("Executing ROS publishers initialization for " << ros::this_node::getName());

            // task status
            std::string topicname;
            utils::TaskDefinitionData util_task_def;
            if (util_task_def.getTaskStatusTopicName(m_task_id, topicname))
            {
                if (topicname.empty())
                {
                    ROS_ERROR_STREAM("TaskHandler::InitROSPublishers - topic name for task ID " << m_task_id << " is empty for node " << ros::this_node::getName());
                }
            }
            else
            {
                ROS_ERROR_STREAM("TaskHandler::InitROSPublishers - topic name for task ID " << m_task_id << " cannot be found for node " << ros::this_node::getName());
            }

            if (!topicname.empty())
            {
                ROS_INFO_STREAM("The node " << ros::this_node::getName() << " is now publishing for task ID " << m_task_id);
                m_ros_pub_task_status = m_nh_global.advertise<task_msgs::TaskStatus>(topicname, 10);
            }
        }
    }

    /// initialises the subscriber for command pub2
    void InitROSSubscriberCommandPub2()
    {
        // subscribe to command pub2
        m_ros_sub_commandpub2 = m_nh_global.subscribe<mrccc_ros_messages::CommandPub2>(K_ROSTOPIC_NAME_MRCCC_ROS_MESSAGES_COMMAND_PUB2,
                                                                                       10,
                                                                                       &BaseHandler::OnROSCommandPub2Callback,
                                                                                       this);
    }

    /**
     * If we are using commandpub2, we need to know the task name so that we can
     * append the ROS params to it
     */
    void InitTaskName()
    {
        utils::TaskDefinitionData util_task_def;
        if (!util_task_def.m_tasks_definition.getTaskDefinitionData().getTaskName(m_task_id, m_task_name))
        {
            ROS_ERROR_STREAM(ros::this_node::getName() << " InitTaskName failed to read task name");
        }
        else if (m_task_name.empty())
        {
            ROS_ERROR_STREAM(ros::this_node::getName() << " InitTaskName task name is empty!");
        }
    }

    /**
     * Callback when the Command Pub2 ROS message is received
     * @param msg Message received
     */
    void OnROSCommandPub2Callback(const mrccc_ros_messages::CommandPub2::ConstPtr &msg)
    {
        if (msg->modes.size() == msg->targets.size())
        {
            /// @todo ROSParam data in CommandPub2
            if (msg->modes.size() == msg->data.size())
            {
                for (unsigned int i = 0; i < msg->modes.size(); ++i)
                {
                    if (msg->targets.at(i) == m_task_id)
                    {
                        if (msg->modes.at(i) == mrccc_ros_messages::CommandPub2::K_MODE_RUN)
                        {
                            /// @todo ROSParam data in CommandPub2
                            runTask(msg->data.at(i));
                        }
                        else
                        {
                            stopTask();
                        }
                        break;
                    }
                    else if (msg->targets.at(i) == mrccc_ros_messages::CommandPub2::K_TARGET_NODE_ALL &&
                             msg->modes.at(i) == mrccc_ros_messages::CommandPub2::K_MODE_IDLE)
                    {
                        // only support stop all nodes
                        stopTask();
                    }
                }
            }
            else
            {
                ROS_ERROR_STREAM(ros::this_node::getName() << " CommandPub2 data and mode size not matching");
            }
        }
    }

    /// the current task status
    int m_curr_task_status;

    /// ROS publisher for task status
    ros::Publisher m_ros_pub_task_status;

    /// ROS subscriber to the command pub2 message
    ros::Subscriber m_ros_sub_commandpub2;

    /// our task ID
    int m_task_id;

    /// name of our task as defined in the task definition file
    std::string m_task_name;
};

} // handler

#endif // BASE_HANDLER_HPP
