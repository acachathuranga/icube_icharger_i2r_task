#include "charger_control_task/charger/charger.h"

WirelessCharger::WirelessCharger(ros::NodeHandle *nh, boost::function<void (Charger::Status)> callback) : nh_(nh), callback_(callback)
{
    std::string charger_enable_action_topic;
    std::string charger_disable_action_topic;
    std::string battery_status_topic;
    std::string charger_control_service;
    std::string cmd_vel_input_topic;
    std::string cmd_vel_output_topic;

    // Fetch parameters
    nh_->param<std::string>(charger_enable_action_topic_param, charger_enable_action_topic, "/start_wireless_charge");
    nh_->param<std::string>(charger_disable_action_topic_param, charger_disable_action_topic, "/stop_wireless_charge");
    nh_->param<std::string>(charger_control_service_param, charger_control_service, "/enable_charger");
    nh_->param<std::string>(battery_status_topic_param, battery_status_topic, "/enableeess");
    nh_->param<std::string>(cmd_vel_input_topic_param, cmd_vel_input_topic, "/base_cmd_vel");
    nh_->param<std::string>(cmd_vel_output_topic_param, cmd_vel_output_topic, "/charging_safe_cmd_vel");

    nh_->param<int>(charger_address_param, charger_address_, 11);
    nh_->param<int>(battery_capacity_param, battery_type_capacity_, 15);
    nh_->param<int>(charging_voltage_param, charging_voltage_, 48);
    nh_->param<int>(charging_current_param, charging_current_, 5);
    nh_->param<int>(max_tries_param, max_tries_, 5);
    nh_->param<int>(charge_interval_mins_param, charge_interval_mins_, 2);
    nh_->param<int>(target_capacity_param, target_capacity_, 90);
    nh_->param<bool>(trickle_charge_param, trickle_charge_, true);

    // Start Charger action client
    start_charger_action_ = new actionlib::SimpleActionClient<wcharger_controller::ChargeAction>(charger_enable_action_topic, true);
    ROS_INFO("Waiting for %s to start.", charger_enable_action_topic.c_str());
    start_charger_action_->waitForServer();
    ROS_INFO("%s initialization complete", charger_enable_action_topic.c_str());

    // Stop Charger action client
    stop_charger_action_ = new actionlib::SimpleActionClient<wcharger_controller::StopChargeAction>(charger_disable_action_topic);
    ROS_INFO("Waiting for %s to start.", charger_disable_action_topic.c_str());
    stop_charger_action_->waitForServer();
    ROS_INFO("%s initialization complete", charger_disable_action_topic.c_str());

    // Cmd_vel publishers and subscribers
    cmd_vel_publisher_ = nh_->advertise<geometry_msgs::Twist>(cmd_vel_output_topic, 1000);
    cmd_vel_subscriber_ = nh_->subscribe<geometry_msgs::Twist>(cmd_vel_input_topic,1000, &WirelessCharger::cmd_vel_callback, this);

    // Charger control service
    charger_control_service_ = nh_->advertiseService(charger_control_service, &WirelessCharger::charger_control_callback, this);

    // Battery State subscriber
    //battery_state_listener_ = nh_->subscribe<sensor_msgs::BatteryState>(battery_status_topic, 1000, &WirelessCharger::battery_state_callback, this);
    battery_state_listener_ = nh_->subscribe<std_msgs::Bool>(battery_status_topic, 1000, &WirelessCharger::battery_state_callback, this);
}

WirelessCharger::~WirelessCharger()
{
    // Delete objects
    delete start_charger_action_;
    delete stop_charger_action_;
}

bool WirelessCharger::enable_charger()
{
    wcharger_controller::ChargeGoal goal;
    goal.charger_address = charger_address_;
    goal.battery_type_capacity = battery_type_capacity_;
    goal.charging_voltage = charging_voltage_;
    goal.charging_current = charging_current_;
    goal.max_tries = max_tries_;
    goal.charge_interval_mins = charge_interval_mins_;
    goal.target_capacity = target_capacity_;
    goal.trickle_charge = trickle_charge_;
    
    if (start_charger_action_->getState() != actionlib::SimpleClientGoalState::ACTIVE)
    {
        start_charger_action_->sendGoal(goal,
        boost::bind(&WirelessCharger::start_charger_done_callback, this, _1, _2),
        actionlib::SimpleActionClient<wcharger_controller::ChargeAction>::SimpleActiveCallback(), 
        boost::bind(&WirelessCharger::start_charger_feedback, this, _1));
    }

    // TODO : Implement chanrging status checking before reporting task status to I2R
    callback_(Charger::Status::CHARGING);
    return true;
}

bool WirelessCharger::disable_charger()
{
    if (stop_charger_action_->getState() != actionlib::SimpleClientGoalState::ACTIVE)
    {
        wcharger_controller::StopChargeGoal goal;
        stop_charger_action_->sendGoal(goal, 
            boost::bind(&WirelessCharger::stop_charger_done_callback, this, _1, _2),
            actionlib::SimpleActionClient<wcharger_controller::StopChargeAction>::SimpleActiveCallback(), 
            boost::bind(&WirelessCharger::stop_charger_feedback, this, _1));

        start_charger_action_->cancelAllGoals();

        // TODO : Implement chanrging status checking before reporting task status to I2R
        callback_(Charger::Status::NOT_CHARGING);
    }
    return true;
}

void WirelessCharger::cancel()
{
    // Cancel all current and pending actions
    start_charger_action_->cancelAllGoals();
    stop_charger_action_->cancelAllGoals();
}

Charger::Status WirelessCharger::get_status()
{
    return status_;
}

void WirelessCharger::cmd_vel_callback(const geometry_msgs::TwistConstPtr &msg)
{
    if (status_ == Charger::Status::NOT_CHARGING)
    {
        // Pass through cmd vel if charger is disabled
        cmd_vel_publisher_.publish(msg);
    }
    else
    {
        // Disable charger if velocity command is detected while charging
        if (( pow(msg->angular.z, 2) + pow(msg->linear.x, 2) ) != 0 )
        {
            ROS_WARN("Charger Controller: Robot velocity command given while charger state is in %s state! Disabling charger now!", status_to_string(status_).c_str());
            disable_charger();
        }
    }
}

bool WirelessCharger::charger_control_callback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
{
    if (request.data)
    {
        ROS_INFO_STREAM("Charger Enable Request Received");
        response.success = enable_charger();
    }
    else
    {
        ROS_INFO_STREAM("Charger Disable  Request Received");
        response.success = disable_charger();
    }
    response.message = "Charger Command Completed";
    
    return true;
}

void WirelessCharger::battery_state_callback(const sensor_msgs::BatteryStateConstPtr &msg)
{
    switch(msg->power_supply_status)
    { 
        case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL: status_ = Charger::Status::CHARGING; break;
        case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING: status_ = Charger::Status::CHARGING; break;
        case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING : status_ = Charger::Status::NOT_CHARGING; break;
        case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING: status_ = Charger::Status::NOT_CHARGING; break;
        case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN: status_ = Charger::Status::UNKNOWN; break;
        default: status_ = Charger::Status::ERROR; break;
    }
}

void WirelessCharger::battery_state_callback(const std_msgs::BoolConstPtr &msg)
{
    if (msg->data)
    {
        status_ = Charger::Status::CHARGING;
    }
    else
    {
        status_ = Charger::Status::NOT_CHARGING;
    }
}

void WirelessCharger::start_charger_done_callback(const actionlib::SimpleClientGoalState& state, const wcharger_controller::ChargeResultConstPtr& result)
{
    if (result->result_state == (int)ChargeRequestResult::Charging)
    {
        //TODO
        //callback_(Charger::Status::CHARGING);
    }
    else
    {
        //TODO
        //callback_(Charger::Status::ERROR);
    }
    ROS_INFO("Task finished in state [%s] [%d]", state.toString().c_str(), result->result_state);
}

void WirelessCharger::start_charger_feedback(const wcharger_controller::ChargeFeedbackConstPtr &feedback)
{
    if (feedback->feedback_state == (int)ChargeRequestFeedback::OutOfRange)
    {
        ROS_INFO("Charger Out of Range");
    }
    else if (feedback->feedback_state == (int)ChargeRequestFeedback::TooClose)
    {
        ROS_INFO("Charger Too Close");
    }
    else if (feedback->feedback_state == (int)ChargeRequestFeedback::TooFar)
    {
        ROS_INFO("Charger Too Far");
    }
}
      
void WirelessCharger::stop_charger_done_callback(const actionlib::SimpleClientGoalState& state, const wcharger_controller::StopChargeResultConstPtr& result)
{
    if (result->result_state == (int)ChargeRequestResult::NotCharging)
    {
        // TODO
        //callback_(Charger::Status::NOT_CHARGING);
    }
    else
    {
        // TODO
        //callback_(Charger::Status::ERROR);
    }
    ROS_INFO("Task finished in state [%s] [%d]", state.toString().c_str(), result->result_state);
}

void WirelessCharger::stop_charger_feedback(const wcharger_controller::StopChargeFeedbackConstPtr &feedback)
{
    // No feedback given for stop charger action
}