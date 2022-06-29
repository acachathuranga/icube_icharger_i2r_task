#ifndef WIRELESS_CHARGER
#define WIRELESS_CHARGER

#include "charger_control_task/charger/charger_interface.h"
#include "charger_control_task/global_defs.h"
#include <ros/ros.h>
#include <string.h>
#include <atomic>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <actionlib/client/simple_action_client.h>
#include "wcharger_controller/ChargeAction.h"
#include "wcharger_controller/StopChargeAction.h"
#include "wcharger_controller/ChargeEnums.hpp"
#include "boost/function.hpp"
#include <math.h>

class WirelessCharger : public Charger
{
    public:
        WirelessCharger(ros::NodeHandle *nh, boost::function<void (Charger::Status)> callback);
        ~WirelessCharger();

        bool enable_charger() override;
        bool disable_charger() override;

        void cancel(void) override;

        Charger::Status get_status() override;

    
    private:
        ros::NodeHandle *nh_;
        std::atomic<Charger::Status> status_;
        boost::function<void (Charger::Status)> callback_;

        actionlib::SimpleActionClient<wcharger_controller::ChargeAction> *start_charger_action_;
        actionlib::SimpleActionClient<wcharger_controller::StopChargeAction> *stop_charger_action_;
        ros::ServiceServer charger_control_service_;
        ros::Subscriber cmd_vel_subscriber_;
        ros::Subscriber battery_state_listener_;
        ros::Publisher  cmd_vel_publisher_;

        int charger_address_;
        int battery_type_capacity_;
        int charging_voltage_;
        int charging_current_;
        int max_tries_;
        int charge_interval_mins_;
        int target_capacity_;
        bool trickle_charge_;

        const std::string charger_enable_action_topic_param = K_CHARGER_ENABLE_ACTION_TOPIC_PARAM;
        const std::string charger_disable_action_topic_param = K_CHARGER_DISABLE_ACTION_TOPIC_PARAM;
        const std::string battery_status_topic_param = K_BATTERY_STATUS_TOPIC_PARAM;
        const std::string charger_control_service_param = K_CHARGER_CONTROL_SERVICE_PARAM;
        const std::string cmd_vel_input_topic_param = K_CMD_VEL_INPUT_TOPIC_PARAM;
        const std::string cmd_vel_output_topic_param = K_CMD_VEL_OUTPUT_TOPIC_PARAM;

        const std::string charger_address_param = K_CHARGER_ADDRESS_PARAM;
        const std::string battery_capacity_param = K_BATTERY_CAPACITY_PARAM;
        const std::string charging_voltage_param = K_CHARGING_VOLTAGE_PARAM;
        const std::string charging_current_param = K_CHARGING_CURRENT_PARAM;
        const std::string max_tries_param = K_MAX_CHARGING_RETRIES_PARAM;
        const std::string charge_interval_mins_param = K_CHARGE_INTERVAL_MINS_PARAM;
        const std::string target_capacity_param = K_TARGET_CAPACITY_PARAM;
        const std::string trickle_charge_param = K_TRICKLE_CHARGE_PARAM;

        // Start Charging action callbacks
        void start_charger_done_callback(const actionlib::SimpleClientGoalState& state, const wcharger_controller::ChargeResultConstPtr& result);
        void start_charger_feedback(const wcharger_controller::ChargeFeedbackConstPtr &feedback);

        // Stop Charging action callbacks
        void stop_charger_done_callback(const actionlib::SimpleClientGoalState& state, const wcharger_controller::StopChargeResultConstPtr& result);
        void stop_charger_feedback(const wcharger_controller::StopChargeFeedbackConstPtr &feedback);

        // Cmd vel callback
        void cmd_vel_callback(const geometry_msgs::TwistConstPtr &msg);

        // Charger Control service callback
        bool charger_control_callback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);

        // Battery state callback
        void battery_state_callback(const sensor_msgs::BatteryStateConstPtr &msg);
        void battery_state_callback(const std_msgs::BoolConstPtr &msg);
};


#endif // WIRELESS_CHARGER
