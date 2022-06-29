#ifndef GLOBAL_DEFS
#define GLOBAL_DEFS

// charger.h 
#define K_CHARGER_ENABLE_ACTION_TOPIC_PARAM "charger_enable_action"
#define K_CHARGER_DISABLE_ACTION_TOPIC_PARAM "charger_disable_action"
#define K_BATTERY_STATUS_TOPIC_PARAM "battery_status_topic"
#define K_CHARGER_CONTROL_SERVICE_PARAM "charger_control_service"
#define K_CMD_VEL_INPUT_TOPIC_PARAM "cmd_vel_input"
#define K_CMD_VEL_OUTPUT_TOPIC_PARAM "cmd_vel_output"

#define K_CHARGER_ADDRESS_PARAM "charger_address"
#define K_BATTERY_CAPACITY_PARAM "battery_type_capacity"
#define K_CHARGING_VOLTAGE_PARAM "charging_voltage"
#define K_CHARGING_CURRENT_PARAM "charging_current"
#define K_MAX_CHARGING_RETRIES_PARAM "max_charging_retries"
#define K_CHARGE_INTERVAL_MINS_PARAM "charge_interval_mins"
#define K_TARGET_CAPACITY_PARAM "target_capacity"
#define K_TRICKLE_CHARGE_PARAM "trickle_charge"

// task_handler.h
#define K_ENABLE_CHARGER 1
#define K_DISABLE_CHARGER 0
#define K_COMMAND_PARAM "command"

#endif