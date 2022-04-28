#include <OLLO.h>
#include "motor_driver.h"
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
/*
   define
*/
#define DISTANCE_THRESHOLD_PASS       500
#define DXL_ID                        1
#define DXL_POSITION_VALUE_CLOSED     3072
#define DXL_POSITION_VALUE_OPENED     2048
#define DXL_POSITION_VALUE_MIDDLE     2560
#define DXL_POSITION_VALUE_SUCCESS    2200
/*
   calsses
*/
ros::NodeHandle nh;
OLLO ollo;
MotorDriver motorDriver;
/*
   variables
*/
static uint16_t sensor_distance[3];
static uint32_t pre_time;
double start_time;
bool is_started[3], pass;
uint8_t level_turn = 1;
/*
   enum
*/
static enum State {WAITING, ENTERED, STOP, PASSED} vehicle_state_;
static enum Level {LEVEL_CLOSED, LEVEL_OPENED, LEVEL_MIDDLE, LEVEL_SUCCESS} level_status_;
static enum Stage_State {STAGE, S1, S1END, S2, S2END, S3, S3END, S3FAIL, S4} stage_state_;
/*
   functions
*/
void fnInitLevel();
void fnReset();
void fnReceiveSensorDistance();
void fnCheckStageStatus();
void fnCheckVehicleStatus();
void fnLevelControl();
void fnControlLevel();
/*
   publisher & fn
*/
// std_msgs::Int8 stage_msg;
// ros::Publisher stage_pub("stage", &stage_msg);
/*
   subscriber & fn
*/
void resetCallback(const std_msgs::Bool &reset_msg);
// void stageCallback(const std_msgs::Int8 &stage_msg);
void vehicleCallback(const std_msgs::Int8 &vehicle_msg);
ros::Subscriber<std_msgs::Bool> reset_sub("reset", resetCallback);
// ros::Subscriber<std_msgs::Int8> stage_sub("stage", stageCallback);
ros::Subscriber<std_msgs::Int8> state_sub("vehicle", vehicleCallback);
