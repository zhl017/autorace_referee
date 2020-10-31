#include <OLLO.h>
#include "motor_driver.h"
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

/*
   define
*/
#define DISTANCE_THRESHOLD_PASS 500
#define DXL_ID_1                  1
#define DXL_ID_2                  2
#define DXL_1_UP                2048
#define DXL_1_DOWN              3048
#define DXL_2_UP                2048
#define DXL_2_DOWN              1048

/*
   classes
*/
ros::NodeHandle nh;
OLLO ollo;
MotorDriver motorDriver;

/*
   variables
*/
static uint16_t sensor_distance[2];
static uint32_t pre_time;

uint8_t led_turn_ = 1;

double training_start_time, training_time;
double match_start_time, match_time;

double light_loop_time;
double random_delay;
double fail_delay;

bool mission_trigger, level_trigger, stopwatch_trigger, mission_start, s1_start, s1_end;
/*
   enum
*/
static enum Mode  {READY_MODE, TRAINING_MODE, MATCH_MODE, FINISH_MODE} mode_;
static enum State {MISSION, FAIL, PASS} state_;
static enum Mission_State {READY, TRAINING_START, TRAINING_FINISH, MATCH_START, MATCH_FINISH} mission_state_;
static enum Stopwatch_State {SETUP, TRAINING_TIMEOUT, MATCH_TIMEOUT} stopWatchState_;
static enum Color {LED_RED, LED_YELLOW, LED_GREEN, LED_ALL_HIGH} led_color_;
static enum Stage_State {STAGE, S1, S1END, S2, S2END, S3, S3END, S3FAIL, S4} stage_state_;
/*
   functions
*/
void fnInitDxl();
void fnInitTrafficLight();
void fnCheckMode();
void fnReset();
void fnReceiveSensorDistance();
void fnCheckVehicleStatus();
void fnLEDControl();
void fnControlLED();
void fnGetRandomDelay();
double fnGetCurrentTime();
/*
   publisher & publish function
*/
void pbVehicle();
void pbStage();

std_msgs::Int8 vehicle_msg;
ros::Publisher vehicle_pub("vehicle", &vehicle_msg);

std_msgs::Int8 stage_msg;
ros::Publisher stage_pub("stage", &stage_msg);
/*
   subscriber & subscribe function
*/
void resetCallback(const std_msgs::Bool &reset_msg);
void stateCallback(const std_msgs::Int8 &state_msg);
ros::Subscriber<std_msgs::Bool> reset_sub("reset", resetCallback);
ros::Subscriber<std_msgs::Int8> state_sub("state", stateCallback);
