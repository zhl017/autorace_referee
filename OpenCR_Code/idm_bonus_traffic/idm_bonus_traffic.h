#include <OLLO.h>
#include "motor_driver.h"

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
OLLO ollo;
MotorDriver motorDriver;

/*
   variables
*/
static uint16_t sensor_distance[2];
static uint32_t pre_time;

uint8_t led_turn_ = 1;

double bonus_time;
double start_time;

double light_loop_time;
double random_delay;
double fail_delay;

bool mission_trigger, level_trigger, stopwatch_trigger, mission_start;
/*
   enum
*/
static enum Mode  {READY_MODE, TRAINING_MODE, MATCH_MODE, FINISH_MODE, BONUS_MODE} mode_;
static enum State {MISSION, FAIL, PASS} state_;
static enum Mission_State {READY, TRAINING_START, TRAINING_FINISH, MATCH_START, MATCH_FINISH} mission_state_;
static enum Stopwatch_State {SETUP, TRAINING_TIMEOUT, MATCH_TIMEOUT} stopWatchState_;
static enum Color {LED_RED, LED_YELLOW, LED_GREEN, LED_ALL_HIGH} led_color_;
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
double fnGetCurrentTime();
