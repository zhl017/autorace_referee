#include <OLLO.h>
#include "motor_driver.h"
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
/*
   functions
*/
void fnInitLevel();
void fnReset();
void fnReceiveSensorDistance();
void fnCheckVehicleStatus();
void fnLevelControl();
void fnControlLevel();
