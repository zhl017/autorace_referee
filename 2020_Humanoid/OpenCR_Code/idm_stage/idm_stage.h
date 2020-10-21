#include <OLLO.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

/*
   define
*/
#define DISTANCE_THRESHOLD_PASS 500

/*
   classes
*/
ros::NodeHandle nh;
OLLO ollo;

/*
   variables
*/
static uint16_t sensor_distance[4];

/*
   enum
*/
static enum Stage_State {STAGE, S1, S1END, S2, S2END, S3, S3END, S3FAIL, S4} stage_state_;

/*
   funcitons
*/
void fnInitState();
void fnReceiveSensorDistance();
void fnCheckStageStatus();
/*
   publisher & fn
*/
void pbStage();

std_msgs::Int8 stage_msg;
ros::Publisher stage_pub("stage", &stage_msg);

/*
   subscriber & fn
*/
void resetCallback(const std_msgs::Bool &reset_msg);
void stageCallback(const std_msgs::Int8 &stage_msg);
ros::Subscriber<std_msgs::Bool> reset_sub("reset", resetCallback);
ros::Subscriber<std_msgs::Int8> stage_sub("stage", stageCallback);
