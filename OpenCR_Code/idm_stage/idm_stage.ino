#include "idm_stage.h"

void setup()
{
  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.advertise(stage_pub);

  nh.subscribe(reset_sub);
  nh.subscribe(stage_sub);

  nh.loginfo("Connected to idm_stage!");

  ollo.begin(1, DMS_SENSOR);
  ollo.begin(2, DMS_SENSOR);
  ollo.begin(3, DMS_SENSOR);
  ollo.begin(4, DMS_SENSOR);

  fnInitState();
}

void loop()
{
  fnReceiveSensorDistance();

  fnCheckStageStatus();

  nh.spinOnce();
}

void fnInitState()
{
  sensor_distance[0] = 0;
  sensor_distance[1] = 0;
  sensor_distance[2] = 0;
  sensor_distance[3] = 0;

  stage_state_ = STAGE;
}

void fnReceiveSensorDistance()
{
  sensor_distance[0] = ollo.read(1, DMS_SENSOR);
  sensor_distance[1] = ollo.read(2, DMS_SENSOR);
  sensor_distance[2] = ollo.read(3, DMS_SENSOR);
  sensor_distance[3] = ollo.read(4, DMS_SENSOR);
  delay(100);
}

void fnCheckStageStatus()
{
  if (stage_state_ == S1)
  {
    if (sensor_distance[0] > DISTANCE_THRESHOLD_PASS)
    {
      stage_state_ = S1END;
      pbStage();
    }
  }
  if (stage_state_ == S1END)
  {
    if (sensor_distance[1] > DISTANCE_THRESHOLD_PASS)
    {
      stage_state_ = S2;
      pbStage();
    }
  }
  if (stage_state_ == S2)
  {
    if (sensor_distance[2] > DISTANCE_THRESHOLD_PASS)
    {
      stage_state_ = S3;
      pbStage();
    }
  }
  if (stage_state_ == S3END)
  {
    if (sensor_distance[3] > DISTANCE_THRESHOLD_PASS)
    {
      stage_state_ = S4;
      pbStage();
    }
  }
}

/**
   Publisher
*/
void pbStage()
{
  stage_msg.data = stage_state_;

  stage_pub.publish(&stage_msg);
}

/**
   Subscriber
*/
void resetCallback(const std_msgs::Bool &reset_msg)
{
  fnInitState();
}

void stageCallback(const std_msgs::Int8 &stage_msg)
{
  if (stage_msg.data == 1)  stage_state_ = S1;
  else if (stage_msg.data == 6 || stage_msg.data == 7) stage_state_ = S3END;
}
