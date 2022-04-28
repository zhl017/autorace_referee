#include "idm_level.h"

void setup()
{
  nh.initNode();
  nh.getHardware()->setBaud(115200);

  // nh.advertise(stage_pub);

  nh.subscribe(reset_sub);
  // nh.subscribe(stage_sub);
  nh.subscribe(state_sub);

  nh.loginfo("Connected to idm_level!");

  ollo.begin(1, DMS_SENSOR);
  ollo.begin(2, DMS_SENSOR);
  ollo.begin(3, DMS_SENSOR);

  ollo.begin(4, TOUCH_SENSOR);

  motorDriver.init();

  fnInitLevel();
}

void loop()
{
  fnReset();

  fnReceiveSensorDistance();

  // fnCheckStageStatus();

  fnCheckVehicleStatus();

  fnLevelControl();

  fnControlLevel();

  nh.spinOnce();
}

void fnInitLevel()
{
  fnSetLevel();
  level_turn = 0;

  sensor_distance[0] = 0;
  sensor_distance[1] = 0;
  sensor_distance[2] = 0;

  is_started[0] = false;
  is_started[1] = false;
  is_started[2] = false;

  pass = true;

  vehicle_state_ = WAITING;
  level_status_ = LEVEL_OPENED;
  stage_state_ = STAGE;
}

void fnReset()
{
  if (ollo.read(4, TOUCH_SENSOR))
  {
    fnInitLevel();
  }
}

void fnReceiveSensorDistance()
{
  sensor_distance[0] = ollo.read(1, DMS_SENSOR);
  sensor_distance[1] = ollo.read(2, DMS_SENSOR);
  sensor_distance[2] = ollo.read(3, DMS_SENSOR);
  delay(100);
}

// void fnCheckStageStatus()
// {
//   if (stage_state_ == S3)
//   {
//     if (sensor_distance[2] > DISTANCE_THRESHOLD_PASS && vehicle_state_ == STOP)
//     {
//       if (pass == true) stage_state_ = S3END;
//       else              stage_state_ = S3FAIL;

//       pbStage();
//     }
//   }
// }

void fnCheckVehicleStatus()
{
  if (sensor_distance[0] > DISTANCE_THRESHOLD_PASS && vehicle_state_ == WAITING)
    vehicle_state_ = ENTERED;
  else if (sensor_distance[1] > DISTANCE_THRESHOLD_PASS && vehicle_state_ == ENTERED)
    vehicle_state_ = STOP;
  else if (sensor_distance[2] > DISTANCE_THRESHOLD_PASS && vehicle_state_ == STOP)
    vehicle_state_ = PASSED;
}

void fnLevelControl()
{
  if (vehicle_state_ == ENTERED)
  {
    if (is_started[0] == false)
    {
      fnSetLevel();
      is_started[0] = true;
    }
    else
    {
      pass = false;
      level_status_ = LEVEL_CLOSED;
    }
  }
  else if (vehicle_state_ == STOP)
  {
    if (is_started[1] == false)
    {
      fnSetLevel();
      is_started[1] = true;
    }
    else
    {
      if (fnGetSince() > 5000.0)
      {
        level_status_ = LEVEL_OPENED;
        pass = true;
      }
      else
      {
        level_status_ = LEVEL_CLOSED;
        pass = false;
      }
    }
  }
  else if (vehicle_state_ == PASSED)
  {
    if (is_started[2] == false)
    {
      fnSetLevel();
      is_started[2] = true;
    }
    else
    {
      if (pass == false)
      {
        level_status_ = LEVEL_MIDDLE;
      }
      else
      {
        if (level_turn == 1)   level_status_ = LEVEL_SUCCESS;
        else                   level_status_ = LEVEL_OPENED;

        if (millis() - pre_time >= 1000)
        {
          level_turn = 1 - level_turn;
          pre_time = millis();
        }
      }
    }
  }
}

void fnControlLevel()
{
  if (level_status_ == LEVEL_OPENED)      motorDriver.controlPosition(DXL_ID, DXL_POSITION_VALUE_OPENED);
  else if (level_status_ == LEVEL_CLOSED)  motorDriver.controlPosition(DXL_ID, DXL_POSITION_VALUE_CLOSED);
  else if (level_status_ == LEVEL_MIDDLE)  motorDriver.controlPosition(DXL_ID, DXL_POSITION_VALUE_MIDDLE);
  else if (level_status_ == LEVEL_SUCCESS) motorDriver.controlPosition(DXL_ID, DXL_POSITION_VALUE_SUCCESS);
}

void fnSetLevel()
{
  start_time = fnGetCurrentTime();
}

double fnGetCurrentTime()
{
  return (double)millis();
}

double fnGetSince()
{
  double elapsed_time;

  elapsed_time = fnGetCurrentTime() - start_time;
  if (elapsed_time < 0.0)  start_time = fnGetCurrentTime();
  return elapsed_time;
}

// void pbStage()
// {
//   stage_msg.data = stage_state_;
//   stage_pub.publish(&stage_msg);
// }

void resetCallback(const std_msgs::Bool &reset_msg)
{
  fnInitLevel();
}

// void stageCallback(const std_msgs::Int8 &stage_msg)
// {
//   if (stage_msg.data == 5) stage_state_ = S3;
// }

void vehicleCallback(const std_msgs::Int8 &vehicle_msg)
{
  if (vehicle_msg.data == 2) fnInitLevel();
}
