#include "idm_traffic.h"

/*
   param
*/
const int GREEN_push = BDPIN_GPIO_6;  //training mode
const int RED_push = BDPIN_GPIO_4;    //match mode
const int RED_YELLOW_delay = 5000;    //RED -> YELLOW delay
const int YELLOW_GREEN_delay = 5000;  //YELLOW -> GREEN delay
const int GREEN_RED_delay = 5000;     //GREEN -> RED delay

/*
   setup
*/
void setup()
{
  //init ros node handle
  nh.initNode();
  nh.getHardware()->setBaud(115200);
  //publisher
  nh.advertise(vehicle_pub);
  nh.advertise(stage_pub);
  //subscriber
  nh.subscribe(reset_sub);
  nh.subscribe(state_sub);

  nh.loginfo("Connected to idm_traffic!");

  //set DMS
  ollo.begin(1, DMS_SENSOR);
  ollo.begin(2, DMS_SENSOR);

  //set RESET_BT
  ollo.begin(4, TOUCH_SENSOR);

  //set LEDS  R:9 G:10 B:11
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  digitalWrite(9, HIGH);
  digitalWrite(10, HIGH);
  digitalWrite(11, HIGH);

  //set BOX
  pinMode(GREEN_push, INPUT_PULLUP);
  pinMode(RED_push, INPUT_PULLUP);

  //init Dynamixel
  motorDriver.init();

  //init paramters
  fnInitTrafficLight();
}

void loop()
{
  fnCheckMode();

  fnReset();

  fnReceiveSensorDistance();

  fnCheckStageStatus();

  fnCheckVehicleStatus();

  fnLEDControl();

  fnControlLED();

  nh.spinOnce();
}

/*
   fuction
*/
void fnInitDxl()  //Dynamixel reset
{
  level_trigger = false;
  motorDriver.controlPosition(DXL_ID_1, DXL_1_DOWN);
  motorDriver.controlPosition(DXL_ID_2, DXL_2_DOWN);
}

void fnInitTrafficLight()
{
  fnInitDxl();

  fnGetRandomDelay();

  sensor_distance[0] = 0;
  sensor_distance[1] = 0;
///  sensor_distance[2] = 0;

  training_start_time = 0.0;
  match_start_time = 0.0;
  training_time = 0.0;
  match_time = 0.0;
  light_loop_time = 0.0;
  fail_delay = 0.0;

  mission_start = false;
  mission_trigger = false;
  stopwatch_trigger = false;
  s1_start = false;
  s1_end = false;

  led_color_ = LED_RED;
  mode_ = READY_MODE;
  state_ = MISSION;
  mission_state_ = READY;
  stopWatchState_ = SETUP;
  stage_state_ = STAGE;

  pbVehicle();
  pbStage();
}

void fnCheckMode()
{
  if (mode_ == READY_MODE && digitalRead(GREEN_push) == LOW)
  {
    training_start_time = fnGetCurrentTime();
    training_time = training_start_time;

    mode_ = TRAINING_MODE;
    mission_state_ = TRAINING_START;

    fnInitDxl();

    pbVehicle();
  }

  else if ((mode_ == TRAINING_MODE && digitalRead(RED_push) == LOW) || stopWatchState_ == TRAINING_TIMEOUT)
  {
    match_start_time = fnGetCurrentTime();
    match_time = match_start_time;

    mode_ = MATCH_MODE;
    mission_state_ = TRAINING_FINISH;
    stopWatchState_ = SETUP;

    fnGetRandomDelay();
    fnInitDxl();

    pbVehicle();
  }

  else if (stopWatchState_ == MATCH_TIMEOUT)
  {
    mode_ = FINISH_MODE;
  }
}

void fnReset()
{
  if (ollo.read(4, TOUCH_SENSOR))
  {
    fnInitDxl();

    if (mode_ == READY_MODE)
    {
      if (led_color_ == LED_RED)           led_color_ = LED_YELLOW;
      else if (led_color_ == LED_YELLOW)   led_color_ = LED_GREEN;
      else if (led_color_ == LED_GREEN)    led_color_ = LED_RED;
      delay(100);
    }
    else if (mode_ == TRAINING_MODE)
    {
      training_time = fnGetCurrentTime();
      fnGetRandomDelay();
    }
    else
      fnInitTrafficLight();
  }
}

void fnReceiveSensorDistance() //get DMS value
{
  sensor_distance[0] = ollo.read(1, DMS_SENSOR);
  sensor_distance[1] = ollo.read(2, DMS_SENSOR);
  delay(100);
}

void fnCheckStageStatus()
{
  if (mode_ == MATCH_MODE)
  {
    if (sensor_distance[0] > DISTANCE_THRESHOLD_PASS && s1_start == false)
    {
      stage_state_ = S1;
      pbStage();
      s1_start = true;
    }
//    if (sensor_distance[2] > DISTANCE_THRESHOLD_PASS && s1_start == true && s1_end == false && level_trigger == true)
//    {
//      stage_state_ = S1END;
//      pbStage();
//      s1_end = true;
//    }
  }
}

void fnCheckVehicleStatus()
{
  if (mode_ == MATCH_MODE)
  {
    if (sensor_distance[0] > DISTANCE_THRESHOLD_PASS && mission_trigger == false)
    {
      if (led_color_ == LED_GREEN) state_ = PASS;
      else
      {
        state_ = FAIL;
        fail_delay = (fnGetCurrentTime() - match_start_time) - random_delay;
      }
      mission_trigger = true;

      if (mission_start == false)
      {
        mission_state_ = MATCH_START;
        pbVehicle();
        mission_start = true;
      }
    }

    if (sensor_distance[0] > DISTANCE_THRESHOLD_PASS && stopwatch_trigger == true)
    {
      mission_state_ = MATCH_FINISH;
      pbVehicle();
      stopwatch_trigger = false;
    }

    if (sensor_distance[2] > DISTANCE_THRESHOLD_PASS && level_trigger == true)
      stopwatch_trigger = true;
  }

  if (sensor_distance[1] > DISTANCE_THRESHOLD_PASS && level_trigger == false)
  {
    if (random(2) == 0)  motorDriver.controlPosition(DXL_ID_1, DXL_1_UP);
    else                 motorDriver.controlPosition(DXL_ID_2, DXL_2_UP);
    level_trigger = true;
  }
}

void fnLEDControl()
{
  if (mode_ == TRAINING_MODE)
  {
    light_loop_time = fnGetCurrentTime() - training_time;

    if (light_loop_time <= 3000)  led_color_ = LED_RED;
    else if (light_loop_time > 3000 && light_loop_time <= 6000) led_color_ = LED_YELLOW;
    else led_color_ = LED_GREEN;
  }

  else if (mode_ == MATCH_MODE)
  {
    if (state_ == MISSION)
    {
      light_loop_time = fnGetCurrentTime() - match_time;

      if (light_loop_time <= random_delay)  led_color_ = LED_RED;
      else if (light_loop_time > random_delay && light_loop_time <= random_delay + YELLOW_GREEN_delay) led_color_ = LED_YELLOW;
      else if (light_loop_time > random_delay && light_loop_time <= 2 * random_delay + YELLOW_GREEN_delay)
      {
        led_color_ = LED_GREEN;
        if (mission_start == false)
        {
          mission_state_ = MATCH_START;
          pbVehicle();
          mission_start = true;
        }
      }
      else
      {
        match_time = fnGetCurrentTime();
        fnGetRandomDelay();
      }
    }

    else
    {
      if (led_turn_ == 1)
      {
        if (state_ == FAIL)       led_color_ = LED_RED;
        else if (state_ == PASS)  led_color_ = LED_GREEN;
      }

      else                        led_color_ = LED_ALL_HIGH;

      if (millis() - pre_time >= 200)
      {
        led_turn_ = 1 - led_turn_;
        pre_time = millis();
      }
    }
  }
}

void fnControlLED()
{
  if (led_color_ == LED_RED)
  {
    digitalWrite(9, LOW);
    digitalWrite(10, HIGH);
    digitalWrite(11, HIGH);
  }
  else if (led_color_ == LED_YELLOW)
  {
    digitalWrite(9, HIGH);
    digitalWrite(10, LOW);
    digitalWrite(11, HIGH);
  }
  else if (led_color_ == LED_GREEN)
  {
    digitalWrite(9, HIGH);
    digitalWrite(10, HIGH);
    digitalWrite(11, LOW);
  }
  else
  {
    digitalWrite(9, HIGH);
    digitalWrite(10, HIGH);
    digitalWrite(11, HIGH);
  }
}

/*
   About time fuction
*/
void fnGetRandomDelay()
{
    random_delay = random(5, 11) * 1000; //5 ~ 10 sec
}

double fnGetCurrentTime()
{
  return (double)millis();
}
/*
   publish msgs
*/
void pbVehicle()
{
  vehicle_msg.data = mission_state_;
  vehicle_pub.publish(&vehicle_msg);
}
void pbStage()
{
  stage_msg.data = stage_state_;
  stage_pub.publish(&stage_msg);
}
/*
   subscribe callback
*/
void resetCallback(const std_msgs::Bool &reset_msg)
{
  fnInitTrafficLight();
}

void stateCallback(const std_msgs::Int8 &state_msg)
{
  switch (state_msg.data)
  {
    case 0:
      {
        stopWatchState_ = SETUP;
        break;
      }
    case 1:
      {
        stopWatchState_ = TRAINING_TIMEOUT;
        break;
      }
    case 2:
      {
        stopWatchState_ = MATCH_TIMEOUT;
        break;
      }
    default:
      break;
  }
}
