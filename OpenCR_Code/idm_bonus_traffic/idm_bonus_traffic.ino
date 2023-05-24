#include "idm_bonus_traffic.h"

/*
   param
*/
const int GREEN_push = BDPIN_GPIO_6;  //training mode
const int RED_push = BDPIN_GPIO_4;    //match mode
const int RED_YELLOW_delay = 5000;    //RED -> YELLOW delay
const int YELLOW_GREEN_delay = 5000;  //YELLOW -> GREEN delay
const int GREEN_RED_delay = 5000;     //GREEN -> RED delay
const int LED_delay = 5000;

/*
   setup
*/
void setup()
{
  //set DMS
  ollo.begin(1, DMS_SENSOR);
  ollo.begin(2, DMS_SENSOR);

  //set RESET_BT
  ollo.begin(4, TOUCH_SENSOR);

  //set LEDS  Red:9 Yellow:10 Green:11 共陽極:HIGH(滅) 共陰極:LOW(滅)
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);

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

  fnCheckVehicleStatus();

  fnLEDControl();

  fnControlLED();
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

  sensor_distance[0] = 0;
  sensor_distance[1] = 0;

  light_loop_time = 0.0;
  start_time = 0.0;
  bonus_time = 0.0;

  mission_trigger = false;
  mission_start = false;

  led_color_ = LED_RED;
  mode_ = READY_MODE;
  state_ = MISSION;
}

void fnCheckMode()
{
  if (mode_ == READY_MODE && ( digitalRead(GREEN_push) == LOW || digitalRead(RED_push) == LOW))
  {
    start_time = fnGetCurrentTime();
    bonus_time = start_time;

    mode_ = BONUS_MODE;

    fnInitDxl();
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
    else if (mode_ == BONUS_MODE)
    {
      bonus_time = fnGetCurrentTime();
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

void fnCheckVehicleStatus()
{
  if (mode_ == BONUS_MODE)
  {
    if (sensor_distance[0] > DISTANCE_THRESHOLD_PASS && mission_trigger == false)
    {
      if (led_color_ == LED_GREEN) state_ = PASS;
      else state_ = FAIL;
      mission_trigger = true;
    }
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
  if (mode_ == BONUS_MODE)
  {
    if (state_ == MISSION)
    {
      light_loop_time = fnGetCurrentTime() - bonus_time;
      if(light_loop_time <= LED_delay) led_color_ = LED_RED;
      else if(light_loop_time > LED_delay && light_loop_time <= (2*LED_delay)) led_color_ = LED_YELLOW;
      else if(light_loop_time > (2*LED_delay) && light_loop_time <= (3*LED_delay))
      {
        led_color_ = LED_GREEN;
        if (mission_start == false)
        {
          mission_start = true;
        }
      } 
      else if(light_loop_time > (3*LED_delay)) bonus_time = fnGetCurrentTime();
    }

    else
      mode_ = FINISH_MODE;
  }

  else if (mode_ == FINISH_MODE)
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
double fnGetCurrentTime()
{
  return (double)millis();
}
