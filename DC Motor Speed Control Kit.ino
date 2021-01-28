#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "Arduino.h"
#include "math.h"
#include "ESP32Encoder.h"
#define TFT_DC 17
#define TFT_CS 4
#define TFT_RST 16
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

ESP32Encoder encoder;

float Speed = 0;
float alpha = 1;

const int vr1_pin = 13;
const int vr2_pin = 33;
const int vr3_pin = 14;
const int vr4_pin = 27;

const int encoderA_pin = 26;
const int encoderB_pin = 25;

const int MotorA_pin = 22;
const int MotorB_pin = 21;

const int freq = 15000;
const int resolution = 12;

uint32_t time_now = 0;
uint32_t time_prve1 = 0;
uint32_t time_prve2 = 0;
uint32_t time_prve3 = 0;

int counter = 0;
float Raw_SensorValue = 0;
float Raw_SensorValue_vr1, Raw_SensorValue_vr2, Raw_SensorValue_vr3, Raw_SensorValue_vr4 = 0;
float Set_point = 0;
float Set_point_plot = 0;

float Kp, Ki, Kd;
float error, sum_error, de_error, prev_error;
float PID_output = 0;

float Speed_plot = 0;
int x_new[250];
int x_new_1[250];
const int originx = 5;
const int originy = 235;
const int sizex = 220;
const int sizey = 170;

int pointer = 0;

float Map(float input, float in_min, float in_max, float min_value, float max_value)
{
  float tmp = (input * 10 - in_min * 10) * (max_value * 10 - min_value * 10) / (in_max * 10 - in_min * 10) + min_value * 10;
  return tmp * 0.1f;
}
float count_to_RPM_or_omega (float counter, float dt , float rotary , float x)
{
  float tmp = ((counter * 10.00f) / (x * dt * rotary));
  //   return tmp * 0.1f * 2 * 3.14159f; // omega
  return tmp * 0.1f * 60.00f; // RPM
}
float cal_coff(float fc, float d_T)
{
  float alpha = 0;
  alpha = (2.0f * M_PI * d_T * fc) / ((2.0f * M_PI * d_T * fc) + 1);
  return alpha;
}

float lpf(float alpha, float x_input, float y_output)
{
  return y_output + alpha * (x_input - y_output);
}
void setup()
{
  pinMode(MotorA_pin, OUTPUT);
  pinMode(MotorB_pin, OUTPUT);

  pinMode(vr1_pin, INPUT);
  pinMode(vr2_pin, INPUT);
  pinMode(vr3_pin, INPUT);
  pinMode(vr4_pin, INPUT);

  pinMode(encoderA_pin, INPUT_PULLDOWN);
  pinMode(encoderB_pin, INPUT_PULLDOWN);

  ledcSetup(0, freq, resolution);
  ledcSetup(1, freq, resolution);

  ledcAttachPin(MotorA_pin, 1);
  ledcAttachPin(MotorB_pin, 0);

  encoder.setCount(0);
  encoder.attachFullQuad(encoderA_pin, encoderB_pin);

  Serial.print(" Fc = 10Hz alpha=");
  Serial.println(cal_coff(10, 0.001));
  alpha = cal_coff(10, 0.001);

  Serial.begin(115200);

  tft.begin(26000000);
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);
  drawGraph();
  Taxt();
  delay(2000);

  xTaskCreatePinnedToCore(
    TaskRead_Speed, "TaskRead_omaga" // A name just for humans
    ,
    1024 * 3 // This stack size can be checked & adjusted by reading the Stack Highwater
    ,
    NULL, 2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,
    NULL, 1);

  xTaskCreatePinnedToCore(
    Task_control, "Task_control" // A name just for humans
    ,
    1024 * 3 // Stack size
    ,
    NULL, 2 // Priority
    ,
    NULL, 1);

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  time_now = millis();
  if (time_now - time_prve1 >= 50)
  {
    time_prve1 = time_now;
    Speed_plot = Map(Speed, 0, 5000, originy - 1, originy - sizey);
    Set_point_plot = Map(Set_point, 0, 5000, originy - 1, originy - sizey);
    Graph();
    Taxt();
  }
  if (time_now - time_prve2 >= 5)
  {
    Serial.print("time_now ");
    Serial.print(time_now);
    Serial.print(" Speed ");
    Serial.print(Speed);
    Serial.print(" Set_point ");
    Serial.println(Set_point);
  }
}
void TaskRead_Speed(void *pvParameters)
{
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 5;

  xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    counter = encoder.getCount();
    encoder.setCount(0);

    Raw_SensorValue = lpf(alpha, counter, Raw_SensorValue);

    Speed = count_to_RPM_or_omega(Raw_SensorValue, 0.005, 96, 4);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Perform action here.
  }
}

void Task_control(void *pvParameters)
{
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 5;

  xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {

    Raw_SensorValue_vr1 = lpf(alpha, analogRead(vr1_pin) , Raw_SensorValue_vr1);
    Raw_SensorValue_vr2 = lpf(alpha, analogRead(vr2_pin) , Raw_SensorValue_vr2);
    Raw_SensorValue_vr3 = lpf(alpha, analogRead(vr3_pin) , Raw_SensorValue_vr3);
    Raw_SensorValue_vr4 = lpf(alpha, analogRead(vr4_pin) , Raw_SensorValue_vr4);

    Set_point = Map(Raw_SensorValue_vr1, 0, 4095, 0, 5000);
    Kp = Map(Raw_SensorValue_vr2, 0, 4095, 0, 1);
    Ki = Map(Raw_SensorValue_vr3, 0, 4095, 0, 0.01);
    Kd = Map(Raw_SensorValue_vr4, 0, 4095, 0, 0.01);

    error = Set_point - Speed;
    sum_error = sum_error + (error * 0.005);
    de_error = (error - prev_error) / 0.005;

    if (sum_error > 4095)
      sum_error = 4095;
    if (sum_error < 0)
      sum_error = 0;

    PID_output = (error * Kp) + (sum_error * Ki) + (de_error * Kd);
    PID_output = PID_output * (4095 / 12.00f); // caculation to Volt


    if (PID_output > 0)
    {
      ledcWrite(1, 0);                          // 0-4095
      ledcWrite(0, constrain(PID_output, 0, 4095)); // 0-4095
    }
    else
    {
      ledcWrite(0, 0);                                // 0-4095
      ledcWrite(1, constrain(fabs(PID_output), 0, 4095)); // 0-4095
    }
    prev_error = error;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void Graph()
{
  x_new[pointer] = Speed_plot;
  x_new_1[pointer] = Set_point_plot;
  for (int x = 7; x < 222; x++)
  {
    int x_tmp = x + pointer;
    if (x_tmp >= 223)
      x_tmp -= 223;

    int x_tmp2 = x + pointer + 1;
    if (x_tmp2 >= 223)
      x_tmp2 -= 223;

    if (x_new[x_tmp2] != 0 && x_new[x_tmp2 + 1] != 0)

    {
      tft.drawLine(x, x_new_1[x_tmp], x + 1, x_new_1[x_tmp + 1], ILI9341_BLACK);

      tft.drawLine(x, x_new_1[x_tmp2], x + 1, x_new_1[x_tmp2 + 1], ILI9341_RED);

      tft.drawLine(x, x_new[x_tmp], x + 1, x_new[x_tmp + 1], ILI9341_BLACK);

      tft.drawLine(x, x_new[x_tmp2], x + 1, x_new[x_tmp2 + 1], ILI9341_ORANGE);
    }
  }

  pointer++;
  if (pointer > 223)
    pointer = 0;
}

void Taxt()
{

  tft.setCursor(180, 0);                        // Set position (x,y)
  tft.setTextColor(ILI9341_RED, ILI9341_BLACK); // Set color of text. First is the color of text and after is color of background
  tft.setTextSize(2);                           // Set text size. Goes from 0 (the smallest) to 20 (very big)
  tft.print("SP=");
  tft.println(Set_point,2);

  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("Kp=");
  tft.println(Kp, 6);

  tft.setCursor(0, 20);
  tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("Ki=");
  tft.println(Ki, 6);

  tft.setCursor(0, 40);
  tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("Kd=");
  tft.println(Kd, 6);

  tft.setCursor(180, 35);
  tft.setTextColor(ILI9341_ORANGE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("RPM=");
  tft.println(Speed,2);

  tft.setCursor(235, 220);
  tft.setTextColor(ILI9341_ORANGE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("T=");
  tft.println(time_now / 1000);
}

void drawGraph()
{
  tft.drawLine(originx, originy, (originx + sizex), originy, ILI9341_WHITE);
  tft.drawLine(originx, originy, originx, (originy - sizey), ILI9341_WHITE);
}
