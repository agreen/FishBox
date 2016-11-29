extern "C" {
#include "pwm.h"
}

#include <OneWire.h>
#include <DallasTemperature.h>

#include <PID_v1.h>

#include "elapsedMillis.h"

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 12 // Lower resolution

///////////////// ATO RELATED /////////////////
#define WATER_LEVEL_PIN  3
#define WATER_COUNT_CRITICAL  10

#define PUMP_PIN  10
#define ATO_TIMEOUT  30000L // 10 seconds

bool water_low = false;
int water_level = 0;
int water_low_count = 0;
elapsedMillis water_level_checked;

bool ato_disabled = false;
bool ato_error = false;
bool ato_running = false;

elapsedMillis ato_error_reported;
elapsedMillis ato_started;
elapsedMillis ato_cooldown;


char printlnString[255];

//int kickCutoff = 75;       // Cutoff* pwm value to apply kickstart
//int kickDuration = 50;     // Duration (in ms) to apply kickstart




int set_pump(int mode)
{
  switch (mode) {
    case 0:
      digitalWrite(PUMP_PIN, LOW);
      Particle.println("Switching Pump: On");
      break;
    case 1:
      digitalWrite(PUMP_PIN, HIGH);
      Particle.println("Switching Pump: Off");
      break;
  }
}




int set_pump_particle(String command) {
  if (command == "on") {
    set_pump(HIGH);
  }
  else if (command == "off")
  {
    set_pump(LOW);
  }
  return 1;
}



double tempF = 0;


//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double scaledOutput;

//Define the aggressive and conservative Tuning Parameters
double aggKp = 10, aggKi = 2, aggKd = 1;
double consKp = 1, consKi = 0.05, consKd = 0.25;

//Specify the links and initial tuning parameters
PID PID_1(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature dst(&oneWire);


int fan_duty = 0;
int fan_1_duty = 0;
int fan_change_amount = 5;         // how many points to fade the LED by
const uint32_t period = 125; // * 200ns ^= 25kHz

unsigned long ul_PreviousMillis = 0UL;
unsigned long ul_Interval = 10000UL;


int set_fan_1_duty_particle(String arg) {
  fan_1_duty = arg.toInt();
//  Particle.println("Setting fan_1_duty to \"" + arg + "\"");
  pwm_set_duty(fan_1_duty, 0); //GPIO14
  pwm_start();

  return 1;
}

void setup_pwm()
{
  uint32 pwm_duty_init[3] = {0, 0, 0}; //initial duty cycle values 0-1000
  uint32 io_info[3][3] = {
    {PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO14, 14},
    {PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13, 13},
    {PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12, 12}
  };
  //pinMode(12, OUTPUT);
  //pinMode(13, OUTPUT);
  //pinMode(14, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);

  pwm_init(period, pwm_duty_init, 3, io_info);
  pwm_start();
}

void setup_particle()
{
  bool status;
  Particle.function("fan1_duty", set_fan_1_duty_particle);
  Particle.function("pump", set_pump_particle);
  
  Particle.variable("water_level", &water_level, particle_core::INT);
  Particle.variable("temp", &tempF, particle_core::DOUBLE);
  Particle.variable("fan_1_duty", &fan_1_duty, particle_core::INT);

  Particle.publish("log","FishBox 1.0");

}

void setup_temp()
{
  // Start up the temp library
  dst.begin();
}

void setup_PID()
{
  //initialize the variables we're linked to
  Input = 78; // Bogus, but want PID to start off in the "off" position
  Setpoint = 78;

  PID_1.SetControllerDirection(REVERSE);
  PID_1.SetOutputLimits(0, 100);

  //turn the PID on
  PID_1.SetMode(AUTOMATIC);
  PID_1.SetTunings(aggKp, aggKi, aggKd);
}

void setup_ATO()
{
  pinMode(WATER_LEVEL_PIN, INPUT_PULLUP);
  pinMode(PUMP_PIN, OUTPUT);

}

void setup()
{
  setup_pwm();
  setup_ATO();

  setup_particle();
  
  ul_PreviousMillis = millis();
}

/**********************************************************************************************************/


void ramp_fan()
{
  unsigned long ul_CurrentMillis = millis();
  if (ul_CurrentMillis - ul_PreviousMillis > ul_Interval)
  {
    ul_PreviousMillis = ul_CurrentMillis;

    fan_duty = fan_duty + fan_change_amount;

    if (fan_duty == 0 || fan_duty >= period) {
      //fadeAmount = -fadeAmount ;
      fan_duty = 0;
    }
    pwm_set_duty(fan_duty, 0); //GPIO14
    pwm_set_duty(fan_duty, 1); //GPIO13
    pwm_set_duty(fan_duty, 2); //GPIO12
    pwm_start();
  }
}

void check_water_level()
{
  water_level = digitalRead(WATER_LEVEL_PIN);
  
  if (water_level_checked >= 10000L)
  {
    if (water_level == LOW)
    {
      water_low_count++;
      if (water_low_count >= WATER_COUNT_CRITICAL)
      {
        water_low = true;
        if (!ato_running) 
        {
          Particle.publish("log", "Water level is critically low.");
        }
      }
      else
      {
        sprintf(printlnString, "Water level low, count: %d", water_low_count);
        Particle.publish("log", printlnString);
      }
    }
    else
    {
      if (water_low_count > 0)
      {
        Particle.publish("log", "Water level reset");
      }
      water_low_count = 0;
      water_low = false;
    }
    water_level_checked = 0;
  }
}

void ato_run()
{
// Check if we ARE running
// Check if we need to stop and stop
// else
// Check if we need to run
// Check if we CAN run
// Run

  if (ato_running)
  {
    if (water_low && ato_started >= ATO_TIMEOUT)
    {
      ato_error = true;
      ato_started = 0;
      set_pump(LOW);
      ato_running = false;
      Particle.publish("log","*** ERROR: ATO timeout");
    }
    else if (!water_low || water_level == HIGH)
    {
      set_pump(LOW);
      ato_running = false;
      water_low = false;
      sprintf(printlnString, "ATO Stopped (completed) in %d milliseconds", (long)ato_started);
      Particle.publish("log", printlnString);
    }
  }
  else 
  {
    if (water_low)
    {
      if (ato_disabled || ato_error) {
        if (ato_error_reported > (120 * 1000L))
        {
          Particle.publish("log", "ATO Required, but disabled or error.");
        }
        return;
      }
      Particle.publish("log", "ATO Starting...");
      set_pump(HIGH);
      ato_started = 0;
      ato_running = true;
    }
  }
/*  
  if (ato_disabled == false)
  {
    if (ato_error == false)
    {
      if (water_low == true)
      {
        if (ato_running == false)
        {
          ato_started = 0;
          ato_running = true;
          set_pump(HIGH);
          Particle.publish("log", "Starting ATO.");
        }
        else
        {
          if (ato_started >= ATO_TIMEOUT)
          {
            ato_error = true;
            water_low = false;
            ato_started = 0;
            set_pump(LOW);
            Particle.publish("log","*** ERROR: ATO timeout.");
          }
          else if (water_level == LOW)
          {
            ato_started = 0;
            water_low = false;
            ato_running = false;
            Particle.publish("log","ATO stopped (1).");
            set_pump(LOW);
          }
        }
      }
      else if (ato_running == true)
      {
        ato_started = 0;
        set_pump(LOW);
        ato_running = false;
        Particle.publish("log","ATO stopped (2)");
      }
    }
    else
    {
      Particle.publish("log","*** ATO Required, but disabled due to error.");
    }
  }
*/
}

void other_stuff()
{
  //water_level = digitalRead(inPin);
  //  digitalWrite(10,HIGH);


  // dst.requestTemperatures();
  //tempF = dst.getTempFByIndex(0);

  //Input = tempF;

  //double gap = abs(Setpoint - Input); //distance away from setpoint

  //if(gap<2)
  //{  //we're close to setpoint, use conservative tuning parameters
  //    myPID.SetTunings(consKp, consKi, consKd);
  //    Serial.println("Using Conservative settings");
  //  }
  //  else
  //  {
  //we're far from setpoint, use aggressive tuning parameters
  //myPID.SetTunings(aggKp, aggKi, aggKd);
  //    Serial.println("Using Aggressive settings");
  //  }

  //  myPID.Compute();
  // Serial.print("Setting PWM value to: ");
  // Serial.println(Output);

  //  if (Output >= 1) {
  //    scaledOutput = fscale(1, 100, 80, 255, Output, 0);
  //    pwmWrite(led, scaledOutput);
  //  }
  //  else {
  //    scaledOutput = 0;
  //    pwmWrite(led, 0);
  //  }

  // if (scaledOutput > 80) {
  // Kick power to get motor started
  //  if ( scaledOutput < kickCutoff) {
  //    pwmWrite(led, 255);
  //    delay(kickDuration);
  //  }
  //use this functions instead of analogWrite on 'initialized' pins
  //  pwmWrite(led, scaledOutput);
  //analogWrite(3,Output);
  // }
  // else
  // {
  //  pwmWrite(led, 0);
  // }
}


void loop()
{
  //ramp_fan();
  check_water_level();
  ato_run();
  delay(0);
}





