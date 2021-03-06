#include <Servo.h>

#define PIN_SERVO 10
#define PIN_IR A0
#define PIN_LED 9

// configurable parameters
#define _DUTY_MIN 553 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1476 // servo neutral position (90 degree)
#define _DUTY_MAX 2399 // servo full counterclockwise position (180 degree)

#define _SERVO_SPEED 100 // servo speed limit (unit: degree/second)
#define INTERVAL 20  // servo update interval

unsigned long last_sampling_time; // unit: ms
float duty_chg_per_interval; // maximum duty difference per interval
float toggle_interval, toggle_interval_cnt;
float pause_time; // unit: sec
Servo myservo;
float duty_target, duty_curr, last_dist = 480;
float a = 86, b = 450;

void setup() {
    // initialize GPIO pins
    myservo.attach(PIN_SERVO); 
    duty_target = duty_curr = _DUTY_NEU;
    //myservo.writeMicroseconds(duty_curr);
    pinMode(PIN_LED,OUTPUT);
    digitalWrite(PIN_LED, 1);
    
    // initialize serial port
    Serial.begin(57600);
  
    // convert angle speed into duty change per interval.
    //duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * INTERVAL / 1000;
    duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * INTERVAL / 1000;
  
    // initialize variables for servo update.
    pause_time = 1;
    toggle_interval = (180.0 / _SERVO_SPEED + pause_time) * 1000 / INTERVAL;
    toggle_interval_cnt = toggle_interval;
    
    // initialize last sampling time
    last_sampling_time = 0;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
    float raw_dist = ir_distance();
    float dist_cali = 150 + 300.0 / (b - a) * (raw_dist - a);
    //if(millis() < last_sampling_time + INTERVAL) return;

  // adjust duty_curr toward duty_target by duty_chg_per_interval
  if(duty_target > duty_curr) {
    duty_curr += duty_chg_per_interval;
    if(duty_curr > duty_target) duty_curr = duty_target;
  }
  else {
    duty_curr -= duty_chg_per_interval;
    if(duty_curr < duty_target) duty_curr = duty_target;
  }
  float d = dist_cali - last_dist;
  myservo.writeMicroseconds(duty_curr);
  
  Serial.print("Min:1000,duty_target:");
  Serial.print(duty_target);
  Serial.print(",duty_curr:");
  Serial.print(duty_curr);
  Serial.print(",Max:2000");
  Serial.print("dist_cali");
  Serial.print(dist_cali);
  Serial.print("d: ");
  Serial.println(d*30);


  duty_target = _DUTY_NEU - (dist_cali - 255 ) - (d * 30);
  /*if(dist_cali > 255) //??????
    duty_target = _DUTY_NEU - 125;//(_DUTY_MIN + _DUTY_NEU) / 2;
  else //?????????
    duty_target = _DUTY_NEU + 100;//(_DUTY_MAX + _DUTY_NEU) / 2;
   */

// update last sampling time
  last_dist = dist_cali;
  last_sampling_time += INTERVAL;
}
