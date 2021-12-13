#include <Servo.h>

#define PIN_SERVO 10
#define PIN_IR A0
#define PIN_LED 9

// configurable parameters
#define _DUTY_MIN 553 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1320 // servo neutral position (90 degree)
#define _DUTY_MAX 2399 // servo full counterclockwise position (180 degree)

#define _SERVO_SPEED 100 // servo speed limit (unit: degree/second)
#define INTERVAL 20  // servo update interval

unsigned long last_sampling_time; // unit: ms
float duty_chg_per_interval; // maximum duty difference per interval
float toggle_interval, toggle_interval_cnt;
float pause_time; // unit: sec
Servo myservo;
float duty_target, duty_curr, dist_raw, dist_mod = 255;
float a = 86, b = 450;
float _KP = 0.4,_KD = 100;
float error_prev, error_curr;

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
    dist_mod = dist_cali * 0.7 + dist_mod * 0.3;
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
  myservo.writeMicroseconds(duty_curr);
  
  error_curr = dist_cali;
  float pterm = _KP * (255 - dist_cali);
  float dterm = _KD * (error_curr - error_prev);
  float control = pterm - dterm;
  duty_target = _DUTY_NEU + control;

  if(duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;
  if(duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;

  error_prev = error_curr;

  
  Serial.print("dist_ir:");
  Serial.print(raw_dist);
  Serial.print(",pterm:");
  Serial.print(map(pterm,-1000,1000,510,610));
  Serial.print(",duty_target:");
  Serial.print(map(duty_target,1000,2000,410,510));
  Serial.print(",duty_curr:");
  Serial.print(map(duty_curr,1000,2000,410,510));
  Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  

  /*if(dist_cali > 255) //멀다
    duty_target = _DUTY_NEU - 125;//(_DUTY_MIN + _DUTY_NEU) / 2;
  else //가깝다
    duty_target = _DUTY_NEU + 100;//(_DUTY_MAX + _DUTY_NEU) / 2;
   */

// update last sampling time
  last_sampling_time += INTERVAL;
}
