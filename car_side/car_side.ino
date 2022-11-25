//pwma white
//vcc orange
//pwmb yellow
//AN2 blue
//AN1 green
//BN1 purple
//BN2 brown

#include <esp_now.h>
#include <WiFi.h>

#include <Adafruit_NeoPixel.h>
#ifndef PSTR
 #define PSTR // Make Arduino Due happy
#endif

#define PIN        12
#define NUMPIXELS 2

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

unsigned long t_last_com; //time since last communication

int hsv_colour_r;
int hsv_colour_l;
bool first_update = true;

int pwmA_pin = 16; //white
int pwmB_pin = 17; //yellow
int max_spd = 255;
int min_spd = 60;

int rmI1 = 25; //green
int rmI2 = 26; //blue
int lmI1 = 27; // H out1 = H | L out1 = L | H short break | L stop purple
int lmI2 = 33; // L out2 = L | H out2 = H | H short break | L stop brown
int STDBY = 34; //Low is off (current save) //not connected so far

// setting PWM properties
const int freq = 2000;
const int ledChannel_l = 2;
const int ledChannel_r = 4;
const int resolution = 8;

//previous values to check if we need to update.
int rs_prev;
int ls_prev;
int dl_prev;
int dr_prev;

typedef struct stuct_message {

//  int vals[5];
//  uint8_t a1;
  uint16_t t1;
  uint16_t t2; //int
//  uint16_t t3;
  uint16_t speedval;
  uint16_t breakval;
//  int a2;
} struct_message;

struct_message control_data;

void robot_stop(){
  digitalWrite(rmI1,LOW);
  digitalWrite(rmI2,LOW);
  digitalWrite(lmI1,LOW);
  digitalWrite(lmI2,LOW);
}

void robot_setup(){
  pinMode(rmI1,OUTPUT);
  pinMode(rmI2,OUTPUT);
  pinMode(lmI1,OUTPUT);
  pinMode(lmI2,OUTPUT);
  
  ledcSetup(ledChannel_l, freq, resolution);
  ledcSetup(ledChannel_r, freq, resolution);

  ledcAttachPin(pwmA_pin, ledChannel_l);
  ledcAttachPin(pwmB_pin, ledChannel_r);


}

void left_motor_drive(int spd, int dir){
//  Serial.print("in left motor drive ");
//  Serial.println(spd);
  ls_prev = spd; dl_prev = dir;
  if(dir>0){
    digitalWrite(lmI1, HIGH);
    digitalWrite(lmI2, LOW);
  } else {
    digitalWrite(lmI2, HIGH);
    digitalWrite(lmI1, LOW);
   }

  ledcWrite(ledChannel_l, spd);
}

void right_motor_drive(int spd, int dir){
  rs_prev = spd; dr_prev = dir;
 if(dir>0){
  digitalWrite(rmI1, HIGH);
  digitalWrite(rmI2, LOW);
 } else {
  digitalWrite(rmI2, HIGH);
  digitalWrite(rmI1, LOW);
 }

 ledcWrite(ledChannel_r, spd);
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  Serial.println("incoming");
  memcpy(&control_data, incomingData, sizeof(control_data));
  Serial.println(sizeof(control_data));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("xpin = ");
  Serial.println(control_data.t1);
  Serial.print("ypin = ");
  Serial.println(control_data.t2);
  Serial.print("speedval = ");
  Serial.println(control_data.speedval);
  Serial.print("breakval = ");
  Serial.println(control_data.breakval);
  t_last_com  = millis();
  if (control_data.breakval==0) {
    robot_stop();
  } else { //change speed
    update_robot();
  }
} //void on

void update_robot(){
  int dir;
  int fwd_spd;
  int right_spd;
  int left_spd;
  int dir_lft;
  int dir_rgt;
  
  if(control_data.t2>1850){ //bias a little bit forward
    //forward
    dir = 1;
    fwd_spd = map(control_data.t2, 2000, 4095, 0,max_spd+10);
  } else {
    dir = -1;
    fwd_spd = map(control_data.t2, 2000, 0, 0,max_spd+10);    
  }

  dir_lft = dir;
  dir_rgt = dir;
  
  if(control_data.t1>1960){
    //left
    right_spd = fwd_spd + map(control_data.t1, 2000, 4095, 0,max_spd+10);
    left_spd = fwd_spd - map(control_data.t1, 2000, 4095, 0,max_spd+10);
    
  } else {
    right_spd = fwd_spd - map(control_data.t1, 2000, 0, 0,max_spd+10);
    left_spd = fwd_spd + map(control_data.t1, 2000, 0, 0,max_spd+10);
  }
  if(right_spd < 0){
    right_spd = -right_spd;
    dir_rgt = -dir;
  }
  if(left_spd < 0){
    left_spd = -left_spd;
    dir_lft = -dir;
  }
  if(right_spd > max_spd) right_spd = max_spd;
  if(left_spd > max_spd) left_spd = max_spd;

  //set colour according to speed
  hsv_colour_r = map(dir_rgt*right_spd, -255,255,3000,32000);
  Serial.println(hsv_colour_r);
  pixels.setPixelColor(0, pixels.gamma32(pixels.ColorHSV((uint32_t ) hsv_colour_r,255,max_spd)));
  
  hsv_colour_l = map(dir_lft*left_spd, -255,255,35000,62000);
  pixels.setPixelColor(1, pixels.gamma32(pixels.ColorHSV((uint32_t ) hsv_colour_l,255,max_spd)));
  pixels.show();
  //we need to stop the motor if speed less that certain value.
  if (right_spd < min_spd && left_spd < min_spd){
    robot_stop();
  } else {
    if(first_update){
      right_motor_drive(right_spd, dir_rgt);
      left_motor_drive(left_spd, dir_lft);
      first_update = false;
    } else {
      left_motor_drive(left_spd, dir_lft);//dir_lft);
      right_motor_drive(right_spd, dir_rgt);
//      if(abs(right_spd - rs_prev)>10 || dr_prev != dir_rgt) {
//        right_motor_drive(right_spd, dir_rgt);
//      }
//      if(abs(left_spd - ls_prev) > 10 || dl_prev != dir_lft){
//         left_motor_drive(left_spd, dir_lft);
//      }
    }  
  }
} //update robot

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  robot_setup(); 
//  left_motor_drive(255, 1);
//  delay(200);
  robot_stop();
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
  t_last_com  = millis();
}
 
void loop() {
  
  if((millis()-t_last_com) > 500){
    //if more than 500 ms since last communication stop robot.
    //this will make sure robot doesn't run away if communication signal is broken.
    robot_stop(); 
  }
}
