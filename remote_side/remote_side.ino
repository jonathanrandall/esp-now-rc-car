//For sketch to find mac address see:
//https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/
//

#include <esp_now.h>
#include <WiFi.h>

const int ypin = 35;
const int xpin = 34;
const int speedpin = 32;
const int breakbutton = 15;
unsigned long t_next_send = 230; //send every 230 ms(?)
unsigned long t_last_send;


uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0x99, 0xCE, 0x5C};
// variable for storing the potentiometer value
int potValue = 0;

typedef struct stuct_message {
  uint16_t t1;
  uint16_t t2; //int
//  uint16_t t3=0;
  uint16_t speedval;
  uint16_t breakval;
//  uint16_t a2;
} struct_message;


struct_message control_data;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
void setup() {
  Serial.begin(115200);
  
  pinMode(breakbutton, INPUT_PULLUP);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  t_last_send= millis()-t_next_send;
  
}

int i=0;
int j=0;

void loop() {
  // Reading potentiometer value
  Serial.println(millis());
  control_data.t1 = 0x0000 | analogRead(xpin);
//   control_data.t1 = j;
  delay(20);
  Serial.print("xpin = ");
  Serial.println(control_data.t1);
  control_data.t2 = 0x0000 | analogRead(ypin);
  if (control_data.t2>1900) control_data.t2+=120;
  delay(20);
  Serial.print("ypin = ");
  Serial.println(control_data.t2);
  control_data.speedval =  0x0000 | analogRead(speedpin);
  Serial.print("speedpin = ");
  Serial.println(control_data.speedval);
  control_data.breakval = 0x0000 | digitalRead(breakbutton);
  Serial.print("breakpin = ");
  Serial.println(control_data.breakval);
  // Send message via ESP-NOW
  if((millis()-t_last_send>t_next_send)|| control_data.breakval==0){
    //if want robot to break, send straight away
    t_last_send = millis();
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &control_data, sizeof(control_data));
     
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
  }
  delay(20);
}
