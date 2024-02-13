#include <esp_now.h>
#include <WiFi.h>

#include <ArduinoJson.h>
#include "Ticker.h"
#include <ESP32Encoder.h>




/* 
 REPLACE WITH MOTION CONTROLLER MAC Address.
 The serial console prints "WiFi MAC: XX:XX:XX:XX:XX:XX" on startup
*/

// 24:D7:EB:B7:A7:2C
uint8_t remotePeerAddress[] = {0x24, 0xD7, 0xEB, 0xB7, 0xA7, 0x2C}; // TODO: Implement broadcast with security


typedef struct espnow_add_peer_msg {
  byte mac_adddress[6];
} espnow_add_peer_msg;

// Structure example to send data
// Must match the receiver structure
typedef struct espnow_message {
  char a[32];
  int b;
  float c;
  bool d;
} espnow_message;

typedef struct espnow_message_mpg {
  char a[32];
  int b;
  float c;
  bool d;
  int mpg1;
} espnow_message_mpg;

espnow_message_mpg mpgData;

// Feedback packet (with max 6 axis) that is sent from motion controller
struct fbPacket {
    uint8_t control;
    uint8_t io;
    int32_t pos[6];
    int32_t vel[6];
    uint32_t udp_seq_num;
};


espnow_message myData; // Create a espnow_message struct called myData
espnow_add_peer_msg espnowAddPeerMsg; // Used to send local mac address to motion controller. Gets added to its esp-now peer list enabling bi-directional communication
esp_now_peer_info_t peerInfo;

fbPacket fb = { 0 };

long lastPacketRxTimeMs = 0;
bool runLoops = false;
bool espnow_peer_configured = false;

// callback when esp-now data is sent including status
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("ESP-NOW: Unable to send data to peer. Resetting configured flag");
    espnow_peer_configured = false;
  }
}

// callback function that will be executed when new esp-now data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
    lastPacketRxTimeMs = millis();
    //Serial.printf("ESP-NOW RX: %d\r\n", len);
    espnow_peer_configured = true;
    if (len == sizeof(myData)) { // if rx example data struct
      memcpy(&myData, incomingData, sizeof(myData));

    } else if (len == sizeof(fb)) { // if rx feedback packet struct. Print it for testing
      memcpy(&fb, incomingData, sizeof(fb));
      Serial.printf("FB: Control: %d, IO: %d, UDPSeq: %d\r\n", fb.control, fb.io, fb.udp_seq_num);
      Serial.printf("FB: POS0: %d, POS1: %d, POS2: %d\r\n", fb.pos[0], fb.pos[1], fb.pos[2]);
      Serial.printf("FB: VEL0: %d, VEL1: %d, VEL2: %d\r\n", fb.vel[0], fb.vel[1], fb.vel[2]);
    }
}

void loop_Core0_EspNowSenderTask(void* parameter) 
{
    Serial.printf("loop_Core0_EspNowSenderTask running...\r\n");

    while (runLoops) {
      
      if (!espnow_peer_configured) { // Register peer. Send the local wifi mac to the remote motion controller
          
          memcpy(peerInfo.peer_addr, remotePeerAddress, 6);
          peerInfo.channel = 0;  
          peerInfo.encrypt = false;
          
          esp_now_del_peer(peerInfo.peer_addr); // clear old peers before adding new one
          
          // Add peer
          if (esp_now_add_peer(&peerInfo) != ESP_OK){
            Serial.println("Failed to add peer. Retry in 5s...");
            continue;
          }
          
          WiFi.macAddress(espnowAddPeerMsg.mac_adddress); // Get local wifi mac and store in esp-now peer struct

          Serial.printf("ESP-NOW: Sending MAC: '%02X:%02X:%02X:%02X:%02X:%02X' to peer...\r\n", 
            espnowAddPeerMsg.mac_adddress[0],espnowAddPeerMsg.mac_adddress[1],espnowAddPeerMsg.mac_adddress[2],
            espnowAddPeerMsg.mac_adddress[3],espnowAddPeerMsg.mac_adddress[4],espnowAddPeerMsg.mac_adddress[5],
            espnowAddPeerMsg.mac_adddress[6]);

          esp_err_t result = esp_now_send(remotePeerAddress, (uint8_t*) &espnowAddPeerMsg, sizeof(espnowAddPeerMsg)); // send esp-now addPeerMsg

          if (result == ESP_OK) {
            Serial.println("Successfully sent an addPeerMessage to remote node");
            espnow_peer_configured = true;
          }
          else {
            Serial.println("ERROR: Could not send an addPeerMessage to remote node. Retry in 5s...");
            espnow_peer_configured = false;
          }
        
        } else { // esp-now peer is configured - send test data struct every 5s. Check if last rx packet >3s then mark as stale connection and retry adding peer every 5s
          long now_LastRxPktTime = millis();
          if (now_LastRxPktTime - lastPacketRxTimeMs > 3000) {
              Serial.println("Timeout expired receiving message from peer. Reconnecting...");
              espnow_peer_configured = false;
          } else {
            esp_err_t result = esp_now_send(remotePeerAddress, (uint8_t*) &myData, sizeof(myData));
            if (result != ESP_OK) {
              Serial.println("ERROR: Could not send an ping to remote node. Retry in 5s...");
              espnow_peer_configured = false;
            }
          }
        }
        
        vTaskDelay(5000 / portTICK_PERIOD_MS); // Task Sleep 5s
    }
    
    Serial.printf("Exiting loop_Core0_EspNowSenderTask\r\n");
    vTaskDelete(NULL);
}

// Encoder stuff

Ticker sendTimer;
Ticker encoderTimer;

ESP32Encoder encoder;
int lastCnt = 0;

void sendMPG(){
  esp_err_t result = esp_now_send(remotePeerAddress, (uint8_t*) &mpgData, sizeof(mpgData));
  if (result != ESP_OK) {
    Serial.println("ERROR: Could not send MPG update...");
    espnow_peer_configured = false;
  }
}

void checkEncoder(){
  int cnt = encoder.getCount();
  if(cnt != lastCnt){
    Serial.printf("\n\t %i \n",cnt);
    lastCnt = cnt;
    // TODO, fix this, cast 64 bit int to 32
    mpgData.mpg1 = (int)encoder.getCount();
    sendMPG();
  }
}

void setup() {
  // setup encoder
  ESP32Encoder::useInternalWeakPullResistors = puType::UP;
  encoder.attachHalfQuad(32, 25);
  encoderTimer.attach_ms(10,checkEncoder);

  Serial.begin(115200);
  Serial.println("Starting up...");

  // Set device as a Wi-Fi Station and AP Mode (Required for bi-directional ESP-NOW). No SSID is configured
  WiFi.mode(WIFI_MODE_APSTA);
  
  Serial.printf("WiFi MAC: %s\r\n", WiFi.macAddress().c_str());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for TX&RX CBs
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  runLoops = true;

  xTaskCreatePinnedToCore(
        loop_Core0_EspNowSenderTask, // Task function.
        "loop_Core0_EspNowSenderTask", // name of task.
        2048, // Stack size of task
        NULL, // parameter of the task
        1, // priority of the task
        NULL, // Task handle to keep track of created task
        0); // pin task to core 0
  
  Serial.println("Setup complete");

}
 
void loop() {
  // Do whatever you like here
  delay(2000);
}
