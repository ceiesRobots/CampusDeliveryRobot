#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
//#include <esp_int_wdt.h>
//#include <esp_task_wdt.h>

WiFiServer tcpServer(8000);
uint8_t DataPacket[22];
uint8_t CmdPacket[8];


//##################################[IP-address-Configurations]#########################################
//Static IP address configuration
IPAddress local_IP(192, 168, 1, 2);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);


////////////////////////////////////////////////////
// esp8266 IP address : 192.168.1.2               //
// jetson nano IP address : 192.168.1.3           //
// gateway : 192.168.1.1                          //
// subnet mask : 255.255.255.0 (24)               //
////////////////////////////////////////////////////





// ########################## DEFINES ##########################
#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      100         // [-] Maximum speed for testing

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  wheelR_cnt;
   int16_t  wheelL_cnt;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback NewFeedback;

unsigned long iTimeSend = 0;
int iTestMax = SPEED_MAX_TEST;
int iTest = 0;
uint8_t rcv_ctr = 0;
uint8_t snd_ctr = 0;
unsigned long timeNowUART;
unsigned long timePrevUART;
unsigned long timeNowTCP;
unsigned long timePrevTCP;
bool DPready;
#define WDT_TIMEOUT 3




void setup() {
  DPready = false;//Data Packet not ready
  for(int i=0;i<22;i++)
    DataPacket[i] = 0;

  for(int i=0;i<8;i++)
    CmdPacket[i] = 0;  

  Serial.begin(115200);
  initWiFi();
  timeNowUART = millis();
  timePrevUART = timeNowUART;  
  timeNowTCP = timeNowUART;  
  timePrevTCP = timeNowUART;  
  ESP.wdtEnable(WDT_TIMEOUT*1000);

}




void loop(){
    ESP.wdtFeed();                              //Reset Watchdog Timer
    ReceiveUART();
    WiFiClient client = tcpServer.available();
    if (client) {
      timePrevTCP = millis() + 1000;            //Give the new connection a 1 second margin before timeout
      while (client.connected()){
        ESP.wdtFeed();//Reset Watchdog Timer
        ReceiveUART();
        timeNowTCP = millis();
        if(client.available()>7){ 
          client.read(CmdPacket,8);
          SendUART();
          client.write(DataPacket, 22);         //Disabled for testing
          DPready = true;                       //ReceiveUART() can now write new values to this packet
          timePrevTCP = timeNowTCP;
        }
        if(timeNowTCP > timePrevTCP + 1500){    //Timeout if a command is not received at least every 1.5 seconds
           break;
        }
      }
      client.stop();
    }
}




void initWiFi() {
  //Match the following settings with WiFi Router
  const char *ssid = "KAU_Delivery_Robot";
  const char *password = "1234567890";

  // Configures static IP address
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("Failed to configure");
  }
  
  WiFi.begin(ssid, password);
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
  tcpServer.begin();
}





// ################## Send Command Packet to Hoverboard UART ##################
void SendUART() {
    timeNowUART = millis();
    if (timeNowUART > timePrevUART + 100){    //Try this every 100 msec
      if(validCP()){//check the validity of command packet
        Serial.write((uint8_t *) &CmdPacket, sizeof(CmdPacket));
        timePrevUART = timeNowUART;
      }
    }
}





// ################## Check if Command Packet is valid ##################
bool validCP(){
  bool flag = false;
    if((CmdPacket[0] ^ CmdPacket[2] ^ CmdPacket[4]) == CmdPacket[6])
      flag = true;
    else
      flag = false;
      
    if((CmdPacket[1] ^ CmdPacket[3] ^ CmdPacket[5]) == CmdPacket[7])
      flag = true;
    else
      flag = false;
      
  return flag;
}





// ########################## Send a direct command to Hoverboard UART ##########################
void SendCmdUART(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);
  // Write to Serial
  Serial.write((uint8_t *) &Command, sizeof(Command));
}





// ################## Receive Data Packet from Hoverboard UART ##################
void ReceiveUART(){
    if (Serial.available()) {
        incomingByte     = Serial.read();                                   // Read the incoming byte
        bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
    // Copy received data
    if (bufStartFrame == START_FRAME) {                     // Initialize if new data is detected
        p       = (byte *)&NewFeedback;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;  
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
        *p++    = incomingByte; 
        idx++;
    } 
    // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback)) {
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                            ^ NewFeedback.wheelR_cnt ^ NewFeedback.wheelL_cnt ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

        // Check validity of the new data
        if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
            // Copy the new data
            if(DPready){//Don't copy if previous packet was not sent.
              memcpy(&DataPacket, &NewFeedback, sizeof(SerialFeedback));
              DPready = false;
            }
            rcv_ctr = rcv_ctr + 1;
        }
        idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }
    // Update previous states
    incomingBytePrev = incomingByte;
    }
}
