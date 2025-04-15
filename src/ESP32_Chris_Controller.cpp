#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <NetworkUdp.h>
#include <ArduinoOTA.h>
#include <FS.h>
#include <LittleFS.h>  //File system on ESP32
#include <WiFiManager.h> 
#include <Stepper.h>  // include the stepper library
#include<esp_now.h>
#include <WebServer.h>  // include the WebServer library to be able to handle web requests



/**WARNING*********************************************************
 * *******WARNING**************************************************
 * **************WARNING******************************************
 * Warning:  Before flashing a new ESP32, first upload code with 
 * "FORMAT_LITTLEFS_IF_FAILED" set to "true"  Then reflash with 
 * "FORMAT_LITTLEFS_IF_FAILED" set to "false".  Otherwise the file system
 * will not be formatted, and will not properly store login information.  The
 * ESP32 may then no  longer be able to connect via WiFi.  If a USB is not
 * available, the ESP32 may then become trash!!!
 * 
 */

#define FORMAT_LITTLEFS_IF_FAILED false//only need to format FS the first time

// ULN2003 Motor Driver Pins
#define IN1 18
#define IN2 8
#define IN3 10
#define IN4 11
#define POT 4  //Potentiometer Pin number
#define OTAButton              GPIO_NUM_17     // OTA update 
#define cmdUp 3  //Command from the remoe to go up one step
#define cmdDown 4 //Command from the remote to go down one step
#define cmdGoTo 5 //Command from the remote to go to a specific value
#define cmdStatus 6  //command to send status to Remote
#define MaxADC 3500 //maximum stepper motor ADC value
#define MinADC 500  //minimum stepper motor ADC value

//const uint32_t Interval1 = 10000; // Interval for the first stepper movement
//const uint32_t Interval2 = 7500; // Interval for the second stepper movement
//uint32_t previousMillis = 0; // Store the last time function 1 was executed
//bool function1Executed = false; // Flag to check if function 1 has been executed
const int stepsPerRevolution = 2048;  //number of steps per revolution
uint16_t StepValue = 128;
int16_t upStep=-StepValue;
int16_t downStep=-upStep;
uint16_t CurrentStep=0;
uint32_t LastOTAPress=0; // Last time the OTA button was pressed
uint32_t OTAPressInterval=500; // Interval to check for OTA button press
bool DeliverySuccess=false; // Flag to check if the data was delivered successfully
bool NewData=false; //flag to check if we've recieved a new data command
volatile bool OTAMode=false; // Flag to check if OTA mode is activated
volatile bool updateOTA=false; // Flag to check if OTA update is needed
uint8_t RemoteAddress[]={0x68, 0xB6, 0xB3, 0x08, 0xF1, 0xFE}; //MAC address of Chris Remote

struct DataStruct
{
  uint8_t cmdESP_Now;  //Command for ESP-NOW
  uint16_t potADC; //ADC Value
};

DataStruct ControllerData={0,0}; //Data structure to hold data from the controller

// initialize the stepper library
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

// Create an instance of the web server on port 80
WebServer server(80);

WiFiManager wifiManager;  //start wifiManager

//Creates a varaible called peerInfo of the data structury type esp_now_peer_info_t to 
//hold information about the peer
esp_now_peer_info_t peerInfo; 

String ESPHostName="Chris_Controller";

/**
 * @brief start the LittleFS file system.  For first time, set FORMAT_LITTLEFS_IF_FAILED to true
 * to format memory the first time.  Otherwise, set to false.
 * 
 */
void prepareLittleFS()
{
  if(!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)){
    Serial.print("LittleFS mount failed");
    return;
  }
}

/**
 * @brief looks to see if the OTAMode button is pressed 
 * 
 * Note: IRAM_ATTR should be used with ESP32 chips.  This places the interupt routine
 * in RAM.  This is faster than keeping it in flash.  Also, the interrupt may be 
 * missed if flash is being used at the time of interrupt if it's not placed in RAM
 */
void IRAM_ATTR OTAButtonPress()
{
  if(millis()-LastOTAPress>OTAPressInterval) // prevent button bouncing
  {
    LastOTAPress=millis(); // Update the last press time
    OTAMode =!OTAMode;
    updateOTA=true;
  }
}

/**
 * @brief Initializes the WiFi Connection
 * 
 */
void initWiFi()
{
  Serial.println("Booting");
  WiFi.hostname(ESPHostName.c_str());
  WiFi.mode(WIFI_STA);

  wifiManager.setTimeout(120);
  //wifiManager.resetSettings();  //For testing, reset credentials
 

  // Automatically connect using saved credentials,
  // if connection fails, it starts an access point with the specified name ( "Salt Monitor"),
  // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
  // then goes into a blocking loop awaiting configuration and will return success result
 
  bool res;
  // res = wifiManager.autoConnect(); // auto generated AP name from chipid
  res = wifiManager.autoConnect("Chris_Controller"); // anonymous ap
  //res = wifiManager.autoConnect("Feed Monitor"); // anonymous ap
  

if(!res) 
{
  Serial.println("Failed to connect");
  ESP.restart();
} else 
{
  //if you get here you have connected to the WiFi    
  Serial.println("connected... :)");
}   
/*
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
*/
}

/**
 * @brief Initializes ArduinoOTA, Keep this code to allow WiFi updates
 *   
 * // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32"); note use WiFi.HostName() in initWiFi instead

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
 */
void initOTA()
{

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }
      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    });
    

  ArduinoOTA.begin();


  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.getHostname());

}

/**
 * @brief Get the Sys Mode object from the file system.  If the file does not exist, write Mode_Normal to the file system.
 * 
 */
void getFileData()
{
  //Read SysMode from Flash, if file does not exist, write Mode_Normal.
  if(LittleFS.exists("/OTAdata.txt"))
  {
    File myFile=LittleFS.open("/OTAdata.txt",FILE_READ);
    myFile.read((byte *)&OTAMode, sizeof(OTAMode));
    myFile.close();
    char buffer[20];
    sprintf(buffer, "OTA Mode = %d", OTAMode); // Convert the integer to a string
    Serial.println(buffer);
  } else {
    File myFile=LittleFS.open("/OTAdata.txt",FILE_WRITE);
    myFile.write((byte *)&OTAMode, sizeof(OTAMode));
    myFile.close();
  }

}

/**
 * @brief Function to read ADC value from GPIO4
 * 
 * @return int 
 */
int readADC() {
  uint32_t adcValue = 0;
  for(int i = 0; i < 75; i++){
    adcValue = adcValue + analogRead(POT);
    //delay(5);
  }
  adcValue = adcValue / 75;
  return adcValue;
}

int readADCVolts()
{
uint32_t adcVoltsIn = 0;
for (int i = 0; i < 75; i++)
{
  adcVoltsIn = adcVoltsIn + analogReadMilliVolts(POT);
  //delay(5);
}
adcVoltsIn= adcVoltsIn/75;
return adcVoltsIn;
}

void StepperAllStop()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW); 
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

/**
 * @brief Function to handle requests to the "/ADC" endpoint
 * 
 */
void handle_ADC() {
  int adcValue = readADC(); // Get the ADC value
  int adcVolts= readADCVolts();

  String response = "ADC Value: " + String(adcValue) + "\n";
  response += "ADC Volts: " + String(adcVolts) + " mV\n";
  response += "OTAMode: " + String(OTAMode ? "true" : "false") + "\n";
  server.send(200, "text/plain", response); // Send response
}

/**
 * @brief Fucntion called wihen new data is sent. This function simply prints 
 * if the message was successfully delivered or not. If the message is delivered
 *  successfully, the status variable returns 0, so we can set our success message 
 * to “Delivery Success”:
 * 
 * @param mac_addr 
 * @param status //from the esp_now_send_status_t data type structure
 */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
   if (status==ESP_NOW_SEND_SUCCESS){DeliverySuccess=true;}  else {DeliverySuccess=false;}
 }
 
 /**
  * @brief The OnDataRecv() function will be called when a new packet arrives.
  * 
  * @param mac 
  * @param incomingData 
  * @param len Length of incoming data, Can only have a max of 250 bytes so use integer
  */
 void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
   memcpy(&ControllerData, incomingData, sizeof(ControllerData));
  NewData=true;
 }
 
/**
 * @brief Send  data to the Chris Remote 
 *
 * @param OutData data to send.  
 */
void SendData(DataStruct OutData)
{
    //Send Data
    esp_err_t result = esp_now_send(RemoteAddress, (uint8_t *) &OutData, sizeof(OutData) );
/*
    if (result==ESP_OK){
      //test code
      char buffer[100];
      sprintf(buffer,"Sent with Success, Outdata = %d",OutData);
      Serial.println(buffer);
    }
    else {
      Serial.println("Error sending the data");
    }
*/
}

/**
 * @brief updates the browser with the MAC address
 * 
 */
void handle_MAC(){
  String macAddress = WiFi.macAddress();
  server.send(200, "text/plain", macAddress); // Send the MAC address as a response
}

/**
 * @brief Initializes the ESP_NOW network
 * 
 */
void initESP_NOW()
{
 
  WiFi.mode(WIFI_STA);//Set the device as a WiFi Station
  esp_now_init();  //initialize ESP-NOW
  esp_now_register_send_cb(OnDataSent); //register for Send Call back to get status of transmitted packet
  esp_now_register_recv_cb(OnDataRecv); //register call back function for when data is recieved
  memcpy(peerInfo.peer_addr,RemoteAddress,sizeof(RemoteAddress));
  peerInfo.channel=0;
  peerInfo.encrypt=false;
  esp_now_add_peer(&peerInfo);  //Add peer
}

void setup() 
{
  Serial.begin(115200);
  prepareLittleFS();
  getFileData(); // Get the system mode from the file system

  if(OTAMode)
  {
    initWiFi();
    initOTA();
    // Define the route for the "/ADC" endpoint
    server.on("/ADC", HTTP_GET, handle_ADC); // Use the handle_ADC function
    server.on("/MAC", HTTP_GET, handle_MAC); // Send the MAC address as a response
    // Start the server
    server.begin();
    Serial.println("HTTP server started");
  }else{
    initESP_NOW();
  }

  pinMode(OTAButton, INPUT_PULLUP); // Set OTAButton pin as input
  attachInterrupt(digitalPinToInterrupt(OTAButton), OTAButtonPress, FALLING); // Attach interrupt to OTAButton
    
  // set the speed at 8 rpm
  myStepper.setSpeed(8);

  pinMode(POT, INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_2_5db);
  StepperAllStop();
}

void loop() 
{
  ArduinoOTA.handle();  //handles Over The Air updates
  if(OTAMode) {server.handleClient();} // Handle incoming client requests

  if(updateOTA) // If OTA update is needed
  {
    updateOTA=false; // Reset the flag
    File myFile=LittleFS.open("/OTAdata.txt",FILE_WRITE);
    myFile.write((byte *)&OTAMode, sizeof(OTAMode));
    myFile.close();
    ESP.restart(); // Restart the ESP32 to apply changes
  }

  //Handle ESP_NOW commands:
  if(NewData){ //a new data command has been recieved
    switch (ControllerData.cmdESP_Now)
    {
      case cmdUp:
        myStepper.step(upStep); // Move the stepper motor up
        ControllerData.cmdESP_Now=cmdStatus;
        ControllerData.potADC=readADC();
        SendData(ControllerData);
        break;
      case cmdDown:
        myStepper.step(downStep); // Move the stepper motor down
        ControllerData.cmdESP_Now=cmdStatus;
        ControllerData.potADC=readADC();
        SendData(ControllerData);
        break;
      case cmdGoTo:     
        CurrentStep=readADC();
        if(ControllerData.potADC > MaxADC){ControllerData.potADC=MaxADC;} // Limit the ADC value to the maximum
        if(ControllerData.potADC < MinADC){ControllerData.potADC=MinADC;} // Limit the ADC value to the minimum
        if(ControllerData.potADC > CurrentStep) // Move the stepper motor to the specified position
        {
          while(CurrentStep < ControllerData.potADC)
          {
            myStepper.step(upStep); // Move the stepper motor up
            CurrentStep=readADC();
          }
        } else if(ControllerData.potADC < CurrentStep) // Move the stepper motor to the specified position
        {
          while(CurrentStep > ControllerData.potADC)
          {
            myStepper.step(downStep); // Move the stepper motor down
            CurrentStep=readADC();
          }
        }
        ControllerData.cmdESP_Now=cmdStatus;
        break;
      case cmdStatus:
        StepperAllStop();
        ControllerData.potADC=readADC();
        SendData(ControllerData);
        NewData=false; // Reset the flag  
        break;
      default:
        break;
    }
  }
  
  
 /*
  if (!function1Executed) {
    // Check if it's time to execute function 1
    if (millis() - previousMillis >= Interval1) {
        uint32_t analogValue = readADC();
        while (analogValue < MaxADC)
        {
          if(OTAMode){break;}
          myStepper.step(upStep);
          analogValue = readADC();
        }
        previousMillis = millis(); // Update the last execution time
        function1Executed = true; // Set the flag to indicate function 1 has been executed
        StepperAllStop(); // Stop the stepper motor after function 1 execution
    }
  } else {
    // If function 1 has been executed, check for function 2
    if (millis() - previousMillis >= Interval2) {
      uint32_t analogValue=readADC();
      while (analogValue > MinADC)
      {
        if(OTAMode){break;}
        myStepper.step(downStep);
        analogValue = readADC();
      }
        previousMillis = millis(); // Update the last execution time
        function1Executed = false; // Reset the flag to indicate function 1 can be executed again
        StepperAllStop(); // Stop the stepper motor after function 2 execution
    }

  }

  // Add a small delay to avoid overwhelming the server
  delay(100);
*/
}