#include <Arduino.h>
#include <NetworkUdp.h>
#include <WiFiManager.h> 
#include <Stepper.h>  // include the stepper library
#include <WebServer.h>  // include the WebServer library to be able to handle web requests

#include "WiFiOTAHelper.h"
#include "Definitions.h" // Definitions header file
#include "ESPNOW_Func.h"



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


//uint8_t RemoteAddress[]={0x68, 0xB6, 0xB3, 0x08, 0xF1, 0xFE}; //MAC address of Chris Remote, Production
uint8_t RemoteAddress[6]={0x68, 0xB6, 0xB3, 0x08, 0xDE, 0x1E}; //MAC address of Chris Remote, Test



// initialize the stepper library
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

// Create an instance of the web server on port 80
WebServer server(80);

WiFiManager wifiManager;  //start wifiManager

String ESPHostName="Chris_Controller";


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
 * @brief updates the browser with the MAC address
 * 
 */
void handle_MAC(){
  String macAddress = WiFi.macAddress();
  server.send(200, "text/plain", macAddress); // Send the MAC address as a response
}



void setup() 
{
  Serial.begin(115200);
  prepareLittleFS(FORMAT_LITTLEFS_IF_FAILED);
  getFileData(); // Get the system mode from the file system

  if(OTAMode)
  {
    initWiFi(ESPHostName, wifiManager);
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
  
}