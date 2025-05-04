#ifndef DEFINITIONS_H
#define DEFINITIONS_H
#include <Arduino.h>

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

extern const int stepsPerRevolution;  //number of steps per revolution
extern uint16_t StepValue;
extern int16_t upStep;
extern int16_t downStep;
extern uint16_t CurrentStep;
extern uint32_t LastOTAPress; // Last time the OTA button was pressed
extern uint32_t OTAPressInterval; // Interval to check for OTA button press
extern bool DeliverySuccess; // Flag to check if the data was delivered successfully
extern bool NewData; //flag to check if we've recieved a new data command
extern volatile bool OTAMode; // Flag to check if OTA mode is activated
extern volatile bool updateOTA; // Flag to check if OTA update is needed

struct DataStruct
{
  uint8_t cmdESP_Now;  //Command for ESP-NOW
  uint16_t potADC; //ADC Value
};

extern DataStruct ControllerData; //Data structure to hold data from the controller

#endif // DEFINITIONS_H