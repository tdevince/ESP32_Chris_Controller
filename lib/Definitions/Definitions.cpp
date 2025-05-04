#include "Definitions.h"
#include <Arduino.h>   



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


DataStruct ControllerData={0,0}; //Data structure to hold data from the controller