/* Hardware: Arduino Mini Pro 5V/16MHz
 * Connections:
 * - RG-11 Rain Sensor: Digital Pin 2; not used for SMAAWA Models
 * - 4-20mA Depth Sensor: Analog Pin A6
 * - Battery Monitoring: Analog Pin A2
 * - SIM800L Module: TX to D8, RX to D9, 
 * - Power Control on D12
 * - Peripheral Power Control: D6
 * - New Calibration function added Nov 25
 */
 
//#define DEBUG_MODE  // Comment out for production

#include <LowPower.h> // Low-Power library
#include <SoftwareSerial.h> // Serial library
#include <EEPROM.h>
SoftwareSerial Serial2(8,9); //SIM800L Rx=8, Tx=9


#define RG9_Pin 2// RG-11 is connected to digital pin 2
#define POWER (6) //power peripherals
#define POWERSIM (12) //power SIM-BUCK/PCB
#define ANALOG_PIN (A6) //PCB, A6 Arduino MINI PRO 
#define ANALOG_BATT_PIN (A2) //PCB
#define RANGE 5000 // Depth measuring range 5000mm (for water)
#define CURRENT_INIT 4.1// Current @ 0mm (uint: mA)--->modulate to calibrate to Zero (4mA to 4.2mA)
#define DENSITY_WATER 1  // Pure water density normalized to 1
#define DENSITY_BLEACH 1.104  // https://www.alenusa.com/wp-content/uploads/2020/04/2016-cloralen-regular-bleach.pdf
#define DENSITY_GASOLINE 0.74  // Gasoline density
#define PRINT_INTERVAL_MIN 10000  // 10 second
#define SIM_INTERVAL_MIN 20000  // 20 second

#define EEPROM_CAL_ADDR 0
#define EEPROM_MAGIC 0xCAFE  // Magic number to verify valid data

volatile byte DEVICE_STATE;// volatile variable
unsigned long SIM_InitialTime;
int16_t dataVoltage;
float dataCurrent, depth; //unit:mA
float voltage;

const unsigned long StateMachineinterval = 65000; //StateMachineInterval (1 min)
unsigned long timepoint_measure;
unsigned long StateMachinetimer;
unsigned long time_now = 0;
 
//Sensor readings stored in two char array
char BattVal[4]; // "80" = [0,1,2], size of the sting + 1
char DepthVal[8]="00.0"; // "0.99"= [0,1,2,3,4]-->[0,.,9,9,\0];
char rateVal[8]="00.0"; // cm/cycle
char Token[16] = "SMAAWA_001"; //Corredor Verde
bool flag = false;                                   
bool flagINT = true;
int value;
int mode;
int period = 5000;
const char* hologramSMSNumber = "+447937405250"; // TODO: Replace with actual number-->SIM ID: 3136471

volatile bool justWoke = false;
extern volatile unsigned long timer0_millis;  // Access the global millis counter
//volatile int counterIndex = 0;

// ===== Calibration coefficients =====
// Calibration coefficients (stored in SRAM, could use EEPROM for persistence)
struct CalibrationCoeffs {
  float rawLow;
  float rawHigh;
  float refLow;
  float refHigh;
  bool isCalibrated;
} coeffs = {0.0, 5000.0, 0.0, 5000.0,false}; // Default values; all in mm (0-5000mm = 0-5m)

const char* awsServerName = "AT+HTTPPARA=\"URL\",\"http://34.224.5.68/SetURL?deviceID=%s&distance=%s&rate=%s&batt=%s\"";

//"http://34.224.5.68/SetURL?deviceID=SMAAWA_001&distance=50.5&rate=-22.3&batt=90"


// ===== SAVE COEFFICIENTS TO EEPROM =====
void saveCoefficientsToEEPROM() {
  int addr = EEPROM_CAL_ADDR;
  
  // Write magic number first
  EEPROM.put(addr, EEPROM_MAGIC);
  addr += sizeof(uint16_t);
  
  // Write calibration structure
  EEPROM.put(addr, coeffs);
  
  Serial.println(F("✓ Calibration saved to EEPROM"));
}

// ===== LOAD COEFFICIENTS FROM EEPROM =====

void loadCoefficientsFromEEPROM() {
  int addr = EEPROM_CAL_ADDR;
  uint16_t magic;
  
  // Read magic number
  EEPROM.get(addr, magic);
  addr += sizeof(uint16_t);
  
  if (magic == EEPROM_MAGIC) {
    // Valid data exists, load it
    EEPROM.get(addr, coeffs);
    Serial.println(F("✓ Calibration loaded from EEPROM"));
    printCoefficients();
  } else {
    Serial.println(F("⚠ No valid calibration in EEPROM, using defaults"));
    coeffs.isCalibrated = false;
  }
}


// ===== SETUP =====
void setup(){
  DEVICE_STATE = 0;
  Serial.begin(19200);
  Serial2.begin(9600);// SIM8008 baud rate
  Serial.println("SMAAWA with RG-9 Rain Event Detector  | smability.io");
  
  #ifdef DEBUG_MODE
  Serial.println(F("*** DEBUG MODE ENABLED ***"));
  Serial.println(F("Send 'C' for calibration test"));
  #endif

  // **NEW: Load calibration from EEPROM on startup**
  loadCoefficientsFromEEPROM();
  
  pinMode(RG9_Pin, INPUT);
  pinMode(ANALOG_PIN, INPUT);

  pinMode(POWERSIM,OUTPUT);          //SIM800L ON/OFF
  pinMode(POWER,OUTPUT);             //PowerSensor ON/OFF

  StateMachinetimer = 0;             // ✅ Initialize to 0
  
  attachInterrupt(digitalPinToInterrupt(RG9_Pin), rgisr, LOW);
}

// ===== ISR for Rain Mode =====
// Interrrupt handler routine that is triggered when the rg-9 detects rain
void rgisr() {
     DEVICE_STATE = 1;
}
// end of rg-9 rain detection interrupt handler 

// ===== MAIN LOOP =====
void loop(){
  DEVICE_STATES();
}

// ===== STATE-MACHINE MODES ===== 
void DEVICE_STATES(){
  switch (DEVICE_STATE)
  {
    
    case 0: //Normal Mode
    
     Serial.println(F(" DEVICE_STATE: NORMAL MODE ")); // 65 sec sampling

      if (justWoke) {
        sleep();
        justWoke = false;
      }

      mode = 0;
      
      detachInterrupt (digitalPinToInterrupt (RG9_Pin)); 
      SIM_STATES(mode); //65 sec sampling
      
      break;
   
    case 1: //Rain Mode; if not used a pullup resistor is already added at PCB level
      
      Serial.println(F(" DEVICE_STATE: RAIN MODE "));
      
      mode = 1;
      
      detachInterrupt (digitalPinToInterrupt (RG9_Pin));  //Disable interrupts. Clears the global interrupt flag, in SREG so prevent any form of interrupt occurring
      SIM_STATES(mode);
      attachInterrupt(digitalPinToInterrupt(RG9_Pin), rgisr, LOW); //interrupt is Activated after a switch-casee break. Enables interrupts, sets the bit and switches interrupts on
      
      DEVICE_STATE = 0; //Reset DEVICE_STATE to 0
   
      break;
  }  
}
// ===== MAIN STATE-MACHINE =====
//SIM states
enum SIM
    {
      START,SENSORS,READSENSOR,
      PWRSIM,CHECKSMS,ATTACHGPRS,
      INITHTTP,SENDPARA,
      ENDGPRS,SIMOFF
    };

SIM SIM_state  = START;

void SIM_STATES(int state){
  
  int countloop = 0; // set countloop to 0
  
  switch (SIM_state)
  {
   
    case START:{
    
      Serial.println(F("    STATE 0: START SENSORS    "));
      
      flag = false; 
      powerOFFSIM();
      SIM_state = SENSORS;
        
    } break; //goes to the closing brace (final brace) of switch-case body and continues with the next instruction-the interrupt-
      
    case SENSORS:{
    
      Serial.println(F("    STATE 1: POWER ON SENSORS   "));
      
      if(state == 1){//Rain mode
        powerONSensor();
        SIM_state = READSENSOR;
      }
      else {//Normal mode
        
        //ON Sensors
        powerONSensor();
     
        SIM_state = PWRSIM;
      }
            
    } break;

   case READSENSOR:{
       
      //2, 3, 4 ..sensor measurement. (Read pressure sensor value in rain mode)
      if (flag == true){
        Serial.println(F("    STATE 2: READ SENSOR RAIN  "));
        readsensor();
        SIM_state = SENDPARA;
      }
      else{
         //SIM_state = PWRSIM; -->only if an optic sensor is attached
         //if ( (millis () - StateMachinetimer) >= StateMachineinterval){
          
          Serial.println(F("    STATE 2: READ SENSOR DRY   "));
          
          battery();

          float currentDepth = readsensor();        // Take one reading
          Serial.println(currentDepth);             // Print
          dtostrf(currentDepth, 3, 2, DepthVal);    // Convert
          
          float currentRate = ratesensor(currentDepth);         // pass the value
          Serial.println(currentRate);
          dtostrf(currentRate, 3, 2, rateVal);
         
          StateMachinetimer = millis();

          SIM_state = SENDPARA;
          
         //}
      }
     
   }  break;
      
    case PWRSIM:{
   
      Serial.println(F("    STATE 3: POWER ON SIM800L   "));

      countloop = 0; // rest while loop counter
      
      powerONSIM(); //Power ON SIM800L
      
      SIM_InitialTime = millis(); //Record SIM -ON- time

      battery();

      sendATcommand("AT", "OK", 2000);
  
     
      sendATcommand("AT+CFUN=1", "OK", 2000);

           
      sendATcommand("AT+CREG=1", "OK", 200);  // Activating CREG notifications

      // All 0s means While is TRUE, countloop increments, we need at least one 1 to break the loop
      while ((sendATcommand("AT+CREG?", "+CREG: 1,1", 1000) // before 0,1,
              || sendATcommand("AT+CREG?", "+CREG: 0,5", 1000) 
              || sendATcommand("AT+CREG?", "+CREG: 0,1", 1000)
              || sendATcommand("AT+CREG?", "+CREG: 2,1", 1000)
              || sendATcommand("AT+CREG?", "+CREG: 1,5", 1000)) != 1 )
              
              {countloop++; if(countloop >= 4){break;}};  // If it does not find network think what to do..  

      sendATcommand("AT+CREG=0", "OK", 5000); // Deactivating CREG notifications; -->SIM gets stuck here!

                                  
      if (countloop < 4){ 
          Serial.println(F("Registered to the Network!"));
          SIM_state = CHECKSMS;

          Serial.println(F("Waiting for network SMS push..."));
          delay(10000);
         
          // ✅ Enable SMS for calibration commands
          sendATcommand("AT+CMGF=1", "OK", 2000);
          sendATcommand("AT+CNMI=2,1,0,0,0", "OK", 2000);
          sendATcommand("AT+CPMS=\"SM\",\"SM\",\"SM\"", "OK", 2000);
      
          while (Serial2.available()) {
            Serial2.read();
          }
          Serial.println(F("✓ Serial2 buffer flushed"));  
      } else{
          //SIM_state = SIMOFF;   //otherwise will hang if not registered, we will try again in one hour
          SIM_state = ENDGPRS;
          Serial.println(F("End GPRS, not Registered to the Network!"));
      }
            
    } break;

    case CHECKSMS:{
      Serial.println(F("    STATE 3.5: CHECK FOR SMS   "));
      
      // 1. Flush buffer first
      while(Serial2.available()) Serial2.read();

      // 2. Send command to list UNREAD messages directly
      Serial2.println("AT+CMGL=\"REC UNREAD\"");
      
      // 3. Immediately go to parsing function (wait up to 5 seconds for data)
      // We pass 'true' to indicate we just sent a command and expect data
      if (checkIncomingHologramData()) {
          Serial.println(F("✓ Calibration SMS Found & Processed"));
          
          // Cleanup messages
          sendATcommand("AT+CMGD=1,4", "OK", 2000); 
          
          SIM_state = ATTACHGPRS; // Or restart loop
      } else {
          Serial.println(F("No Calibration SMS found"));
          SIM_state = ATTACHGPRS;
      }
      
    } break;
    
    case ATTACHGPRS:{       //start ATTACHGPRS state
    
      Serial.println(F("    STATE 4: ATTACH GPRS    "));

      countloop = 0; // reset while loop counter

      // 1. Buffer Initialization
      constexpr size_t MAX_STR_LEN = 50;
      char aux_str[MAX_STR_LEN];
      
      memset(aux_str, '\0', sizeof(aux_str)); //Why assign '\0' to a memory location?

      
      sendATcommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"", "OK", 2000);
      
      //hologram
       
      snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"APN\",\"%s\"", "hologram");
      sendATcommand(aux_str, "OK", 2000);
      
      
      //Telcel
      /*
      snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"APN\",\"%s\"", "internet.itelcel.com");
      sendATcommand(aux_str, "OK", 2000);
      
      snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"USER\",\"%s\"", "webgprs");
      sendATcommand(aux_str, "OK", 2000);
      
      snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"PWD\",\"%s\"", "webgprs2002");
      sendATcommand(aux_str, "OK", 2000);
      */
      
     
           //+SAPBR: 1,1                                                       //before 6000-for both-gets stuck here!
      while ((sendATcommand("AT+SAPBR=1,1", "OK", 10000) || sendATcommand("AT+SAPBR=2,1", "+SAPBR: 1,1,\"", 10000)) != 1) // (1 != 1)--> (0)-->connected,  countloop = 0..or 1
              {countloop++;if(countloop >= 2){break;}}; // if it's (1), try again up to 10 times; countloop >= 5
             
      if  (countloop < 2){ //countloop < 5
        SIM_state = INITHTTP;
        Serial.println(F("Connected to the Network!")); //(0<5)-->(1)-->connected
      }
      else{
        //Normal mode state
        SIM_state = ENDGPRS; 

        Serial.println(F("End GPRS, Not Connected to the Network!"));
      }
      //end ATTACHGPRS state
      
    } break;
    
    case INITHTTP:{
      
      Serial.println(F("    STATE 5: INITHTTP   "));

      countloop = 0; // reset while loop counter
      
      //start INITHTTP
      
      while (sendATcommand("AT+HTTPINIT", "OK", 10000) != 1)
      {countloop++;if(countloop >= 3){break;}}; 
      
      delay(100);
      
      sendATcommand("AT+HTTPPARA=\"CID\",1", "OK", 5000);
      
      
      if  (countloop < 3){ //countloop < 3
        
        SIM_state = READSENSOR;
        Serial.println(F("Init HTTP functional!"));
      }
      else{
         //SIM_state =  INITHTTP; //try again-->how many times?, for both modes-->infinite loop(2)
         SIM_state = ENDGPRS; // or ENDGPRS session, for both modes?
         Serial.println(F("Init HTTP not functional!"));
      }
      //end INITHTTP
            
    } break;
      
    case SENDPARA:{
    
      Serial.println(F("    STATE 6: SENDPARA   "));
   
      //start SENDPARA
      countloop = 0; // reset loop counter 
      
      //char aux_str_[400]; //auxiliary string //200; increase aux_str_[210]
      
      // 1. Buffer Initialization
      constexpr size_t MAX_URL_LEN = 256;  // Adjust based on server requirements
      char aux_str_[MAX_URL_LEN];
      
      //Initialize buffer (optional but safe)
      memset(aux_str_, '\0', sizeof(aux_str_)); // with memory assignation? //200

      //Serial.println(depth);
      
      // 2. Safely format URL with truncation checks
      snprintf(aux_str_, sizeof(aux_str_), awsServerName,Token,DepthVal,rateVal,BattVal);
   
      // 3. Check for truncation
      if (strlen(aux_str_) >= sizeof(aux_str_) - 1) {
        Serial.println(F("URL truncated!"));
      }
      
      // 4. Send AT command with extended timeout
      sendATcommand(aux_str_, "OK", 2000); // send URL, increase time to 600? org:520   2,601,0
      
                                                                                             // AT+HTTPACTION=1 (POST), AT+HTTPACTION=0 (GET)
                                                                                             //AT+HTTPACTION=Method,StatusCode,DataLen 
                                                            // '3' can be removed--> 0,200
      while (sendATcommand("AT+HTTPACTION=0", "+HTTPACTION: 0,200,3", 15000)!= 1){           //Why 0,200,3 is the expected anwser? is 3 the amount of sent data in bytes?; AT+HTTPACTION=0"
       delay(3); // before = 3; this delay is important
       countloop++;if(countloop >= 3){break;}; // we can try 5 times instead of 10           //+HTTPACTION: 0,200,1000 response where 200 is the OK and 1000 is the payload size.
      }

      if (countloop >= 3){ //countloop >= 10
          Serial.println(F("Server connection failed"));
          SIM_state = ENDGPRS;
      }
      else{
        Serial.println(F("Data sent successfully"));
        sendATcommand("AT+HTTPREAD", "ok!", 5000);   //before "!!"                                         // recevices: --ok!-- from Smability platform

        if (state == 1){ //rain mode
          //Serial.print(" DEVICE_STATE: ");  
         
          SIM_state = READSENSOR;
          flag = true;
          
        } else { //state = 0, dry mode
          //Serial.print(" DEVICE_STATE: ");  
          
          //SIM_state = ENDGPRS; -->use only if optical sensor is attached
          
          SIM_state = SIMOFF;
          justWoke = true;
          
        }
      }
      
    } break;
      
    case ENDGPRS:{
      
      Serial.println(F("    STATE 7: ENDGPRS    "));
      
      //start ENDGPRS
      
      sendATcommand("AT+HTTPTERM", "OK", 4000); // closing http if open
      sendATcommand("AT+CLTS=0", "OK", 2000); // "Get Local Time" Stamp Disabled
      //sendATcommand("AT+HTTPACTION=0", "OK", 2000); //? 
      sendATcommand("AT+SAPBR=0,1", "OK", 4000); // closing bearer if open
      sendATcommand("AT+CGATT=0","OK",2000); //DE-Atach from the the network
      sendATcommand("AT+CIPSHUT", "OK", 2000); //De-Attach to GPRS network
      delay(1000);
      
      SIM_state = SIMOFF;
      
    } break;
      
    case SIMOFF:{
      
      Serial.println(F("    STATE 8: SIMOFF   "));
      
      //start SIMOFF
      
      countloop = 0; // while loop reset    
      
      while (sendATcommand("AT+CPOWD=1", "DOWN", 5000) != 1) 
        {countloop++; if(countloop >= 2){break;}};
      
      Serial.print(F("Elapsed Time in Seconds: "));
      Serial.println((millis() - SIM_InitialTime)/1000); // Display SIM ON time in seconds
      //end SIMOFF
      
      state = 0;
      powerOFFSensor();
      delay(500);
      
      
      powerOFFSIM ();
      delay(500);
      
     
      Serial.flush();
 
      SIM_state = START; // Start over
      
      //sleep();
      
    } break;

  }
  delay(1); //stability
}

// ===== CHECK INCOMING HOLOGRAM DATA =====
bool checkIncomingHologramData() {
    Serial.println(F("=== Reading SMS Buffer ==="));
    
    String response = "";
    unsigned long startTime = millis();
    bool dataStarted = false;

    // Wait loop: Wait for data to arrive, then read until silence
    while (millis() - startTime < 4000) { // 4 second timeout
        while (Serial2.available()) {
            char c = Serial2.read();
            response += c;
            startTime = millis(); // Reset timeout if we are receiving data
            dataStarted = true;
            
            // Memory protection for Arduino Mini Pro (2KB RAM)
            if (response.length() > 250) {
                // If buffer gets too big, keep only the end where the command might be
                // This prevents crashing
                response = response.substring(response.length() - 150); 
            }
        }
        // If we haven't received data yet, keep waiting. 
        // If we have received data and buffer is now empty, break to parse.
        if (dataStarted && !Serial2.available()) {
            delay(100); // Wait a tiny bit to ensure message is complete
            if (!Serial2.available()) break; 
        }
    }

    if (response.length() == 0) return false;

    Serial.println(F("[RAW DATA]"));
    // Serial.println(response); // Uncomment for full debug (careful with memory)

    // --- PARSING LOGIC ---
    // Look for "CAL:"
    int calIndex = response.indexOf("CAL:");
    
    if (calIndex >= 0) {
        Serial.println(F("Found 'CAL:' header..."));
        
        // Extract the substring starting at CAL:
        String potentialCmd = response.substring(calIndex);
        
        // Find the end of the line (newline or carriage return)
        int endLine = potentialCmd.indexOf('\r');
        if (endLine == -1) endLine = potentialCmd.indexOf('\n');
        if (endLine == -1) endLine = potentialCmd.length();
        
        // Isolate just the command: "CAL:7.5,5028.5,0.0,5000.0"
        String cleanCommand = potentialCmd.substring(0, endLine);
        cleanCommand.trim(); // Remove whitespace
        
        // Check for double quotes usually found in SMS response like "CAL:..."
        int quoteIndex = cleanCommand.indexOf('"');
        if (quoteIndex != -1) {
             cleanCommand = cleanCommand.substring(0, quoteIndex);
        }

        Serial.print(F("Processing: "));
        Serial.println(cleanCommand);
        
        processCalibrationCommand(cleanCommand);
        return true;
    }
    
    return false;
}


// ===== PROCESS CALIBRATION COMMAND =====
void processCalibrationCommand(String data) {
  Serial.println(F("\n=== Processing Calibration Command ==="));

  // 1. Basic Cleanup
  data.trim();
  int calIndex = data.indexOf("CAL:");
  
  if (calIndex == -1) {
    Serial.println(F("✗ Error: No 'CAL:' prefix found"));
    return;
  }

  // 2. Isolate the numbers string (e.g., "7.5,5028.5,0.0,5000.0")
  // We use substring to skip "CAL:" 
  String payload = data.substring(calIndex + 4);
  payload.trim(); // Remove any remaining whitespace/newlines

  float tempValues[4]; // Temporary storage
  int count = 0;       // How many numbers we found
  int startIndex = 0;  // Start position of current number

  // 3. Parsing Loop
  for (int i = 0; i <= payload.length(); i++) {
    // Trigger if we find a comma OR we reached the end of the string
    if (payload.charAt(i) == ',' || i == payload.length()) {
      
      // Extract the individual number string
      String numStr = payload.substring(startIndex, i);
      numStr.trim(); // CRITICAL: Remove hidden \r or \n
      
      if (numStr.length() > 0) {
        if (count < 4) {
          tempValues[count] = numStr.toFloat();
          count++;
        }
      }
      
      startIndex = i + 1; // Move start index past the comma
    }
  }

  // 4. Validation: Did we get exactly 4 numbers?
  if (count != 4) {
    Serial.print(F("✗ Error: Expected 4 values, got "));
    Serial.println(count);
    sendCalibrationResponse(false);
    return;
  }

  // 5. Logic Validation: Check Low vs High ranges
  // rawLow < rawHigh
  if (tempValues[0] >= tempValues[1]) {
    Serial.println(F("✗ Error: Raw Low must be less than Raw High"));
    sendCalibrationResponse(false);
    return;
  }
  // refLow < refHigh
  if (tempValues[2] >= tempValues[3]) {
    Serial.println(F("✗ Error: Ref Low must be less than Ref High"));
    sendCalibrationResponse(false);
    return;
  }

  // 6. Apply Values (Atomic Update)
  coeffs.rawLow  = tempValues[0];
  coeffs.rawHigh = tempValues[1];
  coeffs.refLow  = tempValues[2];
  coeffs.refHigh = tempValues[3];
  coeffs.isCalibrated = true;

  // 7. Save to EEPROM and Confirm
  saveCoefficientsToEEPROM();
  
  Serial.println(F("✓ Calibration Success. New Coefficients:"));
  printCoefficients();
  sendCalibrationResponse(true);
}

// ===== SEND CALIBRATION RESPONSE =====
void sendCalibrationResponse(bool success) {
  String response;
  
  if (success) {
    response = "CAL_SUCCESS:a=" + String(coeffs.rawLow, 2) + ",b=" + String(coeffs.rawHigh, 4) + ",c=" + String(coeffs.refLow, 4) + ",d=" + String(coeffs.refHigh, 4);
    Serial.println(F("✓ CALIBRATION APPLIED & SAVED"));
    Serial.println(response);
    Serial.println(F("Next data upload will show calibrated values"));
  } else {
    response = "CAL_ERROR:Invalid coefficients format";
    Serial.println(response);
    Serial.println(F("Check SMS format: CAL:rawLow,rawHigh,refLow,refHigh"));
  }
  
  //Serial.println("Sending response: " + response);
  //sendDataToHologram(response);
}

// ===== PRINT COEFFICIENTS =====
void printCoefficients() {
  Serial.println("a = " + String(coeffs.rawLow, 2));
  Serial.println("b = " + String(coeffs.rawHigh, 4));
  Serial.println("c = " + String(coeffs.refLow, 4));
  Serial.println("d = " + String(coeffs.refHigh, 4)); 
}

// ===== SEND DATA TO HOLOGRAM =====
void sendDataToHologram(String data) {
  
  Serial.println(F("=== Sending to Hologram via SMS ==="));
 
  Serial.println("Data to send: " + data); 

  int countloop = 0;

  // Enable SMS text mode
  while (sendATcommand("AT+CMGF=1", "OK", 2000) != 1) {
    countloop++;
    if (countloop >= 2) {
      Serial.println(F("Failed to set SMS mode"));
      return;
    }
  }
  
  // Hologram SMS ingestion number (get from your dashboard)
  //const char* hologramSMSNumber = "+13128787221"; // TODO: Replace with actual number
  
  char smsCmd[50];
  snprintf(smsCmd, sizeof(smsCmd), "AT+CMGS=\"%s\"", hologramSMSNumber);
  
  if (sendATcommand(smsCmd, ">", 5000) == 1) {
    Serial.println(F("Ready to send SMS..."));
    
    Serial2.print(data);
    delay(100);
    Serial2.write(26); // Ctrl+Z
    
    unsigned long smsStart = millis();
    while (millis() - smsStart < 30000) {
      if (Serial2.available()) {
        String response = Serial2.readString();
        if (response.indexOf("+CMGS:") >= 0) {
          Serial.println(F("✓ SMS sent to Hologram"));
          return;
        }
        if (response.indexOf("ERROR") >= 0) {
          Serial.println(F("✗ SMS error"));
          return;
        }
      }
      delay(500);
    }
    Serial.println(F("⚠ SMS timeout"));
  } else {
    Serial.println(F("Failed to initiate SMS"));
  }
    
}
// ===== SLEEP FUNCTION =====
void sleep(){
  
    attachInterrupt(digitalPinToInterrupt(RG9_Pin), rgisr, LOW);
    
    Serial.println(F("Going to sleep..."));
    
    Serial.flush();                                     // Wait for serial to finish sending

    powerOFFSensor();                                   //Switch Level Sensor OFF
   
    sendATcommand("AT+CSCLK=2", "OK", 1000);            //Activates automatic sleep mode.

    for (int i = 0; i < 33; i ++){                       //7=1 min sleep approx; 4*8=32 sec; it has a limit; // 7 x 9.2s=64.4s; 6 × ~9.2s = ~55s; not exactly 8 sec but 9.2 sec
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
    }
    Serial.print(F("Awake from sleep. Cycle: "));

    /*                                                    // Advance millis() by 8 000 ms*4 so timing logic keeps working
    noInterrupts();
    timer0_millis += 65000UL;
    interrupts();
    */
    sendATcommand("AT+CSCLK=0", "OK", 1000);            //Disables sleep mode

    powerONSensor();                                    //Switch Level Sensor ON
    
}

double readsensor(){
  
  depth = round(avr_depth())/10.0; // depth measure in cm
 
  //sanity check
  if (depth < 0) {
    depth = 0.0;
  }
  
  return depth;
  
}


//===== Discharge or charging rate FUNCTION =====
double ratesensor(float currentDepth){
  static float previousDepth = 0.0;
  static bool firstReading = true;
  float diff;
  
  if (firstReading) {
    // First ever reading - no previous value to compare
    previousDepth = currentDepth;
    firstReading = false;
    return 0.0;
  }
  
  // Calculate rate: current - previous
  diff = currentDepth - previousDepth;
  
  // Store current reading for next cycle
  previousDepth = currentDepth;
  
  return diff;
} 

//===== print battery in  % =====  
void battery(){

  dtostrf(round(BattPercentage()),2,0,BattVal);
  // Print Battery Percentange
  Serial.println(round(BattPercentage())); 
  }
  
//read pressure sensor raw
float get_depth(){
  dataVoltage = analogRead(ANALOG_PIN); //0-1024 counts
  voltage = dataVoltage * (5000.0 / 1024.0); // in mV
  dataCurrent = (voltage / 120.0); //Sense Resistor:120ohm; in mA
  return dataCurrent;
  }
  
//===== depth average function in mm ===== 
 float avr_depth(){
  
  float DepthAvg;
  float sum;
  int i;
  int samples = 20 ;
  float depth[samples];
  sum = 0.0;

  //analogRead(ANALOG_PIN); 
  //delay(10);

  for (i = 0; i < samples; i++){
    depth[i] = (get_depth() - CURRENT_INIT)*((RANGE/DENSITY_WATER)/16.0);
    sum = sum + depth[i];
    //Serial.println(depth[i]); //debug
    delay(2);
  }
  
  DepthAvg = sum/(samples);
  
  // Apply two-point calibration if enabled
  if (coeffs.isCalibrated) {
    float rawRange = coeffs.rawHigh - coeffs.rawLow;
    float refRange = coeffs.refHigh - coeffs.refLow;
    
    // Prevent division by zero
    if (abs(rawRange) < 0.001) {
      Serial.println(F("Warning: Invalid calibration range"));
      return DepthAvg; // Return uncalibrated value
    }
    
    // Apply calibration formula
    float calibratedDepth = (((DepthAvg - coeffs.rawLow) * refRange) / rawRange) + coeffs.refLow;
    
    Serial.print(F("Raw: "));
    Serial.print(DepthAvg);
    Serial.print(F(" -> Calibrated: "));
    Serial.println(calibratedDepth);
    
    return calibratedDepth;
  }

  Serial.println(DepthAvg); // debug
  return DepthAvg;
  
 }

//===== battery raw =====  
float BattVolt(){
  float voltage;
  int value;
  value = analogRead(ANALOG_BATT_PIN);
  voltage = (value * 5.00/1023)/0.5; //use this eq if R1= 100k and R2=100k
  //voltage = (value * 5.01/1023)/0.32; //use this eq if R1= 47k and R2=100k
  return voltage;
  }

//===== battery in % =====  
float BattPercentage(){
  float Batpercentage;
  Batpercentage = ((BattVolt()-3)/1.2)*100; //3.7v nominal value
  if (Batpercentage < 0) Batpercentage = 0; //do not display negative values
  return Batpercentage;
  }
  
//===== Power ON/OFF sensor & SIM  =====  
void powerONSensor()
  {
  digitalWrite (POWER, HIGH);  // turn power ON
  delay (1500); // give time to power up, increase to one second more, before=1000   
  }  // end of powerOnPeripherals

void powerOFFSensor()
  {
  digitalWrite (POWER, LOW);  // turn power OFF
  delay (1); // give time to power up    
  }  // end of powerOFFPeripherals

void powerONSIM()
  {
  digitalWrite (POWERSIM, HIGH);  // turn power ON
  delay (1000); // give time to power up    
  }  // end of powerOnPeripherals

void powerOFFSIM()
  {
  digitalWrite (POWERSIM, LOW);  // turn power OFF
  delay (1); // give time to power up    
  }  // end of powerOFFPeripherals
  
//===== Calibration coefficients in EEPROM Debug mode ===== 
/* 
void printEEPROMDebug() {
    #ifdef DEBUG_MODE
    Serial.println(F("\n=== EEPROM DEBUG ==="));
    
    int addr = EEPROM_CAL_ADDR;
    uint16_t magic;
    
    EEPROM.get(addr, magic);
    Serial.print(F("Magic: 0x"));
    Serial.println(magic, HEX);
    
    if (magic == EEPROM_MAGIC) {
        addr += sizeof(uint16_t);
        
        CalibrationCoeffs tempCoeffs;
        EEPROM.get(addr, tempCoeffs);
        
        Serial.println(F("Stored in EEPROM:"));
        Serial.print(F("  rawLow:  ")); Serial.println(tempCoeffs.rawLow, 4);
        Serial.print(F("  rawHigh: ")); Serial.println(tempCoeffs.rawHigh, 4);
        Serial.print(F("  refLow:  ")); Serial.println(tempCoeffs.refLow, 4);
        Serial.print(F("  refHigh: ")); Serial.println(tempCoeffs.refHigh, 4);
        Serial.print(F("  isCalibrated: ")); Serial.println(tempCoeffs.isCalibrated);
    } else {
        Serial.println(F("No valid calibration in EEPROM"));
    }
    
    Serial.println(F("===================\n"));
    #endif
}
*/

/***********************************************************************************
 * AT-COMMAD SIM800L FUNCTION
 * 
************************************************************************************/
int8_t sendATcommand(const char* ATcommand, const char* expected_answer, unsigned int timeout){
    
    uint8_t x=0,  answer=0;
    //char response[500];

    // Replace with:
    constexpr size_t MAX_RESPONSE_LEN = 200;
    char response[MAX_RESPONSE_LEN] = {0};
    
    unsigned long previous;

    memset(response, '\0', 200);    // Initialize the string

    delay(10);
    
    while( Serial2.available() > 0) Serial2.read();               // Clean the input buffer
      
    //if (ATcommand[0] != '\0')
    //{
      noInterrupts();
      Serial2.println(ATcommand);    // Send the AT command
      interrupts();
    //}
    

    x = 0;
    previous = millis();


    // this loop waits for the answer
    do{
        if(Serial2.available() != 0){
            // if there are data in the UART input buffer, reads it and checks for the asnwer
            response[x] = Serial2.read();
            delay(1);
            x++;
            
            // ADDED: Buffer overflow protection
            if (x >= MAX_RESPONSE_LEN - 1) {
                Serial.println(F("Warn: Buffer full"));
                break;
            }
            
            // check if the desired answer  is in the response of the module
            if (strstr(response, expected_answer) != NULL){
                answer = 1;
            }
  
        }

    }
    // Waits for the asnwer with time out
    while((answer == 0) && ((millis() - previous) < timeout));
    
    /*
    noInterrupts();
    Serial.println(response);    // Send the AT command
    interrupts();
    */
    return answer;
    
}
