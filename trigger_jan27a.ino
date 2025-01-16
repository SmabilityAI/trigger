#include <LowPower.h> // Low-Power library
#include <SoftwareSerial.h> // Serial library
SoftwareSerial Serial2(8,9); //SIM800L Rx=8, Tx=9

#define RG9_Pin 2// RG-11 is connected to digital pin 2
#define POWER (6) //power peripherals
#define POWERSIM (12) //power SIM-BUCK/PCB
//#define ANALOG_PIN (A5) //PCB, A6 Arduino MINI PRO, A5 Arduino UNO
#define ANALOG_PIN (A6) //PCB, A6 Arduino MINI PRO, A5 Arduino UNO
#define ANALOG_BATT_PIN (A2) //PCB
#define RANGE 5000 // Depth measuring range 5000mm (for water)
#define CURRENT_INIT 4.0// Current @ 0mm (uint: mA)--->modulate to calibrate to Zero (4mA to 4.2mA)
#define DENSITY_WATER 1  // Pure water density normalized to 1
#define DENSITY_BLEACH 1.104  // https://www.alenusa.com/wp-content/uploads/2020/04/2016-cloralen-regular-bleach.pdf
#define DENSITY_GASOLINE 0.74  // Gasoline density
#define PRINT_INTERVAL_MIN 10000  // 10 second
#define SIM_INTERVAL_MIN 20000  // 20 second

volatile byte DEVICE_STATE;// volatile variable
unsigned long SIM_InitialTime;
int16_t dataVoltage;
float dataCurrent, depth; //unit:mA
float voltage;
unsigned long timepoint_measure; 
//Sensor readings stored in two char array
char BattVal[3]; // "80" = [0,1,2], size of the sting + 1
char DepthVal[5]; // "0.99"= [0,1,2,3,4]-->[0,.,9,9,\0];
char Token[33] = "";
bool flag = false;
bool flagINT = true;
int value;
int mode;

void setup(){
  DEVICE_STATE = 0;
  Serial.begin(19200);
  Serial2.begin(9600);// SIM8008 baud rate
  Serial.println("Hydreon RG-9 Rain Event Detector  | smability.io");
  delay(100);
  pinMode(RG9_Pin, INPUT);
  pinMode(ANALOG_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RG9_Pin), rgisr, LOW);
}

// Interrrupt handler routine that is triggered when the rg-9 detects rain
void rgisr() {
     DEVICE_STATE = 1;
}
// end of rg-9 rain detection interrupt handler 
void loop(){
  //cli();//Disable interrupts..clears the global interrupt flag, in SREG so prevent any form of interrupt occurring
  DEVICE_STATES();
  //sei();//Enables interrupts, sets the bit and switches interrupts on
}

void DEVICE_STATES(){
  switch (DEVICE_STATE)
  {
    
    case 0: //Normal Mode
    
      Serial.println(F(" DEVICE_STATE: 0 ")); // 1hr sampling
      
      mode = 0;
      
      detachInterrupt (digitalPinToInterrupt (RG9_Pin)); 
      SIM_STATES(mode); //1 hr sampling
      
      break;
   
    case 1: //Rain Mode
      
      Serial.println(F(" DEVICE_STATE: 1 "));
      
      mode = 1;
      
      detachInterrupt (digitalPinToInterrupt (RG9_Pin)); 
      SIM_STATES(mode);
      attachInterrupt(digitalPinToInterrupt(RG9_Pin), rgisr, LOW); //interrupt is Activated after a switch-casee break
      
      DEVICE_STATE = 0; //Reset DEVICE_STATE to 0
   
      break;
  }  
}

//SIM states
enum SIM
    {
      START,SENSORS,READSENSOR,
      PWRSIM,ATTACHGPRS,
      INITHTTP,SENDPARA,
      ENDGPRS,SIMOFF
    };

SIM SIM_state  = START;

void SIM_STATES(int state){
  
  int countloop = 0; // set countloop to 0
  
  switch (SIM_state)
  {
   
    case START:
    
      Serial.println(F("    STATE 0: START SENSORS    "));
      
      flag = false; 
      powerOFFSIM();
      delay(500);
      SIM_state = SENSORS;
        
      break; //goes to the closing brace (final brace) of switch-case body and continues with the next instruction-the interrupt-
      
    case SENSORS:
    
      Serial.println(F("    STATE 1: POWER ON SENSORS   "));
      
      if(state == 1){//Rain mode
        powerONSensor();
        SIM_state = READSENSOR;
      }
      else {//Normal mode
        powerONSensor();
        delay(500); //wait some time
        readsensor();
        //OFF Sensors
        delay(500); //wait some time
        powerOFFSensor();
     
        SIM_state = PWRSIM;
      }
            
      break;

   case READSENSOR:
    
      Serial.println(F("    STATE 2: READ SENSOR   "));
       
      //2, 3, 4 ..sensor measurement. (Read pressure sensor value in rain mode)
      if (flag == true){
        readsensor();
        delay(500);
        //battery();--> theres some conflict with this variable here
        //delay(500);
        SIM_state = SENDPARA;
      }
      else{
        readsensor();
        delay(500);
        SIM_state = PWRSIM;
      }
     
      break;
      
    case PWRSIM:
   
      Serial.println(F("    STATE 3: POWER ON SIM800L   "));

      countloop = 0; // rest while loop counter
      
      
      powerONSIM(); //Power ON SIM800L
      
      delay(2000); //wait some time

      //sendATcommand("AT+CFUN=0", "OK", 2000);
      //delay(1000);

      //sendATcommand("AT+CFUN=1", "OK", 2000);
      //delay(1000);
      
      ////sendATcommand("AT", "OK", 1000);
      //sendATcommand("AT+CSQ", "OK", 700);
      //delay(4000);
      //sendATcommand("AT+CGATT=1", "OK", 1000);
      //SIM is OFF-fly mode-minimun functionality
      //sendATcommand("AT+CFUN=0", "OK", 2000);
      //delay(1000);
      //SIM is ON
      //sendATcommand("AT+CFUN=1", "OK", 1100);
      
      SIM_InitialTime = millis(); //Record SIM -ON- time

      //convert battery % to string and display battery value
      battery();

      delay(500);

      sendATcommand("AT", "OK", 2000);
     
      //sendATcommand("AT+CSQ", "OK", 700);
     
      sendATcommand("AT+CFUN=1", "OK", 2000);

      //sendATcommand("AT+CIPSHUT", "OK", 1000); //de-Attach to GPRS network
      //delay(1000);
      
           
      sendATcommand("AT+CREG=1", "OK", 200);  // Activating CREG notifications
      //delay(1000);
      // All 0s means While is TRUE, countloop increments, we need at least one 1 to break the loop
      while ((sendATcommand("AT+CREG?", "+CREG: 1,1", 1000) // before 0,1,
              || sendATcommand("AT+CREG?", "+CREG: 0,5", 1000) 
              || sendATcommand("AT+CREG?", "+CREG: 0,1", 1000)
              || sendATcommand("AT+CREG?", "+CREG: 2,1", 1000)) != 1 )
              {countloop++; if(countloop >= 4){break;}};  // If it does not find network think what to do..  

      sendATcommand("AT+CREG=0", "OK", 5000); // Deactivating CREG notifications; -->SIM gets stuck here!

     
      //sendATcommand("AT+CSQ", "OK", 1000);
      //delay(2000);
                                  
      if (countloop < 4){ 
         
         SIM_state = ATTACHGPRS;
         Serial.println(F("Registered to the Network!"));
      }
      else{
          //SIM_state = SIMOFF;   //otherwise will hang if not registered, we will try again in one hour
          SIM_state = ENDGPRS;
          Serial.println(F("End GPRS, not Registered to the Network!"));
      }
            
      break;
      
    case ATTACHGPRS:
    
      Serial.println(F("    STATE 4: ATTACH GPRS    "));

      countloop = 0; // reset while loop counter

      //start ATTACHGPRS state
      
      char aux_str[50];
      
      memset(aux_str, '\0', sizeof(aux_str)); //Why assign '\0' to a memory location?

      //sendATcommand("AT+CGATT=1", "OK", 2000); //Attach to GPRS network
      
      //delay(1000);
      //sendATcommand("AT+CGATT?", "OK", 1000); //Attach to GPRS network-->return ERROR
      //delay(1000);
      //sendATcommand("AT+CGATT=1", "OK", 25000);
      //sendATcommand("AT+SAPBR=0,1", "OK", 4000); // closing bearer if open
      
      sendATcommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"", "OK", 2000);
      
      
      
      //hologram
       /*
      snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"APN\",\"%s\"", "hologram");
      sendATcommand(aux_str, "OK", 2000);
      */
      //hologram
      //sendATcommand("AT+CSTT=hologram","OK",2000);
      //delay(1000);
      
      //Telcel
      
      snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"APN\",\"%s\"", "internet.itelcel.com");
      sendATcommand(aux_str, "OK", 2000);
      
      snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"USER\",\"%s\"", "webgprs");
      sendATcommand(aux_str, "OK", 2000);
      
      snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"PWD\",\"%s\"", "webgprs2002");
      sendATcommand(aux_str, "OK", 2000);
      
      //delay(1000);
      
     
           //+SAPBR: 1,1                                                       //before 6000-for both-gets stuck here!
      while ((sendATcommand("AT+SAPBR=1,1", "OK", 10000) || sendATcommand("AT+SAPBR=2,1", "+SAPBR: 1,1,\"", 10000)) != 1) // (1 != 1)--> (0)-->connected,  countloop = 0..or 1
              {countloop++;if(countloop >= 2){break;}}; // if it's (1), try again up to 10 times; countloop >= 5
             
      if  (countloop < 2){ //countloop < 5
        SIM_state = INITHTTP;
        Serial.println(F("Connected to the Network!")); //(0<5)-->(1)-->connected
      }
      else{
        //Normal mode state
        SIM_state = ENDGPRS; //Can we go first to ENDGPRS then SIMOFF State?
        //SIM_state = SIMOFF;
        
         //SIM is OFF-fly mode-minimun functionality
       //sendATcommand("AT+CFUN=0", "OK", 1000);
       //SIM is ON
       //sendATcommand("AT+CFUN=1", "OK", 1000);

        Serial.println(F("End GPRS, Not Connected to the Network!"));
      }
      //end ATTACHGPRS state
      
      break;
      
    case INITHTTP:
      
      Serial.println(F("    STATE 5: INITHTTP   "));

      countloop = 0; // reset while loop counter
      
      //start INITHTTP
      
      while (sendATcommand("AT+HTTPINIT", "OK", 10000) != 1)
      {countloop++;if(countloop >= 3){break;}}; 
      
      delay(100);
      
      sendATcommand("AT+HTTPPARA=\"CID\",1", "OK", 5000);
      
      
      if  (countloop < 3){ //countloop < 3
        
        SIM_state = SENDPARA;
        Serial.println(F("Init HTTP functional!"));
      }
      else{
         //SIM_state =  INITHTTP; //try again-->how many times?, for both modes-->infinite loop(2)
         SIM_state = ENDGPRS; // or ENDGPRS session, for both modes?
         Serial.println(F("Init HTTP not functional!"));
      }
      //end INITHTTP
            
      break;
      
    case SENDPARA:
    
      Serial.println(F("    STATE 6: SENDPARA   "));
   
      //start SENDPARA
      countloop = 0; // reset loop counter 
      
      char aux_str_[210]; //auxiliary string //200; increase aux_str_[210]
     
      memset(aux_str_, '\0', 210); // with memory assignation? //200

      Serial.println(depth);
      
      //dtostrf(depth/1000.0, 4, 2, DepthVal); // depth is float variable, 4 is the lenght of the string what will be created, 2 number of digits after the decimal point to print, DepthVal the array to store the results

      /* Smability-V1.001
       * device serial:XXXX
       * device token: YYYY
       * variables: Battery, Depth 
      */

      snprintf(aux_str_, sizeof(aux_str_), "AT+HTTPPARA=\"URL\",\"http://smability.sidtecmx.com/Device//SetURL?distance=%s&batt=%s&deviceID=%s \"",DepthVal,BattVal,Token);
     
      /*
       * URL example:
       * http://smability.sidtecmx.com/Device//SetURL?distance=0.00&batt=99&deviceID=b90f7eb37aab6954ea222b3eeb780c60
       * 
      */
      
      sendATcommand(aux_str_, "OK", 520); // send URL, increase time to 600? org:520   2,601,0
      
                                                                                             // AT+HTTPACTION=1 (POST), AT+HTTPACTION=0 (GET)
                                                                                             //AT+HTTPACTION=Method,StatusCode,DataLen 
                                                            // '3' can be removed--> 0,200
      while (sendATcommand("AT+HTTPACTION=0", "+HTTPACTION: 0,200,3", 15000)!= 1){           //Why 0,200,3 is the expected anwser? is 3 the amount of sent data in bytes?; AT+HTTPACTION=0"
       delay(3); // before = 3; this delay is important
       countloop++;if(countloop >= 3){break;}; // we can try 5 times instead of 10           //+HTTPACTION: 0,200,1000 response where 200 is the OK and 1000 is the payload size.
      }

      if (countloop >= 3){ //countloop >= 10
          Serial.println(F("Failure in sending data"));
          SIM_state = ENDGPRS;
      }
      else{
        Serial.println(F("Success in sending data"));
        sendATcommand("AT+HTTPREAD", "+HTTPREAD: 3", 5000);   //before "!!"                                         // recevices: --ok!-- from Smability platform

        if (state == 1){ 
          //Serial.print(" DEVICE_STATE: ");  
          //Serial.println(state);
          SIM_state = READSENSOR;
          flag = true;
        }
        else{ //state = 0
          //Serial.print(" DEVICE_STATE: ");  
          //Serial.println(state);
          SIM_state = ENDGPRS;
        }
      }
      
      break;
      
    case ENDGPRS:
      
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
      
      break;
      
    case SIMOFF:
      
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
      
      sleep();
      
      break;
  }
  delay(1); //stability
}

// 5 second sampling mode function; use if stament within the main STATE-Machine SIM_STATES()

void sleep(){
    attachInterrupt(digitalPinToInterrupt(RG9_Pin), rgisr, LOW);
    for (int i = 0; i < 35; i ++){ //7=1 min sleep approx; 420=1hr approx
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
    }
}
//read pressure snesor value
void readsensor(){
  depth = round(avr_depth())/1000.0; // retrieve depth measure
  //sanity check
  if (depth < 0) depth = 0.0;
  //display depth in meters
  Serial.println(depth); // in meters
  //convert to string
  dtostrf(depth, 3, 2, DepthVal);
  }


void battery(){
  delay(100);
  dtostrf(round(BattPercentage()),3,0,BattVal);
  // Print Battery Percentange
  Serial.println(round(BattPercentage())); 
  }
  
//read pressure sensor
float get_depth(){
  dataVoltage = analogRead(ANALOG_PIN); //0-1024 counts
  voltage = dataVoltage * (5000.0 / 1024.0); // in mV
  dataCurrent = (voltage / 120.0); //Sense Resistor:120ohm; in mA
  return dataCurrent;
  }
  
//average function
 float avr_depth(){
  
  float DepthAvg;
  float sum;
  int i;
  int samples = 10 ;
  float depth[samples];
  sum = 0.0;

  for (i = 0; i<samples; i++){
    depth[i] = (get_depth() - CURRENT_INIT)*((RANGE/DENSITY_WATER)/16.0);
    sum = sum + depth[i];
    delay(1);
  }
  
  DepthAvg = sum/(samples);
  return DepthAvg;
 }

float BattVolt(){
  float voltage;
  int value;
  value = analogRead(ANALOG_BATT_PIN);
  voltage = (value * 5.00/1023)/0.5; //use this eq if R1= 100k and R2=100k
  //voltage = (value * 5.01/1023)/0.32; //use this eq if R1= 47k and R2=100k
  return voltage;
  }

float BattPercentage(){
  float Batpercentage;
  //Batpercentage = (BattVolt()-6)*30.3; // only for alkaline batteries; 9.3V Max
  //Batpercentage = ((BattVolt()-6)/(4.7))*100; //for AA alkaline and lithium batteries Max: 10.7v Min: 6.0v (pack of 6 AA batteries in series)
  delay(500);
  Batpercentage = ((BattVolt()-3)/1.2)*100; //1.5v nominal value
  if (Batpercentage < 0) Batpercentage = 0; //do not display negative values
  return Batpercentage;
  }
  
// Power ON/OFF sensor & SIM
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

/***********************************************************************************
 * AT-COMMAD SIM800L FUNCTION
 * 
************************************************************************************/
int8_t sendATcommand(char* ATcommand,  char* expected_answer, unsigned int timeout){
    
    uint8_t x=0,  answer=0;
    char response[270];
    unsigned long previous;

    memset(response, '\0', 270);    // Initialize the string

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
            //delay(5);
            x++;
            // check if the desired answer  is in the response of the module
            if (strstr(response, expected_answer) != NULL){
                answer = 1;
            }
            /*
            if (strstr(response, "+FTPPUT: 1,6") != NULL){
              Serial.println(F("FTP Error Code"));
                answer = 2;
            }
            */
            if (strstr(response, "+HTTPACTION: 0,6") != NULL){
              //Serial.println(F("HTTP Error Code"));
                answer = 2;
                
            }
            if (strstr(response, "ERROR") != NULL){
              //Serial.println(F("AT Command ERROR"));
                answer = 2;
            }
            /*
            if (strstr(response, "Location Not Fix") != NULL){
              Serial.println(F("GPS not found"));
                answer = 2;
            }
            */
            /* Why not?
            if (strstr(response, "+CREG: 0,2") != NULL)
            {
              Serial.println(F("Network not found"));
                answer = 2;
            }
            */
            
        }

    }
    // Waits for the asnwer with time out
    while((answer == 0) && ((millis() - previous) < timeout));
    
    //noInterrupts();
    //Serial.println(response);    // Send the AT command
    //interrupts();
    /*
    if((millis() - previous) > timeout){
      //noInterrupts();
      Serial.print(F("Time exceeded: "));
      Serial.println(millis() - previous);
      //interrupts();
      }
    */ 
    return answer;
    
}
