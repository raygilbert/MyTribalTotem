/**
 * Totem Firmware
 * (c) MyTribalTotem.com
 *
 * 
 */
#include <EEPROM.h>




/* SAVE MEMORY */
#define RF22_MAX_MSG_LEN      100



#include <RF22Datagram.h>
#include <RF22.h>
#include <SPI.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>


#define DEBUG            1
//#define DDEBUG           0
#define DEMO             1

#define MAX_TOTEM_LEN    24      /* Max size of totem NOT including NULL terminator */

/* Beacon more frequently in demo mode as it 'shows' better */
#ifdef DEMO
#define BEACON_INTERVAL  15000   /* How often to send beacon */
#else
#define BEACON_INTERVAL  60000   /* How often to send beacon */
#endif

#define RECV_WAIT_MIN    1000    /* Min time to block on recv call */
#define RECV_WAIT_MAX    5000    /* CAREFUL - if we wait TOO long on recv then the watchdog will reset */
#define QUIET_TIMEOUT    (BEACON_INTERVAL*2)
#define SYNC_BYTE        21
#define LED              5       /* Analog PIN 5 has the LED */
#define RED              5
#define GREEN            4
#define BLUE             3
#define RAND_ROTATE      1       /* Randomize over this number of sends to determine how often to change colors */
#define RAND_BASE        1       /* Add random factor to this base to get total # of sends to wait */
/*Serial protocol defines*/
#define RESPONSE_PREFIX       "++" /* 2x + */
#define DEFAULT_TOTEM_CODE    "MYTRIBALTOTEM.COM"
#define WAIT_FOR_END_OF_DATA  100  /* Milliseconds to wait for no messages to ensure we have entire buffered command */

/** Serial Command Set **/
#define SET_TOTEM             "AT&S50="              /* Command to set totem */
#define GET_TOTEM             "ATS50?"
#define MAX_INPUT_STRING      32                    /* Max length of serial input string */
#define SERIAL_ERROR_MSG      "Command not recognized"
#define PROGRAMMING_MODE_DURATION  10000  /* ms to wait for silence on serial port before exiting programming mode */

/*  EEPROM Table
Item        Size        Value        Notes
TableID     1           21           Indicates our table.  Must match 21
TableVer    1           1            Verison # of table.  Currently 1
TotemLen    1           ?            Length of totem
Totem       multi       data         Contains bytes that are the totem
*/

#define EEPROM_TABLE_ID       21
#define EEPROM_TABLE_VER      1



String         gTotemString;
String         gSerialRespString;
/**
 * Storage for totem code 
 */
//char          gTotemCode[MAX_TOTEM_LEN+1];

/**
 * Serial command I/O buffers
 */
//char          gSerialResponseMsg[MAX_INPUT_STRING+1];
//char          gInputString[MAX_INPUT_STRING+1];

/**
 * Different states the totem can be in 
 */
typedef enum {
  eUndefined_State = 0,
  eProgramming_State,
  eBeaconing_Inactive_State,
  eBeaconing_Active_State,
  eListening_Active_State 
} 
eTotemState;

uint32_t thecolors[] = {
  0xFF0000,            /* red */
  0x00FF00,            /* green */
  0x0000FF,            /* blue */
};


/*-----------------------------
 * Barking Dog protocol messages 
 -----------------------------*/
//URI Version inspired by DuckDuckGo - https://duckduckgo.com/params.html
// http://duckduckgo.com/?q=search&kp=-1&kl=us-en
//totem:<totemID>?proto=<proto version number unsigned byte>&state=on|off&ic=<indicator color code #12346&seq=<unsigned byte>=&rid=<unsigned byte>
//totem:                  [6 bytes]
//totemID: MAX_TOTEM_LEN  [24 bytes]
//?proto=1 byte data      [8 bytes]    
//&state=on/off           [10 bytes]
//&ic=#RRGGBB             [11 bytes]
//&seq=1 byte data        [6 bytes]
//&rid=1 byte data        [6 bytes]
#define MAX_BEACON_LEN    100 /*overkill */


/* Variables */
RF22Datagram   grf22(0);                    /* Radio instance */
int            seed;                        /* Random number generator seed */
unsigned long  gTimeOfNextBeacon;           /* Time to send next beacon */
unsigned long  gTimeLastBeaconRecvd;        /* Time last matching beacon received */
//uint8_t        buf[RF22_MAX_MESSAGE_LEN];   /* Buffer to recv msgs */
uint8_t        buf[MAX_BEACON_LEN];         /* Buffer to recv msgs */
uint8_t        gSequenceNum;                /* Message sequence number */
eTotemState    gState;                      /* State of totem */
uint8_t        gRed,gBlue,gGreen;           /* LED colors */
bool           gLEDOn;                      /* On/off */
uint8_t        rotateCounter;               /* how often to change led color */
uint8_t        gLastColorIndex;             /* index of last color chosen */
bool           gbReset;                     /* Force reset by suppressing patting watchdog timer */


/*------------------------------------------------------------------------------------
 * Setup 
 ------------------------------------------------------------------------------------*/
void setup() 
{
  Serial.begin(9600);
  
  // Print identifiers so USB scans can indentify totems
  Serial.println("++TOTEM");
  Serial.println("www.mytribaltotem.com");
  gbReset = false;

  if (!grf22.init())
    Serial.println("grf22 init failed");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb2_4Fd36

  //grf22.setTxPower(RF22_TXPOW_17DBM);  // power mode we determined in Ray's house w/out antenna

  seed = analogRead(0);
  randomSeed(seed);

  gState = eProgramming_State;
//  sprintf(gTotemCode,DEFAULT_TOTEM_CODE);        //Stores gTotemCode programmed
  gTotemString = DEFAULT_TOTEM_CODE;
  memoryTest();
  initTotem();
  
  pinMode(LED, OUTPUT);   //Set the pin to output 
  pinMode(RED, OUTPUT);   //Set the pin to output 
  pinMode(GREEN, OUTPUT);   //Set the pin to output 
  pinMode(BLUE, OUTPUT);   //Set the pin to output 

  cycleLEDs();
  
  rotateCounter = random(RAND_ROTATE)+RAND_BASE;
  gLastColorIndex = random(sizeof(thecolors)/sizeof(thecolors[0]));  /* need init value */
  calcLED();
  gLEDOn = false;
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/
void initTotem()
{
  gTotemString=DEFAULT_TOTEM_CODE;
  if (validateEEPROM() ) {
    readTotem();  
  }
  else {
    Serial.println("NEED TO INIT EEPROM");
    writeTotem();
  }
}
/*-------------------------------------------------------------------------------------------------------------------------------------------------*/
bool validateEEPROM()
{
 byte value;
 int address = 0;
 
 /* Init to default */
 gTotemString = DEFAULT_TOTEM_CODE;
 value = EEPROM.read(address++);
 if (EEPROM_TABLE_ID == value)
 {
      value = EEPROM.read(address++);
      if (EEPROM_TABLE_VER == value)
      {
        return true;
      }
      else {
        Serial.println("Invalid table ver");
      }
 } else {
   Serial.println("Invalid table ID");
 }
 return false;
  
}
/*-------------------------------------------------------------------------------------------------------------------------------------------------*/
void readTotem()
{
 byte value,length;
 int address = 2;

 if (validateEEPROM()) {

    /* Get totem length */
    length = EEPROM.read(address++);
    
    /* start outside of the loop so we init the string */
    value = EEPROM.read(address++);
    gTotemString =String(value);          

    for (int i=1;i<length;i++)
    {
          value = EEPROM.read(address++);
          gTotemString +=String(value);          
    }
    Serial.print("readTotem found ");
    Serial.println(gTotemString);
 }
 else {
   Serial.println("readTotem - use default totem");
   gTotemString = DEFAULT_TOTEM_CODE;   
 }
 
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/
void writeTotem()
{
 int address = 0;
 Serial.println("Writing totem code to EEPROM");
 EEPROM.write(address++,EEPROM_TABLE_ID);
 EEPROM.write(address++,EEPROM_TABLE_VER);
 EEPROM.write(address++,gTotemString.length());
 for (int q=0;q<gTotemString.length();q++)
 {
    EEPROM.write(address++,gTotemString.charAt(q)); 
 }
 gbReset = true;
 Serial.println("TOTEM WILL RESET IN A 30-60 SECONDS TO PROCESS EEPROM UPDATE");
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------*/
void setState(uint8_t newstate)
{
#ifdef DEBUG
  Serial.print("Old state - ");
  Serial.println(gState);
#endif
  //flashLED(3);                          /* Flash on state change */
  gState = (eTotemState)newstate;  

#ifdef DEBUG
  Serial.print("New state - ");
  Serial.println(gState);
#endif

}   
/*---------------------------------------------------------------------------------------------------------------------*/
void calcLED()
{
  uint8_t  index;
  uint32_t color;

  do {
    index = random(sizeof(thecolors)/sizeof(thecolors[0]));
  } while (gLastColorIndex == index);
  gLastColorIndex = index;
  color = thecolors[index];
  Serial.print("Random color selected: ");  
  Serial.println(color,HEX);
#ifdef DDEBUG
  Serial.print("Index ");
  Serial.println(index,DEC);
#endif
  gBlue   = color;
  gGreen  = (color>>8);
  gRed    = (color>>16);

#ifdef DDEBUG
  Serial.print("Red:   ");
  Serial.println(gRed,HEX);
  Serial.print("Green: ");
  Serial.println(gGreen,HEX);
  Serial.print("Blue:  ");
  Serial.println(gBlue,HEX);
#endif

}
//void calcLED()
//{
//    /* Set initial colors */  
//  {
//    gRed   = random(255);
//    gGreen = random(255);
//    gBlue  = random(255);
//    gLEDOn = false;
//  } while (0 == gRed == gGreen == gBlue);
//  Serial.print("Red:   ");Serial.println(gRed,HEX);
//  Serial.print("Green: ");Serial.println(gGreen,HEX);
//  Serial.print("Blue:  ");Serial.println(gBlue,HEX);
//  
//}
/*---------------------------------------------------------------------------------------------------------------------*/
/**
 * Compute time for next beacon
 * Determines when the next beacon should fire
 */
void calcNextBeacon()
{
  unsigned long now = millis();

  /* randomize the beacon timing so that it happens twice / interval to allow for noise and errors */
  gTimeOfNextBeacon = now + random(BEACON_INTERVAL/2);

}
#if 0
/*---------------------------------------------------------------------------------------------------------------------*/
/**
 * Compute how long to block waiting on recv() before we need to send our next beacon 
 */
uint16_t calcRecvTime()
{
  uint16_t wait;  

  if (millis() >= gTimeOfNextBeacon)
  {
    wait=0;
  }
  else 
  {
    wait = gTimeOfNextBeacon - millis();    
  }

  if (wait < RECV_WAIT_MIN)
    wait = RECV_WAIT_MIN;

  return wait;  

}
#endif
/*---------------------------------------------------------------------------------------------------------------------*/
/**
 * validateBeacon
 *
 * @param length of beacon message
 *
 * @returns true if valid beacon message, false otherwise
 */
bool validateBeacon(String beacon)
{
  /* TODO - complete this */
  if (!beacon.startsWith("totem:"))
  { 
#ifdef DEBUG
    Serial.println(">> ERROR: Recieved a not totem");
#endif
    return false;
  }
  return true;  

}
/*---------------------------------------------------------------------------------------------------------------------*//**
 * isMatchingBeacon
 * Use after validating beacon
 * 
 * @param stateFromBeacon use to return state value extracted from beacon
 * 
 * returns true if beacon matches, false if not
 */
bool isMatchingBeacon(bool *stateFromBeacon,String beacon)
{
  String pattern = "totem:";
//  pattern += gTotemCode;
  pattern += gTotemString;
  pattern += "?";
//  if (beacon.startsWith("totem:REWRITE?"))
  Serial.print("Checking totem for pattern "); Serial.println(pattern);
  if (beacon.startsWith(pattern))
  {
    if(-1 != beacon.indexOf("state=on")) {
      *stateFromBeacon=true;
    }
    else
    {
      *stateFromBeacon=false;
    }
    return true;
  }
  return false;
}
/*---------------------------------------------------------------------------------------------------------------------*/
/**
 * Pull colors from beacon message 
 * TODO - rewrite?  Too messy?
 *
 * ic=<indicator color code #12346
 */
bool getBeaconColor(String beacon)
{
  int index;
  char color[7]; /* 6 bytes of colors (2red, 2 green, 2 blue) + NULL */
  char tmp[5];
  memset(color,0x0,sizeof(color));

  if (-1 != (index= beacon.indexOf("ic=#")))
  {
    /* we have a color */
    String foo=beacon.substring(index+ 4,index+4+7);  /* now we have color bit */
    foo.toCharArray(color,sizeof(color));
#ifdef DEBUG
    Serial.print("Found color ");
    Serial.println(color);
#endif
    memset(tmp,0x0,sizeof(tmp));
    sprintf(tmp,"0x");

    memcpy(&tmp[2],color,2);
    gRed = strtoul(tmp,NULL,0);

    memcpy(&tmp[2],&color[2],2);    
    gGreen = strtoul(tmp,NULL,0);

    memcpy(&tmp[2],&color[4],2);
    gBlue = strtoul(tmp,NULL,0);
#ifdef DDEBUG
    Serial.print("Found Red: "),Serial.println(gRed,HEX);
    Serial.print("Found Green: "),Serial.println(gGreen,HEX);    
    Serial.print("Found Blue: "),Serial.println(gBlue,HEX);
#endif
  
    return true;    
  }
  else
  {
    return false;
  }

}
/*---------------------------------------------------------------------------------------------------------------------*/
/**
 * Packs beacon message
 *
 * ////totem:<totemID>?proto=<proto version number integer>&state=on|off&ic=<indicator color code #12346&seq=&rid
 */
String packBeacon()
{
  String beacon = "totem:";
//  beacon += "REWRITE";
  //beacon += gTotemCode;
  beacon += gTotemString;
  beacon += "?";
  beacon += "proto=1";
  if ( (eBeaconing_Active_State == gState) || (eListening_Active_State ==gState) )
      beacon += "&state=on";  
  else
      beacon += "&state=off";
      
  beacon += "&ic=#";
  if (gRed < 16)
    beacon+="0";
  beacon += String(gRed,HEX);
  if (gGreen < 16)
    beacon+="0";
  beacon += String(gGreen,HEX);
  if (gBlue < 16)
    beacon+="0";
  beacon += String(gBlue,HEX);
  beacon += "&seq=";
  beacon += String(++gSequenceNum,DEC);
  beacon += "&rid=";
  beacon += seed;
  Serial.print("Packed beacon ");
  Serial.println(beacon);
  return beacon;
}
#if 0
/*---------------------------------------------------------------------------------------------------------------------*/
void ledOff()
{
  analogWrite(RED,0);
  analogWrite(BLUE,0);
  analogWrite(GREEN,0);
  gLEDOn = false;
}
/*---------------------------------------------------------------------------------------------------------------------*/
void ledOn()
{
  analogWrite(RED,gRed);
  analogWrite(BLUE,gBlue);
  analogWrite(GREEN,gGreen);
  gLEDOn = true;

}
#endif
/*---------------------------------------------------------------------------------------------------------------------*/
void turnLED(bool on)
{
  analogWrite(RED,on?gRed:0);
  analogWrite(BLUE,on?gBlue:0);
  analogWrite(GREEN,on?gGreen:0);
  gLEDOn = on;
  
}
/*---------------------------------------------------------------------------------------------------------------------*/
void flashLED(uint8_t count)
{
  bool initLEDState = gLEDOn;
 
  do   
  {
    turnLED(!initLEDState);
    delay(250);
    turnLED(initLEDState);
    delay(250);
    count--;
  }
  while (count);
}
/*---------------------------------------------------------------------------------------------------------------------*/
void setLED()
{
  switch (gState)
  {
  case eBeaconing_Active_State:
  case eListening_Active_State:
    // digitalWrite(LED,HIGH);
    //ledOn();
    turnLED(true);
    break;
  default:
    //digitalWrite(LED,LOW);
    //ledOff();
    turnLED(false);
  } 

}
/*---------------------------------------------------------------------------------------------------------------------*/
void updateState()
{
  if ( (millis() > gTimeLastBeaconRecvd) && ((millis()-gTimeLastBeaconRecvd) > QUIET_TIMEOUT) )
  {
    if (eBeaconing_Active_State == gState)
    {
#ifdef DEBUG
      Serial.println("Silent for too long.  Just listen");
#endif
      setState(eListening_Active_State);
      /* set last beacon again so that we get another QUIET_TIMEOUT in the listening state*/
      gTimeLastBeaconRecvd = millis();
    }
    else if (eListening_Active_State == gState)
    {
#ifdef DEBUG
      Serial.println("Silent for too long.  Going inactive");
#endif
      setState(eBeaconing_Inactive_State);

      calcLED();      /* set new color */
    }
  }
}
/*----------------------------------------------------------------------------------------------------------------------------------------------------*/
void loop()
{
  //uint16_t timeout=0;
  unsigned long tnow;

  /* Programming mode loop */
  /* re-use tnow for monitoring the progamming state */
  tnow = millis();
  while (eProgramming_State == gState)
  {
    if (Serial.available() > 0)    {        
      tnow = millis();  /* reset timer that monitors how long we have been in programming mode */
      handleSerial();
    }  else if (millis() >= (tnow + PROGRAMMING_MODE_DURATION))  {
      /* Leave programming state and start the totem */
      setState(eBeaconing_Inactive_State);
    }
  }
  
  memoryTest();
  
  /***********************/
  /* Main processing loop */
  /* Setup watchdog timer */
  /***********************/
  Serial.println("Enable watchdog");
  MCUSR=0;
  wdt_enable(WDTO_8S);
  
  calcNextBeacon();
  gTimeOfNextBeacon = millis() + random(100,3000);  /* send first message in 100 ms to 3 seconds */
  while (1)
  {
    if (!gbReset)
      wdt_reset();
      
    if (millis() >= gTimeOfNextBeacon ) 
    {
      memoryTest();
      /* If we have sent rotateCounter msgs, change the led color and set new counter */   
      if (0 >= --rotateCounter)
      {
        rotateCounter = random(RAND_ROTATE)+RAND_BASE;
        calcLED();       
      }
      String beacon=packBeacon();
      char  beaconChars[beacon.length()+1];
      beacon.toCharArray(beaconChars,sizeof(beaconChars));
#ifdef DDEBUG
      Serial.print("Sending message of size ");
      Serial.println(beacon.length());
#endif

      //flashLED(2); /* flash light on send */   
      cycleLEDs();   
      
    
      if (grf22.sendto((uint8_t*)beaconChars, beacon.length(),RF22_BROADCAST_ADDRESS)) 
      {
        grf22.waitPacketSent();
        calcNextBeacon();
      }
    }

    updateState();
    setLED();

//    timeout = calcRecvTime();
    

    tnow = millis();
#ifdef DEBUG
    Serial.print("Now is ");    
    Serial.println(tnow);
//    Serial.print("Waiting interval ");    
//    Serial.println(timeout);
#endif

    // Now wait for a reply
    //if (grf22.waitAvailableTimeout(timeout))
    if (grf22.waitAvailableTimeout(random(RECV_WAIT_MIN,RECV_WAIT_MAX)))
    { 
      // Should be a message for us now   
      memset(buf,0x0,sizeof(buf));
      uint8_t len = sizeof(buf)-1;       /*leave NULL at end of buff since we are converting it to a string */
      if (grf22.recv(buf, &len))
      {
        String received = (char*)buf;

        if (validateBeacon(received))
        {
          bool state;
          Serial.print(">>>Got beacon- ");
          
          flashLED(2);
          
          if (isMatchingBeacon(&state,received))
          {
            Serial.println("MATCH"); 
            /*Take on color of sender */
            getBeaconColor(received);   


            if(state) {
              gTimeLastBeaconRecvd=millis();
            }

            switch (gState)
            {
            case eBeaconing_Inactive_State:
              Serial.println("Go active and bark back");
              /* go active and bark back */
              setState(eBeaconing_Active_State);
              /* Force barkback */
              gTimeOfNextBeacon=0;              
              break;

            case eBeaconing_Active_State:
              Serial.println("Reset bark timer");
              /* We heard a matching beacon, so no need to send ours, someone else did */
              calcNextBeacon();     
              break;

            case eListening_Active_State:
              Serial.println("Go active");
              /* go active  */
              setState(eBeaconing_Active_State);
              calcNextBeacon();   
              break;
            } /*case */
            
            setLED();
          }
          else
          {
            Serial.println("no match");       
          }
        }
        dumpBuff(buf,len);
      }

    }

  }
}
/*----------------------------------------------------------------------------------------------------------------------------------------------------*/
void dumpBuff(uint8_t* buf,int len)
{
#ifdef DDEBUG
  //  // Print the data
  uint16_t i, j;
  for (i = 0; i < len; i += 16)
  {
    // Hex
    for (j = 0; j < 16 && i+j < len; j++)
    {
      if (buf[i+j] < 16)
        Serial.print("0"); // Sigh, Serial.print does not know how to pad hex
      Serial.print(buf[i+j], HEX);
      Serial.print(" ");
    }
    // Padding on last block
    while (j++ < 16)
      Serial.print("   ");

    Serial.print("   ");
    // ASCII
    for (j = 0; j < 16 && i+j < len; j++)
      Serial.print(isprint(buf[i+j]) ? buf[i+j] : '.', BYTE);
    Serial.println(""); 
  }   
  Serial.println(""); 
#endif
}

/*----------------------------------------------------------------------------------------------------------------------------------------------------*/
void cycleLEDs()
{
  uint8_t  orig_red   = gRed;
  uint8_t  orig_green = gGreen;
  uint8_t  orig_blue  = gBlue;
 
  for (uint8_t i=0;i<(sizeof(thecolors)/sizeof(thecolors[0]));i++)
  {
    uint32_t color = thecolors[i];

#ifdef DDEBUG
    Serial.print("Testing color: ");  
    Serial.println(color,HEX);
    Serial.print("Index ");
    Serial.println(i,DEC);
#endif

    gBlue   = color;
    gGreen  = (color>>8);
    gRed    = (color>>16);
#ifdef DDEBUG
    Serial.print("Red:   ");
    Serial.println(gRed,HEX);
    Serial.print("Green: ");
    Serial.println(gGreen,HEX);
    Serial.print("Blue:  ");
    Serial.println(gBlue,HEX);
#endif
    //ledOn();
    turnLED(true);
    delay(150);
  }
  gRed = orig_red;
  gGreen = orig_green;
  gBlue = orig_blue;
  turnLED(gLEDOn);
 
}
/*----------------------------------------------------------------------------------------------------------------------------------------------------*/
/**
 * Parse the string that was passed in via the serial port
 *
 * @param cmd  Input string
 *
 * @returns  0 for success, non-zero for error code
 *
 * Note - all responses echo the command to allow matching of a response to a command
 * Command Set
 *
 * AT&S50="string"  where string is the totem Code
 * ATS50? Returns current programmed totem code
 * ATI0 Returns TOTEM
 * ATI1 returns the verison # of software
 */
int  parseCommand(String cmd)
//int  parseCommand(char *pCmd)
{

  int ret=-1;
  gSerialRespString = RESPONSE_PREFIX;
 
  Serial.print("parseCommand parsing");
  //Serial.println(pCmd);
  Serial.println(cmd);
  //if (0 == strncmp(pCmd,SET_TOTEM,strlen(SET_TOTEM)))
  if (cmd.startsWith(SET_TOTEM))
  {
    ret=0;
    gTotemString = cmd.substring(strlen(SET_TOTEM));

    gSerialRespString +=SET_TOTEM;
    gSerialRespString += RESPONSE_PREFIX;
    gSerialRespString +="OK";
    writeTotem();
//    sprintf(gTotemCode,&pCmd[strlen(SET_TOTEM)]);
//    /* strip any extra chars like newlines from totem */
//    char *pendoftotem = &gTotemCode[0];
//    while (isalpha(*pendoftotem))
//      pendoftotem++;
//    *pendoftotem = '\0';
//    sprintf(gSerialResponseMsg,"%s%s%s%s",RESPONSE_PREFIX,SET_TOTEM,RESPONSE_PREFIX,"OK");
  }
  //  else if (cmd.startsWith("ATS50?"))
  else if (cmd.startsWith(GET_TOTEM))
  //else if (0==strncmp(pCmd,GET_TOTEM,strlen(GET_TOTEM)))
  {
      ret=0;
      gSerialRespString+=GET_TOTEM; 
       gSerialRespString+=RESPONSE_PREFIX;
       gSerialRespString += gTotemString; /* Return the totem code in the response */
      //sprintf(gSerialResponseMsg,"%s%s%s%s",RESPONSE_PREFIX,GET_TOTEM,RESPONSE_PREFIX,gTotemCode);
      Serial.print("Totem code is ");
      Serial.println(gTotemString);
  
  }
  //  else if (cmd.startsWith(GET_ID_STRING))
  //  else if (0==strncmp(pCmd,GET_ID_STRING,strlen(GET_ID_STRING)))
  //  {
  //    ret=0;
  //    gSerialResponseMsg+=GET_ID_STRING; 
  //    gSerialResponseMsg+=RESPONSE_PREFIX;
  //    gSerialResponseMsg += "TOTEM"; 
  //  }
  //  //  else if (cmd.startsWith("ATI1"))
  ////  else if (cmd.startsWith(GET_SW_VERSION))
  //  else if (0==strncmp(pCmd,GET_SW_VERSION,strlen(GET_SW_VERSION)))
  //  {
  //    ret=0;
  //    gSerialResponseMsg+=GET_SW_VERSION; 
  //    gSerialResponseMsg+=RESPONSE_PREFIX;
  //    gSerialResponseMsg += VERSION; /* Return the totem code in the response */
  //  }

  if (-1 == ret)
  {
    gSerialRespString +="ERROR";  /* send the error string */
    //sprintf(gSerialResponseMsg,"%s",RESPONSE_PREFIX,"ERROR");
    /* Print the error status and command */
    Serial.print(SERIAL_ERROR_MSG);
    Serial.print(" ");
    Serial.println(cmd);
    //Serial.println(pCmd);
  }

  /* Send message back to serial port */
//  Serial.println(gSerialResponseMsg);
  Serial.println(gSerialRespString);
  /* Return error code */
  return ret;
}
/*-------------------------------------------------------------------------------------------*/
void handleSerial()
{
  int byteread;
//  #define MAX_INPUT_STRING  32
  String inputString;
//  char inputString[MAX_INPUT_STRING+1];
  //int i;

  /* Serial Read Code
   *
   * The buffering is a little odd so we want to make sure that 
   * we allow for pauses to get all the data in.  At 9600 baud, we have
   * 1200 bytes per second.  So, we code to processing below to 
   * look for a period of no-data to indicate that we have the entire command
   *-------------------------------------- */
  if ( Serial.available() > 0)
  {
    //memset(inputString,0x0,sizeof(inputString));
    //    Serial.print("Have bytes on buffer: ");
    //    Serial.println(serialBytes);   
    
    //i=0;
    while ( (byteread=Serial.read()) && (byteread != -1) )
    {
      //inputString[i++] = char(byteread);  /** todo - add code to prevent buffer overrun */

      inputString = inputString + String(byte(byteread));

      /* This can be done better w/out a delay but take the cheap road for now */
      delay(WAIT_FOR_END_OF_DATA);
    }

    /* Clean off any junk from the input */
    inputString = inputString.trim();
    /* Now, we have a full message */
    Serial.print("Received Command ");
    Serial.println(inputString);
    parseCommand(inputString);    
  } 
}

#if 0
// this function will return the number of bytes currently free in RAM
int memoryTest() {
  int byteCounter = 0; // initialize a counter
  byte *byteArray; // create a pointer to a byte array
  // More on pointers here: http://en.wikipedia.org/wiki/Pointer#C_pointers

  // use the malloc function to repeatedly attempt allocating a certain number of bytes to memory
  // More on malloc here: http://en.wikipedia.org/wiki/Malloc
  while ( (byteArray = (byte*) malloc (byteCounter * sizeof(byte))) != NULL ) {
    byteCounter++; // if allocation was successful, then up the count for the next try
    free(byteArray); // free memory after allocating it
  }
  
  free(byteArray); // also free memory after the function finishes
  return byteCounter; // send back the highest number of bytes successfully allocated
}
#endif

uint8_t * heapptr, * stackptr;
int memoryTest() {
  stackptr = (uint8_t *)malloc(4);          // use stackptr temporarily
  heapptr = stackptr;                     // save value of heap pointer
  free(stackptr);      // free up the memory again (sets stackptr to 0)
  stackptr =  (uint8_t *)(SP);           // save value of stack pointer
  Serial.print("DEBUG - free mem is ");
  Serial.println(stackptr-heapptr);
}
