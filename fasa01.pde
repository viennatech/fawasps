/*  
 *  This is My latest attempt at  FA-AG900 
 using codegrabbed from here and there.  
 is working with a waspmote plug and sense smart ag.  Xbee 900 sending frames to meshlium in standardized format
 *    
 */
#include <WaspSensorAgr_v20.h>
#include <WaspFrame.h>
#include <WaspXBee900.h>   
  
// Various Variables described below.
int s = 300;   // Used in SOS Flasher
int o = 800;	// Used in SOS Flasher
int   batteryLevel;  // To hold Battery charge
long  sequenceNumber = 0;      // To count number of loops         
float digitalTemperature;    
float digitalHumidity;      
float anemometer;             
float pluviometer;          
int   vane;
float connectorFFloatValue;

//Xbee 900 Stuff initializes here
packetXBee* packet; 
// PAN (Personal Area Network) Identifier
uint8_t  panID[2] = {0x7F,0xAF}; 
// Define the Encryption mode: 1 (enabled) or 0 (disabled)
uint8_t encryptionMode = 0;
// Define the AES 16-byte Encryption Key
char  encryptionKey[] = "LIBELIUM"; 
// Define the nodeID                           
char  nodeID[10] = "FASA01";              
char* macAddress="0013A200409419CE";  //MAC of the Receiving unit.  In our case a Meshlium
char* sleepTime = "00:00:14:22";    // How long to sleep between measurements
// variable to store RSSI value
int rssi;
// 900 send retries counter
int retries=0;
// maximum number of retries when sending
#define MAX_RETRIES 3  
  
void setup() {
  // 0. Init USB port for debugging
  USB.ON();
  RTC.ON();
  USB.print(F("Initializing - FA_Agri_900_201  at:  "));
 USB.println(RTC.getTime());
USB.println(F(" "));

 // Check battery level: if below 30% wait here checking every 30 minutes

  do
  {    
    PWR.getBatteryLevel();
    // Getting Battery Level
    batteryLevel = PWR.getBatteryLevel();
    USB.printf("Battery Level: %d\r\n",batteryLevel);

    if (batteryLevel > 29)
    {
      break; 
    }
    else
    {
      USB.println(F("Waiting for battery to charge to 30%, sleep for 30 minutes"));
      sosFlasher();  //To flash the LED prior to sleeping.  
      PWR.deepSleep("00:00:30:00",RTC_OFFSET,RTC_ALM1_MODE1,ALL_OFF);
      //re-enable ports
      USB.ON();
      RTC.ON();
    }
  } 
  while (1);
  //  end of Battery test section.
  
frame.setID(nodeID);  //Creating the frame now using the frame class and node id specified above.

// 1.2 Set Frame Size (Link encryp disabled, AES encryp disabled)
  frame.setFrameSize(XBEE_900, DISABLED, DISABLED);
  USB.print(F("\nframe size (900, UNICAST_64B, XBee encryp Disabled, AES encryp Disabled):"));
  USB.println(frame.getFrameSize(),DEC);  
  USB.println(); 
  
  // 1.3 Create new frame
  frame.createFrame(ASCII);  

  // 1.4 Set frame fields (String - char*)
  frame.addSensor(SENSOR_STR, (char*) "Reporting for duty at:");
  frame.addSensor(SENSOR_STR,(RTC.getTime()));
  frame.addSensor(SENSOR_BAT, batteryLevel);
  
  // 1.5 Print frame
  frame.showFrame();


  ////////////////////////////////////////////////
  // 2. Send initial message
  ////////////////////////////////////////////////

  // 2.1 Switch on the XBee module
  xbee900.ON();  
  delay(400);

  // 2.2 Memory allocation
  packet = (packetXBee*) calloc(1,sizeof(packetXBee));

  // 2.3 Choose transmission mode: UNICAST or BROADCAST
  packet -> mode=UNICAST;

  // 2.4 Set destination XBee parameters to packet
  xbee900.setDestinationParams(packet, macAddress, frame.buffer, frame.length); 

  // 2.5 Initial message transmission
  xbee900.sendXBee(packet);

// 2.6 Check TX flag
  if ( xbee900.error_TX == 0 ) 
  {
    // Getting RSSI using the API function
    // This function returns the last received packet's RSSI
    xbee900.getRSSI();
    delay(100);
    //get rssi from getRSSI function and make conversion
      rssi = xbee900.valueRSSI[0];
      rssi *= -1;  
      
    USB.print("ok with RSSI(dBm): ");
    USB.println(rssi,DEC);
	okFlasher();
  }
  else 
  {
    USB.println(F("error in initial packet send"));
	sosFlasher();
  }

  // 2.7 Free memory
  free(packet);
  packet=NULL;

  // 2.8 Communication module to OFF
  xbee900.OFF();
  delay(100);
 
}

void character(int speed) {
	Utils.setExternalLED(LED_ON); 
	delay(speed);
	Utils.setExternalLED(LED_OFF);
	delay(300);
}

void loop() {
 
  
  
  //Turn on the sensor board
    SensorAgrv20.ON();
	//supply stabilization delay
    delay(1000);
    //Turn on the RTC
    RTC.ON();

// Step 9. Turn on the sensors

    //En el caso de la placa de eventos no aplica

    SensorAgrv20.setSensorMode(SENS_ON, SENS_AGR_SENSIRION);
    delay(1000); // to prevent power surge allow them time to come on.

//    SensorAgrv20.setSensorMode(SENS_ON, SENS_AGR_ANEMOMETER);
//    delay(1000); // to prevent power surge allow them time to come on.

    SensorAgrv20.setSensorMode(SENS_ON, SENS_AGR_LEAF_WETNESS);

//  now a loop added to warm up the sensors
 USB.println(F("Warming sensors"));
  unsigned long previous = millis();
  while (millis() - previous < 10000)
  {
    USB.print(".");
    delay(2000);
    // Condition to avoid an overflow (DO NOT REMOVE)
	if (millis() < previous)
	{
	  previous = millis();	
	}   
  }

// Step 10. Read the sensors

    

    // First dummy reading for analog-to-digital converter channel selection
    PWR.getBatteryLevel();
    // Getting Battery Level
    batteryLevel = PWR.getBatteryLevel();
  
    //Sensor temperature reading
    digitalTemperature = SensorAgrv20.readValue(SENS_AGR_SENSIRION, SENSIRION_TEMP);
   
    //Sensor humidty reading
    digitalHumidity = SensorAgrv20.readValue(SENS_AGR_SENSIRION, SENSIRION_HUM);
   
    //Sensor temperature reading
 //   anemometer = SensorAgrv20.readValue(SENS_AGR_ANEMOMETER);
   
    //Sensor temperature reading
//    pluviometer = SensorAgrv20.readValue(SENS_AGR_PLUVIOMETER);
   
    //Sensor temperature reading
//    vane = SensorAgrv20.readValue(SENS_AGR_VANE);
 
    //Sensor wetness reading
    connectorFFloatValue = SensorAgrv20.readValue(SENS_AGR_LEAF_WETNESS);
  
// Step 11. Turn off the sensors

    //En el caso de la placa de eventos no aplica

    SensorAgrv20.setSensorMode(SENS_OFF, SENS_AGR_SENSIRION);

  //  SensorAgrv20.setSensorMode(SENS_OFF, SENS_AGR_ANEMOMETER);

    SensorAgrv20.setSensorMode(SENS_OFF, SENS_AGR_LEAF_WETNESS);

 ////////////////////////////////////////////////
  // 4. Message composition
  ////////////////////////////////////////////////

  // 4.1 Create new frame
  frame.createFrame(ASCII);  

  // 4.2 Add frame fields
       frame.addSensor(SENSOR_BAT, batteryLevel);
//        frame.addSensor(SENSOR_TIME, RTC.hour, RTC.minute, RTC.second );
	frame.addSensor(SENSOR_TCB, digitalTemperature);
	frame.addSensor(SENSOR_HUMB, digitalHumidity);
//	frame.addSensor(SENSOR_ANE, anemometer);
//	frame.addSensor(SENSOR_PLV, pluviometer);
//	frame.addSensor(SENSOR_WV,(uint8_t) SensorAgrv20.vaneDirection);
        frame.addSensor( SENSOR_WV, vane );
	frame.addSensor(SENSOR_LW ,(float) connectorFFloatValue);
        frame.addSensor(SENSOR_IN_TEMP, (float) RTC.getTemperature());

 /* 

Current ASCII Frame:
Length: 99
Frame Type:  128
frame (HEX): 3C3D3E80082334303333363932303823464153414F4E452332234241543A393323494E5F54454D503A32382E3530235443423A32332E37302348554D423A32372E3523414E453A302E303023504C563A302E30302357563A33234C573A322E36363523
frame (STR): <=>#403369208#FASAONE#2#BAT:93#IN_TEMP:28.50#TCB:23.70#HUMB:27.5#ANE:0.00#PLV:0.00#WV:3#LW:2.665#
*/
  frame.showFrame();  // to display it in the serial monitor
 //Switch on the XBee module
  xbee900.ON();  
Utils.setExternalLED(LED_ON);  //turn on light to show the packet is being sent.
    //supply stabilization delay
    delay(800);
   // 5.2 Memory allocation
  packet = (packetXBee*) calloc(1,sizeof(packetXBee));

  // 5.3 Choose transmission mode: UNICAST or BROADCAST
  packet -> mode = UNICAST;

  // 5.4 Set destination XBee parameters to packet
  xbee900.setDestinationParams( packet, macAddress, frame.buffer, frame.length);  

  // 5.5 Send XBee packet
  xbee900.sendXBee(packet);
  Utils.setExternalLED(LED_OFF);  //turn off light to make it appear to blink
   
  // 5.5.1 retry sending if necessary for a maximum of MAX_RETRIES
  retries=0;
  while( xbee900.error_TX != 0 ) 
  {
    if( retries >= MAX_RETRIES )
    {
     sosFlasher();
	 delay(250);
	 sosFlasher();
      break;
    }
    
    retries++;

 
    xbee900.sendXBee(packet);          
  }
  

  // 5.6 Check TX flag 
  if( xbee900.error_TX == 0 ) 
  {
       // Getting RSSI using the API function
    // This function returns the last received packet's RSSI
    xbee900.getRSSI();
    delay(100);
    //get rssi from getRSSI function and make conversion
      rssi = xbee900.valueRSSI[0];
      rssi *= -1;  
      
    USB.print("ok with RSSI(dBm): ");
    USB.println(rssi,DEC);
    okFlasher();
  }
  else 
  {
    USB.println(F("error"));
 sosFlasher();
 }

  // 5.7 Free memory
  free(packet);
  packet = NULL;

  // 5.8 Communication module to OFF
  xbee900.OFF();
  delay(100);
//Increase the sequence number after wake up
    sequenceNumber++;
USB.println(sequenceNumber);
if (sequenceNumber > 100)  //to force a re-init every so often.  
    {
      PWR.reboot(); 
    }
    else
  {}
    PWR.deepSleep(sleepTime,RTC_OFFSET,RTC_ALM1_MODE1,ALL_OFF);
    //Increase the sequence number after wake up
    

}

void sosFlasher(){
  // A nicer way to flash S.O.S. 
  for (int x = 1; x <= 3; x++) {
    character(s);
  }
  delay(100);
  for (int x = 1; x <= 3; x++) {
    character(o);
  }
  delay(100);
  for (int x = 1; x <= 3; x++) {
    character(s);
  }
  delay(2000);
}
void okFlasher(){
  Utils.setExternalLED(LED_ON);
    delay(2000);
 Utils.setExternalLED(LED_OFF);
 delay(2000);
 Utils.setExternalLED(LED_ON);
    delay(2000);
 Utils.setExternalLED(LED_OFF);
 delay(4000);
}
