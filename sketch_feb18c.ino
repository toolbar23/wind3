#define NMEA2000_FRAME_ERROR_DEBUG 1
#define NMEA2000_FRAME_IN_DEBUG 1
#define NMEA2000_FRAME_OUT_DEBUG 1
#define NMEA2000_MSG_DEBUG 1
#define NMEA2000_BUF_DEBUG 1
#define NMEA2000_DEBUG 1

#define ENABLE_TEMP true
#define ENABLE_WIND_INPUT true
#define DEBUG_WIND true

#define DEBUG_NMEA2000 false
int tempReceiveCounter = 0;
int lastWindAngleUncorrected = 0;
#define TEMP_SEND_EVERY_NTH 5
#define DEBUG_TEMP false

#define USE_N2K_CAN 1
#define N2k_SPI_CS_PIN 3
#define N2k_CAN_INT_PIN 7
#define USE_MCP_CAN_CLOCK_SET 16



#include <Arduino.h>

#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object


#include <N2kMessages.h>

#include <vector>
#include <numeric>
#include <movingAvg.h>

#define HONEYWELL_PIN A0

movingAvg honeywellSensor(10);  // define the moving average object

using namespace std;


class RotationSensor {
public:
  static int newValue;
  static int oldValue;
};

class WindSensor {
public:
  static double windSpeedKnots;
  static int windAngleDegrees;
};

typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg);
} tNMEA2000Handler;

void WindSpeed(const tN2kMsg &N2kMsg);

tNMEA2000Handler NMEA2000Handlers[] = {
  { 130306L, &WindSpeed },
  { 0, 0 }
};


// Initialize static variables for RotationSensor Class
int RotationSensor::newValue{ 0 };
int RotationSensor::oldValue{ 0 };

double WindSensor::windSpeedKnots{ 0.0 };
int WindSensor::windAngleDegrees{ 0 };
double offset = 0;
int what = 0;

// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM = { 130306L, 0 };  // This is the PGN for Wind

void setup() {

  Serial.begin(115200);
 // while (!Serial);

  Serial1.begin(4800);
 // while (!Serial1);
  Serial.println("connected to wind instrument");
 analogReference(AR_DEFAULT);

  // Set Product information
  NMEA2000.SetProductInformation("00000001",                   // Manufacturer's Model serial code
                                 100,                          // Manufacturer's product code
                                 "Mast Rotation Compensator",  // Manufacturer's Model ID
                                 "1.0",                        // Manufacturer's Software version code
                                 "Paul MRC"                    // Manufacturer's Model version
  );
  // Set device information
  NMEA2000.SetDeviceInformation(112,  // Unique number. Use e.g. Serial number.
                                130,  // Device function=Atmospheric. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                25,   // Device class=External Environment. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046  // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );

  //NMEA2000.SetForwardStream(&Serial);
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode,31);    // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly
  //NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, 31);  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly
  //NMEA2000.EnableForward();
 // NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  NMEA2000.SetN2kCANReceiveFrameBufSize(50);
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.Open();



  pinMode(HONEYWELL_PIN, INPUT);
  analogReference(AR_DEFAULT);
  analogReadResolution(12);

  honeywellSensor.begin();    //Instantiates the moving average object

}



void loop() {

/*
  while (Serial1.available()) {
    // get the new byte:
    char inChar = (char)Serial1.read();
 
    processWindCharacter(inChar);
  }
*/

  NMEA2000.ParseMessages();

  
  double m = readAnalogRotationValue();
  if ( m<0 ) {
    m=360+m;
  }
 
  double windAngleInRad = DegToRad(m);


 debug("se d ") ;
   debug(m);
    debug(" -> ") ;
    debug(windAngleInRad);
  debugln("");
    tN2kMsg N2kMsg;

  
      SetN2kWindSpeed(N2kMsg, 1, 5, windAngleInRad, N2kWind_Apprent);
      NMEA2000.SendMsg(N2kMsg);

      

}

double ReadWindAngle(int rotateout) {
  return DegToRad(rotateout);  //
}

double ReadWindSpeed() {
  return WindSensor::windSpeedKnots;  //Read the parsed windspeed from NMEA stream
}

#define WindUpdatePeriod 1000

int readWindAngleInput() {
  return WindSensor::windAngleDegrees;
}

double readAnalogRotationValue() {  //returns mastRotate value when called. This is in degrees, and corresponds to the current value of the Honeywell sensor

  // Define Constants
  const int lowset = 460;//4095 * 0.1;
  const int highset = 3500;//4095 * 0.9;

delay(200);
//analogRead(HONEYWELL_PIN);  
//delay(250);

  int adc1 = analogRead(HONEYWELL_PIN);   
  debug("adc1 ") ;
  debug(adc1);
  int  oldValue = 0;
  if (adc1<=3650 ) {
 int newValue = honeywellSensor.reading(adc1);  // calculate the moving average
   oldValue = RotationSensor::oldValue;

  if (newValue < 3600) {  //writes value to oldsensor if below highset threshold
    oldValue = newValue;
  }

  // Update values for new and old values (for the next loop iteration)
  RotationSensor::newValue = newValue;
  RotationSensor::oldValue = oldValue;
  } else {
    oldValue = honeywellSensor.getAvg();
  }


  double mastRotate = map(oldValue, lowset, highset, -50, 50);  //maps 12 bit number to degrees of rotation
  debug(" angle ") ;
   debug(mastRotate);
  debugln("");  //reads bitshifted value of Honeywell position
 

  return mastRotate;
}

void WindSpeed(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double windSpeedMetersSeconds;
  double windAngle;
  tN2kWindReference WindReference;

debug("received windspeed ");

  if (ParseN2kWindSpeed(N2kMsg, SID, windSpeedMetersSeconds, windAngle, WindReference)) {
    double windSpeedKnots = windSpeedMetersSeconds * 1.94384;  //maybe * .01?
    int windAngleDegrees = windAngle * 57.2958;                //maybe * .0001?
debug("received windspeed ");
debug(windAngleDegrees);
debugln("");
    // Update Static Object Values for Wind Velocity and Angle
    WindSensor::windSpeedKnots = windSpeedKnots;
    WindSensor::windAngleDegrees = windAngleDegrees;
  }
}

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {  //NMEA 2000 message handler
  int iHandler;
  // Find handler
  //Serial.println("got nmea2k message");
  for (iHandler = 0; NMEA2000Handlers[iHandler].PGN != 0 && !(N2kMsg.PGN == NMEA2000Handlers[iHandler].PGN); iHandler++)
    ;
  if (NMEA2000Handlers[iHandler].PGN != 0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg);
  }
}

void SendN2kWind(int rotateout) {
  static unsigned long WindUpdated = millis();
  tN2kMsg N2kMsg;

  if (WindUpdated + WindUpdatePeriod < millis()) {
    SetN2kWindSpeed(N2kMsg, 1, ReadWindSpeed(), ReadWindAngle(rotateout), N2kWind_Apprent);  // Typo in N2kWindApprent is intentional
    WindUpdated = millis();
    NMEA2000.SendMsg(N2kMsg);
  }
}

String windInputBuffer = "";

void processWindCharacter(char inChar) {

  // add it to the windInputBuffer:
  windInputBuffer += inChar;

  // if the incoming character is a newline, set a flag
  // so the main loop can do something about it:

  int ascii = inChar;

  if (inChar == '\n' || inChar == '\r') {
    processNMEA0183Line(windInputBuffer);
    windInputBuffer = "";
  }
}
void processNMEA0183Line(String s) {
  debugln(s);

  if (ENABLE_TEMP && getValue(s, ',', 0).equals("$WIXDR") && tempReceiveCounter-- == 0) {  // for temp-lines

    tempReceiveCounter = TEMP_SEND_EVERY_NTH;

    String tempString = getValue(s, ',', 2);
    double temp = StringToDouble(tempString);

    if (DEBUG_TEMP) {
      debug("got temp ");
      debugln(temp);
    }

    if (DEBUG_TEMP || DEBUG_NMEA2000) {
      debug("nmea-send temp, temp ");
      debugln(temp);
    }

    tN2kMsg N2kMsg;
    SetN2kTemperature(N2kMsg, 1, 1, N2kts_OutsideTemperature, CToKelvin(temp));
    NMEA2000.SendMsg(N2kMsg);

  }

  else if (ENABLE_WIND_INPUT && getValue(s, ',', 0).equals("$IIMWV")) {  // for wind-lines

    String windAngleString = getValue(s, ',', 1);
    String windSpeedString = getValue(s, ',', 3);

    double windAngle = StringToDouble(windAngleString);
    lastWindAngleUncorrected = windAngle;

    double windSpeed = StringToDouble(windSpeedString);

    double mastAngle = readAnalogRotationValue();


    if (DEBUG_WIND) {
      debug("got wind, sentence ");
      debug(s);
      debug(" speed ");debug(windSpeed);
      debug(", angle uncorrected ");
      debug(windAngle);
      debug(", mastAngle ");
      debug(mastAngle);
      debug(", offset ");
      //   debug(storage.offsetMastSensors);
    }

    windAngle += mastAngle + offset;
    windAngle = fmod(windAngle, 360.0);

    windAngle = round(windAngle);
    if (windAngle < 0) {
      windAngle = 360 + windAngle;
    }

    if (DEBUG_WIND) {
      debug(", calculated ");
      debugln(windAngle);
    }

    tN2kMsg N2kMsg;

    double windAngleInRad = DegToRad(windAngle);
    if (windSpeed > -1) {

      if (DEBUG_WIND || DEBUG_NMEA2000) {
        debug("nmea-send wind, speed ");
        debug(windSpeed);
        debug(", angle ");
        debugln(windAngle);
      }

      SetN2kWindSpeed(N2kMsg, 1, KnotsToMetersPerSecond(windSpeed), windAngleInRad, N2kWind_Apprent);
      NMEA2000.SendMsg(N2kMsg);
      // WindHandler(N2kMsg);
    }
  }
}


double KnotsToMetersPerSecond(double knots) {
  return knots * 0.514444;
}




/*
    Helper
*/



double StringToDouble(String s) {
  char buf[15];  //make this the size of the String
  s.toCharArray(buf, 15);

  char *endptr;
  float num;
  num = strtod(buf, &endptr);
  // if (*endptr != '\0')
  //  return -1;
  return num;
}


String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}


void debugln(String s) {
  Serial.println(s);
}


void debugln(double s) {
  Serial.println(s);
}


void debug(String s) {
  Serial.print(s);
}


void debug(double s) {
  Serial.print(s);
}

