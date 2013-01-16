//This LCD03 library is a version I modified from the original
//with extra options.  I will supply this library in the zip.
#include <LCD03_I2C.h>
#include <Metro.h>  //used by the Rush KV code.
//Config.h, GlobalVariables.h, and Serial.INO are just copies 
//of the files taken from the Rush_KV_2_1 code project by Kataventos.
//link - http://code.google.com/p/rush-osd-development/downloads/list
//It is the on-board OSD which is making the requests for GPS positions and 
//then relaying it through to the OSD's second Tx out pin to the telemetry module.
//Without this code running on either a Rushduino OSD or MinimOSD, you 
//will need to set up this tracker code to make the MWP calls via two way telemetry, i.e. -
//blankserialRequest(MSP_RAW_GPS) will return the required vars for GPS info
//blankserialRequest(MSP_ATTITUDE) will return the drone heading (just for display for now)
//blankserialRequest(MSP_ALTITUDE) will return the drone altitude (baro returns ground offset alt in cm)
#include "Config.h"
#include "GlobalVariables.h"
#include <Servo.h>

//#define TEST_MOTION

LCD03_I2C LCD(0x63,20,4);

Servo headingServo;
Servo altangleServo;

boolean Home_Captured = false;
boolean offsetCaptured = false;

String Setup_Field = "   ";
String Setup_State = "INIT";

int Button_Delay_Count = 0;
int Button_Hold_Count = 0;

long LCD_Timer = 0;
long GPS_Timer = 0;

//Tweak these PWM values to get the servo travel you require.
//You will notice when your min/max value is past what your servo supports
//when it twitches or just stops moving when using #define TEST_MOTION (above)
//Note: I was able to get extended ranges from the digital servos I used.
//Also, if the heading servo gear combo cannot reach full +/- 180 then you may
//need to define extra vars for physical max/min servo movement rather than just the
//potential 180 deg throws defined by the current min/max below.  Apply these to the 'if' tests
//for max min PWM.
int H_PWM_midpoint = 1450;
int max_H_PWM = 2350;
int min_H_PWM = 550;
int current_H_PWM = H_PWM_midpoint;

int I_PWM_midpoint = 1480;
int max_I_PWM = 2340;
int min_I_PWM = 620;
int current_I_PWM = I_PWM_midpoint;

float GPS_lat = 0;
float GPS_lon = 0;

int32_t Home_latitude_int = 0;
int32_t Home_longitude_int = 0;
float Home_latitude = 0;
float Home_longitude = 0;

float heading = 0;
float hdgOffset = 0;
float distance = 0;
float angle = 0;


void setup() {

  LCD.Clear();
  LCD.Backlight_On();
  LCD.PrintChar(1, 1, "MultiWii 2.1 Tracker");

  Serial.begin(115200);

  headingServo.attach(6);
  altangleServo.attach(9);
  
  headingServo.writeMicroseconds(current_H_PWM);
  altangleServo.writeMicroseconds(current_I_PWM);

  pinMode(3, INPUT);  //Offset Left Button
  pinMode(4, INPUT);  //Offset Right Button
  pinMode(5, INPUT);  //Capture Offset Heading.  This sets the difference between the current angle and zero angle
  pinMode(13, OUTPUT);  //Feedback LED
  
  digitalWrite(3, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);
  
  //init vars
  MwAltitude = 0;
  MwHeading = 0;
  GPS_fix = false;
  GPS_latitude = 0;
  GPS_longitude = 0;
  GPS_altitude = 0;  
      
  delay(500);
  
  #ifdef TEST_MOTION
    test_servo_motion()
  #endif
  
}

void loop(){

  //read MSP stream
  serialMSPreceive();
   
  ++GPS_Timer;
  //controls frequency of GPS/servo processing updates
  if (GPS_Timer == 1000) {
    if (Setup_State == "INIT") {
      if (GPS_latitude != 0) {
        GPS_lat = (float)GPS_latitude / 10000000;
        GPS_lon = (float)GPS_longitude / 10000000;  
      }
      //if button pressed, set current LAT LON as home pos
      if ((digitalRead(5) == LOW) && (GPS_latitude != 0)) {
        Home_latitude_int = GPS_latitude;
        Home_longitude_int = GPS_longitude;
        
        //Illuminate LED to indicate that the home position has been captured
        digitalWrite(13, HIGH);
        Setup_State = "HOME_CAPTURED";
        Setup_Field = "LAT";
        delay(1000);
      }  
    }
    
    if (Setup_State == "HOME_CAPTURED") {
      //In this state you can modify the home lat lon if you made a record in Google Maps where the exact 
      //coordinate would be for your antenna.
      //Toggle switch to increase or decrease home LAT/LON int32_t by 1 (float rounding issue will be visible on LCD)
      if (Setup_Field == "LAT") {
        if (digitalRead(3) == LOW) {
          Home_latitude_int += 1;
        }
        if (digitalRead(4) == LOW) {
          Home_latitude_int -= 1;
        }
      } else if (Setup_Field == "LON") {
        if (digitalRead(3) == LOW) {
          Home_longitude_int += 1;
        }
        if (digitalRead(4) == LOW) {
          Home_longitude_int -= 1;
        }
      }

      //button press to alternate between LAT or LON to modify the home position  
      //Long button press to indicate the home LAT LON is correct
      if (digitalRead(5) == LOW) { 
        ++Button_Hold_Count;
        LCD.PrintChar(19,1,"  ");
      } else {
        if (Button_Hold_Count > 0) {
          if (Button_Hold_Count < 10) {
            //Toggle LAT LON
            if (Setup_Field == "LAT") {
              Setup_Field = "LON";
            } else {
              Setup_Field = "LAT";
            }
          } else if (Button_Hold_Count >= 10) {
            Setup_Field = "   ";
            Setup_State = "HOME_SET";
            //Toggle LED to indicate confirmation
            digitalWrite(13, LOW);
            delay(250);
            digitalWrite(13, HIGH);
            delay(500);
          }    
          Button_Hold_Count=0;
        }
      }

      Home_latitude = (float)Home_latitude_int / 10000000;
      Home_longitude = (float)Home_longitude_int / 10000000;
      
    }
    
    if (Setup_State == "HOME_SET") {    
      //drone is moved over 20m directly ahead of tracker zero angle and button pressed to set the offset. 
      
      Home_latitude = (float)Home_latitude_int / 10000000;
      Home_longitude = (float)Home_longitude_int / 10000000;
      
      if (GPS_latitude != 0) {
        GPS_lat = (float)GPS_latitude / 10000000;
        GPS_lon = (float)GPS_longitude / 10000000;
      }
      
      heading = calc_bearing(Home_latitude,Home_longitude,GPS_lat,GPS_lon);
      float turn_angle = 0;
      
      if ((heading <= 360) && (heading >= 180)) {
        turn_angle = -360 + heading;
      } else {
        turn_angle = heading;
      }   
      
      //Button press confirms you have moved the drone directly out by 20m or more 
      //and you are ready to start tracking
      if (digitalRead(5) == LOW) {
        hdgOffset = turn_angle;
        Setup_State = "READY";
        //2 sec delay after you hit the button in case it moves
        delay(2000);
      }
    }
    
    if (Setup_State == "READY") {     
      //Home pos and offset are defined so now the gimbals can be moved.
      if (GPS_latitude != 0) {
        GPS_lat = (float)GPS_latitude / 10000000;
        GPS_lon = (float)GPS_longitude / 10000000;   
      } 
      //distance drone is from home pos
      distance = calc_dist(Home_latitude,Home_longitude,GPS_lat,GPS_lon);
      
      //inclination calculations
      angle = 180 - Inclination(distance,((float)MwAltitude/100),0,0); 
      float inclination_PWM_amount = 0;
      if (angle > 0) {
        inclination_PWM_amount = (angle / 90) * (max_I_PWM - I_PWM_midpoint);
      } else if (angle < 0) {
        inclination_PWM_amount = (angle / 90) * (I_PWM_midpoint - min_I_PWM);
      } 
      
      current_I_PWM = I_PWM_midpoint - (int)inclination_PWM_amount;
      
      if (current_I_PWM < min_I_PWM) {
        current_I_PWM = min_I_PWM;
      }
      
      if (current_I_PWM > max_I_PWM) { 
        current_I_PWM = max_I_PWM;
      }
      
      //heading calculations
      heading = calc_bearing(Home_latitude,Home_longitude,GPS_lat,GPS_lon);
      
      float turn_angle = 0;
      
      if ((heading <= 360) && (heading >= 180)) {
        turn_angle = -360 + heading;
      } else {
        turn_angle = heading;
      }
      
      float heading_PWM_amount = 0;
      if (turn_angle > 0) {
        heading_PWM_amount = (turn_angle/180)* ((float)H_PWM_midpoint - (float)min_H_PWM);
      } else if (turn_angle < 0) {
        heading_PWM_amount = (turn_angle/180)* ((float)max_H_PWM - (float)H_PWM_midpoint);
      }
      
      current_H_PWM = H_PWM_midpoint - (int)heading_PWM_amount;
      
      if (current_H_PWM < min_H_PWM) {
        current_H_PWM = min_H_PWM;
      }
      
      if (current_H_PWM > max_H_PWM) { 
        current_H_PWM = max_H_PWM;
      }
      
      //set servo positions
      altangleServo.writeMicroseconds(current_I_PWM); 
      headingServo.writeMicroseconds(current_H_PWM); 
      
      //adjust heading offset when toggle switch is left or right
      if (digitalRead(3) == LOW) {
        hdgOffset += 1;
      }
      
      if (digitalRead(4) == LOW) {
        hdgOffset -= 1;
      }
    }

    GPS_Timer=0;
  }
  
  
  ++LCD_Timer;
  //controls frequency of LCD updates
  if (LCD_Timer == 2000) {
    LCD.PrintChar(1, 1, "#S:      ");
    LCD.PrintInt(4, 1, GPS_numSat, 10);
    LCD.PrintChar(8, 1, "S:       ");
    LCD.PrintInt(10, 1, GPS_speed, 10);
    LCD.PrintChar(15, 1, "H:       ");
    LCD.PrintInt(17, 1, MwHeading, 10);
      
    if ((Setup_State == "INIT") || (Setup_State == "HOME_CAPTURED")) {
      
      if (Setup_Field == "LAT") {
        LCD.PrintChar(1, 3, "LAT                 ");
        LCD.PrintFloat(5, 3, Home_latitude, 10,6);
        LCD.PrintChar(1, 4, "LON                 ");
        LCD.PrintFloat(5, 4, Home_longitude, 10,6);    
        LCD.PrintChar(20, 3, "<");
        LCD.PrintChar(20, 4, " ");
      }
      
      if (Setup_Field == "LON") {
        LCD.PrintChar(1, 3, "LAT                 ");
        LCD.PrintFloat(5, 3, Home_latitude, 10,6);
        LCD.PrintChar(1, 4, "LON                 ");
        LCD.PrintFloat(5, 4, Home_longitude, 10,6);  
        LCD.PrintChar(20, 3, " ");
        LCD.PrintChar(20, 4, "<");
      }
      
      if (Setup_Field == "   ") {
        LCD.PrintChar(1, 3, "LAT                 ");
        LCD.PrintFloat(5, 3, GPS_lat, 10,6);
        LCD.PrintChar(1, 4, "LON                 ");
        LCD.PrintFloat(5, 4, GPS_lon, 10,6);
      }

    } else {
      LCD.PrintChar(1, 2, "ALT       ");
      LCD.PrintInt(5, 2, MwAltitude, 10);
      LCD.PrintChar(9, 2, "cm");     
      LCD.PrintChar(11, 2, "INC       ");
      LCD.PrintFloat(15, 2, angle, 4, 1);   

      LCD.PrintChar(1, 3, "DST       ");
      LCD.PrintFloat(5, 3, distance, 4, 1);
      LCD.PrintChar(9, 3, "m");
      LCD.PrintChar(11, 3, "HDG       ");
      LCD.PrintFloat(15, 3, heading, 4, 1);
      
      if (Setup_State = "READY") 
      {
        LCD.PrintChar(1, 4, "OFFSET :            ");
        LCD.PrintFloat(10, 4, hdgOffset, 4,1);
      } else {
        LCD.PrintChar(1, 4, "                    ");
      }
    }
    
    LCD_Timer = 0;
  }
}


float Inclination(float From_X, float From_Y, float To_X, float To_Y) {

  float DistanceX;
  float DistanceY;
  float const NotQuiteZero = 0.00001;
  long const Ninety = 90;
  long const TwoSeventy = 270;
  float const RadsToDegs = 57.29578;
    
  //Given two points, return the angle of the line from P1 to P2.
  if (From_Y == To_Y)  { 
    To_Y = From_Y - NotQuiteZero; //Prevent div-by-zero error.
  }
  if (To_Y < From_Y) { 
    return ((float)Ninety + atan((To_X - From_X) / (To_Y - From_Y)) * RadsToDegs);
  } else {
    return ((float)TwoSeventy + atan((To_X - From_X) / (To_Y - From_Y)) * RadsToDegs);
  }
}

float calc_bearing(float flat1, float flon1, float flat2, float flon2) {
  float calc;
  float bear_calc;

  float x = 69.1 * (flat2 - flat1); 
  float y = 69.1 * (flon2 - flon1) * cos(flat1/57.3);
 
  calc=atan2(y,x);

  bear_calc= degrees(calc);

  if(bear_calc<=1) {
    bear_calc=360+bear_calc; 
  }
  
  //set offset
  bear_calc = bear_calc - hdgOffset;
  if (bear_calc > 360) {
    bear_calc = bear_calc - 360;
  } else if (bear_calc < 0) {
    bear_calc = bear_calc + 360;
  }
  
  return bear_calc;
}


float calc_dist(float flat1, float flon1, float flat2, float flon2) {
 float x = 69.1 * (flat2 - flat1); 
 float y = 69.1 * (flon2 - flon1) * cos(flat1/57.3);
 
 return (float)sqrt((float)(x*x) + (float)(y*y))*1609.344; 
}

float to_float_6(long value) 
{
  return (float)value/(float)1000000;
}


void test_servo_motion() {
  //test inclination servo range
  while(current_I_PWM > min_I_PWM) {
    current_I_PWM--;
    altangleServo.writeMicroseconds(current_I_PWM);
    delay(2);
  } 

  while(current_I_PWM < max_I_PWM) {
    current_I_PWM++;
    altangleServo.writeMicroseconds(current_I_PWM);
    delay(2);
  } 

  while(current_I_PWM > I_PWM_midpoint) {
    current_I_PWM--;
    altangleServo.writeMicroseconds(current_I_PWM);
    delay(2);
  }  
  delay(1000);
  
  //test heading servo range
  while(current_H_PWM > min_H_PWM) {
    current_H_PWM--;
    headingServo.writeMicroseconds(current_H_PWM);
    delay(2);
  } 
  
  while(current_H_PWM < max_H_PWM){
    current_H_PWM++;
    headingServo.writeMicroseconds(current_H_PWM);
    delay(2);
  } 

  while(current_H_PWM > H_PWM_midpoint){
    current_H_PWM--;
    headingServo.writeMicroseconds(current_H_PWM);
    delay(2);
  } 
  delay(1000);
  
}




