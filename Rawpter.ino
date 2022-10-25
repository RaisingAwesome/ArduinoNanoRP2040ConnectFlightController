//Rawpter 1.0 by Sean J. Miller
//Flight Controller code for an Arduino Nano RP2040 Connect
//Go to https://raisingawesome.site/projects for more info
//MIT License - use for any purpose you want

#include <SPI.h>
#include <WiFiNINA.h> //WiFi
#include <Arduino_LSM6DSOX.h> //IMU
#include <Servo.h>
#include <MadgwickAHRS.h>

//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //                           
//========================================================================================================================//
//EASYCHAIR is used to put in some key dummy variables so you can sit in the easy chair with just the Arduino on a cord wiggling it around and seeing how it responds.
//Set EASYCHAIR to false when you are wanting to install it in the drone.
#define EASYCHAIR false
//UPONLYMODE is used to take the radio out of it other than thrust.  The intent is to get it tuned so it can at least take off.  Should be false once tuned.
#define UPONLYMODE false
//The LOOP_TIMING is based on the IMU.  For the Arduino_LSM6DSOX, it is 104Hz.  So, the loop time is set a little longer so the IMU has time to update from the control change.
#define LOOP_TIMING 100

//The battery alarm code handles 14.8 or 7.4V LIPOs.  Set to your type of battery. If not hooked up, it will pull down and beep often.
#define BATTERYTYPE 14.8
int batteryVoltage=1023; //just a default for the battery monitoring routine 

//Radio failsafe values for every channel in the event that bad reciever data is detected.
//These are for it to stay stable and descend safely versus totally cutting throttle and drop like a rock.
unsigned long PWM_throttle_zero = 1000; //used when we want to take throttle to zero.  Failsafe is something higher as it is expected that failsafe is a value needed to safely land.
unsigned long PWM_throttle_fs = 1000; //throttle  will allow it to descend to the ground if you adjust the throttlecutswitch_fs to 2000
unsigned long PWM_roll_fs = 1500; //ail pretty much in the middle so it quits turning
unsigned long PWM_elevation_fs = 1500; //elev
unsigned long PWM_rudd_fs = 1500; //rudd
unsigned long PWM_ThrottleCutSwitch_fs = 1000; //SWA less than 1300, cut throttle - must config a switch to Channel 5 in your remote.

bool failsafed=false;
//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
float AccErrorX = 0.01;
float AccErrorY = -0.01;
float AccErrorZ = 0.01;
float Accel_filter=0.14;
float GyroErrorX = 0.42;
float GyroErrorY= 0.07;
float GyroErrorZ = -0.05;
float Gyro_filter = 0.14;

//Controller parameters (take note of defaults before modifying!): 
float i_limit = 100.0;     //Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 30.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode (default 30.0)
float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode (default 30.0)
float maxYaw = 160.0;     //Max yaw rate in deg/sec (default 160.0)

float Kp_roll_angle = 0.1;    //Roll P-gain - angle mode default .2
float Ki_roll_angle = 0.1;    //Roll I-gain - angle mode default .3
float Kd_roll_angle = 0.05;   //Roll D-gain - angle mode default 0.05

float Kp_pitch_angle = 0.1;   //Pitch P-gain - angle mode default .2
float Ki_pitch_angle = 0.1;   //Pitch I-gain - angle mode default .3
float Kd_pitch_angle = 0.05;  //Pitch D-gain - angle mode default .05

float Kp_yaw = 0.3;           //Yaw P-gain default .3
float Ki_yaw = 0.05;          //Yaw I-gain default .05
float Kd_yaw = 0.00015;       //Yaw D-gain default .00015 (be careful when increasing too high, motors will begin to overheat!)

//========================================================================================================================//
//                                                     DECLARE PINS                                                       //                           
//========================================================================================================================//                                          
//Radio:
//used so I quit getting confused begween the ARduino pins and the Radio Channel pins.
const int stickRightHorizontal=2; //right horizontal stick
const int stickRightVertical=3; //right vertical stick
const int stickLeftVertical=4; //left vertical stick
const int stickLeftHorizontal=5; //left horizontal stick
const int SwitchA=6; //SWA switch

const int throttlePin = stickLeftVertical; //throttle - up and down on the 
const int rollPin = stickRightHorizontal; //ail (roll)
const int upDownPin = stickRightVertical; //ele
const int ruddPin = stickLeftHorizontal; //rudd
const int throttleCutSwitchPin = SwitchA; //gear (throttle cut)

//variables for reading PWM from the radio receiver
unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6; 
unsigned long channel_1_raw=0;
unsigned long channel_2_raw=0;
unsigned long channel_3_raw=0;
unsigned long channel_4_raw=0;
unsigned long channel_5_raw=0;

int ppm_counter = 0;
unsigned long time_ms = 0;
int throttleCutCounter=0;
int throttleNotCutCounter=0;
//Motor Electronic Speed Control Modules (ESC):
const int m1Pin = 10; //10
const int m2Pin = 15; //15
const int m3Pin = 16; //16
const int m4Pin = 14; //14
Servo m1PWM, m2PWM, m3PWM, m4PWM;

//========================================================================================================================//
//DECLARE GLOBAL VARIABLES
//========================================================================================================================//

//General stuff for controlling timing of things
float deltaTime; float mt=0;
float invFreq = (1.0/LOOP_TIMING)*1000000.0;

unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
bool beeping=false; //for beeping when the battery is getting low.
bool throttle_is_cut=true; //used to force the pilot to manually set the throttle to zero after the switch is used to throttle cut

//Radio communication:
unsigned long PWM_throttle,PWM_roll, PWM_Elevation, PWM_Rudd, PWM_ThrottleCutSwitch;
unsigned long PWM_throttle_prev,PWM_roll_prev, PWM_Elevation_prev, PWM_Rudd_prev;

//IMU:
Madgwick IMU_Data; //the library tool that will convert the IMU data to angular data
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;

//Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;

//Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

//Mixer
float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled;
int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM;
unsigned long buzzer_millis; unsigned long buzzer_spacing=10000;

//WiFi Begin
int keyIndex = 0;
int status = WL_IDLE_STATUS;
bool ALLOW_WIFI=true;

WiFiServer server(80);
//WiFi End

//========================================================================================================================//
//BEGIN THE CLASSIC SETUP AND LOOP
//========================================================================================================================//

void setup() {
  //Bootup operations 
  IMU_Data.begin(LOOP_TIMING); //initiate the calculations to determine the drone attitude angles
  setupSerial();
  setupBatteryMonitor();
  setupDrone();
  if (ALLOW_WIFI) setupWiFi(); //At first power on, a WiFi hotspot is set up for talking to the drone. (SSID Rawpter, 12345678)
}

void loop() {
  tick(); //stamp the start time of the loop to keep our timing to 2000Hz.  See tock() below.
  //loopBuzzer();
  //The main thread that infinitely loops - poling sensors and taking action.
  
  if (ALLOW_WIFI) loopWiFi();
  loopDrone();
  tock(LOOP_TIMING);
}

void setupDrone() {  
  //Initialize all pins

  pinMode(m1Pin, OUTPUT);
  pinMode(m2Pin, OUTPUT);
  pinMode(m3Pin, OUTPUT);
  pinMode(m4Pin, OUTPUT);
  m1PWM.attach(m1Pin,1060,1860);
  m2PWM.attach(m2Pin,1060,1860);
  m3PWM.attach(m3Pin,1060,1860);
  m4PWM.attach(m4Pin,1060,1860);
  //Set built in LED to turn on to signal startup
  delay(5);

  //Initialize radio communication
  radioSetup();
  setToFailsafe();
  PWM_throttle = PWM_throttle_zero; //zero may not necessarily be the failsafe, but on startup we want zero.
  
  //Initialize IMU communication
  IMUinit();

  //Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
  //calculate_IMU_error(); //Calibration parameters printed to serial monitor. Paste these in the user specified variables section, then comment this out forever.

  if (getRadioPWM(1)>1800&&getRadioPWM(1)<2400&!EASYCHAIR) calibrateESCs(); //if the throttle is up, first calibrate before going into the loop
  
  m1_command_PWM = 0; //Will send the default for motor stopped for Simonk firmware
  m2_command_PWM = 0;
  m3_command_PWM = 0;
  m4_command_PWM = 0;

  while (getRadioPWM(1)>1060&&getRadioPWM(1)<2400&!EASYCHAIR&&getRadioPWM(5)<1300) //wait until the throttle is turned down and throttlecut switch is not engaged before allowing anything else to happen.
  {
    delay(1000);
  }
  //calibrateAttitude(); This is only good if you are sure to start level.
}

void loopDrone() {
  getIMUdata(); //Pulls raw gyro andaccelerometer data from IMU and applies LP filters to remove noise
  Madgwick6DOF(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ); //Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)
  getDesiredAnglesAndThrottle(); //Convert raw commands to normalized values based on saturated control limits
  PIDControlCalcs(); //The PID functions. Stabilize on angle setpoint from getDesiredAnglesAndThrottle
  controlMixer(); //Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
  scaleCommands(); //Scales motor commands to 0-1
  throttleCut(); //Directly sets motor commands to off based on channel 5 being switched
  Troubleshooting(); //will do the print routines that are uncommented
  commandMotors(); //Sends command pulses to each ESC pin to drive the motors
  getRadioSticks(); //Gets the PWM from the radio receiver
  failSafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup
}

void setupBatteryMonitor()
{
  buzzer_millis=millis();  
  pinMode(9,OUTPUT);
  pinMode(A6,INPUT);
}

void loopBuzzer()
{ //this monitors the battery.  the lower it gets, the faster it beeps.
  
  if (!beeping){    
    if ( millis()-buzzer_millis>(buzzer_spacing) )
    {
        digitalWrite(9,HIGH);
        beeping=true;
        buzzer_millis=millis();
    }
  } else
  {
    if (millis()-buzzer_millis>80)
    {
      beeping=false;
      buzzer_millis=millis();
      digitalWrite(9,LOW);  
    }
  }

  batteryVoltage=analogRead(A6);
  if (BATTERYTYPE==14.8)
  {
    if (batteryVoltage>604) buzzer_spacing=40000;
    else if (batteryVoltage>580) buzzer_spacing=30000;
    else if (batteryVoltage>556) buzzer_spacing=20000;
    else if (batteryVoltage>534) buzzer_spacing=10000;
    else if (batteryVoltage>510) buzzer_spacing=500;
    else buzzer_spacing=100;
  } else
  {
    if (batteryVoltage>302) buzzer_spacing=40000;
    else if (batteryVoltage>290) buzzer_spacing=30000;
    else if (batteryVoltage>278) buzzer_spacing=20000;
    else if (batteryVoltage>267) buzzer_spacing=10000;
    else if (batteryVoltage>255) buzzer_spacing=500;
    else buzzer_spacing=100;
  }
}

void Troubleshooting() {
  //Print data at 100hz (uncomment one at a time for troubleshooting)
    //printRadioData();     //Prints radio pwm values (expected: 1000 to 2000)
    //printDesiredState();  //Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
    //printGyroData();      //Prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
    //printAccelData();     //Prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
    //printRollPitchYaw();  //Prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
    //printPIDoutput();     //Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
    //printMotorCommands(); //Prints the values being written to the motors (expected: 1000 to 2000)
    //printtock();      //Prints the time between loops in microseconds (expected: microseconds between loop iterations and less the Gyro/Acc update speed.  Set by LOOP_TIMING)
    printJSON();
  }


void setupSerial(){
  Serial.begin(2000000);
  delay(1000);
}

void setupWiFi()
{
  char ssid[] = "Rawpter";        
  char pass[] = "12345678";    

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    // don't continue because this is probably not an RP2040
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // Just picked this out of the air.  Throw back to Jeff Gordon
  WiFi.config(IPAddress(192, 168, 2, 4));

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    // don't continue
    while (true);
  }

  // wait 1 seconds for connection:
  delay(3000);

  // start the web server on port 80
  server.begin();
}

void loopWiFi() {
    // compare the previous status to the current status
  WiFiClient client = server.available();   // listen for incoming clients
  if (client) {                             // if you get a client,
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        if (c == '\n') {                   
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
        if (currentLine.endsWith("GET /?"))
        {
          setTheValuesFromUserForm(client);
          //Handle clicking the Begin button
          MakeWebPage(client,"<meta http-equiv = \"refresh\" content = \"0; url = http://192.168.2.4 \"/>");
        }
        else if (currentLine.endsWith("GET / HTTP"))
        { //Handle hitting the basic page (1st connection)
          MakeWebPage(client,"<h1>Rawpter V1.0 </h1>" + GetParameters() +"<br><input class='mt-2 btn btn-primary' type=submit value='submit' />");
        }
      }
    }
    // close the connection:
    client.stop();
  }
}
void setTheValuesFromUserForm(WiFiClient client)
{
  String currentLine="";
  while (client.connected())
  {            // loop while the client's connected
    if (client.available()) {             // if there's bytes to read from the client,
      char c = client.read();             // read a byte, then
      if (c == '\n') {                   
        // if the current line is blank, you got two newline characters in a row.
        // that's the end of the client HTTP request, so send a response:
        if (currentLine.length() == 0) {
          break;
        } else {    // if you got a newline, then clear currentLine:
          currentLine = "";
        }
      } else if (c != '\r') {  // if you got anything else but a carriage return character,
        currentLine += c;      // add it to the end of the currentLine
      }
      if (currentLine.endsWith("kp_roll_angle="))
      {
          String myValue="";

          while (!currentLine.endsWith("&"))
          {
            c = client.read();
            if (c!='&') myValue+=c;
            currentLine+=c;
          }
          Kp_roll_angle=myValue.toFloat();
      }
      if (currentLine.endsWith("ki_roll_angle="))
      {
          String myValue="";

          while (!currentLine.endsWith("&"))
          {
            c = client.read();
            if (c!='&') myValue+=c;
            currentLine+=c;
          }
          Ki_roll_angle=myValue.toFloat();
      }
      if (currentLine.endsWith("kd_roll_angle="))
      {
          String myValue="";

          while (!currentLine.endsWith("&"))
          {
            c = client.read();
            if (c!='&') myValue+=c;
            currentLine+=c;
          }
          Kd_roll_angle=myValue.toFloat();
      }
    
      //
      if (currentLine.endsWith("kp_pitch_angle="))
      {
          String myValue="";

          while (!currentLine.endsWith("&"))
          {
            c = client.read();
            if (c!='&') myValue+=c;
            currentLine+=c;
          }
          Kp_pitch_angle=myValue.toFloat();
      }
      if (currentLine.endsWith("ki_pitch_angle="))
      {
          String myValue="";

          while (!currentLine.endsWith("&"))
          {
            c = client.read();
            if (c!='&') myValue+=c;
            currentLine+=c;
          }
          Ki_pitch_angle=myValue.toFloat();
      }
      if (currentLine.endsWith("kd_pitch_angle="))
      {
          String myValue="";

          while (!currentLine.endsWith("&"))
          {
            c = client.read();
            if (c!='&') myValue+=c;
            currentLine+=c;
          }
          Kd_pitch_angle=myValue.toFloat();
      }
      //yaw
      if (currentLine.endsWith("kp_yaw="))
      {
          String myValue="";

          while (!currentLine.endsWith("&"))
          {
            c = client.read();
            if (c!='&') myValue+=c;
            currentLine+=c;
          }
          Kp_yaw=myValue.toFloat();
      }
      if (currentLine.endsWith("ki_yaw="))
      {
          String myValue="";

          while (!currentLine.endsWith("&"))
          {
            c = client.read();
            if (c!='&') myValue+=c;
            currentLine+=c;
          }
          Ki_yaw=myValue.toFloat();
      }
      if (currentLine.endsWith("kd_yaw="))
      {
          String myValue="";

          while (!currentLine.endsWith("&"))
          {
            c = client.read();
            if (c!='&') myValue+=c;
            currentLine+=c;
          }
          Kd_yaw=myValue.toFloat();
          return;
      }
    }  
  }
}
String GetParameters()
{
  String myString="<table class=table><thead class=thead-dark><th></th><th>Kp</th><th>Ki</th><th>Kd</th></thead>";
  myString=myString + "<tr><td>Roll:</td><td><input name=kp_roll_angle style='width:70px;' type=text value='" + String(Kp_roll_angle) + "'></td><td><input style='width:70px;' name=ki_roll_angle type=text value='" + String(Ki_roll_angle) + "'></td><td><input name=kd_roll_angle type=text style='width:70px;' value='" + String(Kd_roll_angle) + "'></td></tr>";
  myString=myString + "<tr><td>Pitch:</td><td><input name=kp_pitch_angle  style='width:70px;' type=text value='" + String(Kp_pitch_angle) + "'></td><td><input  style='width:70px;' name=ki_pitch_angle type=text value='" + String(Ki_pitch_angle) + "'></td><td><input name=kd_pitch_angle type=text  style='width:70px;' value='" + String(Kd_pitch_angle) + "'></td></tr>";
  myString=myString + "<tr><td>Yaw:</td><td><input name=kp_yaw type=text  style='width:70px;' value='" + String(Kp_yaw) + "'></td><td><input  style='width:70px;' type=text name=ki_yaw value='" + String(Ki_yaw) + "'></td><td><input type=text name=kd_yaw style='width:70px;' value='" + String(Kd_yaw) + "'><input type=hidden name=ender value='0'></td></tr>";
  myString=myString + "</table><br>Tip: To use your parameters beyond this flight session, snapshot the screen for reference and update the code.<br>";
  return myString;
}

void MakeWebPage(WiFiClient client, String html)
{
  // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
  // and a content-type so the client knows what's coming, then a blank line:
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:text/html");
  client.println();

  // the content of the HTTP response follows the header:
  client.print("<head>");
  client.print("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
  client.print("<link href=\"https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/css/bootstrap.min.css\" rel=\"stylesheet\" integrity=\"sha384-1BmE4kWBq78iYhFldvKuhfTAU6auU8tT94WrHftjDbrCEXSU1oBoqyl2QvZ6jIW3\" crossorigin=\"anonymous\">");
  client.print("</head>");
  client.print("<style>");
  client.print("</style>");
  client.print("<form method=get><div class='container'>");
  client.print(html);
  client.print("</div></form>");
  client.print("<script src=\"https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/js/bootstrap.bundle.min.js\" integrity=\"sha384-ka7Sk0Gln4gmtz2MlQnikT1wXgYsOg+OMhuP+IlRH9sENBO0LRn5q+8nbTov4+1p\" crossorigin=\"anonymous\"></script>");
  client.println(); // The HTTP response ends with another blank line:
}

void tick()
{
  //Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;
  current_time = micros();
  deltaTime = (current_time - prev_time)/1000000.0; //division takes it from micros to seconds.  1000000 ms=1 second
}
//========================================================================================================================//
//                                                      FUNCTIONS                                                         //                           
//========================================================================================================================//

void controlMixer() {
  //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
   * should have -roll_PID. Front two should have -pitch_PID and the back two should have +pitch_PID etc... every motor has
   * normalized (0 to 1) thro_des command for throttle control. Can also apply direct unstabilized commands from the transmitter with 
   * roll_passthru, pitch_passthru, and yaw_passthu. mX_command_scaled and sX_command scaled variables are used in scaleCommands() 
   * in preparation to be sent to the motor ESCs
   * 
   *Relevant variables:
   *thro_des - direct thottle control
   *roll_PID, pitch_PID, yaw_PID - stabilized axis variables
   *roll_passthru, pitch_passthru, yaw_passthru - direct unstabilized command passthrough
   */
   
  //Quad mixing
  //m1_command_scaled = thro_des - pitch_PID + roll_PID + yaw_PID; //Front left
  //m2_command_scaled = thro_des - pitch_PID - roll_PID - yaw_PID; //Front right
  //m3_command_scaled = thro_des + pitch_PID - roll_PID + yaw_PID; //Back Right
  //m4_command_scaled = thro_des + pitch_PID + roll_PID - yaw_PID; //Back Left
  m1_command_scaled = thro_des + pitch_PID + roll_PID + yaw_PID; //Front left
  m2_command_scaled = thro_des + pitch_PID - roll_PID - yaw_PID; //Front right
  m3_command_scaled = thro_des - pitch_PID - roll_PID + yaw_PID; //Back Right
  m4_command_scaled = thro_des - pitch_PID + roll_PID - yaw_PID; //Back Left
}

void IMUinit() {
  //DESCRIPTION: Initialize IMU
  if (!IMU.begin()) {  
    while (true) ;
  }  
  delay(15);
}

void getIMUdata() {
  //DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data
 //Accelerometer
 
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(AccX, AccY, AccZ);
    AccX = AccX - AccErrorX;
    AccY = AccY - AccErrorY;
    AccY = AccY - AccErrorZ;
    //Correct the outputs with the calculated error values
    AccX = (1.0 - Accel_filter)*AccX_prev + Accel_filter*AccX;
    AccY = (1.0 - Accel_filter)*AccY_prev + Accel_filter*AccY;
    AccZ = (1.0 - Accel_filter)*AccZ_prev + Accel_filter*AccZ;
    AccX_prev = AccX;
    AccY_prev = AccY;
    AccZ_prev = AccZ;
  } else return;
  if (IMU.gyroscopeAvailable()) {
  //Gyro
    IMU.readGyroscope(GyroX, GyroY, GyroZ);
    //Correct the outputs with the calculated error values
    GyroX = GyroX - GyroErrorX;
    GyroY = GyroY - GyroErrorY;
    GyroZ = GyroZ - GyroErrorZ;
    GyroX = (1.0 -Gyro_filter)*GyroX_prev +Gyro_filter*GyroX;
    GyroY = (1.0 -Gyro_filter)*GyroY_prev +Gyro_filter*GyroY;
    GyroZ = (1.0 -Gyro_filter)*GyroZ_prev +Gyro_filter*GyroZ;
    GyroX_prev = GyroX;
    GyroY_prev = GyroY;
    GyroZ_prev = GyroZ;
  }
}

void calculate_IMU_error() {
  //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
  /*
   * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift in the
   * measurement.
   */
  float AcX,AcY,AcZ,GyX,GyY,GyZ;
  
  //Read IMU values 12000 times
  int c = 0;
  while (c < 12000) {
    
    IMU.readAcceleration(AcX, AcY, AcZ);
    IMU.readGyroscope(GyX, GyY, GyZ);
    AccX  = AcX;
    AccY  = AcY;
    AccZ  = AcZ;
    GyroX = GyX;
    GyroY = GyY;
    GyroZ = GyZ;
    
    //Sum all readings
    AccErrorX  = AccErrorX + AccX;
    AccErrorY  = AccErrorY + AccY;
    AccErrorZ  = AccErrorZ + AccZ;
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
  }
  //Divide the sum by 12000 to get the error value
  AccErrorX  = AccErrorX / c;
  AccErrorY  = AccErrorY / c;
  AccErrorZ  = AccErrorZ / c - 1.0;
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;

  Serial.print("float AccErrorX = ");
  Serial.print(AccErrorX);
  Serial.println(";");
  Serial.print("float AccErrorY = ");
  Serial.print(AccErrorY);
  Serial.println(";");
  Serial.print("float AccErrorZ = ");
  Serial.print(AccErrorZ);
  Serial.println(";");
  
  Serial.print("float GyroErrorX = ");
  Serial.print(GyroErrorX);
  Serial.println(";");
  Serial.print("float GyroErrorY = ");
  Serial.print(GyroErrorY);
  Serial.println(";");
  Serial.print("float GyroErrorZ = ");
  Serial.print(GyroErrorZ);
  Serial.println(";");

  Serial.println("Paste these values in user specified variables section and comment out calculate_IMU_error() in void setup.");
  while(true) delay(1000);
}

void calibrateAttitude() {
  //DESCRIPTION: Used to warm up the main loop to allow the madwick filter to converge before commands can be sent to the actuators
  //Assuming vehicle is powered up on level surface!
  /*
   * This function is used on startup to warm up the attitude estimation and is what causes startup to take a few seconds
   * to boot. 
   */
  //Warm up IMU and madgwick filter in simulated main loop
  for (int i = 0; i <= 10000; i++) {
    tick();
    getIMUdata();
    Madgwick6DOF(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ);
    tock(LOOP_TIMING); //gyro update frequency
  }
}
void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az)
{
      // update the filter, which computes orientation
    IMU_Data.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll_IMU = IMU_Data.getRoll();
    pitch_IMU = IMU_Data.getPitch();
    yaw_IMU = IMU_Data.getYaw();
}

void getDesiredAnglesAndThrottle() {
  //DESCRIPTION: Normalizes desired control values to appropriate values
  /*
   * Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
   * RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
   * roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
   * (rate mode). yaw_des is scaled to be within max yaw in degrees/sec. Also creates roll_passthru, pitch_passthru, and
   * yaw_passthru variables, to be used in commanding motors/servos with direct unstabilized commands in controlMixer().
   */
  thro_des = (PWM_throttle - 1000.0)/1000.0; //Between 0 and 1
  roll_des = (PWM_roll - 1500.0)/500.0; //Between -1 and 1
  pitch_des = (PWM_Elevation - 1500.0)/500.0; //Between -1 and 1
  yaw_des = (PWM_Rudd - 1500.0)/500.0; //Between -1 and 1 
  roll_passthru = roll_des/2.0; //Between -0.5 and 0.5
  pitch_passthru = pitch_des/2.0; //Between -0.5 and 0.5
  yaw_passthru = yaw_des/2.0; //Between -0.5 and 0.5
  
  //Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0); //Between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll; //Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; //Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw; //Between -maxYaw and +maxYaw
  roll_passthru = constrain(roll_passthru, -0.5, 0.5);
  pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
  yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
}

void PIDControlCalcs() {
  //DESCRIPTION: Computes control commands based on state error (angle)
  /*
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 
   * getDesiredAnglesAndThrottle(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
   * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
   */
  
  //Roll
  if (UPONLYMODE) roll_des=0;

  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll*deltaTime;
  if (PWM_throttle < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }

  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = GyroX; //(roll_des-roll_IMU-roll_des-previous_IMU)/dt=current angular velocity since last IMU read and therefore GyroX in deg/s
  roll_PID = 0.01*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  if (UPONLYMODE) pitch_des=0;
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch*deltaTime;
  if (PWM_throttle < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = GyroY;
  pitch_PID = .01*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ versus angle.  In other words, your stick is setting y axis rotation speed - not the angle to get to.
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*deltaTime;
  if (PWM_throttle < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/deltaTime; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range
  
  if (UPONLYMODE) yaw_PID=0;
  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;

}

void scaleCommands() {
  //DESCRIPTION: Scale normalized actuator commands to values for ESC protocol
  /*
   * The actual pulse width is set at the servo attach.
   */
  //Constrain commands to motors within Simonk bounds
  m1_command_PWM = m1_command_scaled*180;
  m2_command_PWM = m2_command_scaled*180;
  m3_command_PWM = m3_command_scaled*180;
  m4_command_PWM = m4_command_scaled*180;

  m1_command_PWM = constrain(m1_command_PWM, 0, 180);
  m2_command_PWM = constrain(m2_command_PWM, 0, 180);
  m3_command_PWM = constrain(m3_command_PWM, 0, 180);
  m4_command_PWM = constrain(m4_command_PWM, 0, 180);
}

void getRadioSticks() {
  //DESCRIPTION: Get raw PWM values for every channel from the radio
  /*
   * Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of 
   * the loop.
   * The raw radio commands are filtered with a first order low-pass filter to eliminate any really high frequency noise. 
   */

    PWM_throttle = getRadioPWM(1);
    PWM_roll = getRadioPWM(2);
    PWM_Elevation = getRadioPWM(3);
    PWM_Rudd = getRadioPWM(4);
    PWM_ThrottleCutSwitch = getRadioPWM(5);

  //Low-pass the critical commands and update previous values
  float b = 0.7; //Lower=slower, higher=noiser default 0.7

  PWM_throttle = (1.0 - b)*PWM_throttle_prev + b*PWM_throttle;
  PWM_roll = (1.0 - b)*PWM_roll_prev + b*PWM_roll;
  PWM_Elevation = (1.0 - b)*PWM_Elevation_prev + b*PWM_Elevation;
  PWM_Rudd = (1.0 - b)*PWM_Rudd_prev + b*PWM_Rudd;

  PWM_throttle_prev = PWM_throttle;
  PWM_roll_prev =PWM_roll;
  PWM_Elevation_prev = PWM_Elevation;
  PWM_Rudd_prev = PWM_Rudd;
  
}
void setToFailsafe()
{
  PWM_throttle = PWM_throttle_fs;
  PWM_roll = PWM_roll_fs;
  PWM_Elevation = PWM_elevation_fs;
  PWM_Rudd = PWM_rudd_fs;
  PWM_ThrottleCutSwitch = PWM_ThrottleCutSwitch_fs; //this is so the throttle cut routine doesn't override the fail safes.
}

void failSafe() {
  //DESCRIPTION: If radio gives garbage values, set all commands to default values
  /*
   * Radio connection failsafe used to check if the getRadioSticks() function is returning acceptable pwm values. If any of 
   * the commands are lower than 800 or higher than 2200, then we can be certain that there is an issue with the radio
   * connection (most likely hardware related). If any of the channels show this failure, then all of the radio commands 
   * channel_x_pwm are set to default failsafe values specified in the setup. Comment out this function when troubleshooting 
   * your radio connection in case any extreme values are triggering this function to overwrite the printed variables.
   */
  

  //Triggers for failure criteria
  //unsigned minVal = 800;
  //unsigned maxVal = 2200;
  //if (PWM_ThrottleCutSwitch>maxVal || PWM_ThrottleCutSwitch<minVal || PWM_throttle > maxVal || PWM_throttle < minVal || PWM_roll > maxVal ||PWM_roll < minVal || PWM_Elevation > maxVal || PWM_Elevation < minVal || PWM_Rudd > maxVal || PWM_Rudd < minVal) 
  
  
  if (PWM_throttle < 800 || PWM_throttle > 2200) //this is the less conservative version to get through this routine faster.
  {
    failsafed=true;
    setToFailsafe();
  } else failsafed=false;
  
  if (EASYCHAIR) PWM_throttle=1500;//For testing in the easy chair with the Arduino out of the drone.  See the compiler directive at the top of the code.
}

void commandMotors() {
  //DESCRIPTION: Send pulses to motor pins
  m1PWM.write(m1_command_PWM);
  m2PWM.write(m2_command_PWM);
  m3PWM.write(m3_command_PWM);
  m4PWM.write(m4_command_PWM);
}

void calibrateESCs() {
  //DESCRIPTION: Used in void setup() to allow standard ESC calibration procedure with the radio to take place.
  /*  
   *  Simulates the void loop(), but only for the purpose of providing throttle pass through to the motors, so that you can
   *  power up with throttle at full, let ESCs begin arming sequence, and lower throttle to zero.
   */
  m1PWM.write(180);
  m2PWM.write(180);
  m3PWM.write(180);
  m4PWM.write(180);
  delay(5000);
  m1PWM.write(0);
  m2PWM.write(0);
  m3PWM.write(0);
  m4PWM.write(0);
  delay(5000);
}

void throttleCut() {
  //DESCRIPTION: Directly set actuator outputs to minimum value if triggered
  /*
   * Monitors the state of radio command PWM_throttle and directly sets the mx_command_PWM values to minimum (1060 is
   * minimum , 0 is minimum for standard PWM servo library used) if channel 5 is high. This is the last function 
   * called before commandMotors() is called so that the last thing checked is if the user is giving permission to command
   * the motors to anything other than minimum value. Safety first. 
   */
  if (EASYCHAIR) {
    //This is set above in compiler directives for when you are testing the Arduino outside of the drone.
    throttle_is_cut=false;
    return;
  }
  if (throttle_is_cut) killMotors(); //make sure we keep those motors at 0 if the throttle was cut.

  if (PWM_ThrottleCutSwitch < 1300) { 
    //throttleCutCounter will ensure it is not just a blip that has caused a false cut.
    if (++throttleCutCounter>20) killMotors(); 
    return;    
  }
  if (throttle_is_cut&&PWM_ThrottleCutSwitch>1500)
  { 
    //reset only if throttle is down to prevent a jolting suprise 
    if (PWM_throttle<1040&&++throttleNotCutCounter>20){  
      throttle_is_cut=false;
      throttleNotCutCounter=0;
      throttleCutCounter=0;
    } else killMotors();
    return;
  }  
}

void killMotors(){
  //sets the PWM to its lowest value to shut off a motor such as whne the throttle cut switch is fliped.
    throttle_is_cut=true;    
    m1_command_PWM = 0;
    m2_command_PWM = 0;
    m3_command_PWM = 0;
    m4_command_PWM = 0;
}

void tock(int freq) {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. I have matched 
   * it to the Gyro update frequency.
   */  
  //Sit in loop until appropriate time has passed
  while (invFreq > (micros() - current_time)) ;
}

void printRadioData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F(" CH1: "));
    Serial.print(getRadioPWM(1));
    Serial.print(F(" CH2: "));
    Serial.print(getRadioPWM(2));
    Serial.print(F(" CH3: "));
    Serial.print(getRadioPWM(3));
    Serial.print(F(" CH4: "));
    Serial.println(getRadioPWM(4));
    Serial.print(F(" CH5: "));
    Serial.print(PWM_ThrottleCutSwitch);
  }
}

void printDesiredState() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("thro_des: "));
    Serial.print(thro_des);
    Serial.print(F(" roll_des: "));
    Serial.print(roll_des);
    Serial.print(F(" pitch_des: "));
    Serial.print(pitch_des);
    Serial.print(F(" yaw_des: "));
    Serial.println(yaw_des);
  }
}

void printGyroData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("GyroX: "));
    Serial.print(GyroX);
    Serial.print(F(" GyroY: "));
    Serial.print(GyroY);
    Serial.print(F(" GyroZ: "));
    Serial.println(GyroZ);
  }
}

void printAccelData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("AccX: "));
    Serial.print(AccX);
    Serial.print(F(" AccY: "));
    Serial.print(AccY);
    Serial.print(F(" AccZ: "));
    Serial.println(AccZ);
  }
}
void printJSON(){
  if (current_time - print_counter > 25000) { //Don't go too fast or it slows down the main loop
    print_counter = micros();
    Serial.print(F("{\"roll\": "));
    Serial.print(roll_IMU);
    Serial.print(F(", \"pitch\": "));
    Serial.print(pitch_IMU);
    Serial.print(F(", \"yaw\": "));
    Serial.print(yaw_IMU);
    Serial.print(F(", \"ErrorRoll\": "));
    Serial.print(error_roll);
    Serial.print(F(", \"IntegralRoll\": "));
    Serial.print(integral_roll);
    Serial.print(F(", \"DerivativeRoll\": "));
    Serial.print(derivative_roll);
    
    Serial.print(F(", \"RollKp\": "));
    Serial.print(Kp_roll_angle);
    Serial.print(F(", \"RollKi\": "));
    Serial.print(Ki_roll_angle);
    Serial.print(F(", \"RollKd\": "));
    Serial.print(Kd_roll_angle);

    Serial.print(F(", \"m1\": "));
    Serial.print(m1_command_PWM);
    Serial.print(F(", \"m2\": "));
    Serial.print(m2_command_PWM);
    Serial.print(F(", \"m3\": "));
    Serial.print(m3_command_PWM);
    Serial.print(F(", \"m4\": "));
    Serial.print(m4_command_PWM);
    
    Serial.print(F(", \"AccX\": "));
    Serial.print(AccX);
    Serial.print(F(", \"AccY\": "));
    Serial.print(AccY);
    Serial.print(F(", \"AccZ\": "));
    Serial.print(AccZ);

    Serial.print(F(", \"GyroX\": "));
    Serial.print(GyroX);
    Serial.print(F(", \"GyroY\": "));
    Serial.print(GyroY);
    Serial.print(F(", \"GyroZ\": "));
    Serial.print(GyroZ);
    Serial.print(F(", \"RollIMU\": "));
    Serial.print(roll_IMU);
    Serial.print(F(", \"PitchIMU\": "));
    Serial.print(pitch_IMU);
    Serial.print(F(", \"YawIMU\": "));
    Serial.print(yaw_IMU);

    Serial.print(F(", \"ThroDes\": "));
    Serial.print(thro_des);
    Serial.print(F(", \"RollDes\": "));
    Serial.print(roll_des);
    Serial.print(F(", \"PitchDes\": "));
    Serial.print(pitch_des);
    Serial.print(F(", \"YawDes\": "));
    Serial.print(yaw_des);
    Serial.print(F(", \"Pitch_PID\": "));
    Serial.print(pitch_PID);
    Serial.print(F(", \"Roll_PID\": "));
    Serial.print(roll_PID);
    Serial.print(F(", \"Yaw_PID\": "));
    Serial.print(yaw_PID);

    Serial.print(F(", \"PWM_throttle\": "));
    Serial.print(PWM_throttle);
    Serial.print(F(", \"PWM_roll\": "));
    Serial.print(PWM_roll);
    Serial.print(F(", \"PWM_Elevation\": "));
    Serial.print(PWM_Elevation);
    Serial.print(F(", \"PWM_Rudd\": "));
    Serial.print(PWM_Rudd);
    Serial.print(F(", \"PWM_ThrottleCutSwitch\": "));
    Serial.print(PWM_ThrottleCutSwitch);
    Serial.print(F(", \"Throttle_is_Cut\": "));
    Serial.print(throttle_is_cut);

    Serial.print(F(", \"Failsafe\": "));
    Serial.print(failsafed);
    
    Serial.print(F(", \"DeltaTime\": "));
    Serial.print(deltaTime*1000000.0);
    Serial.println("}");
  }
}

void printRollPitchYaw() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll: "));
    Serial.print(roll_IMU);
    Serial.print(F(" pitch: "));
    Serial.print(pitch_IMU);
    Serial.print(F(" yaw: "));
    Serial.println(yaw_IMU);
  }
}

void printPIDoutput() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll_PID: "));
    Serial.print(roll_PID);
    Serial.print(F(" pitch_PID: "));
    Serial.print(pitch_PID);
    Serial.print(F(" yaw_PID: "));
    Serial.println(yaw_PID);
  }
}

void printMotorCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("m1_command: "));
    Serial.print(m1_command_PWM);
    Serial.print(F(" m2_command: "));
    Serial.print(m2_command_PWM);
    Serial.print(F(" m3_command: "));
    Serial.print(m3_command_PWM);
    Serial.print(F(" m4_command: "));
    Serial.println(m4_command_PWM);
  }
}

void printtock() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("deltaTime = "));
    Serial.println(deltaTime*1000000.0);
  }
}

//========================================================================================================================//

//This file contains all necessary functions and code used for radio communication to avoid cluttering the main code

void radioSetup()
{
    //PWM Receiver
    //Declare interrupt pins 
    pinMode(throttlePin, INPUT_PULLUP);
    pinMode(rollPin, INPUT_PULLUP);
    pinMode(upDownPin, INPUT_PULLUP);
    pinMode(ruddPin, INPUT_PULLUP);
    pinMode(throttleCutSwitchPin, INPUT_PULLUP);
    
    delay(20);
    //Attach interrupt and point to corresponding ISR functions
    attachInterrupt(digitalPinToInterrupt(throttlePin), getCh1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rollPin), getCh2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(upDownPin), getCh3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ruddPin), getCh4, CHANGE);
    attachInterrupt(digitalPinToInterrupt(throttleCutSwitchPin), getCh5, CHANGE);
    
    delay(20);
}

unsigned long getRadioPWM(int ch_num) {
  //DESCRIPTION: Get current radio commands from interrupt routines 
  unsigned long returnPWM = 0;
  
  if (ch_num == 1) {
    returnPWM = channel_1_raw;
  }
  else if (ch_num == 2) {
    returnPWM = channel_2_raw;
  }
  else if (ch_num == 3) {
    returnPWM = channel_3_raw;
  }
  else if (ch_num == 4) {
    returnPWM = channel_4_raw;
  }
  else if (ch_num == 5) {
    returnPWM = channel_5_raw;
  }
  return returnPWM;
}

//========================================================================================================================//

//INTERRUPT SERVICE ROUTINES (for reading PWM and PPM)

void getCh1() {
  int trigger = digitalRead(throttlePin);
  if(trigger == 1) {
    rising_edge_start_1 = micros();
  }
  else if(trigger == 0) {
    channel_1_raw = micros() - rising_edge_start_1;
  }
}

void getCh2() {
  int trigger = digitalRead(rollPin);
  if(trigger == 1) {
    rising_edge_start_2 = micros();
  }
  else if(trigger == 0) {
    channel_2_raw = micros() - rising_edge_start_2;
  }
}

void getCh3() {
  int trigger = digitalRead(upDownPin);
  if(trigger == 1) {
    rising_edge_start_3 = micros();
  }
  else if(trigger == 0) {
    channel_3_raw = micros() - rising_edge_start_3;
  }
}

void getCh4() {
  int trigger = digitalRead(ruddPin);
  if(trigger == 1) {
    rising_edge_start_4 = micros();
  }
  else if(trigger == 0) {
    channel_4_raw = micros() - rising_edge_start_4;
  }
}

void getCh5() {
  int trigger = digitalRead(throttleCutSwitchPin);
  if(trigger == 1) {
    rising_edge_start_5 = micros();
  }
  else if(trigger == 0) {
    channel_5_raw = micros() - rising_edge_start_5;
  }
}
