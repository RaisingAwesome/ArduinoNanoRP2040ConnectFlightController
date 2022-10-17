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
#define DEBUG false

//EASYCHAIR is used to put in some key dummy variables so you can sit in the easy chair with just the Arduino on a cord wiggling it around and seeing how it responds.
//Set EASYCHAIR to false when you are wanting to install it in the drone.
#define EASYCHAIR false

//The IMUTIMING is based on the IMU.  For the Arduino_LSM6DSOX, it is 104Hz.
#define IMUTIMING 104

//The battery alarm code handles 14.8 or 7.4V LIPOs.  Set to your type of battery. If not hooked up, it will pull down and beep often.
#define BATTERYTYPE 14.8
int batteryVoltage=1023; //just a default for the battery monitoring routine 

//Radio failsafe values for every channel in the event that bad reciever data is detected.
//These are for it to stay stable and descend safely versus totally cutting throttle and drop like a rock.
unsigned long PWM_throttle_zero = 1000; //used when we want to take throttle to zero.  Failsafe is something higher as it is expected that failsafe is a value needed to safely land.
unsigned long PWM_throttle_fs = 1100; //throttle  will allow it to descend to the ground if you adjust the throttlecutswitch_fs to 2000
unsigned long PWM_roll_fs = 1500; //ail pretty much in the middle so it quits turning
unsigned long PWM_elevation_fs = 1500; //elev
unsigned long PWM_rudd_fs = 1500; //rudd
unsigned long PWM_ThrottleCutSwitch_fs = 1000; //SWA less than 1300, cut throttle - must config a switch to Channel 5 in your remote.

bool failsafed=false;
//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
float AccErrorX = 0.01;
float AccErrorY = -0.01;
float AccErrorZ = 0.01;
float GyroErrorX = 0.42;
float GyroErrorY= 0.07;
float GyroErrorZ = -0.05;

//Controller parameters (take note of defaults before modifying!): 
float i_limit = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 30.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode (default 30.0)
float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode (default 30.0)
float maxYaw = 160.0;     //Max yaw rate in deg/sec (default 160.0)

float Kp_roll_angle = 0.2;    //Roll P-gain - angle mode 
float Ki_roll_angle = 0.3;    //Roll I-gain - angle mode
float Kd_roll_angle = 0.05;   //Roll D-gain - angle mode

float Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
float Ki_pitch_angle = 0.3;   //Pitch I-gain - angle mode
float Kd_pitch_angle = 0.05;  //Pitch D-gain - angle mode

float Kp_roll_rate = 0.15;    //Roll P-gain - rate mode
float Ki_roll_rate = 0.2;     //Roll I-gain - rate mode
float Kd_roll_rate = 0.0002;  //Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
float Kp_pitch_rate = 0.15;   //Pitch P-gain - rate mode
float Ki_pitch_rate = 0.2;    //Pitch I-gain - rate mode
float Kd_pitch_rate = 0.0002; //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

float Kp_yaw = 0.3;           //Yaw P-gain
float Ki_yaw = 0.05;          //Yaw I-gain
float Kd_yaw = 0.00015;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)

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
unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw;
int ppm_counter = 0;
unsigned long time_ms = 0;

//Motor Electronic Speed Control Modules (ESC):
const int m1Pin = 14;
const int m2Pin = 15;
const int m3Pin = 16;
const int m4Pin = 10; // (17 should have worked, but doesn't)
Servo m1PWM, m2PWM, m3PWM, m4PWM;

//========================================================================================================================//
//DECLARE GLOBAL VARIABLES
//========================================================================================================================//

//General stuff for controlling timing of things
float deltaTime;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;
bool beeping=false; //for beeping when the battery is getting low.
bool throttle_is_cut=true; //used to force the pilot to manually set the throttle to zero after the switch is used to throttle cut

//Radio communication:
unsigned long PWM_throttle,PWM_roll, PWM_Elevation, PWM_Rudd, PWM_ThrottleCutSwitch;
unsigned long PWM_throttle_prev,PWM_roll_prev, PWM_Elevation_prev, PWM_Rudd_prev;

//IMU:
Madgwick filter; //the library tool that will convert the IMU data to angular data
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
bool ALLOW_WIFI=false;

WiFiServer server(80);
//WiFi End

//========================================================================================================================//
//BEGIN THE CLASSIC SETUP AND LOOP
//========================================================================================================================//

void setup() {
  //Bootup operations 
  filter.begin(IMUTIMING); //initiate the calculations to determine the drone attitude angles
  setupSerial();
  setupBatteryMonitor();
  setupDrone();
  if (ALLOW_WIFI) setupWiFi(); //At first power on, a WiFi hotspot is set up for talking to the drone. (SSID Rawpter, 12345678)
}

void loop() {
  tick(); //stamp the start time of the loop to keep our timing to 2000Hz.  See tock() below.
  loopBuzzer();
  //The main thread that infinitely loops - poling sensors and taking action.
  if (ALLOW_WIFI) loopWiFi(); else loopDrone(); //For the first few seconds after bootup, you have the opportunity to connect to the wifi.

  tock(IMUTIMING); //Do not exceed 2000Hz, all filter parameters tuned to 104Hz by default
}

void setupDrone() {  
  //Initialize all pins
  if (DEBUG) Serial.println("initializing1");
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
  PWM_throttle = PWM_throttle_zero;
  PWM_roll = PWM_roll_fs;
  PWM_Elevation = PWM_elevation_fs;
  PWM_Rudd = PWM_rudd_fs;
  PWM_ThrottleCutSwitch = PWM_ThrottleCutSwitch_fs;
  
  //Initialize IMU communication
  IMUinit();

  delay(5);

  //Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
  //calculate_IMU_error(); //Calibration parameters printed to serial monitor. Paste these in the user specified variables section, then comment this out forever.

  delay(5);
  if (DEBUG) Serial.println("Initializing");

  if (getRadioPWM(1)>1800&!EASYCHAIR) calibrateESCs(); //if the throttle is up, first calibrate before going into the loop
  
  m1_command_PWM = 0; //Will send the default for motor stopped for Simonk firmware
  m2_command_PWM = 0;
  m3_command_PWM = 0;
  m4_command_PWM = 0;

  while (getRadioPWM(1)>1060&!EASYCHAIR&&getRadioPWM(5)<1300) //wait until the throttle is turned down before allowing anything else to happen.
  {
    delay(1000);
  }
  //calibrateAttitude();
}
void loopDrone() {
  loopBlink(); //Indicates that we are in main loop with short blink every 1.5 seconds
  getIMUdata(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
  Madgwick6DOF(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ); //Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)
  getDesiredAnglesAndThrottle(); //Convert raw commands to normalized values based on saturated control limits
  controlANGLE(); //Stabilize on angle setpoint from getDesiredAnglesAndThrottle
  controlMixer(); //Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
  scaleCommands(); //Scales motor commands to 0-1
  throttleCut(); //Directly sets motor commands to off based on channel 5 being switched
  Troubleshooting(); //will do the print routines if uncommented
  commandMotors(); //Sends command pulses to each motor pin
  getRadioSticks(); //Gets the PWM from the receiver
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
      if (DEBUG) Serial.println("About to go low");
      beeping=false;
      buzzer_millis=millis();
      digitalWrite(9,LOW);  
      if (DEBUG) Serial.println("About to go lower");
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
    //printtock();      //Prints the time between loops in microseconds (expected: microseconds between loop iterations)
    printJSON();
  }


void setupSerial(){
  Serial.begin(230400);
  
  delay(1000);
  if (DEBUG) Serial.println("I got mine!");
}

void setupWiFi()
{
  char ssid[] = "Rawpter";        
  char pass[] = "12345678";    

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    if (DEBUG) Serial.println("Communication with WiFi module failed!");
    // don't continue because this is probably not an RP2040
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    if (DEBUG) Serial.println("Please upgrade the firmware");
  }

  // Just picked this out of the air.  Throw back to Jeff Gordon
  WiFi.config(IPAddress(192, 168, 2, 4));

  // print the network name (SSID);
  if (DEBUG) Serial.print("Creating access point named: ");
  if (DEBUG) Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    if (DEBUG) Serial.println("Creating access point failed");
    // don't continue
    while (true);
  }

  // wait 1 seconds for connection:
  delay(1000);

  // start the web server on port 80
  server.begin();

  // you're connected now, so print out the status
  printWiFiStatus();
}

void loopWiFi() {
    // compare the previous status to the current status
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      if (DEBUG) Serial.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      if (DEBUG) Serial.println("Device disconnected from AP");
    }
  }
  
  WiFiClient client = server.available();   // listen for incoming clients

   if (client) {                             // if you get a client,
    if (DEBUG) Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

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
        if (currentLine.endsWith("GET /Begin"))
        {
          //Handle clicking the Begin button
          MakeWebPage(client,"<h1>Rawpter V1.0 </h1><div class='alert alert-success'>Starting Drone!</div>");
          ALLOW_WIFI=false;             
        } 
        else if (currentLine.endsWith("GET / HTTP"))
        { //Handle hitting the basic page (1st connection)
          MakeWebPage(client,"<h1>Rawpter V1.0 </h1><a href='/Begin' class='btn btn-secondary'>Begin</a>");
        }
      }
    }
    // close the connection:
    client.stop();
    if (DEBUG) Serial.println("client disconnected");
  }
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
  client.print("<div class='container'>");
  client.print(html);
  client.print("</div>");
  client.print("<script src=\"https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/js/bootstrap.bundle.min.js\" integrity=\"sha384-ka7Sk0Gln4gmtz2MlQnikT1wXgYsOg+OMhuP+IlRH9sENBO0LRn5q+8nbTov4+1p\" crossorigin=\"anonymous\"></script>");
  client.println(); // The HTTP response ends with another blank line:
}
void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  if (DEBUG) Serial.print("SSID: ");
  if (DEBUG) Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  if (DEBUG) Serial.print("IP Address: ");
  if (DEBUG) Serial.println(ip);

  // print where to go in a browser:
  if (DEBUG) Serial.print("To see this page in action, open a browser to http://");
  if (DEBUG) Serial.println(ip);
}

void tick()
{
  //Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;      
  current_time = micros();      
  deltaTime = (current_time - prev_time)/1000000.0;
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
   
  //Quad mixing - EXAMPLE
  m1_command_scaled = thro_des - pitch_PID + roll_PID + yaw_PID; //Front left
  m2_command_scaled = thro_des - pitch_PID - roll_PID - yaw_PID; //Front right
  m3_command_scaled = thro_des + pitch_PID - roll_PID + yaw_PID; //Back Right
  m4_command_scaled = thro_des + pitch_PID + roll_PID - yaw_PID; //Back Left
}

void IMUinit() {
  //DESCRIPTION: Initialize IMU
  if (!IMU.begin()) {
    if (DEBUG) Serial.println("Failed to initialize IMU!");
    while (true) {if (DEBUG) Serial.println("IMU Failed"); delay(2000);}
  }
}

void getIMUdata() {
  //DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data
 //Accelerometer
 
  if (IMU.accelerationAvailable()) {
  float AcX,AcY,AcZ;

    IMU.readAcceleration(AcX, AcY, AcZ);
    AccX = AcX; //G's
    AccY = AcY;
    AccZ = AcZ;
    //Correct the outputs with the calculated error values
    AccX = AccX - AccErrorX;
    AccY = AccY - AccErrorY;

  }
  if (IMU.gyroscopeAvailable()) {
  //Gyro
    float GyX,GyY,GyZ;
    IMU.readGyroscope(GyX, GyY, GyZ);
    GyroX = GyX; //deg/sec
    GyroY = GyY;
    GyroZ = GyZ;
    //Correct the outputs with the calculated error values
    GyroX = GyroX - GyroErrorX;
    GyroY = GyroY - GyroErrorY;
    GyroZ = GyroZ - GyroErrorZ;
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
    tock(IMUTIMING); //do not exceed 2000Hz
  }
}
void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az)
{
      // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll_IMU = filter.getRoll();
    pitch_IMU = filter.getPitch();
    yaw_IMU = filter.getYaw();
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

void controlANGLE() {
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
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll*deltaTime;
  if (PWM_throttle < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = GyroX;
  roll_PID = 0.01*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch*deltaTime;
  if (PWM_throttle < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = GyroY;
  pitch_PID = .01*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*deltaTime;
  if (PWM_throttle < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/deltaTime; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

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
  m1_command_PWM = m1_command_scaled*180;
  m2_command_PWM = m2_command_scaled*180;
  m3_command_PWM = m3_command_scaled*180;
  m4_command_PWM = m4_command_scaled*180;
  //Constrain commands to motors within Simonk bounds
  //1400, full throttle at 2000us
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
  float b = 0.7; //Lower=slower, higher=noiser

  PWM_throttle = (1.0 - b)*PWM_throttle_prev + b*PWM_throttle;
  PWM_roll = (1.0 - b)*PWM_roll_prev + b*PWM_roll;
  PWM_Elevation = (1.0 - b)*PWM_Elevation_prev + b*PWM_Elevation;
  PWM_Rudd = (1.0 - b)*PWM_Rudd_prev + b*PWM_Rudd;

  PWM_throttle_prev = PWM_throttle;
  PWM_roll_prev =PWM_roll;
  PWM_Elevation_prev = PWM_Elevation;
  PWM_Rudd_prev = PWM_Rudd;
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
  
  unsigned minVal = 800;
  unsigned maxVal = 2200;

  //Triggers for failure criteria
  if (PWM_ThrottleCutSwitch>maxVal || PWM_ThrottleCutSwitch<minVal || PWM_throttle > maxVal || PWM_throttle < minVal || PWM_roll > maxVal ||PWM_roll < minVal || PWM_Elevation > maxVal || PWM_Elevation < minVal || PWM_Rudd > maxVal || PWM_Rudd < minVal) 
  {
    failsafed=true;
    PWM_throttle = PWM_throttle_fs;
    PWM_roll = PWM_roll_fs;
    PWM_Elevation = PWM_elevation_fs;
    PWM_Rudd = PWM_rudd_fs;
    PWM_ThrottleCutSwitch=PWM_ThrottleCutSwitch_fs; //this is so the throttle cut routine doesn't override the fail safes.
  } else failsafed=false;
  if (EASYCHAIR) PWM_throttle=2000;//For testing in the easy chair with the Arduino out of the drone.  See the compiler directive at the top of the code.
}

void commandMotors() {
  //DESCRIPTION: Send pulses to motor pins
  m1PWM.write(m1_command_PWM);
  m2PWM.write(m2_command_PWM);
  m3PWM.write(m3_command_PWM);
  m4PWM.write(m4_command_PWM);
  if (DEBUG) Serial.println("Made it to bottom of commandmotors");  
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
  if (PWM_ThrottleCutSwitch < 1300) { killMotors(); return; }
  if (throttle_is_cut&&PWM_ThrottleCutSwitch>1500)
  {
    if (PWM_throttle<1040){
      throttle_is_cut=false;
    } else killMotors();
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
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

void loopBlink() {
  //DESCRIPTION: Blink LED on board to indicate main loop is running

  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    digitalWrite(LED_BUILTIN, blinkAlternate);

    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 100000;
    }
    else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 2000000;
    }
  }
}

void printRadioData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F(" CH1: "));
    Serial.print(PWM_throttle);
    Serial.print(F(" CH2: "));
    Serial.print(PWM_roll);
    Serial.print(F(" CH3: "));
    Serial.print(PWM_Elevation);
    Serial.print(F(" CH4: "));
    Serial.println(PWM_Rudd);
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
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("{\"roll\": "));
    Serial.print(roll_IMU);
    Serial.print(F(", \"pitch\": "));
    Serial.print(pitch_IMU);
    Serial.print(F(", \"yaw\": "));
    Serial.print(yaw_IMU);
    
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

    Serial.print(F(", \"ThroDes\": "));
    Serial.print(thro_des);
    Serial.print(F(", \"RollDes\": "));
    Serial.print(roll_des);
    Serial.print(F(", \"PitchDes\": "));
    Serial.print(pitch_des);
    Serial.print(F(", \"YawDes\": "));
    Serial.print(yaw_des);

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

    Serial.print(F(", \"Failsafe\": "));
    Serial.print(failsafed);


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
