//Rawpter 1.7 by Sean J. Miller
//Flight Controller code for an Arduino Nano RP2040 Connect
//Go to https://raisingawesome.site/projects for more info
//MIT License - use at your own risk

#include <SPI.h>
#include <WiFiNINA.h>          //WiFi
#include <Arduino_LSM6DSOX.h>  //IMU get from the Arduino IDE Library Manager
#include <Servo.h>             //get from the Arduino IDE Library Manager

//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //
//========================================================================================================================//
//EASYCHAIR is used to put in some key dummy variables so you can sit in the easy chair with just the Arduino on a cord wiggling it around and seeing how it responds.
//Set EASYCHAIR to false when you are wanting to install it in the drone.
#define EASYCHAIR false

//The LOOP_TIMING is based on the IMU.  For the Arduino_LSM6DSOX, it is 104Hz.  So, the loop time is set a little longer so the IMU has time to update from the control change.
#define LOOP_TIMING 100

//The battery alarm code handles 14.8 or 7.4V LIPOs.  Set to your type of battery. If not hooked up, it will pull down and beep often.
#define BATTERYTYPE 14.8
#define BUZZER_PIN 9
int batteryVoltage = 777;             //just a default for the battery monitoring routine
unsigned long next_voltage_check = 0;  //used in loopBuzzer to check for voltage on the main battery.
bool beeping = false;         //For tracking beeping when the battery is getting low.

//Radio failsafe values for every channel in the event that bad reciever data is detected.
//These are for it to stay stable and descend safely versus totally cutting throttle and drop like a rock.
unsigned long PWM_throttle_zero = 1000;         //used when we want to take throttle to zero.  Failsafe is something higher as it is expected that failsafe is a value needed to safely land.
unsigned long PWM_throttle_fs = 1000;           //throttle  will allow it to descend to the ground if you adjust the throttlecutswitch_fs to 2000
unsigned long PWM_roll_fs = 1500;               //ail pretty much in the middle so it quits turning
unsigned long PWM_elevation_fs = 1500;          //elev
unsigned long PWM_rudd_fs = 1500;               //rudd
unsigned long PWM_ThrottleCutSwitch_fs = 1000;  //SWA less than 1300, cut throttle - must config a switch to Channel 5 in your remote.
bool UPONLYMODE = false;                        //UPONLYMODE is used to take the radio out of it other than thrust.  The intent is to get it tuned so it can at least take off.  Should be false once tuned.
float stick_dampener = 0.1;                     //0.1-1 Lower=slower, higher=noiser default 0.7

bool failsafed = false;
//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
float AccErrorX = -0.03;
float AccErrorY = 0.00;
float AccErrorZ = 0.01;
float GyroErrorX = 0.37;
float GyroErrorY = -0.04;
float GyroErrorZ = -0.45;

float Gyro_filter = .97;
float Accel_filter = .97;

float B_madgwick = 0.02;  //(default 0.04)
float q0 = 1.0f;          //Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

//Controller parameters (this is where you "tune it".  It's best to use the WiFi interface to do it live and then update once its tuned.):
float i_limit = 60;   //Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 18.0;   //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode (default 30.0)
float maxPitch = 18.0;  //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode (default 30.0)
float maxYaw = 10.0;    //Max yaw rate in deg/sec (default 160.0)
float maxMotor = 0.8;

float Kp_roll_angle = 10;  //Roll P-gain
float Ki_roll_angle = 8;  //Roll I-gain
float Kd_roll_angle = 2;  //Roll D-gain

float Kp_pitch_angle = 15;  //Pitch P-gain
float Ki_pitch_angle = 8;   //Pitch I-gain
float Kd_pitch_angle = 2;  //Pitch D-gain

float Kp_yaw = 0;  //Yaw P-gain default 30
float Ki_yaw = 0;  //Yaw I-gain default 5
float Kd_yaw = 0;  //Yaw D-gain default .015 (be careful when increasing too high, motors will begin to overheat!)

//========================================================================================================================//
//                                                     DECLARE PINS                                                       //
//========================================================================================================================//
//Radio:
//used so I quit getting confused begween the ARduino pins and the Radio Channel pins.
const int stickRightHorizontal = 2;  //right horizontal stick
const int stickRightVertical = 3;    //right vertical stick
const int stickLeftVertical = 4;     //left vertical stick
const int stickLeftHorizontal = 5;   //left horizontal stick
const int SwitchA = 6;               //SWA switch

const int throttlePin = stickLeftVertical;  //throttle - up and down on the
const int rollPin = stickRightHorizontal;   //ail (roll)
const int upDownPin = stickRightVertical;   //ele
const int ruddPin = stickLeftHorizontal;    //rudd
const int throttleCutSwitchPin = SwitchA;   //gear (throttle cut)

//variables for reading PWM from the radio receiver
unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6;
unsigned long channel_1_raw = 0;
unsigned long channel_2_raw = 0;
unsigned long channel_3_raw = 0;
unsigned long channel_4_raw = 0;
unsigned long channel_5_raw = 0;

int ppm_counter = 0;
unsigned long time_ms = 0;
int throttleCutCounter = 0;
int throttleNotCutCounter = 0;
//Motor Electronic Speed Control Modules (ESC):
const int m1Pin = 10;  //10
const int m2Pin = 15;  //15
const int m3Pin = 16;  //16
const int m4Pin = 14;  //14
Servo m1PWM, m2PWM, m3PWM, m4PWM;

//========================================================================================================================//
//DECLARE GLOBAL VARIABLES
//========================================================================================================================//

//General stuff for controlling timing of things
float deltaTime;
float invFreq = (1.0 / LOOP_TIMING) * 1000000.0;

unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
bool throttle_is_cut = true;  //used to force the pilot to manually set the throttle to zero after the switch is used to throttle cut

//Radio communication:
unsigned long PWM_throttle, PWM_roll, PWM_Elevation, PWM_Rudd, PWM_ThrottleCutSwitch;
unsigned long PWM_throttle_prev, PWM_roll_prev, PWM_Elevation_prev, PWM_Rudd_prev;

//IMU:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;

//Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;

//Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

//Mixer
float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled;

int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM;
unsigned long buzzer_millis;
unsigned long buzzer_spacing = 10000;

//WiFi Begin
int keyIndex = 0;
int status = WL_IDLE_STATUS;
bool ALLOW_WIFI = true;

WiFiServer server(80);
//WiFi End

//========================================================================================================================//
//BEGIN THE CLASSIC SETUP AND LOOP
//========================================================================================================================//

void setup() {
  //Bootup operations
  
  analogReadResolution(12); //The RP2040 has a 12 bit ADC versus the standard 10 from the Uno
  setupSerial();
  if (ALLOW_WIFI) setupWiFi();  //At first power on, a WiFi hotspot is set up for talking to the drone. (SSID Rawpter, 12345678)
  setupDrone();
  setupBatteryMonitor();
}

void loop() {
  tick();  //stamp the start time of the loop to keep our timing to 100Hz.  See tock() below.
  if (throttle_is_cut) 
  {
    //We are only going to allow these to slow down the loop if the throttle is cut.  Otherwise, the loop speed is too irregular.
    loopBuzzer();
    if (ALLOW_WIFI) loopWiFi();
  }
  loopDrone();
  tock();
}

void setupDrone() {
  //Initialize all pins
  pinMode(m1Pin, OUTPUT);
  pinMode(m2Pin, OUTPUT);
  pinMode(m3Pin, OUTPUT);
  pinMode(m4Pin, OUTPUT);
  m1PWM.attach(m1Pin, 1060, 1860);
  m2PWM.attach(m2Pin, 1060, 1860);
  m3PWM.attach(m3Pin, 1060, 1860);
  m4PWM.attach(m4Pin, 1060, 1860);
  //Set built in LED to turn on to signal startup
  delay(5);

  //Initialize radio communication
  radioSetup();
  setToFailsafe();
  PWM_throttle = PWM_throttle_zero;  //zero may not necessarily be the failsafe, but on startup we want zero.

  //Initialize IMU communication
  IMUinit();

  //Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
 // calculate_IMU_error(); //Calibration parameters printed to serial monitor. Paste these in the user specified variables section, then comment this out forever.

  if (getRadioPWM(1) > 1800 && getRadioPWM(1) < 2400 & !EASYCHAIR) calibrateESCs();  //if the throttle is up, first calibrate ESCs before going into the loop

  m1_command_PWM = 0;  //Will send the default for motor stopped for Simonk firmware
  m2_command_PWM = 0;
  m3_command_PWM = 0;
  m4_command_PWM = 0;


  while (getRadioPWM(1) > 1060 && getRadioPWM(1) < 2400 && !EASYCHAIR && getRadioPWM(5) < 1300)  //wait until the throttle is turned down and throttlecut switch is not engaged before allowing anything else to happen.
  {
    delay(1000);
  }
}

void loopDrone() {
  getIMUdata();                                            //Pulls raw gyro andaccelerometer data from IMU and applies LP filters to remove noise
  Madgwick6DOF(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ);  //Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)
  getDesiredAnglesAndThrottle();                           //Convert raw commands to normalized values based on saturated control limits
  PIDControlCalcs();                                       //The PID functions. Stabilize on angle setpoint from getDesiredAnglesAndThrottle
  controlMixer();                                          //Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
  scaleCommands();                                         //Scales motor commands to 0-1
  throttleCut();                                           //Directly sets motor commands to off based on channel 5 being switched
    #if EASYCHAIR
    Troubleshooting();                                       //will do the print routines that are uncommented
    #endif
  commandMotors();                                         //Sends command pulses to each ESC pin to drive the motors
  getRadioSticks();                                        //Gets the PWM from the radio receiver
  failSafe();                                              //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup
}

void setupBatteryMonitor() 
{
  buzzer_millis = millis();
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(A6, INPUT);
}

void loopBuzzer() 
{  //this monitors the battery.  the lower it gets, the faster it beeps.
  unsigned long myTime=millis();
  if (!beeping) {
    if (myTime - buzzer_millis > (buzzer_spacing)) {
      digitalWrite(BUZZER_PIN, HIGH);
      beeping = true;
      buzzer_millis = myTime;
    }
  } else {
    if (myTime - buzzer_millis > 80) {
      beeping = false;
      buzzer_millis = myTime;
      digitalWrite(BUZZER_PIN, LOW);
    }
  }

  if (myTime > next_voltage_check) {
    next_voltage_check = myTime + 10000;  //checkvoltage once every 10 seconds versus every loop.    
    batteryVoltage = analogRead(A6);
    if (BATTERYTYPE == 14.8) {
      //based on 330K and 51K voltage divider that takes 16.8V to 2.25V
      if (batteryVoltage > 1457) buzzer_spacing = 40000;
      else if (batteryVoltage > 1440 ) buzzer_spacing = 30000;
      else if (batteryVoltage > 1400) buzzer_spacing = 20000;
      else if (batteryVoltage > 1350) buzzer_spacing = 10000;
      else if (batteryVoltage > 1301) buzzer_spacing = 500;
      else buzzer_spacing = 100;
    } else {
      //Depends on your resistors and battery choice
    }
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
  #if EASYCHAIR
    printJSON();
  #endif
}

void setupSerial() {
  Serial.begin(2000000);
  delay(100);
}

void setupWiFi() {
  char ssid[] = "_Rawpter";
  char pass[] = "12345678";

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    // don't continue because this is probably not an RP2040
    while (true)
      ;
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
    while (true)
      ;
  }

  // wait 1 seconds for connection:
  delay(1000);

  // start the web server on port 80
  server.begin();
}

void loopWiFi() {
  // compare the previous status to the current status
  WiFiClient client = server.available();  // listen for incoming clients
  if (client) {                            // if you get a client,
    String currentLine = "";               // make a String to hold incoming data from the client
    while (client.connected()) {           // loop while the client's connected
      if (client.available()) {            // if there's bytes to read from the client,
        char c = client.read();            // read a byte, then
        if (c == '\n') {
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            break;
          } else {  // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
        if (currentLine.endsWith("GET /?")) {
          setTheValuesFromUserForm(client);
          //Handle clicking the Begin button
          MakeWebPage(client, "<meta http-equiv = \"refresh\" content = \"0; url = http://192.168.2.4 \"/>");
        } else if (currentLine.endsWith("GET / HTTP")) {  //Handle hitting the basic page (1st connection)
          String myMsg = "<h1>Rawpter V1.7</h1><small>by Raising Awesome</small><br>";
          //For the battery voltage calc, you can use ohm's law on your chosen voltage divider resistors and get the voltage ratio of the 12bit ADC.  I simply recorded values against fed voltages from my bench power supply and fit 
          //a line.  Good ole' y=mx+b.
          myMsg += "<b>Snapshot:</b><br>Desired Roll=" + String(roll_des) + "&#176;&nbsp;&nbsp;&nbsp;IMU Roll=" + String(roll_IMU) + "&#176;<br>Desired Pitch= " + String(pitch_des) + "&#176;&nbsp;&nbsp;&nbsp;IMU Pitch=" + String(pitch_IMU) + "&#176;<br>Loop Time= " + String(int(round(1 / (deltaTime)))) + "&nbsp;&nbsp;&nbsp;Throttle PWM=" + String(PWM_throttle) + "<br>Battery=" + String( (4.4695/104.0+((float)batteryVoltage/104.0)),1) + "V (" + String(batteryVoltage) + ")<br>";
          if (batteryVoltage<1350) myMsg+="<br><div class='alert alert-danger'>DANGER: BATTERY LOW!</div><br>";
          myMsg += GetParameters() + "<br><input class='mt-2 btn btn-primary' type=submit value='submit' />";
          MakeWebPage(client, myMsg);
        }
      }
    }
    // close the connection:
    client.stop();
  }
}

void setTheValuesFromUserForm(WiFiClient client) {
  String currentLine = "";
  UPONLYMODE = false;
  while (client.connected()) {  // loop while the client's connected
    if (client.available()) {   // if there's bytes to read from the client,
      char c = client.read();   // read a byte, then
      if (c == '\n') {
        // if the current line is blank, you got two newline characters in a row.
        // that's the end of the client HTTP request, so send a response:
        if (currentLine.length() == 0) {
          break;
        } else {  // if you got a newline, then clear currentLine:
          currentLine = "";
        }
      } else if (c != '\r') {  // if you got anything else but a carriage return character,
        currentLine += c;      // add it to the end of the currentLine
      }
      if (currentLine.endsWith("maxMotor=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        maxMotor = myValue.toFloat();
      }
      if (currentLine.endsWith("stick_dampener=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        stick_dampener = myValue.toFloat();
      }

      if (currentLine.endsWith("i_limit=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        i_limit = myValue.toFloat();
      }
      if (currentLine.endsWith("Accel_filter=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        Accel_filter = myValue.toFloat();
      }
      if (currentLine.endsWith("Gyro_filter=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        Gyro_filter = myValue.toFloat();
      }

      if (currentLine.endsWith("UPONLYMODE=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        if (myValue == "true") UPONLYMODE = true;
        else UPONLYMODE = false;
      }

      if (currentLine.endsWith("kp_roll_angle=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        Kp_roll_angle = myValue.toFloat();
      }

      if (currentLine.endsWith("ki_roll_angle=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        Ki_roll_angle = myValue.toFloat();
      }
      if (currentLine.endsWith("kd_roll_angle=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        Kd_roll_angle = myValue.toFloat();
      }

      //
      if (currentLine.endsWith("kp_pitch_angle=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        Kp_pitch_angle = myValue.toFloat();
      }
      if (currentLine.endsWith("ki_pitch_angle=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        Ki_pitch_angle = myValue.toFloat();
      }
      if (currentLine.endsWith("kd_pitch_angle=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        Kd_pitch_angle = myValue.toFloat();
      }
      //yaw
      if (currentLine.endsWith("kp_yaw=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        Kp_yaw = myValue.toFloat();
      }
      if (currentLine.endsWith("ki_yaw=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        Ki_yaw = myValue.toFloat();
      }
      if (currentLine.endsWith("kd_yaw=")) {
        String myValue = "";

        while (!currentLine.endsWith("&")) {
          c = client.read();
          if (c != '&') myValue += c;
          currentLine += c;
        }
        Kd_yaw = myValue.toFloat();
        return;
      }
    }
  }
}

String GetParameters() {
  String myString;
  myString = myString + "<table><tr><td>Max Motor Speed (0.0-1.0):</td><td><input type=number step=.001 name=maxMotor style='width:70px;' value='" + String(maxMotor) + "'></td></tr>";
  myString = myString + "<tr><td>Stick Dampening (0.01-1.0):<br>0.1=slow/steady, 1.0=noisy/fast</td><td><input type=number step=.001 name=stick_dampener style='width:70px;' value='" + String(stick_dampener) + "'></td></tr>";
  myString = myString + "<tr><td>Integral Accumulation (25 default):</td><td><input type=number step=.001 name=i_limit style='width:70px;' value='" + String(i_limit) + "'></td></tr>";
  myString = myString + "<tr><td>Accel Dampening (0.1-1.0):<br>0.1=slow/steady, 1.0=noisy/fast</td><td><input type=number step=.001 name=Accel_filter style='width:70px;' value='" + String(Accel_filter) + "'></td></tr>";
  myString = myString + "<tr><td>Gyro Dampening (0.1-1.0 ):<br>0.1=slow/steady, 1.0=noisy/fast</td><td><input type=number step=.001 name=Gyro_filter style='width:70px;' value='" + String(Gyro_filter) + "'></td></tr>";
  String myVal;
  if (!UPONLYMODE) myVal = "";
  else myVal = "checked";
  myString = myString + "<tr><td>Up Mode Only:</td><td><input type=CHECKBOX name=UPONLYMODE value='true' " + (myVal) + "></td></tr>";
  myString = myString + "</table>";
  myString = myString + "<table class=table><thead class=thead-dark><th></th><th>Kp</th><th>Ki</th><th>Kd</th></thead>";
  myString = myString + "<tr><td>Roll:</td><td><input name=kp_roll_angle style='width:70px;' type=number step=.001 value='" + String(Kp_roll_angle) + "'></td><td><input style='width:70px;' name=ki_roll_angle type=number step=.001 value='" + String(Ki_roll_angle) + "'></td><td><input name=kd_roll_angle type=number step=.001 style='width:70px;' value='" + String(Kd_roll_angle) + "'></td></tr>";
  myString = myString + "<tr><td>Pitch:</td><td><input name=kp_pitch_angle  style='width:70px;' type=number step=.001 value='" + String(Kp_pitch_angle) + "'></td><td><input  style='width:70px;' name=ki_pitch_angle type=number step=.001 value='" + String(Ki_pitch_angle) + "'></td><td><input name=kd_pitch_angle type=number step=.001 style='width:70px;' value='" + String(Kd_pitch_angle) + "'></td></tr>";
  myString = myString + "<tr><td>Yaw:</td><td><input name=kp_yaw type=number step=.001 style='width:70px;' value='" + String(Kp_yaw) + "'></td><td><input  style='width:70px;' type=number step=.001 name=ki_yaw value='" + String(Ki_yaw) + "'></td><td><input type=number step=.001 name=kd_yaw style='width:70px;' value='" + String(Kd_yaw) + "'></td></tr><input type=hidden name=ender value='0'>";
  myString = myString + "</table><br>Tip: To use your parameters beyond this flight session, snapshot the screen for reference and update the code.<br>";
  return myString;
}

void MakeWebPage(WiFiClient client, String html) {
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
  client.print(tuningProcedure());
  client.print("</div></form>");
  client.print("<script src=\"https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/js/bootstrap.bundle.min.js\" integrity=\"sha384-ka7Sk0Gln4gmtz2MlQnikT1wXgYsOg+OMhuP+IlRH9sENBO0LRn5q+8nbTov4+1p\" crossorigin=\"anonymous\"></script>");
  client.println();  // The HTTP response ends with another blank line:
}

void tick() {
  //Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;
  current_time = micros();
  deltaTime = (current_time - prev_time) / 1000000.0;  //division takes it from micros to seconds.  1000000 ms=1 second
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
   * normalized (0 to 1) thro_des command for throttle control.
   * 
   *Relevant variables:
   *thro_des - direct thottle control
   *roll_PID, pitch_PID, yaw_PID - stabilized axis variables  
   */

  //Quad mixing. maxMotor is used to keep the motors from being too violent if you have a big battery and concers about that.
  m1_command_scaled = maxMotor * (thro_des) - pitch_PID + roll_PID + yaw_PID;    //Front left
  m2_command_scaled = maxMotor * (thro_des) - pitch_PID - roll_PID - yaw_PID;    //Front right
  m3_command_scaled = maxMotor * (thro_des) + pitch_PID - roll_PID + yaw_PID;    //Back Right
  m4_command_scaled = maxMotor * (thro_des) + pitch_PID + roll_PID - yaw_PID;    //Back Left

  m1_command_scaled = constrain(m1_command_scaled, 0, 1.0);
  m2_command_scaled = constrain(m2_command_scaled, 0, 1.0);
  m3_command_scaled = constrain(m3_command_scaled, 0, 1.0);
  m4_command_scaled = constrain(m4_command_scaled, 0, 1.0);
}

void IMUinit() {
  //DESCRIPTION: Initialize IMU
  if (!IMU.begin()) {
    while (true)
      ;
  }
  delay(15);
}

void getIMUdata() {
  //DESCRIPTION: Request full dataset from IMU

  //Accelerometer
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(AccX, AccY, AccZ);
    AccX = AccX - AccErrorX;
    AccY = AccY - AccErrorY;
    AccY = AccY - AccErrorZ;
    //Correct the outputs with the calculated error values
    /*
    //This was used to suppress noise.  However, it just slows things down.  There is already internal filters to the IMU.
    AccX = (1.0 - Accel_filter) * AccX_prev + Accel_filter * AccX;
    AccY = (1.0 - Accel_filter) * AccY_prev + Accel_filter * AccY;
    AccZ = (1.0 - Accel_filter) * AccZ_prev + Accel_filter * AccZ;
    AccX_prev = AccX;
    AccY_prev = AccY;
    AccZ_prev = AccZ;
    */
  } else return;
  if (IMU.gyroscopeAvailable()) {
    //Gyro
    IMU.readGyroscope(GyroX, GyroY, GyroZ);
    //Correct the outputs with the calculated error values
    GyroX = GyroX - GyroErrorX;
    GyroY = GyroY - GyroErrorY;
    GyroZ = GyroZ - GyroErrorZ;
    /*
    //This was used to suppress noise.  However, it just slows things down.  There is already internal filters to the IMU.
    GyroX = (1.0 - Gyro_filter) * GyroX_prev + Gyro_filter * GyroX;
    GyroY = (1.0 - Gyro_filter) * GyroY_prev + Gyro_filter * GyroY;
    GyroZ = (1.0 - Gyro_filter) * GyroZ_prev + Gyro_filter * GyroZ;
    GyroX_prev = GyroX;
    GyroY_prev = GyroY;
    GyroZ_prev = GyroZ;
    */
  }
}

void calculate_IMU_error() {
  //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
  /*
   * The error values it computes are applied to the raw gyro and 
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata().
   * this will correct for sensor drift and crookedness in your copter.
   */
  float AcX, AcY, AcZ, GyX, GyY, GyZ;

  Serial.println("Calculating IMU Error with 10000 iterations. Please stand by...");

  //Read IMU values 1000 times.  Should take around 2 minutes at 104Hz IMU timing.
  int c = 0;AccErrorX=0;AccErrorY=0;AccErrorZ=0;GyroErrorX=0;GyroErrorY=0;GyroErrorZ=0;
  while (c < 10000) {
    while (!IMU.accelerationAvailable()&!IMU.gyroscopeAvailable()) delay(1);
    IMU.readAcceleration(AcX, AcY, AcZ);
    IMU.readGyroscope(GyX, GyY, GyZ);
    AccX = AcX;
    AccY = AcY;
    AccZ = AcZ;
    GyroX = GyX;
    GyroY = GyY;
    GyroZ = GyZ;

    //Sum all readings
    AccErrorX = AccErrorX + AccX;
    AccErrorY = AccErrorY + AccY;
    AccErrorZ = AccErrorZ + AccZ;
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
    Serial.println(String(10000-c));
  }
  //Divide the sum by 12000 to get the error value
  AccErrorX = AccErrorX / c;
  AccErrorY = AccErrorY / c;
  AccErrorZ = AccErrorZ / c - 1.0;
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
  while (true) delay(1000);
}

void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
   * available (for example when using the recommended MPU6050 IMU for the default setup).
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);  //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * deltaTime;
  q1 += qDot2 * deltaTime;
  q2 += qDot3 * deltaTime;
  q3 += qDot4 * deltaTime;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //Compute angles
  roll_IMU = atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * 57.29577951;  //degrees
  pitch_IMU = -asin(-2.0f * (q1 * q3 - q0 * q2)) * 57.29577951;                 //degrees
  yaw_IMU = -atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * 57.29577951;  //degrees
}

void getDesiredAnglesAndThrottle() {
  //DESCRIPTION: Normalizes desired control values to appropriate values
  /*
   * Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
   * RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
   * roll_des and pitch_des are scaled to be within max roll/pitch amount in degrees
   * yaw_des is scaled to be within max yaw in degrees/sec.
   */
  thro_des = (PWM_throttle - 1000.0) / 1000.0;   //Between 0 and 1
  roll_des = (PWM_roll - 1500.0) / 500.0;        //Between -1 and 1
  pitch_des = (PWM_Elevation - 1500.0) / 500.0;  //Between -1 and 1
  yaw_des = (PWM_Rudd - 1500.0) / 500.0;         //Between -1 and 1

  //Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0);                //Between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0) * maxRoll;     //Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0) * maxPitch;  //Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0) * maxYaw;        //Between -maxYaw and +maxYaw
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


  if (PWM_throttle < 1030)
  { //This will keep the motors from spinning with the throttle at zero should the drone be sitting unlevel.
    integral_roll_prev = 0;
    integral_pitch_prev = 0;
    error_yaw_prev = 0;
    integral_yaw_prev = 0;
    roll_PID=0;
    pitch_PID=0;
    yaw_PID=0;
    return;
  }
  if (UPONLYMODE)
  {
    //For troubleshooting.  UPONLYMODE can be set with the web interface.
    roll_des = 0;
    pitch_des = 0;
    yaw_PID = 0;
  }

  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll * deltaTime;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Limit integrator to prevent saturating
  derivative_roll = GyroX; //(roll_des-roll_IMU-roll_des-previous_IMU)/dt=current angular velocity since last IMU read and therefore GyroX in deg/s
  roll_PID = 0.0001 * (Kp_roll_angle * error_roll + Ki_roll_angle * integral_roll - Kd_roll_angle * derivative_roll);  //Scaled by .0001 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch * deltaTime;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit);
  derivative_pitch = GyroY;
  pitch_PID = .0001 * (Kp_pitch_angle * error_pitch + Ki_pitch_angle * integral_pitch - Kd_pitch_angle * derivative_pitch);  //Scaled by .0001 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ versus angle.  In other words, your stick is setting y axis rotation speed - not the angle to get to.
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * deltaTime;
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit);
  derivative_yaw = (error_yaw - error_yaw_prev) / deltaTime;
  yaw_PID = .0001 * (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw);  //Scaled by .0001 to bring within -1 to 1 range
  
  //Update roll variables
  integral_roll_prev = integral_roll;
  integral_pitch_prev = integral_pitch;
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void scaleCommands() {
  //DESCRIPTION: Scale normalized actuator commands to values for ESC protocol
  /*
   * The actual pulse width is set at the servo attach.
   */
  //Scale to Servo PWM 0-180 degrees for stop to full speed.  No need to constrain since mx_command_scaled already is.
  m1_command_PWM = m1_command_scaled * 180;
  m2_command_PWM = m2_command_scaled * 180;
  m3_command_PWM = m3_command_scaled * 180;
  m4_command_PWM = m4_command_scaled * 180;  
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
  if (PWM_throttle - PWM_throttle_prev < 0)
  {
    //Going down  - slow
    PWM_throttle = (.95) * PWM_throttle_prev + 0.05 * PWM_throttle;
  } else 
  { //Going up - fast
    PWM_throttle = (stick_dampener)*PWM_throttle_prev + (1 - stick_dampener) * PWM_throttle;
  }
  PWM_roll = (1.0 - stick_dampener) * PWM_roll_prev + stick_dampener * PWM_roll;
  PWM_Elevation = (1.0 - stick_dampener) * PWM_Elevation_prev + stick_dampener * PWM_Elevation;
  PWM_Rudd = (1.0 - stick_dampener) * PWM_Rudd_prev + stick_dampener * PWM_Rudd;

  PWM_throttle_prev = PWM_throttle;
  PWM_roll_prev = PWM_roll;
  PWM_Elevation_prev = PWM_Elevation;
  PWM_Rudd_prev = PWM_Rudd;
}

void setToFailsafe() {
  PWM_throttle = PWM_throttle_fs;
  PWM_roll = PWM_roll_fs;
  PWM_Elevation = PWM_elevation_fs;
  PWM_Rudd = PWM_rudd_fs;
  PWM_ThrottleCutSwitch = PWM_ThrottleCutSwitch_fs;  //this is so the throttle cut routine doesn't override the fail safes.
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


  if (PWM_throttle < 800 || PWM_throttle > 2200)  //this is the less conservative version to get through this routine faster.
  {
    failsafed = true;
    setToFailsafe();
  } else failsafed = false;
  
  #if EASYCHAIR
    PWM_throttle = 1500;  //For testing in the easy chair with the Arduino out of the drone.  See the compiler directive at the top of the code.
  #endif
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
  #if EASYCHAIR 
    //This is set above in compiler directives for when you are testing the Arduino outside of the drone.
    throttle_is_cut = false;
    return;
  #endif

  if (throttle_is_cut) killMotors();  //make sure we keep those motors at 0 if the throttle was cut.

  if (PWM_ThrottleCutSwitch < 1300) {
    //throttleCutCounter will ensure it is not just a blip that has caused a false cut.
    if (++throttleCutCounter > 10) killMotors();
    return;
  }
  if (throttle_is_cut && PWM_ThrottleCutSwitch > 1500) {
    //reset only if throttle is down to prevent a jolting suprise
    if (PWM_throttle < 1040 && ++throttleNotCutCounter > 10) {
      throttle_is_cut = false;
      throttleNotCutCounter = 0;
      throttleCutCounter = 0;
    } else killMotors();
    return;
  }

  //Handle when things are going upside down
  if (roll_IMU > 55 || roll_IMU < -55 || pitch_IMU > 55 || pitch_IMU < -55) {
    if (++throttleCutCounter > 4) killMotors();
    return;
  }
  throttleNotCutCounter = 0;
  throttleCutCounter = 0;
}

void killMotors() {
  //sets the PWM to its lowest value to shut off a motor such as whne the throttle cut switch is fliped.
  throttle_is_cut = true;
  throttleCutCounter=10; //to prevent overflowing float
  m1_command_PWM = 0;
  m2_command_PWM = 0;
  m3_command_PWM = 0;
  m4_command_PWM = 0;
}

void tock() {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. I have matched 
   * it just below the Gyro update frequency.  invFreq is set at the top of the code.
   */
  //Sit in loop until appropriate time has passed while checking for any WiFi client activity.
  while (invFreq > (micros() - current_time)) {
    if (ALLOW_WIFI&&throttle_is_cut) loopWiFi();
  };
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
void printJSON() {
  if (current_time - print_counter > 34000) {  //Don't go too fast or it slows down the main loop
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
    Serial.print(deltaTime * 1000000.0);
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
    Serial.println(deltaTime * 1000000.0);
  }
}

//========================================================================================================================//

//This file contains all necessary functions and code used for radio communication to avoid cluttering the main code

void radioSetup() {
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
  } else if (ch_num == 2) {
    returnPWM = channel_2_raw;
  } else if (ch_num == 3) {
    returnPWM = channel_3_raw;
  } else if (ch_num == 4) {
    returnPWM = channel_4_raw;
  } else if (ch_num == 5) {
    returnPWM = channel_5_raw;
  }
  return returnPWM;
}

//========================================================================================================================//

//INTERRUPT SERVICE ROUTINES (for reading PWM and PPM)

void getCh1() {
  int trigger = digitalRead(throttlePin);
  if (trigger == 1) {
    rising_edge_start_1 = micros();
  } else if (trigger == 0) {
    channel_1_raw = micros() - rising_edge_start_1;
  }
}

void getCh2() {
  int trigger = digitalRead(rollPin);
  if (trigger == 1) {
    rising_edge_start_2 = micros();
  } else if (trigger == 0) {
    channel_2_raw = micros() - rising_edge_start_2;
  }
}

void getCh3() {
  int trigger = digitalRead(upDownPin);
  if (trigger == 1) {
    rising_edge_start_3 = micros();
  } else if (trigger == 0) {
    channel_3_raw = micros() - rising_edge_start_3;
  }
}

void getCh4() {
  int trigger = digitalRead(ruddPin);
  if (trigger == 1) {
    rising_edge_start_4 = micros();
  } else if (trigger == 0) {
    channel_4_raw = micros() - rising_edge_start_4;
  }
}

void getCh5() {
  int trigger = digitalRead(throttleCutSwitchPin);
  if (trigger == 1) {
    rising_edge_start_5 = micros();
  } else if (trigger == 0) {
    channel_5_raw = micros() - rising_edge_start_5;
  }
}

String tuningProcedure() {
  return "<hr><h3><b>Ziegler-Nichols</b></h3>  <p>The Ziegler-Nichols tuning method is one of the most famous ways to experimentally tune a PID controller. The basic algorithm is as follows:</p><ol><li>Turn off the Integral and Derivative components for the controller; only use Proportional control.</li><li>Slowly increase the gain (i.e. <em>K<sub>p</sub></em>, the Proportion constant) until the process starts to oscillate<br />This final gain value is known as the ultimate gain, or <em>K<sub>u</sub></em><br />The period of oscillation is the ultimate period, or <em>T<sub>u</sub></em></li><li>Use the following table to derive the PID variables</li></ol></div></div><div class=\"sqs-block code-block sqs-block-code\" data-block-type=\"23\" id=\"block-yui_3_17_2_1_1598301478634_207710\"><div class=\"sqs-block-content\"><style type=\"text/css\">.tg  {border-collapse:collapse;border-spacing:0;}.tg td{border-color:black;border-style:solid;border-width:1px;  overflow:hidden;padding:10px 5px;word-break:normal;}.tg th{border-color:black;border-style:solid;border-width:1px;  font-weight:normal;overflow:hidden;padding:10px 5px;word-break:normal;}.tg .tg-gr1d{background-color:#e9f4fc;border-color:#337494;font-weight:bold;text-align:left;vertical-align:top}.tg .tg-m4ds{border-color:#337494;text-align:center;vertical-align:top}.tg .tg-bxxa{border-color:#337494;text-align:left;vertical-align:top}.tg .tg-hk19{background-color:#e9f4fc;border-color:#337494;font-weight:bold;text-align:center;vertical-align:top}.tg .tg-vaq8{background-color:#e9f4fc;border-color:#337494;text-align:left;vertical-align:top}.tg .tg-e1cu{background-color:#e9f4fc;border-color:#337494;text-align:center;vertical-align:top}</style><table class=\"table\"><thead>  <tr>    <th class=\"tg-gr1d\">Controller</th>    <th class=\"tg-hk19\">K<sub>p</sub></th>    <th class=\"tg-hk19\">T<sub>i</sub></th>    <th class=\"tg-hk19\">T<sub>d</sub></th>  </tr></thead><tbody>  <tr>    <td class=\"tg-bxxa\">P</td>    <td class=\"tg-m4ds\">K<sub>u</sub>/2</td>    <td class=\"tg-m4ds\"></td>    <td class=\"tg-m4ds\"></td>  </tr>  <tr>    <td class=\"tg-vaq8\">PI</td>    <td class=\"tg-e1cu\">K<sub>u</sub>/2.5</td>    <td class=\"tg-e1cu\">T<sub>u</sub>/1.25</td>    <td class=\"tg-e1cu\"></td>  </tr>  <tr>    <td class=\"tg-bxxa\">PID</td>    <td class=\"tg-m4ds\">0.6K<sub>u</sub></td>    <td class=\"tg-m4ds\">T<sub>u</sub>/2</td>    <td class=\"tg-m4ds\">T<sub>u</sub>/8</td>  </tr>  <tr>    <td class=\"tg-vaq8\">Pessen Integral Rule<br></td>    <td class=\"tg-e1cu\">0.7K<sub>u</sub></td>    <td class=\"tg-e1cu\">0.4T<sub>u</sub></td>    <td class=\"tg-e1cu\">0.15T<sub>u</sub></td>  </tr>  <tr>    <td class=\"tg-bxxa\">Moderate overshoot</td>    <td class=\"tg-m4ds\">K<sub>u</sub>/3</td>    <td class=\"tg-m4ds\">T<sub>u</sub>/2</td>    <td class=\"tg-m4ds\">T<sub>u</sub>/3</td>  </tr>  <tr>    <td class=\"tg-vaq8\">No overshoot</td>    <td class=\"tg-e1cu\">K<sub>u</sub>/5</td>    <td class=\"tg-e1cu\">T<sub>u</sub>/2</td>    <tdclass=\"tg-e1cu\">T<sub>u</sub>/3</td>  </tr></tbody></table>";
}
//HELPER FUNCTIONS

float invSqrt(float x) {
  //Fast inverse sqrt for madgwick filter
  /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
  /*
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
  */
  return 1.0 / sqrtf(x);  //Teensy is fast enough to just take the compute penalty lol suck it arduino nano
}
