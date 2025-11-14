// ROV Substrate Deployment and Control System
// Microcontroller: Arduino Uno

// --- LIBRARIES ---
#include <Servo.h>          // Required to control ESCs using PWM signals (treats ESCs like servos)
// NOTE: SoftwareSerial library has been removed. We now use Hardware Serial (Pins 0 & 1) 
// to prevent interrupt conflicts with the Servo library that caused thruster jolting.
#include <TinyGPS++.h>      // Library for parsing NMEA sentences and getting clean Lat/Lon

// --- PIN DEFINITIONS ---

// 1. ESC THRUSTER PINS (Digital Pins)
// Connect these to the signal pins of your ESCs.
#define THRUSTER_FORWARD_BACKWARD_1_PIN 9  // Right Horizontal Thruster
#define THRUSTER_FORWARD_BACKWARD_2_PIN 10 // Left Horizontal Thruster
#define THRUSTER_DIVE_UP_1_PIN 11          // Front Vertical Thruster
#define THRUSTER_DIVE_DOWN_2_PIN 12        // Back Vertical Thruster

// 2. RELAY PINS (Digital Pins)
#define LIGHT_RELAY_PIN 6           // Controls the Mini Driving Light
#define ACTUATOR_RELAY_PIN 7        // Controls the Linear Actuator/Valve Drop

// 3. GPS PINS (HardwareSerial - Pins 0 and 1)
// WIRING CHANGE REQUIRED: GPS TX -> Arduino Pin 0 (RX); GPS RX -> Arduino Pin 1 (TX)
// WARNING: Disconnect GPS before uploading code. Cannot use USB Serial Monitor when GPS is connected.
// We use the built-in 'Serial' object for hardware communication.

// GPS Object
TinyGPSPlus gps;

// 4. JOYSTICK INPUT PINS (Analog and Digital)
// Joystick 1: Forward/Backward, Left/Right (Movement)
#define J1_Y_AXIS_PIN A0 // Forward/Backward control
#define J1_X_AXIS_PIN A1 // Left/Right control
#define J1_SWITCH_PIN 4  // Button for GPS/Substrate Drop (Digital Pin)

// Joystick 2: Dive Up/Down
#define J2_Y_AXIS_PIN A2 // Dive Up/Down control
#define J2_SWITCH_PIN 5  // Button for Light Toggle (Digital Pin)

// --- ESC OBJECTS ---
Servo thrusterFB_1;
Servo thrusterFB_2;
Servo thrusterUD_1;
Servo thrusterUD_2;

// --- PWM CONSTANTS (Microseconds for internal calculations) ---
// We keep these in Microseconds for joystick mapping accuracy, but convert to 0-180
// angle before sending to the ESC.
const int US_MIN = 1000;    // Full Reverse (Corresponds to 0 in 0-180 angle)
const int US_STOP = 1500;   // Neutral/Stop (Corresponds to 90 in 0-180 angle)
const int US_MAX = 2000;    // Full Forward (Corresponds to 180 in 0-180 angle)
const int DEADZONE = 100;   // Deadzone for analog stick center (out of 1024). Increased to 100 for better noise filtering.

// --- ESC ANGLE CONSTANTS (0-180) ---
const int ANGLE_STOP = 90;
const int ANGLE_MAX = 180;
const int ANGLE_MIN = 0;


// --- GLOBAL STATE VARIABLES ---
long lastGPSDropTime = 0;
const long dropCooldown = 5000; // 5 seconds cooldown for drop mechanism
bool lightState = false;        // Tracks the current state of the light (ON/OFF)

// --- FUNCTION PROTOTYPES ---
void setupESCs();
void readAndSetThrusters();
void handleDropMechanism();
void handleLightToggle();
void readGPSCoordinates(); 
int convertUSToAngle(int usValue); // New conversion helper function
void updateGPS(); 

// =================================================================
// SETUP
// =================================================================

void setup() {
  // IMPORTANT: Initializes Hardware Serial (Pins 0/1) for GPS.
  // You MUST set your upload software's Serial Monitor to the GPS module's baud rate (9600).
  Serial.begin(9600); 
  Serial.println("ROV Control System Initializing...");
  Serial.println("GPS Module Initialized (TinyGPS++ ready on Hardware Serial).");
  Serial.println("WARNING: Disconnect GPS from Pins 0/1 before uploading new code.");

  // Initialize digital pins for Relays (start LOW/OFF)
  pinMode(LIGHT_RELAY_PIN, OUTPUT);
  digitalWrite(LIGHT_RELAY_PIN, LOW);
  pinMode(ACTUATOR_RELAY_PIN, OUTPUT);
  digitalWrite(ACTUATOR_RELAY_PIN, LOW);

  // Initialize joystick switch pins (using INPUT_PULLUP)
  pinMode(J1_SWITCH_PIN, INPUT_PULLUP);
  pinMode(J2_SWITCH_PIN, INPUT_PULLUP);

  // Attach ESC objects to their respective pins
  // We use the 1000-2000 microseconds range here to override the default
  // 544-2400 range, ensuring 1500us is perfectly centered.
  thrusterFB_1.attach(THRUSTER_FORWARD_BACKWARD_1_PIN, 1000, 2000);
  thrusterFB_2.attach(THRUSTER_FORWARD_BACKWARD_2_PIN, 1000, 2000);
  thrusterUD_1.attach(THRUSTER_DIVE_UP_1_PIN, 1000, 2000);
  thrusterUD_2.attach(THRUSTER_DIVE_DOWN_2_PIN, 1000, 2000);

  // Important: Calibrate/Arm ESCs using the 0-180 logic (which maps to 1000-2000)
  setupESCs();

  Serial.println("Initialization Complete. ROV Ready.");
}

// =================================================================
// MAIN LOOP
// =================================================================

void loop() {
  // 0. CRITICAL: Update GPS data using Hardware Serial
  updateGPS();
  
  // 1. Read and apply thruster power based on joystick input
  readAndSetThrusters();

  // 2. Handle GPS/Substrate Drop mechanism (Joystick 1 Switch)
  handleDropMechanism();

  // 3. Handle Driving Light Toggle (Joystick 2 Switch)
  handleLightToggle();

  delay(10);
}

// =================================================================
// CONTROL FUNCTIONS
// =================================================================

/**
 * @brief Reads incoming GPS data from the Hardware Serial port (Pins 0/1).
 * By using Hardware Serial instead of SoftwareSerial, we eliminate the 
 * interrupt conflict that caused the thrusters to jolt.
 */
void updateGPS() {
  // We use the main 'Serial' object, which is tied to the hardware UART (Pins 0/1).
  while (Serial.available()) {
    gps.encode(Serial.read());
  }
}


/**
 * @brief Converts a 1000-2000 microsecond value (used for internal math) 
 * to the 0-180 angle value (used for Servo.write() compatibility).
 * @param usValue The pulse width in microseconds (1000 to 2000).
 * @return The corresponding angle (0 to 180).
 */
int convertUSToAngle(int usValue) {
    // Map 1000-2000 range to 0-180 range
    return map(usValue, US_MIN, US_MAX, ANGLE_MIN, ANGLE_MAX);
}

/**
 * @brief Initializes and arms the ESCs using the 0-180 angle system (90 is neutral).
 */
void setupESCs() {
  Serial.println("Arming ESCs... Please wait.");
  
  // 1. Send High Signal (Max Angle)
  thrusterFB_1.write(ANGLE_MAX);
  thrusterFB_2.write(ANGLE_MAX);
  thrusterUD_1.write(ANGLE_MAX);
  thrusterUD_2.write(ANGLE_MAX);
  delay(1000); 

  // 2. Send Low Signal (Min Angle)
  thrusterFB_1.write(ANGLE_MIN);
  thrusterFB_2.write(ANGLE_MIN);
  thrusterUD_1.write(ANGLE_MIN);
  thrusterUD_2.write(ANGLE_MIN);
  delay(1000); 

  // 3. Set all to neutral/stop position (90)
  thrusterFB_1.write(ANGLE_STOP);
  thrusterFB_2.write(ANGLE_STOP);
  thrusterUD_1.write(ANGLE_STOP);
  thrusterUD_2.write(ANGLE_STOP);
  delay(1000); 
  Serial.println("ESCs Armed (STOP position 90 angle).");
}


/**
 * @brief Reads joystick analog inputs and controls all 4 thrusters, using 
 * the 0-180 angle format for Servo.write().
 */
void readAndSetThrusters() {
  // --- READ ANALOG INPUTS (0 to 1023) ---
  int J1_Y_Val = analogRead(J1_Y_AXIS_PIN); // Forward/Backward
  int J1_X_Val = analogRead(J1_X_AXIS_PIN); // Left/Right (Yaw)
  int J2_Y_Val = analogRead(J2_Y_AXIS_PIN); // Dive Up/Down (Heave)

  // --- 1. HORIZONTAL THRUSTERS (Forward/Yaw Mixing) ---
  
  // 1a. Forward/Backward Thrust Component (J1 Y-Axis)
  // J1_Y_Val near 0 (Forward on stick) now maps to +500 (Forward Thrust).
  int thrust = map(J1_Y_Val, 0, 1023, 500, -500); 
  // Apply Deadzone: If centered, force 0 thrust
  if (abs(J1_Y_Val - 512) < DEADZONE) {
    thrust = 0; 
  }
  
  // 1b. Left/Right Yaw Component (J1 X-Axis)
  // RE-ENABLED YAW MIXING: Map 0-1023 input to a rotation value from -500 (Left Yaw) to +500 (Right Yaw)
  int rotation = map(J1_X_Val, 0, 1023, -500, 500);
  // Apply Deadzone: If centered, force 0 rotation
  if (abs(J1_X_Val - 512) < DEADZONE) {
    rotation = 0; 
  }
  
  // Calculate final normalized speed for each motor: Thrust +/- Rotation
  // Standard Differential: Left = Thrust + Rotation, Right = Thrust - Rotation
  
  int norm_speed_right = constrain(thrust - rotation, -500, 500); 
  int norm_speed_left = constrain(thrust + rotation, -500, 500); 

  // Convert normalized speed (-500 to 500) back to Microseconds (1000 to 2000)
  int speed_right_us = US_STOP + norm_speed_right;
  int speed_left_us = US_STOP + norm_speed_left;

  // *** FINAL SWAPPED OUTPUT ASSIGNMENT FOR WIRING CORRECTION ***
  // Based on your hardware mistake:
  // FB1 (Pin 9) is connected to the Left Thruster.
  // FB2 (Pin 10) is connected to the Right Thruster.
  thrusterFB_1.write(convertUSToAngle(speed_left_us)); 
  thrusterFB_2.write(convertUSToAngle(speed_right_us));


  // --- 2. VERTICAL THRUSTERS (Dive Up/Down) ---

  // 2a. Dive Up/Down Mapping (J2 Y-Axis)
  int ud_speed_us = map(J2_Y_Val, 0, 1023, US_MIN, US_MAX);
  
  // Apply deadzone for the vertical thrusters
  if (abs(J2_Y_Val - 512) < DEADZONE) {
    ud_speed_us = US_STOP; 
  }
  
  // Constrain and convert to 0-180 angle and apply
  int ud_angle = convertUSToAngle(constrain(ud_speed_us, US_MIN, US_MAX));

  thrusterUD_1.write(ud_angle);
  thrusterUD_2.write(ud_angle);
  
  
  // // Optional: Uncomment the lines below for debugging motor speeds and angles:
  // Serial.print("FB Angle 1 (Left): "); Serial.print(convertUSToAngle(speed_left_us));
  // Serial.print(" | FB Angle 2 (Right): "); Serial.print(convertUSToAngle(speed_right_us));
  // Serial.print(" | UD Angle: "); Serial.println(ud_angle);
  
}


/**
 * @brief Handles the GPS logging and Substrate Drop actuator trigger.
 * * Activated by pressing the J1 Switch.
 */
void handleDropMechanism() {
  if (digitalRead(J1_SWITCH_PIN) == LOW) {
    if (millis() - lastGPSDropTime > dropCooldown) {
      Serial.println("--- DROP MECHANISM ACTIVATED ---");

      // 1. Read and Log GPS Coordinates
      readGPSCoordinates(); 
      
      // 2. Activate Linear Actuator Relay (Substrate Drop)
      digitalWrite(ACTUATOR_RELAY_PIN, HIGH);
      Serial.println("Actuator ON (Substrate dropping)...");
      delay(2000); // Hold actuator for 2 seconds
      digitalWrite(ACTUATOR_RELAY_PIN, LOW);
      Serial.println("Actuator OFF (Valve closed).");
      
      // 3. Reset Cooldown Timer
      lastGPSDropTime = millis();
    } else {
      Serial.print("Drop mechanism on cooldown. Remaining: ");
      Serial.print((dropCooldown - (millis() - lastGPSDropTime)) / 1000.0);
      Serial.println("s");
    }
  }
}

/**
 * @brief Handles the Light Relay toggle.
 * * Activated by pressing the J2 Switch.
 */
void handleLightToggle() {
  if (digitalRead(J2_SWITCH_PIN) == LOW) {
    delay(200); 

    if (digitalRead(J2_SWITCH_PIN) == LOW) {
      lightState = !lightState; 
      digitalWrite(LIGHT_RELAY_PIN, lightState ? HIGH : LOW);
      Serial.print("Driving Light: ");
      Serial.println(lightState ? "ON" : "OFF");
      
      while (digitalRead(J2_SWITCH_PIN) == LOW) {
        delay(50);
      }
    }
  }
}

/**
 * @brief Reads the last valid GPS coordinates using TinyGPS++ and prints them to the local USB serial monitor.
 */
void readGPSCoordinates() {
  if (gps.location.isValid()) {
    float lat = gps.location.lat();
    float lon = gps.location.lng();
    
    Serial.println("--- Substrate Drop Coordinates Captured ---");
    Serial.print("Latitude: "); Serial.println(lat, 6);
    Serial.print("Longitude: "); Serial.println(lon, 6);
    
    Serial.println("\n*** PASTEABLE COORDINATES ***");
    Serial.print(lat, 6);
    Serial.print(",");
    Serial.println(lon, 6);
    Serial.println("*****************************");
    
    Serial.print("Satellites: "); Serial.println(gps.satellites.value());
    Serial.print("Age (ms): "); Serial.println(gps.location.age());
    Serial.println("-------------------------------------------");
    
  } else {
    Serial.println("ERROR: GPS location is not yet valid. Ensure the antenna is above water and wait for a fix.");
    
    if (Serial.available() < 10) {
        Serial.println("STATUS: No GPS data processed yet. Check wiring (TX/RX).");
    }
  }
}

// =================================================================
// POWER WARNING
// =================================================================

/**
 * !!! POWER WARNING !!!
 * * DO NOT ATTEMPT TO POWER THE 4 THRUSTERS THROUGH THE UTP TETHER.
 * * The ROV must be powered by a dedicated battery (e.g., LiPo) located INSIDE the ROV hull.
 * * The UTP cable ONLY carries the low-current control signals (Joysticks).
 */
