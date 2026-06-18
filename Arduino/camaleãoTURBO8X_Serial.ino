/* * 3DChameleon Turbo 8x - Arduino Uno + CNC Shield
 * Adapted for 8 Colors (Dual Extruder) - Jan/2026
 * -----------------------------------------------------------
 * SERIAL COMMANDS (Sent via Klipper/Terminal):
 * * [ SELECTION ]
 * T0 - T7         : Selects tool and moves physical selector.
 * HOME            : Calibrates selector against the physical hard-stop (Channel 0).
 * IDLE            : Parks selector between lobes (relieves filament tension).
 * * [ MOVEMENT ]
 * LOAD <mm>       : Loads up to the nozzle sensor.
 * LOAD <mm> EX <n>: Loads up to the sensor + <n> extra mm.
 * UNLOAD <mm>     : Retracts filament by the defined distance.
 * STOP_LOAD       : Immediate interruption of the load motor.
 * * [ CALIBRATION AND TESTING ]
 * PRESSURE <val>  : Selector offset (+/- steps) for spring tension adjustment.
 * SPEED <val>     : Extrusion speed (Lower = Faster).
 * STRESS          : Automatic cycle of all combinations (T0-T7).
 * STATUS          : Diagnostic report for sensors, tools, and offsets.
 * * [ BUFFER AUTOMATION ]
 * BUFFER_ON/OFF   : Enables/Disables automatic reloading (Autopilot).
 * PROTECAO_ON/OFF : Enables/Disables pause on jammed or run-out filament.
 * * [ ACCESSORIES ]
 * COLETOR_ON/OFF  : Opens/Closes the purge collector servo.
 * -----------------------------------------------------------
 * PARKING MAP (IDLE):
 * T0/T4 -> Pos 2 | T1/T5 -> Pos 3 | T2/T6 -> Pos 0 | T3/T7 -> Pos 1
 */
#include <Servo.h>
#include <EEPROM.h>

// =================================================================================
// PIN MAPPING - CNC SHIELD (MOTORS)
// =================================================================================
// --- Z-Axis: Extruder Motor 1 (T0, T1, T2, T3) ---
#define extEnable             8   // Z_ENABLE (Usually common)
#define extStep               4   // Z_STEP
#define extDir                7   // Z_DIR

// --- Y-Axis: Extruder Motor 2 (T4, T5, T6, T7) ---
#define ext2Enable            8   // Y_ENABLE
#define ext2Step              3   // Y_STEP
#define ext2Dir               6   // Y_DIR

// --- X-Axis: Selector Motor (Tool Selector) ---
#define selEnable             8   // X_ENABLE
#define selStep               2   // X_STEP
#define selDir                5   // X_DIR

// =================================================================================
// PERIPHERALS AND SENSORS
// =================================================================================
#define SERVO_PIN             11  // Collector/cutter servo
#define FILAMENT_SENSOR_PIN   A3  // Hotend sensor (Not used)

// --- Buffer System (NC Microswitches) ---
#define BUFFER_EMPTY_PIN      A1  // ABORT Pin - Buffer Empty Sensor
#define BUFFER_FULL_PIN       A0  // HOLD Pin - Buffer Full Sensor

// =================================================================================
// TIME AND MEMORY SETTINGS (EEPROM)
// =================================================================================
#define BUFFER_CHECK_INTERVAL 200     // Check interval in ms
#define IDLE_TIMEOUT          300000  // Inactivity timeout for Idle (5 mins)

// --- Permanent Memory Addresses ---
#define EEPROM_EXTRUDER_ADDR  0   // Saves current active tool (T0-T7)
#define EEPROM_PRESSURE_ADDR  4   // Saves selector pressure offset

// =================================================================================
// HARDWARE AND CALIBRATION SETTINGS
// =================================================================================
const float STEPS_PER_MM      = 151.0;  // Steps per mm (Z and Y motors)
const int stepsPerRev         = 200;    // Native motor steps
const int microSteps          = 16;     // Configured driver microstepping
const int defaultBackoff      = 5;      // Backoff distance after homing to align Channel 0 (5mm)

// --- Timings and Delays (Speed) ---
int speedDelay                = 170;    // Extruder speed delay (Z and Y)
const int selectorSpeedDelay  = 60;     // Selector speed delay (X)

// --- Direction Definitions ---
const int counterclockwise    = HIGH;
const int clockwise           = !counterclockwise;

// =================================================================================
// SYSTEM AND TOOL STATES (T0-T7)
// =================================================================================
int currentExtruder           = -1;     // Active tool (physical position)

int lastExtruder              = -1;     // Last used tool (determines Z or Y motor)

int currentPhysPos            = 0;      // Current physical position (0–3)
int selectorPressureOffset    = 0;      // Global pressure offset in steps

// --- Control and Logic Flags ---
bool usarProtecaoFila         = true;   // Trigger error if filament tangles or runs out
bool bufferAtivo              = true;   // Automatic buffer loading control
bool isInIdleMode             = false;  // Indicates if the selector is currently resting
bool sensorRemoteStop         = false;  // Remote stop control via Serial (STOP_LOAD)

// =================================================================================
// SERVO CONTROL (PURGE COLLECTOR)
// =================================================================================
Servo filamentCutter;
int cutterPos                 = 0;
bool reverseServo             = true;

const int COLETOR_STOP        = 1500;   // Stopped servo (Dead center)
const int COLETOR_OPEN_SPEED  = 1300;   // Rotation speed to open
const int COLETOR_CLOSE_SPEED = 1700;   // Rotation speed to close
const int COLETOR_OPEN_TIME   = 300;    // Time required to reach fully open
const int COLETOR_CLOSE_TIME  = 180;    // Time required to reach fully closed

// =================================================================================
// SENSORS, BUFFER, AND TIMERS
// =================================================================================
unsigned long lastBufferCheck    = 0;
unsigned long lastBufferActivity = 0;   // Timestamp of the last physical action
bool lastBufferEmptyState        = false; // State memory for buffer empty sensor
bool lastBufferFullState         = false; // State memory for buffer full sensor

// =================================================================================
// COMMUNICATION AND SYSTEM
// =================================================================================
String serialBuffer           = "";     // Klipper serial input buffer
bool commandReceived          = false;  // Full command block received flag
int loaderMode                = 1;      // Automatic mode (Load/Unload)


// =================================================================================
// DISTANCE CALCULATIONS
// =================================================================================
long distance                 = 10;     // Base rotation distance value
long unloadDistance           = (long)stepsPerRev * microSteps * distance;
long loadDistance             = (long)unloadDistance * 1.1; // +10% safety margin

// --- MEMORY MANAGEMENT (EEPROM) ---
void savePressureOffset(int offset) { 
  EEPROM.put(EEPROM_PRESSURE_ADDR, offset);  // put() only writes if value differs from current
}
void loadPressureOffset() {
  int saved;
  EEPROM.get(EEPROM_PRESSURE_ADDR, saved);
  // If the value is above 150 or contains garbage (blank EEPROM returns -1 or 255), reset to 0
  selectorPressureOffset = (abs(saved) > 150) ? 0 : saved;
}
void saveCurrentExtruder(int extruder) { 
  EEPROM.update(EEPROM_EXTRUDER_ADDR, extruder + 1); 
}
int loadSavedExtruder() { 
  int saved = EEPROM.read(EEPROM_EXTRUDER_ADDR);
  // Returns tool index (0-7) or -1 if the saved value is invalid/blank
  return (saved > 0 && saved <= 8) ? (saved - 1) : -1;
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("STARTUP: BUSY")); // Notify Klipper
  const int pins[] = {extEnable, extStep, extDir, ext2Enable, ext2Step, ext2Dir, selEnable, selStep, selDir};
  for (int i = 0; i < 9; i++) pinMode(pins[i], OUTPUT);

  digitalWrite(extEnable, HIGH);
  digitalWrite(ext2Enable, HIGH);
  digitalWrite(selEnable, LOW);
  
  // --- PIN SETTINGS (SENSORS) ---
  pinMode(FILAMENT_SENSOR_PIN, INPUT_PULLUP);
  pinMode(BUFFER_EMPTY_PIN, INPUT_PULLUP);
  pinMode(BUFFER_FULL_PIN, INPUT_PULLUP);
  filamentCutter.attach(SERVO_PIN);
  filamentCutter.writeMicroseconds(COLETOR_STOP);
  loadPressureOffset(); // Load spring tension value from EEPROM
  homeSelector();   // homeSelector now calibrates, reads EEPROM, and shifts to the correct active tool.
  
  // --- INITIALIZE BUFFER STATES ---
  lastBufferEmptyState = (digitalRead(BUFFER_EMPTY_PIN) == HIGH);
  lastBufferFullState = (digitalRead(BUFFER_FULL_PIN) == HIGH);
  lastBufferActivity = millis();
  isInIdleMode = false;
  
  // --- USER INTERFACE OUTLINE ---
  Serial.println(F("\n--- 3DCHAMELEON TURBO 8X READY ---"));
  Serial.println(F("[SEL] T0-T7, HOME, IDLE, STRESS"));
  Serial.println(F("[EXT] LOAD, UNLOAD, STOP_LOAD"));
  Serial.println(F("[CFG] PRESSURE, SPEED, STATUS, HELP"));
  Serial.println(F("[SYS] BUFFER_ON/OFF, PROT_ON/OFF, COL_ON/OFF"));
  Serial.println(F("------------------------------------------"));
  Serial.println(F("READY")); 
}

void loop() {
  while (Serial.available() > 0) { // --- SERIAL INTERACTION ---
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) commandReceived = true;
      break;
    }
    serialBuffer += c;
  }
  // --- COMMAND INTERPRETATION ---
  if (commandReceived) {
    Serial.print(F("CMD: ")); Serial.println(serialBuffer);
    Serial.println(F("BUSY")); // Inform Klipper processing has started
    lastBufferActivity = millis();
    if (isInIdleMode) {
      isInIdleMode = false;
      Serial.println(F("WAKE_UP: OK"));
    }
    processSerialCommand(serialBuffer);
    serialBuffer = "";
    commandReceived = false;
    Serial.println(F("READY")); // Awaiting next instruction
  }
  // --- AUTOMATIC SYSTEM MONITORING ---
  monitorBufferStateChanges(); // Inform host of hardware sensor state changes
  unsigned long currentTime = millis();
  if (currentTime - lastBufferCheck >= BUFFER_CHECK_INTERVAL) {
    maintainBuffer(); // Handles automated feeding (Z or Y motor)
    lastBufferCheck = currentTime;
  }
  checkIdleTimeout(currentTime); // Enters low-power/idle mode if inactive
  delay(5); // Stabilization loop delay
}

void coletorOn() { // --- PURGE COLLECTOR CONTROLS (360° Continuous Servo) ---
  filamentCutter.writeMicroseconds(COLETOR_OPEN_SPEED); // Engage rotation
  delay(COLETOR_OPEN_TIME);                             // Run duration
  filamentCutter.writeMicroseconds(COLETOR_STOP);       // Brake
  Serial.println(F("COL: OPEN_OK"));
}
void coletorOff() {
  filamentCutter.writeMicroseconds(COLETOR_CLOSE_SPEED); // Engage reverse rotation
  delay(COLETOR_CLOSE_TIME);                             // Run duration
  filamentCutter.writeMicroseconds(COLETOR_STOP);        // Brake
  Serial.println(F("COL: CLOSE_OK"));
}

void monitorBufferStateChanges() { // --- ANALYZE SENSOR PINOUT DATA (A0/A1) --- BUFFER
  bool currentEmpty = (digitalRead(BUFFER_EMPTY_PIN) == HIGH);
  bool currentFull  = (digitalRead(BUFFER_FULL_PIN) == HIGH);
  if (currentEmpty != lastBufferEmptyState) {  // Monitors change on Empty sensor (A1)
    lastBufferEmptyState = currentEmpty;
    lastBufferActivity = millis();
    isInIdleMode = false;
    Serial.print(F("BUF_EMPTY: ")); Serial.println(currentEmpty);
  }
  if (currentFull != lastBufferFullState) { // Monitors change on Full sensor (A0)
    lastBufferFullState = currentFull;
    lastBufferActivity = millis();
    isInIdleMode = false;
    Serial.print(F("BUF_FULL: ")); Serial.println(currentFull);
  }
}

void maintainBuffer() { // --- AUTOMATED BUFFER AUTOPILOT ---
  if (!bufferAtivo) return;
  bool bufferEmpty = (digitalRead(BUFFER_EMPTY_PIN) == LOW);
  unsigned long now = millis();
  if (bufferEmpty) {
    if (!lastBufferEmptyState) {  // Executes only on falling edge detection (now empty, but was verified full previously)
      Serial.println(F("BUFFER: RECHARGING"));
      lastBufferActivity = now;
      // Synchronize the selector position if required
      if ((currentExtruder % 4) != (lastExtruder % 4)) {
        gotoExtruder(currentPhysPos, lastExtruder % 4);
        currentPhysPos = lastExtruder % 4;
      }
      feedBuffer(); 
    }
  }
  checkIdleTimeout(now);
}

// --- IDLE SLEEP MANAGEMENT ---
void checkIdleTimeout(unsigned long currentTime) {
  if (isInIdleMode) return;
  if ((currentTime - lastBufferActivity) >= IDLE_TIMEOUT) { // Active only if inactive past IDLE_TIMEOUT threshold (5 mins)
    Serial.println(F("IDLE: STARTING"));
    int idlePos = (lastExtruder + 2) % 4;  // 8-color scheme logic: Park between selector lobes (last + 2)
    bool filamentInHotend = (digitalRead(FILAMENT_SENSOR_PIN) == HIGH);   // Pre-check safety lock before dropping tension
    bool bufferFull       = (digitalRead(BUFFER_FULL_PIN) == HIGH);
    if (filamentInHotend && !bufferFull) {
      if (currentExtruder != lastExtruder) {
        gotoExtruder(currentExtruder, lastExtruder);
        currentExtruder = lastExtruder;
      }
      feedBuffer();
    }
    gotoExtruder(currentPhysPos, idlePos);  // Guide physical selector to low-tension rest state (0-3)
    currentPhysPos = idlePos; 
    isInIdleMode = true;
    Serial.println(F("IDLE: SLEEPING"));
  }
}
// --- FEED BUFFER VIA ACTUATOR (Z/Y MOTORS) ---
void feedBuffer() {
  int stepPin     = (lastExtruder < 4) ? extStep   : ext2Step;
  int dirPin      = (lastExtruder < 4) ? extDir    : ext2Dir;
  int enPin       = (lastExtruder < 4) ? extEnable : ext2Enable;
  bool direction  = (lastExtruder % 4 < 2) ? clockwise : counterclockwise;
  digitalWrite(enPin, LOW); 
  digitalWrite(dirPin, direction);
  long stepsFed = 0;
  long maxSteps = (long)(400.0 * STEPS_PER_MM); // 400mm path limit
  Serial.println(F("FEED: BUSY"));
  while (digitalRead(BUFFER_FULL_PIN) == HIGH) {
    if (usarProtecaoFila && (stepsFed > maxSteps)) {
      Serial.println(F("PAUSE")); 
      Serial.println(F("ERROR: FILAMENT_STUCK"));
      break; 
    }
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(speedDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(speedDelay);
    stepsFed++;
  }
  digitalWrite(enPin, HIGH); // Kill motor line feed power to manage thermals
  Serial.print(F("FEED: OK (")); 
  Serial.print((float)stepsFed / STEPS_PER_MM); 
  Serial.println(F("mm)"));
}
void printHelp() {
  Serial.println(F("\n========= 3DCHAMELEON COMMAND MANUAL ========="));
  Serial.println(F("T0-T7          : Selects tool and drives physical selector"));
  Serial.println(F("HOME           : Calibrates selector against physical stop"));
  Serial.println(F("IDLE           : Parks between paths to relieve tension"));
  Serial.println(F("STRESS         : Loops automated diagnostic sequence"));
  Serial.println(F("STATUS         : Outputs values and hardware metrics"));
  Serial.println(F("--------------------------------------------------"));
  Serial.println(F("LOAD 100 EX 5  : Loads 100mm (or to sensor) + 5mm overshoot"));
  Serial.println(F("UNLOAD 120     : Extrudes/Retracts 120mm of filament"));
  Serial.println(F("STOP_LOAD      : Kills line feeding actuator immediately"));
  Serial.println(F("--------------------------------------------------"));
  Serial.println(F("PRESSURE -20   : Modulates physical spring tension (steps)"));
  Serial.println(F("SPEED 150      : Designates driver speed delay (lower = faster)"));
  Serial.println(F("BUFFER_ON/OFF  : Toggles automatic line top-offs"));
  Serial.println(F("PROTECAO_ON/OFF: Toggles run-out/tangle pause parameters"));
  Serial.println(F("COLETOR_ON/OFF : Opens/Closes the cutter unit servo loop"));
  Serial.println(F("==================================================\n"));
}
//---------------------------------------------------------------------------------------------------
void processSerialCommand(String command) {
  command.trim();
  command.toUpperCase();
  if (command == F("STOP_LOAD")) { sensorRemoteStop = true; Serial.println(F("STOP: OK")); return; }
  if (command == F("HELP") || command == F("?")) { printHelp(); return; }
  if (command == F("STATUS")) {
    Serial.print(F("T:"));          Serial.println(currentExtruder);
    Serial.print(F("BF_FULL:"));    Serial.println(digitalRead(BUFFER_FULL_PIN));
    Serial.print(F("BF_EMPTY:"));   Serial.println(digitalRead(BUFFER_EMPTY_PIN));
    Serial.print(F("HOTEND:"));     Serial.println(digitalRead(FILAMENT_SENSOR_PIN));
    Serial.print(F("SPD:"));        Serial.println(speedDelay);
    Serial.print(F("PRESS:"));      Serial.println(selectorPressureOffset);
    Serial.print(F("PROT:"));       Serial.println(usarProtecaoFila ? 1 : 0);
    Serial.print(F("BUF_AUTO:"));   Serial.println(bufferAtivo ? 1 : 0);
    return;
  }
  if (command.startsWith(F("PRESSURE "))) {
    int val = command.substring(9).toInt();
    if (val >= -150 && val <= 150) {
      selectorPressureOffset = val;
      savePressureOffset(val); 
      Serial.print(F("PRESSURE: OK ")); Serial.println(val);
    } else Serial.println(F("ERROR: RANGE"));
    return;
  }
  if (command.startsWith(F("SPEED "))) {
    int v = command.substring(6).toInt();
    if (v >= 50) { speedDelay = v; Serial.print(F("SPEED: OK ")); Serial.println(v); }
    else Serial.println(F("ERROR: VALUE"));
    return;
  }
  if (command == F("PROTECAO_ON"))  { usarProtecaoFila = true;  Serial.println(F("PROT: 1")); return; }
  if (command == F("PROTECAO_OFF")) { usarProtecaoFila = false; Serial.println(F("PROT: 0")); return; }
  if (command == F("BUFFER_ON"))   { bufferAtivo = true;  Serial.println(F("BUF: 1"));  return; }
  if (command == F("BUFFER_OFF"))  { bufferAtivo = false; Serial.println(F("BUF: 0"));  return; }
  if (command.startsWith(F("T"))) {  // --- ACTIVE TRACKING (T0-T7) ---
    int tool = command.substring(1).toInt();
    if (tool >= 0 && tool <= 7) { 
      selectTool(tool); 
      //Serial.print(F("T: OK ")); Serial.println(tool); 
    } else Serial.println(F("ERROR: T_RANGE"));
    return;
  }
  if (command.startsWith(F("LOAD"))) {  // --- LOAD & UNLOAD SEQUENCES ---
    float d = 0, e = 0;
    int ex = command.indexOf(F("EX"));
    if (ex > 4) d = command.substring(5, ex).toFloat();
    else if (ex == -1 && command.length() > 4) d = command.substring(5).toFloat();
    if (ex > 0) {
      String p = command.substring(ex);
      p.replace(F("EXT"), ""); p.replace(F("EX"), "");
      e = p.toFloat();
    }
    if (currentExtruder >= 0) executeLoadUnload(currentExtruder, true, d, e);
    else Serial.println(F("ERROR: NO_T"));
    return;
  }
  if (command.startsWith(F("UNLOAD "))) {
    float d = command.substring(7).toFloat();
    if (d > 0 && currentExtruder >= 0) executeLoadUnload(currentExtruder, false, d, 0);
    else Serial.println(F("ERROR: UNLOAD_PARAMS"));
    return;
  }
  // --- SUB-SYSTEM MANAGEMENT ---
  if (command == F("HOME"))         { homeSelector(); Serial.println(F("HOME: OK")); return; }
  if (command == F("IDLE"))         { moveToIdle();   Serial.println(F("IDLE: OK")); return; }
  if (command == F("COLETOR_ON"))   { coletorOn();    return; }
  if (command == F("COLETOR_OFF"))  { coletorOff();   return; }
  if (command == F("STRESS"))       { stressTest();   return; }
  Serial.println(F("ERROR: UNKNOWN_CMD"));
}

//---------------------------------------------------------------------------------------------------

void stressTest() {
  Serial.println(F("STRESS_TEST: START"));
  
  // Tactical index mapping sequence validating tool selection boundaries and parity changes
  int seq[] = {
    0, 1, 0, 2, 0, 3,         // Z Motor tracking (0-3)
    4, 5, 4, 6, 4, 7,         // Y Motor tracking (4-7)
    0, 4, 1, 5, 2, 6, 3, 7,   // Motor handoff transitions (parity validation)
    3, 0, 7, 4,               // Long travel physical movements
    0, 7, 3, 4                // Boundary path crossings
  };
  int total = sizeof(seq) / sizeof(seq[0]);
  for (int i = 0; i < total; i++) {
    int t = seq[i];
    Serial.print(F("STEP: ")); Serial.print(i + 1);
    Serial.print(F("/"));      Serial.print(total);
    Serial.print(F(" | TARGET: T")); Serial.println(t);
    selectTool(t);
    bool dir = (t % 4 < 2) ? clockwise : counterclockwise;
    rotateExtruder(t, dir, (long)(1.5 * STEPS_PER_MM));
    delay(2000); 
  }
  Serial.println(F("STRESS_TEST: DONE"));
  selectTool(0);
}

void selectTool(int toolNumber) { // --- TOOL ASSIGNMENT & SELECTION (T0-T7) ---
  int fromPos = (currentExtruder >= 0) ? (currentExtruder % 4) : 0;
  int targetPos = toolNumber % 4;
  gotoExtruder(fromPos, targetPos);
  currentExtruder = toolNumber;
  lastExtruder = toolNumber;
  currentPhysPos = targetPos;
  saveCurrentExtruder(currentExtruder);
  Serial.print(F("T: OK ")); Serial.println(toolNumber);
}
void moveToIdle() { // --- ENGAGE LOW POWER POSITION (IDLE) ---
  if (isInIdleMode) return;
  int idlePos = (lastExtruder + 2) % 4; 
  int physNow = currentExtruder % 4;
  if (physNow != idlePos) {
    Serial.print(F("IDLE: MOVING ")); Serial.print(physNow);
    Serial.print(F("->")); Serial.println(idlePos);
    gotoExtruder(physNow, idlePos);
    currentPhysPos = idlePos; 
    isInIdleMode = true;
    Serial.println(F("IDLE: OK"));
  }
}
long loadUntilSensor(int tool, bool dir, float dist_mm, float extra_mm) { // --- FEED WITH SERIAL INTERRUPT FAULT PROTECTION (STOP_LOAD) ---
  sensorRemoteStop = false;
  int stepPin = (tool < 4) ? extStep   : ext2Step;
  int dirPin  = (tool < 4) ? extDir    : ext2Dir;
  int enPin   = (tool < 4) ? extEnable : ext2Enable;
  digitalWrite(enPin, LOW); 
  digitalWrite(dirPin, dir);
  long steps = 0;
  long maxSteps = (dist_mm > 0) ? (long)(dist_mm * STEPS_PER_MM) : (long)(3000.0 * STEPS_PER_MM); 
  const int minWait = speedDelay;       
  const int maxWait = speedDelay / 5;   
  long ramp = 50.0 * STEPS_PER_MM; 
  long decelStart = (extra_mm > 0) ? -1 : (maxSteps - ramp);
  Serial.println(F("LOAD: BUSY"));
  while (steps < maxSteps && !sensorRemoteStop) {
    if (digitalRead(FILAMENT_SENSOR_PIN) == LOW) { 
      Serial.println(F("SENSOR: DETECTED"));
      break; 
    }
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'S') {
        String cmd = Serial.readStringUntil('\n');
        if (cmd == F("TOP_LOAD")) sensorRemoteStop = true;
      }
    }
    if (sensorRemoteStop) break;
    int wait = maxWait;
    if (steps < ramp) { // Acceleration curve
      wait = minWait - (int)((minWait - maxWait) * ((float)steps / ramp));
    } else if (decelStart > 0 && steps > decelStart) { // Deceleration curve
      wait = maxWait + (int)((minWait - maxWait) * ((float)(steps - decelStart) / ramp));
    }
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(wait);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(wait);
    steps++;
  }
  if (extra_mm > 0 && !sensorRemoteStop) {
    long extraSteps = (long)(extra_mm * STEPS_PER_MM);
    long eDecel = max(0L, extraSteps - ramp); 
    Serial.println(F("LOAD: EXTRA_PUSH"));
    for (long i = 0; i < extraSteps; i++) {
      int eWait = (i > eDecel) ? 
                  maxWait + (int)((minWait - maxWait) * ((float)(i - eDecel) / ramp)) : 
                  maxWait;
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(eWait);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(eWait);
      steps++;
    }
  }
  digitalWrite(enPin, HIGH);
  Serial.print(F("LOAD: OK (")); 
  Serial.print((float)steps / STEPS_PER_MM); 
  Serial.print(F("mm)"));
  if (sensorRemoteStop) Serial.print(F(" [STOPPED]"));
  Serial.println();
  return steps;
}

void executeLoadUnload(int tool, bool isLoad, float dist, float extra) { // --- ENGAGE EXTRUSION LOGIC (8 COLORS) ---
  Serial.print(isLoad ? F("CMD_LOAD: T") : F("CMD_UNLOAD: T"));
  Serial.print(tool); Serial.println(F(" BUSY"));
  if (currentExtruder != tool) {
    gotoExtruder(currentExtruder % 4, tool % 4);
    currentExtruder = tool;
  }
  bool dir = (tool % 4 < 2) ? clockwise : counterclockwise;  // LOAD: T0,T1 (CW) | T2,T3 (CCW) --- UNLOAD: Reverses logic parameters
  if (!isLoad) dir = !dir;
  if (isLoad) {
    loadUntilSensor(tool, dir, dist, extra);
  } else {
    rotateExtruder(tool, dir, (long)(dist * STEPS_PER_MM));
    Serial.println(F("UNLOAD: OK"));
  }
  lastExtruder = tool;
  saveCurrentExtruder(currentExtruder);
}

// --- SHIFT PHYSICAL SELECTOR POSITION (X AXIS) ---
void gotoExtruder(int from, int to) {
  int physFrom = from % 4;
  int physTo = to % 4;
  if (selectorPressureOffset != 0) { // 1. RESET RUNNING PRESSURE COMPENSATIONS (CENTER ALIGNMENT)
    rotateSelector(selectorPressureOffset > 0, abs(selectorPressureOffset));
    delay(20); 
  }
  if (physFrom == physTo) return;
  int diff = physTo - physFrom;  // 2. DISPLACEMENT CALCULATION
  bool dir = (diff > 0) ? counterclockwise : clockwise;
  long stepsPerPos = (stepsPerRev * microSteps) / 4; // 800 steps per node increment (1/4 turn layout)
  long totalSteps = (long)abs(diff) * stepsPerPos;
  totalSteps += selectorPressureOffset;  // 3. RETRIEVE AND INJECT SPRING COMPENSATIONS
  Serial.print(F("SEL: ")); Serial.print(physFrom);
  Serial.print(F("->")); Serial.print(physTo);
  Serial.print(F(" [")); Serial.print(totalSteps); Serial.println(F(" steps]"));
  rotateSelector(dir, totalSteps);
  isInIdleMode = false; // System active; drop resting flags
}

void rotateExtruder(int tool, bool dir, long steps) { // --- DRIVE EXTRUDER STAGE ARRAYS (Z or Y MOTORS) ---
  if (steps <= 0) return; 
  int stepPin = (tool < 4) ? extStep   : ext2Step;
  int dirPin  = (tool < 4) ? extDir    : ext2Dir;
  int enPin   = (tool < 4) ? extEnable : ext2Enable;
  digitalWrite(enPin, LOW); 
  digitalWrite(dirPin, dir);
  const int minWait = speedDelay;
  const int maxWait = speedDelay / 6;
  if ((float)steps / STEPS_PER_MM < 100.0) { // Acceleration planning bypass for short travels (100mm threshold limit)
    for (long x = 0; x < steps; x++) {
      digitalWrite(stepPin, HIGH); delayMicroseconds(minWait);
      digitalWrite(stepPin, LOW);  delayMicroseconds(minWait);
    }
  } else {
    long accelLimit = steps * 0.08; // Long travel speed management: Curves set to 8% Accel / 82% Cruise / 10% Decel
    long decelLimit = steps * 0.90;
    for (long x = 0; x < steps; x++) {
      int wait = maxWait;
      if (x < accelLimit) { // Accel curve ramping
        wait = minWait - (int)((minWait - maxWait) * ((float)x / accelLimit));
      } 
      else if (x > decelLimit) { // Decel curve ramping
        wait = maxWait + (int)((minWait - maxWait) * ((float)(x - decelLimit) / (steps - decelLimit)));
      }
      digitalWrite(stepPin, HIGH); delayMicroseconds(wait);
      digitalWrite(stepPin, LOW);  delayMicroseconds(wait);
    }
  }
  digitalWrite(enPin, HIGH); // Sleep motor line to prevent heating issues
}

void rotateSelector(bool dir, int steps) { // --- ROTATE SELECTOR SYSTEM (X-AXIS ARDUINO STEP STAGE) ---
  digitalWrite(selEnable, LOW); // Enforce holding current lock
  digitalWrite(selDir, dir);
  for (int x = 0; x < steps; x++) {
    digitalWrite(selStep, HIGH);
    delayMicroseconds(selectorSpeedDelay);
    digitalWrite(selStep, LOW);
    delayMicroseconds(selectorSpeedDelay);
  }
}

void homeSelector() // --- HOME SELECTOR AXIS STAGE RUN ---
{
  // 1. Move into mechanical boundary limit (Clockwise direction)
  rotateSelector(clockwise, (int)(stepsPerRev * microSteps * 1.3));
  delay(100);
  // 2. Step backward to pinpoint center alignment for channel 0
  rotateSelector(counterclockwise, defaultBackoff * microSteps);
  delay(50);
  // 3. Establish absolute physical baseline location definitions
  currentPhysPos = 0;        // Known physical location synced
  isInIdleMode   = false;    // Status running; drop sleep tags
  int saved = loadSavedExtruder();  // 4. Extract last confirmed active storage parameters from EEPROM
  if (saved < 0 || saved > 7) saved = 0;

  lastExtruder    = saved;
  currentExtruder = saved;
  Serial.print(F("HOME: OK | Phys=0 | T"));
  Serial.println(currentExtruder);
}
