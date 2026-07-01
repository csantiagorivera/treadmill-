/*
 * Treadmill Hardware Sync Firmware (ESP32)
 * ----------------------------------------------------
 * Role: emit a speed-reference PWM into the treadmill's OEM lower control board
 *       through a PC817 optocoupler, close a velocity loop using the roller
 *       pulse sensor, and obey a simple serial protocol from the ROS2 bridge.
 *
 * The MCU NEVER switches motor current. High power is handled by the OEM board.
 * The E-stop cuts the contactor coil in hardware; this firmware only SENSES it.
 *
 * NEW — two-way safety:
 *   1. HEARTBEAT_PIN: the robot computer toggles this GPIO constantly (~10 Hz).
 *      If it stops toggling (robot crashes / loses power), the firmware ramps
 *      the belt to zero automatically — no ROS2, no serial needed in that path.
 *   2. The belt's measured velocity is reported back over serial so the ROS2
 *      bridge can publish it and the robot's locomotion stack can close the loop.
 *
 * Requires Arduino-ESP32 core >= 3.0 (uses ledcAttach / ledcWrite(pin,duty)).
 *   For core 2.x swap to ledcSetup(ch,freq,res)+ledcAttachPin(pin,ch)+ledcWrite(ch,duty).
 *
 * Serial protocol @115200, line-based:
 *   PC -> ESP32:
 *     S            start  (state RUNNING, target stays 0 until a V command)
 *     X            stop   (ramp to 0, state IDLE)
 *     V<float>     set target belt velocity in m/s   e.g. V1.25
 *     C            run calibration sweep
 *     R            reset E-stop latch (only clears if E-stop physically released)
 *     P            ping -> "#PONG"
 *   ESP32 -> PC:
 *     T,<state>,<target>,<measured>,<duty>,<estop>,<err>   periodic telemetry
 *     CAL,<duty>,<measured>   per calibration step ;  CAL,DONE at the end
 *     #<text>      human-readable acks / warnings
 */

#include <Arduino.h>

// ---------------- Pin map (adjust to your wiring) ----------------
const int PWM_OUT_PIN      = 25;   // -> PC817 LED (speed reference to OEM board)
const int SPEED_FB_PIN     = 27;   // <- roller pulse sensor (INPUT_PULLUP)
const int ESTOP_SENSE_PIN  = 26;   // <- E-stop aux contact (INPUT_PULLUP, LOW = engaged)
const int HEARTBEAT_PIN    = 33;   // <- robot computer GPIO, toggles ~10 Hz (INPUT_PULLUP)
const int STATUS_LED_PIN   = 2;

// ---------------- PWM (LEDC) ----------------
const int   PWM_FREQ     = 20000;   // 20 kHz carrier
const int   PWM_RES_BITS = 12;      // 0..4095
const int   PWM_MAX      = (1 << PWM_RES_BITS) - 1;
const int   DUTY_MAX     = (int)(0.95f * PWM_MAX);
int         g_minRunDuty = (int)(0.08f * PWM_MAX);

// ---------------- Calibration constants (set via calibrate.py) ----------------
//
// These three values are placeholders. After flashing, you must calibrate in
// two steps before the system works accurately:
//
// STEP 1 — Physical (do this first, no PC needed beyond serial monitor):
//   Put a tape mark on the belt surface. Start the belt with 'S' then 'V0.3'
//   over serial. Let the mark travel exactly 1 metre, then stop with 'X'.
//   The serial monitor shows running pulse counts in the telemetry lines —
//   count the total pulses that arrived over that 1 metre. Divide:
//       PULSES_PER_METER = total_pulses / 1.0
//   Update the value below and reflash before moving to step 2.
//   (Without this being correct, the firmware cannot measure m/s accurately,
//   so step 2 would produce a meaningless curve.)
//
// STEP 2 — Software (run calibrate.py after step 1 is done and reflashed):
//   python3 calibrate.py --port /dev/ttyUSB0
//   The script triggers a full duty cycle sweep, measures real belt velocities
//   using the now-accurate PULSES_PER_METER, fits a straight line, and prints:
//       FF_SLOPE     = <value>
//       FF_INTERCEPT = <value>
//   Paste those two values below and reflash one final time.
//   (These let the firmware guess roughly the right duty cycle for any target
//   speed upfront, so the PI controller only has to correct small residual errors
//   rather than fight from zero every time.)
//
float PULSES_PER_METER = 100.0f;   // <<< STEP 1: replace with your measured value
float FF_SLOPE         = 1200.0f;  // <<< STEP 2: replace with calibrate.py output
float FF_INTERCEPT     = 200.0f;   // <<< STEP 2: replace with calibrate.py output

// ---------------- Velocity / motion limits ----------------
const float V_MAX     = 3.0f;    // m/s hard cap. TUNE to your belt.
const float MAX_ACCEL = 0.5f;    // m/s^2 slew limit — applies to ramp-down too,
                                  // so the belt never cuts instantly on watchdog trigger
const float V_DEADBAND = 0.02f;

// ---------------- PI gains (TUNE after calibration) ----------------
float KP = 1500.0f;
float KI = 600.0f;
const float I_CLAMP = 0.6f * PWM_MAX;

// ---------------- Timing ----------------
const uint32_t CTRL_PERIOD_MS       = 10;    // 100 Hz control loop
const uint32_t TELEM_PERIOD_MS      = 20;    // 50 Hz telemetry
const uint32_t VEL_TIMEOUT_MS       = 700;   // stale V command -> ramp to 0
const uint32_t HEARTBEAT_TIMEOUT_MS = 500;   // no toggle from robot -> ramp to 0

// ---------------- State ----------------
//
// The system is always in exactly one of four states:
//
//   ST_IDLE    — belt stopped, waiting for a start command ('S' over serial).
//                Safe to be in at any time.
//
//   ST_RUNNING — actively controlling belt speed. The PI loop runs, velocity
//                commands are accepted, telemetry is published.
//
//   ST_ESTOP   — emergency stop has been triggered. The firmware detected that
//                ESTOP_SENSE_PIN went LOW, meaning the physical mushroom button
//                was pressed and the contactor cut mains power to the LCB.
//                Note: the contactor already killed the belt independently before
//                the firmware even noticed — the firmware just reflects this and
//                locks itself so the belt cannot silently restart on its own.
//                PWM output is forced to zero and all commands are rejected
//                except 'R' (reset). 'R' only works once the mushroom button has
//                been physically twisted and released (ESTOP_SENSE_PIN goes HIGH
//                again), preventing an accidental restart while the button is
//                still pressed. After a valid 'R' the system returns to ST_IDLE
//                and waits for a deliberate 'S' before moving again.
//
//   ST_CALIB   — calibration sweep in progress (triggered by 'C' command).
//                The PI loop is bypassed; duty cycle is driven manually by the
//                sweep routine. Returns to ST_IDLE when the sweep finishes.
//
enum State { ST_IDLE, ST_RUNNING, ST_ESTOP, ST_CALIB };
State    g_state     = ST_IDLE;
float    g_targetV   = 0.0f;   // velocity ROS2 asked for (m/s)
float    g_rampV     = 0.0f;   // slew-limited version of g_targetV (what the loop actually chases)
float    g_measuredV = 0.0f;   // filtered belt velocity from roller sensor (m/s)
float    g_integral  = 0.0f;   // PI integral accumulator
int      g_duty      = 0;      // current PWM duty cycle sent to the opto
uint32_t g_lastVelCmdMs  = 0;  // timestamp of last 'V' command (for watchdog)
uint32_t g_lastCtrlMs    = 0;
uint32_t g_lastTelemMs   = 0;

// ---------------- Pulse counting (roller sensor) ----------------
volatile uint32_t g_pulseCount = 0;
portMUX_TYPE g_pulseMux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR onPulse() {
  portENTER_CRITICAL_ISR(&g_pulseMux);
  g_pulseCount++;
  portEXIT_CRITICAL_ISR(&g_pulseMux);
}

// ---------------- Hardware heartbeat (robot computer GPIO) ----------------
// The robot computer toggles HEARTBEAT_PIN at ~10 Hz over a physical wire.
// Any edge (RISING or FALLING) counts as a heartbeat.
// If no edge arrives within HEARTBEAT_TIMEOUT_MS the robot is assumed dead
// and the belt ramps to zero — no serial, no ROS2 in that path.
volatile uint32_t g_lastHeartbeatMs = 0;
void IRAM_ATTR onHeartbeat() {
  g_lastHeartbeatMs = millis();
}
bool heartbeatLost() {
  // Only enforce heartbeat while RUNNING — allow IDLE without a robot attached
  if (g_state != ST_RUNNING) return false;
  return (millis() - g_lastHeartbeatMs) > HEARTBEAT_TIMEOUT_MS;
}

// ---------------- Helpers ----------------
void setDuty(int duty) {
  duty = constrain(duty, 0, DUTY_MAX);
  g_duty = duty;
  ledcWrite(PWM_OUT_PIN, duty);
}

bool estopEngaged() { return digitalRead(ESTOP_SENSE_PIN) == LOW; }

void enterEstop() {
  g_state = ST_ESTOP;
  g_targetV = 0; g_rampV = 0; g_integral = 0;
  setDuty(0);
}

float readMeasuredV(float dt) {
  uint32_t c;
  portENTER_CRITICAL(&g_pulseMux);
  c = g_pulseCount; g_pulseCount = 0;
  portEXIT_CRITICAL(&g_pulseMux);
  float v = ((float)c / PULSES_PER_METER) / dt;
  const float alpha = 0.4f;
  g_measuredV = alpha * v + (1.0f - alpha) * g_measuredV;
  return g_measuredV;
}

// ---------------- Calibration sweep ----------------
void runCalibration() {
  g_state = ST_CALIB;
  g_integral = 0;
  Serial.println("#CALIB start");
  for (int d = g_minRunDuty; d <= DUTY_MAX; d += (PWM_MAX / 20)) {
    if (estopEngaged()) { enterEstop(); Serial.println("#CALIB aborted: ESTOP"); return; }
    setDuty(d);
    g_pulseCount = 0; delay(200);
    portENTER_CRITICAL(&g_pulseMux); g_pulseCount = 0; portEXIT_CRITICAL(&g_pulseMux);
    delay(1300);
    uint32_t c; portENTER_CRITICAL(&g_pulseMux); c = g_pulseCount; portEXIT_CRITICAL(&g_pulseMux);
    float v = ((float)c / PULSES_PER_METER) / 1.3f;
    Serial.printf("CAL,%d,%.4f\n", d, v);
  }
  setDuty(0);
  Serial.println("CAL,DONE");
  g_state = ST_IDLE;
}

// ---------------- Serial command parser ----------------
String g_lineBuf;
void handleLine(String s) {
  s.trim();
  if (s.length() == 0) return;
  char cmd = s.charAt(0);
  switch (cmd) {
    case 'S':
      if (g_state == ST_ESTOP) { Serial.println("#blocked: ESTOP active"); break; }
      g_state = ST_RUNNING;
      g_targetV = 0; g_integral = 0;
      g_lastVelCmdMs = millis();
      g_lastHeartbeatMs = millis();  // grace period on start
      Serial.println("#RUNNING");
      break;
    case 'X':
      if (g_state != ST_ESTOP) { g_state = ST_IDLE; g_targetV = 0; }
      Serial.println("#IDLE");
      break;
    case 'V': {
      float v = s.substring(1).toFloat();
      v = constrain(v, 0.0f, V_MAX);
      if (v < V_DEADBAND) v = 0.0f;
      g_targetV = v;
      g_lastVelCmdMs = millis();
      break;
    }
    case 'C':
      if (g_state == ST_ESTOP) { Serial.println("#blocked: ESTOP active"); break; }
      runCalibration();
      break;
    case 'R':
      if (estopEngaged()) Serial.println("#cannot reset: E-stop still engaged");
      else { g_state = ST_IDLE; g_integral = 0; Serial.println("#ESTOP cleared -> IDLE"); }
      break;
    case 'P':
      Serial.println("#PONG");
      break;
    default:
      Serial.printf("#unknown cmd: %c\n", cmd);
  }
}

void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (g_lineBuf.length()) { handleLine(g_lineBuf); g_lineBuf = ""; }
    } else if (g_lineBuf.length() < 32) g_lineBuf += c;
  }
}

// ---------------- Control loop ----------------
void controlStep(float dt) {
  // slew-limit the target (acceleration AND deceleration profile)
  float maxStep = MAX_ACCEL * dt;
  if      (g_targetV > g_rampV + maxStep) g_rampV += maxStep;
  else if (g_targetV < g_rampV - maxStep) g_rampV -= maxStep;
  else                                    g_rampV  = g_targetV;

  float meas = readMeasuredV(dt);

  if (g_state != ST_RUNNING) { g_integral = 0; setDuty(0); return; }

  // PI + feedforward
  float err = g_rampV - meas;
  g_integral += KI * err * dt;
  g_integral  = constrain(g_integral, -I_CLAMP, I_CLAMP);
  float ff  = (g_rampV > 0.001f) ? (FF_SLOPE * g_rampV + FF_INTERCEPT) : 0.0f;
  float out = ff + KP * err + g_integral;

  if (g_rampV < 0.001f) { setDuty(0); g_integral = 0; }
  else setDuty(constrain((int)out, g_minRunDuty, DUTY_MAX));
}

// ---------------- Telemetry ----------------
void sendTelemetry() {
  const char* st = (g_state==ST_IDLE)?"IDLE":
                   (g_state==ST_RUNNING)?"RUNNING":
                   (g_state==ST_ESTOP)?"ESTOP":"CALIB";
  Serial.printf("T,%s,%.3f,%.3f,%d,%d,%.3f\n",
                st, g_rampV, g_measuredV, g_duty,
                estopEngaged()?1:0,
                g_rampV - g_measuredV);
}

void setup() {
  Serial.begin(115200);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(SPEED_FB_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SPEED_FB_PIN), onPulse, FALLING);
  pinMode(ESTOP_SENSE_PIN, INPUT_PULLUP);
  pinMode(HEARTBEAT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HEARTBEAT_PIN), onHeartbeat, CHANGE);  // any edge counts

  ledcAttach(PWM_OUT_PIN, PWM_FREQ, PWM_RES_BITS);
  setDuty(0);

  g_lastCtrlMs = g_lastTelemMs = g_lastHeartbeatMs = millis();
  Serial.println("#treadmill sync ready");
}

void loop() {
  pollSerial();

  // Hardwired E-stop: hardware cuts power independently; we just reflect it
  if (estopEngaged() && g_state != ST_ESTOP) {
    enterEstop();
    Serial.println("#ESTOP engaged");
  }

  // Hardware heartbeat: robot computer GPIO has gone silent -> ramp to zero
  if (heartbeatLost()) {
    static uint32_t lastWarnMs = 0;
    if (millis() - lastWarnMs > 1000) {
      Serial.println("#heartbeat lost -> ramping belt to 0");
      lastWarnMs = millis();
    }
    g_targetV = 0.0f;   // slew limiter handles the gradual ramp-down
  }

  uint32_t now = millis();

  if (now - g_lastCtrlMs >= CTRL_PERIOD_MS) {
    float dt = (now - g_lastCtrlMs) / 1000.0f;
    g_lastCtrlMs = now;

    // Software watchdog: stale V command while running
    if (g_state == ST_RUNNING && (now - g_lastVelCmdMs) > VEL_TIMEOUT_MS) {
      if (g_targetV != 0.0f) Serial.println("#vel_cmd stale -> ramping down");
      g_targetV = 0.0f;
    }
    controlStep(dt);
  }

  if (now - g_lastTelemMs >= TELEM_PERIOD_MS) {
    g_lastTelemMs = now;
    sendTelemetry();
  }

  digitalWrite(STATUS_LED_PIN,
    g_state == ST_RUNNING ? HIGH : (millis() / 250) % 2);
}
