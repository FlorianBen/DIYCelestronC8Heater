#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <SPI.h>
#include <SparkFunBME280.h>
#include <Vrekrer_scpi_parser.h>
#include <Wire.h>
#include <math.h>

#define CLK 2
#define DT 3

// Constants
const int pin_feedback = 7;
const int pin_pwm = 11;
const int pin_bme;
const int screen_width = 128;
const int screen_height = 32;
unsigned long lastButtonPress = 0;

// Screen
Adafruit_SSD1306 display(screen_width, screen_height, &Wire, -1);

// State machine.
enum heater_state { INIT, WAITC, CONFIG, HEAT, ABORT };
uint8_t state = INIT;

// SCPI parser instance.
SCPI_Parser my_instrument;

// Sensor ambient
BME280 sensors_ambient;

// Sensor feedback
OneWire oneWire(pin_feedback);
DallasTemperature sensors_feedback(&oneWire);

// Global variables.
double Tdp;
double temperature;
double hr;

// Feedback
int errco = 0;
double setpoint = 15;
double cmd_signal = 0;
uint8_t pwm_signal;
double feedback = 15;
double Kp, Ki, Kd;
double error, errori, errord, errorl;

// Rotary encoder
int currentStateCLK;
int lastStateCLK;

void setup() {
  // Initialize serial:
  Serial.begin(9600);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }
  display.clearDisplay();

  // Sensor feedback
  sensors_feedback.begin();

  // Sensor ambient
  sensors_ambient.settings.commInterface = I2C_MODE;
  sensors_ambient.settings.I2CAddress = 0x76;
  sensors_ambient.settings.runMode = 3;
  sensors_ambient.settings.tStandby = 0;
  sensors_ambient.settings.filter = 0;
  sensors_ambient.settings.tempOverSample = 1;
  sensors_ambient.settings.pressOverSample = 1;
  sensors_ambient.settings.humidOverSample = 1;
  sensors_ambient.begin();

  // Create SCPI instrument and register commands.
  my_instrument.RegisterCommand("*IDN?", &Identify);       // Identifier string.
  my_instrument.SetCommandTreeBase("HEATer:CONTrol");      // Tree Base.
  my_instrument.RegisterCommand(":Heat", &Heat);           // Start heating.
  my_instrument.RegisterCommand(":Heat?", &GetHeat);       // Heater is running?
  my_instrument.RegisterCommand(":AbortHeat", &AbortHeat); // Abort heating.
  my_instrument.RegisterCommand(":Ambiant?", &GetAmbiantT);  // Get ambient.
  my_instrument.RegisterCommand(":Hr?", &GetHR);       // Get HR.
  my_instrument.RegisterCommand(":Tdp?", &GetDp);      // Get Tdp.
  my_instrument.RegisterCommand(":FEEDback?", &GetFeedback); // Get feedback.

  // Setup PID
  Kp = 6;
  Ki = 0.1;
  Kd = 0.05;
  error = errori = errord = errorl = 0;

  // Setup rotary encoder
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(8, INPUT_PULLUP);
  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);

  // Timer
  set_timer();
}

void loop() {
  uint8_t previous_cmd = 0;
  int btnState;
  // State Machine for heater.
  switch (state) {
  case INIT: // Useless in facts.
    state = WAITC;
    break;
  case WAITC: // Wait order from serial.
    state = WAITC;
    btnState = digitalRead(8);
    if (btnState == LOW) {
      if (millis() - lastButtonPress > 100) {
        state = HEAT;
      }
    }
    break;
  case CONFIG: // Config the stepper class.
    state = WAITC;
    break;
  case HEAT: // Heat
    btnState = digitalRead(8);
    if (btnState == LOW) {
      if (millis() - lastButtonPress > 100) {
        state = ABORT;
      }
    } else {
      state = HEAT;
    }
    if (previous_cmd != pwm_signal)
      analogWrite(pin_pwm, pwm_signal);
    break;
  case ABORT:
    analogWrite(pin_pwm, 0);
    reset_PID();
    state = WAITC;
    break;
  }
  read_feedback();
  read_ambient();
  print_oled();
  delay(200);
}

// Serial command loop.
void serialEvent() {
  while (Serial.available()) {
    // Process incoming command.
    my_instrument.ProcessInput(Serial, "\n");
  }
}

void Identify(SCPI_C commands, SCPI_P parameters, Stream &interface) {
  interface.println(
      "FlorianBen,Celestron Heater,#00,v0.1"); // No check, read only.
}

void Heat(SCPI_C commands, SCPI_P parameters, Stream &interface) {
  if (state != WAITC) { // Entry point of the state machine.
    interface.println("WAR#001");
  } else {
    state = HEAT;
    interface.println("ACK#000");
  }
}

void GetHeat(SCPI_C commands, SCPI_P parameters, Stream &interface) {
  interface.println("Test"); // No check, read only.
}

void AbortHeat(SCPI_C commands, SCPI_P parameters, Stream &interface) {
  if (state != HEAT) { // Abort is only available if the motor turn.
    interface.println("WAR#001");
  } else {
    state = ABORT;
    interface.println("ACK#000");
  }
}

void GetAmbiantT(SCPI_C commands, SCPI_P parameters, Stream &interface) {
  interface.println(temperature); // No check, read only.
}

void GetHR(SCPI_C commands, SCPI_P parameters, Stream &interface) {
  interface.println(hr); // No check, read only.
}

void GetDp(SCPI_C commands, SCPI_P parameters, Stream &interface) {
  interface.println(Tdp); // No check, read only.
}

void GetFeedback(SCPI_C commands, SCPI_P parameters, Stream &interface) {
  interface.println(feedback); // No check, read only.
}

// Interrupt function
ISR(TIMER1_COMPA_vect) {
  if (state == HEAT) {
    calc_PID();
  }
}

// Setup timer
void set_timer() {
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1 = 0;  // initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624; // = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

void updateEncoder() {
  // Read the current state of CLK
  currentStateCLK = digitalRead(CLK);

  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK && currentStateCLK == 1) {

    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(DT) != currentStateCLK) {
      setpoint -= 0.25;
    } else {
      // Encoder is rotating CW so increment
      setpoint += 0.25;
    }
  }
  // Remember last CLK state
  lastStateCLK = currentStateCLK;
}

void print_oled() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.print(F("Ta: "));
  display.print(temperature);
  display.print(F(" "));
  display.print(F("HR: "));
  display.println(hr);

  display.print(F("Dp: "));
  display.print(Tdp);
  display.print(F(" "));
  display.print(F("Fb: "));
  display.println(feedback);

  display.print(F("St: "));
  display.print(setpoint);
  display.print(F(" "));
  display.print(F("State: "));
  display.println(state);

  display.print(F("Cmd: "));
  display.print(cmd_signal);
  display.print(F(" "));
  display.print(F("ERR: "));
  display.print(errco);

  display.display();
}

void read_feedback() {
  sensors_feedback.requestTemperatures();
  double temp_feedback = sensors_feedback.getTempCByIndex(0);
  if (temp_feedback > -127.0) {
    errco = 0;
    feedback = temp_feedback;
  } else {
    errco++;
  }
}

void calc_PID() {
  error = setpoint - feedback;
  errori += error;
  errord = error - errorl;
  cmd_signal = Kp * error + Ki * errori + Kd * errord;
  if (cmd_signal < 0) {
    pwm_signal = 0;
  } else if (cmd_signal > 255) {
    pwm_signal = 255;
  } else {
    pwm_signal = (uint8_t)cmd_signal;
  }
}

void reset_PID() {
  error = errori = errord = errorl = 0;
  cmd_signal = 0;
  pwm_signal = 0;
}

void read_ambient() {
  temperature = sensors_ambient.readTempC();
  hr = sensors_ambient.readFloatHumidity();
  calc_dewpoint();
}

float calc_dewpoint() {
  double a = 6.1121;
  double b = 18.678;
  double c = 257.14;
  double Ps = a * exp((b * temperature) / (c + temperature));
  double Pa = (hr / 100) * Ps;
  Tdp = c * log(Pa / a) / (b - log(Pa / a));
}