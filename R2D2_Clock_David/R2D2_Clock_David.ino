#include "time.h"

#include "WiFiManager.h"
#include "TM1637Display.h"
#include "DFRobotDFPlayerMini.h"
#include "AiEsp32RotaryEncoder.h"
#include "WiFiCredentials.h"

// Light Emitting Diodes (LED) 

#define RED_LED_PIN   32
#define WHITE_LED_PIN 33
// #define LIVENESS_LED_PIN 2

// Rotary Encoder (Button)

#define ROTARY_ENCODER_A_PIN      25
#define ROTARY_ENCODER_B_PIN      26
#define ROTARY_ENCODER_BUTTON_PIN 27
#define ROTARY_ENCODER_STEPS       4
#define ROTARY_ENCODER_VCC_PIN    -1

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(
  ROTARY_ENCODER_A_PIN,
  ROTARY_ENCODER_B_PIN,
  ROTARY_ENCODER_BUTTON_PIN,
  ROTARY_ENCODER_VCC_PIN,
  ROTARY_ENCODER_STEPS);

void IRAM_ATTR ReadEncoderISR() {
  rotaryEncoder.readEncoder_ISR();
}

// MP3 Mini Player

#define FPSerial Serial1

DFRobotDFPlayerMini musicPlayer;

void printDetail(uint8_t type, int value);

// Clock Display (4x 7 Digits)

const int CLOCK_DISPLAY_BRIGHTNESS = 3; // 0-7
TM1637Display clockDisplay(21, 22);

// NTP Client

const char* NTP_SERVER      = "pool.ntp.org";
const long  GMT_OFFSET_SECS = 3600;

long daylightOffsetSecs = 0;

WiFiUDP ntpUDP;

// Internal State

unsigned long lastButtonPress  = 0;
float         redLedLuminosity = 0;

// -------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);

  Serial.println();
  Serial.println(F("Configuring pins ..."));
  // pinMode(LIVENESS_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(WHITE_LED_PIN, OUTPUT);
 
  // Configure LED PWM funtionalities
  if (!ledcAttach(RED_LED_PIN, 5000, 8)) {
    Serial.println(F("Unable to setup PWM for Red LED"));
    Serial.println(F("Aborting startup!"));
    while (true) {
      delay(0);  // Code to compatible with ESP8266 watch dog.
    }
  }
  clockDisplay.setBrightness(CLOCK_DISPLAY_BRIGHTNESS);

  Serial.println();
  Serial.print(F("Connecting to WiFi Network"));
  Serial.print(F(SSID));
  Serial.println(F(" ..."));
  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println(F("Connected to WiFi successfully."));

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  // TODO Handle changing daylight saving...
  /* if ((month * 30 + day) >= 121 && (month * 30 + day) < 331) {
    daylightOffsetSecs = 3600;
    // Change daylight saving time - Summer - change 31/03 at 00:00
    // timeClient.setTimeOffset(UTC_OFFSET_SECONDS * UTC);
  } else {
    daylightOffsetSecs = 0;
    // Change daylight saving time - Winter - change 31/10 at 00:00
    // timeClient.setTimeOffset((UTC_OFFSET_SECONDS * UTC) - 3600);
  } */

  configTime(GMT_OFFSET_SECS, daylightOffsetSecs, NTP_SERVER);

  FPSerial.begin(9600, SERIAL_8N1, 14, 12);
  if (!musicPlayer.begin(FPSerial)) {
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    Serial.println(F("Aborting startup!"));
    while (true) {
      delay(0);  // Code to compatible with ESP8266 watch dog.
    }
  }
  Serial.println(F("DFPlayer Mini online."));

  Serial.println(F("Configure rotary encoder..."));
  rotaryEncoder.begin();
  rotaryEncoder.setup(ReadEncoderISR);
  // minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  rotaryEncoder.setBoundaries(0, 3500, true);
  rotaryEncoder.setAcceleration(250);

  Serial.println(F("Starting!"));

  musicPlayer.volume(20);  // Set volume value. From 0 to 30
  musicPlayer.play(1);     // Play the first mp3
}

void loop() {
  Serial.println(F("Loop!"));

  // timeClient.update();
  struct tm now = getTimeStruct();

  uint8_t dots = (now.tm_sec % 2 == 0) ? 0b01000000 : 0;
  clockDisplay.showNumberDecEx(now.tm_hour, dots, true, 2, 0);
  clockDisplay.showNumberDecEx(now.tm_min,  dots, true, 2, 2);
  printTime(now);

  if (musicPlayer.available()) {
    // Print the detail message from DFPlayer to handle different errors and states.
    printDetail(musicPlayer.readType(), musicPlayer.read());
  }

  ledcWrite(RED_LED_PIN, redLedLuminosity);
  digitalWrite(WHITE_LED_PIN, LOW);

  // If we detect LOW signal, button is pressed
  if (digitalRead(ROTARY_ENCODER_BUTTON_PIN) == LOW) {
    // If 50ms have passed since last LOW pulse, it means that the
    // button has been pressed, released and pressed again
    if (millis() - lastButtonPress > 50) {
      Serial.println("Button pressed!");
      setupTimer();
    }
    // Remember last button press event
    lastButtonPress = millis();
  }

  if (redLedLuminosity < 255) {
    redLedLuminosity += 0.1;
  } else {
    redLedLuminosity = 0;
  }

  delay(200);
}

// -------------------------------------------------------------------------------------------------

void setupTimer() {
  clockDisplay.showNumberDecEx(88, 0b01000000, true, 2, 0);
  clockDisplay.showNumberDecEx(88, 0b01000000, true, 2, 2);

  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(WHITE_LED_PIN, HIGH);
  musicPlayer.play(2);
  delay(500);

  unsigned long waitTime = 0;

  while (digitalRead(ROTARY_ENCODER_BUTTON_PIN) == HIGH) {

    if (rotaryEncoder.encoderChanged()) {
      Serial.println(rotaryEncoder.readEncoder());
      waitTime = rotaryEncoder.readEncoder();
    }
    if (rotaryEncoder.isEncoderButtonClicked()) {
      Serial.println("button pressed");
    }

    displayTime(waitTime);
  }
  countdown(waitTime);
  Serial.println("Sortie de la boucle");
}

void countdown(unsigned long waitTime) {
  musicPlayer.play(8);
  delay(1000);

  unsigned long startTime = getTime();

  while (digitalRead(ROTARY_ENCODER_BUTTON_PIN) == HIGH) {
    unsigned long nowTime = getTime();
    unsigned long deltaTime = nowTime - startTime;
    unsigned long remainingTime = waitTime - deltaTime;
    Serial.print("Start time : "); Serial.println(startTime);
    Serial.print("Now   time : "); Serial.println(nowTime);
    Serial.print("Delta time : "); Serial.println(deltaTime);
    Serial.print("Remaining  : "); Serial.println(remainingTime);
    displayTime(remainingTime);
    delay(500);
    // ledcWrite(RED_LED_PIN, 0);

    if (remainingTime <= 0) {
      musicPlayer.play(3);
      for (int i = 0; i < 9; i++) {
        ledcWrite(RED_LED_PIN, 255);
        waitMilliseconds(random(10, 150));
        digitalWrite(WHITE_LED_PIN, HIGH);
        waitMilliseconds(random(10, 150));
        ledcWrite(RED_LED_PIN, 0);
        waitMilliseconds(random(10, 150));
        digitalWrite(WHITE_LED_PIN, LOW);
        waitMilliseconds(random(10, 150));
      }
      musicPlayer.play(5);
      for (int i = 0; i < 9; i++) {
        ledcWrite(RED_LED_PIN, 255);
        waitMilliseconds(random(10, 150));
        digitalWrite(WHITE_LED_PIN, HIGH);
        waitMilliseconds(random(10, 150));
        ledcWrite(RED_LED_PIN, 0);
        waitMilliseconds(random(10, 150));
        digitalWrite(WHITE_LED_PIN, LOW);
        waitMilliseconds(random(10, 150));
      }
      musicPlayer.play(7);
      for (int i = 0; i < 9; i++) {
        ledcWrite(RED_LED_PIN, 255);
        waitMilliseconds(random(10, 150));
        digitalWrite(WHITE_LED_PIN, HIGH);
        waitMilliseconds(random(10, 150));
        ledcWrite(RED_LED_PIN, 0);
        waitMilliseconds(random(10, 150));
        digitalWrite(WHITE_LED_PIN, LOW);
        waitMilliseconds(random(10, 150));
      }
      break;
    }
  }
}

unsigned long getTime() {
  time_t now;
  getTimeStruct();
  time(&now);
  return now;
}

struct tm getTimeStruct(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
  }
  return timeinfo;
}

void displayTime(unsigned long time) {
  int minutes = time / 60;
  int seconds = (time / 60 - minutes) * 60;
  clockDisplay.showNumberDecEx(minutes, 0b01000000, true, 2, 0);
  clockDisplay.showNumberDecEx(seconds, 0b01000000, true, 2, 2);
}

void printTime(struct tm timeinfo){
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

void waitMilliseconds(uint16_t waitTime) {
  uint32_t start = millis();
  while ((millis() - start) < waitTime) {
    delay(1);
  }
}

void printDetail(uint8_t type, int value) {
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}
