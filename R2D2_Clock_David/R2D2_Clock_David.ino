#include "WiFiManager.h"
#include "NTPClient.h"
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

void PrintDetail(uint8_t type, int value);

// Clock Display (4x 7 Digits)

const int CLOCK_DISPLAY_BRIGHTNESS = 3; // 0-7
TM1637Display clockDisplay(21, 22);

// NTP Client

const int  UTC                = 2;    // UTC + value in hour - Summer time
const long UTC_OFFSET_SECONDS = 3600; // UTC + 2H / Offset in second

WiFiUDP   ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", UTC_OFFSET_SECONDS *UTC);

// Internal State

float counter = 0;
unsigned long lastButtonPress = 0;
int btnState = 0;
int secondes = 0;
int minutes = 0;
float inc_red_led = 0;

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

  timeClient.begin();
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

  timeClient.update();
  uint8_t dots = (timeClient.getSeconds() % 2 == 0) ? 0b01000000 : 0;
  clockDisplay.showNumberDecEx(timeClient.getHours(), dots, true, 2, 0);
  clockDisplay.showNumberDecEx(timeClient.getMinutes(), dots, true, 2, 2);

  Serial.print("Time: ");
  Serial.println(timeClient.getFormattedTime());
  unsigned long epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime((time_t *)&epochTime);
  int currentYear = ptm->tm_year + 1900;
  Serial.print("Year: ");
  Serial.println(currentYear);

  int monthDay = ptm->tm_mday;
  Serial.print("Month day: ");
  Serial.println(monthDay);

  int currentMonth = ptm->tm_mon + 1;
  Serial.print("Month: ");
  Serial.println(currentMonth);

  if (musicPlayer.available()) {
    // Print the detail message from DFPlayer to handle different errors and states.
    PrintDetail(musicPlayer.readType(), musicPlayer.read());
  }

  btnState = digitalRead(ROTARY_ENCODER_BUTTON_PIN);
  ledcWrite(RED_LED_PIN, inc_red_led);
  digitalWrite(WHITE_LED_PIN, LOW);

  // If we detect LOW signal, button is pressed
  if (btnState == LOW) {
    // If 50ms have passed since last LOW pulse, it means that the
    // button has been pressed, released and pressed again
    if (millis() - lastButtonPress > 50) {
      Serial.println("Button pressed!");
      setupTimer();
    }
    // Remember last button press event
    lastButtonPress = millis();
  }

  if (inc_red_led <= 254) {
    inc_red_led = inc_red_led + 0.1;
  } else {
    inc_red_led = 0;
  }

  delay(100);

  if ((currentMonth * 30 + monthDay) >= 121 && (currentMonth * 30 + monthDay) < 331) {
    // Change daylight saving time - Summer - change 31/03 at 00:00
    timeClient.setTimeOffset(UTC_OFFSET_SECONDS * UTC);
  } else {
    // Change daylight saving time - Winter - change 31/10 at 00:00
    timeClient.setTimeOffset((UTC_OFFSET_SECONDS * UTC) - 3600);
  }
}

// -------------------------------------------------------------------------------------------------

void setupTimer() {
  clockDisplay.showNumberDecEx(88, 0b01000000, true, 2, 0);
  clockDisplay.showNumberDecEx(88, 0b01000000, true, 2, 2);

  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(WHITE_LED_PIN, HIGH);
  musicPlayer.play(2);
  delay(500);

  btnState = digitalRead(ROTARY_ENCODER_BUTTON_PIN);
  while (digitalRead(ROTARY_ENCODER_BUTTON_PIN) == HIGH) {

    if (rotaryEncoder.encoderChanged()) {
      Serial.println(rotaryEncoder.readEncoder());
      counter = rotaryEncoder.readEncoder();
    }
    if (rotaryEncoder.isEncoderButtonClicked()) {
      Serial.println("button pressed");
    }

    minutes = counter / 60;
    secondes = ((counter / 60) - minutes) * 60;
    clockDisplay.showNumberDecEx(minutes, 0b01000000, true, 2, 0);
    clockDisplay.showNumberDecEx(secondes, 0b01000000, true, 2, 2);
  }
  Countdown(counter);
  Serial.println("Sortie de la boucle");
}

void Countdown(float timerCounter) {
  musicPlayer.play(8);
  delay(1000);
  btnState = digitalRead(ROTARY_ENCODER_BUTTON_PIN);

  while (btnState == HIGH) {
    btnState = digitalRead(ROTARY_ENCODER_BUTTON_PIN);
    for (int i = 10; i > 0; i--) {
      timerCounter -= 0.1;
      minutes = timerCounter / 60;
      secondes = ((timerCounter / 60) - minutes) * 60;
      clockDisplay.showNumberDecEx(minutes, 0b01000000, true, 2, 0);
      clockDisplay.showNumberDecEx(secondes, 0b01000000, true, 2, 2);
      delay(70);
      digitalWrite(RED_LED_PIN, LOW);
    }

    if (timerCounter <= 0) {
      musicPlayer.play(3);
      for (int i = 0; i < 9; i++) {
        ledcWrite(RED_LED_PIN, 255);
        WaitMilliseconds(random(10, 150));
        digitalWrite(WHITE_LED_PIN, HIGH);
        WaitMilliseconds(random(10, 150));
        ledcWrite(RED_LED_PIN, 0);
        WaitMilliseconds(random(10, 150));
        digitalWrite(WHITE_LED_PIN, LOW);
        WaitMilliseconds(random(10, 150));
      }
      musicPlayer.play(5);
      for (int i = 0; i < 9; i++) {
        ledcWrite(RED_LED_PIN, 255);
        WaitMilliseconds(random(10, 150));
        digitalWrite(WHITE_LED_PIN, HIGH);
        WaitMilliseconds(random(10, 150));
        ledcWrite(RED_LED_PIN, 0);
        WaitMilliseconds(random(10, 150));
        digitalWrite(WHITE_LED_PIN, LOW);
        WaitMilliseconds(random(10, 150));
      }
      musicPlayer.play(7);
      for (int i = 0; i < 9; i++) {
        ledcWrite(RED_LED_PIN, 255);
        WaitMilliseconds(random(10, 150));
        digitalWrite(WHITE_LED_PIN, HIGH);
        WaitMilliseconds(random(10, 150));
        ledcWrite(RED_LED_PIN, 0);
        WaitMilliseconds(random(10, 150));
        digitalWrite(WHITE_LED_PIN, LOW);
        WaitMilliseconds(random(10, 150));
      }
      btnState = LOW;
      counter = 0;
    };
  }
}

void WaitMilliseconds(uint16_t waitTime) {
  uint32_t start = millis();
  while ((millis() - start) < waitTime) {
    delay(1);
  }
}

void PrintDetail(uint8_t type, int value) {
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
