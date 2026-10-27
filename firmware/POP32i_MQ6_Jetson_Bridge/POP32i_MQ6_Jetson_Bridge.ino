#include <POP32.h>
#include <string.h>
#include <stdlib.h>

// DT-SENSE MQ-6 RX -> POP32i PB6 (TX)
// DT-SENSE MQ-6 TX -> POP32i PB7 (RX, verify 5V tolerance)
// GND common, MQ-6 powered from regulated servo 5V.
#define MQ6_RX PB7
#define MQ6_TX PB6
#define BUZZER_PIN PB5

HardwareSerial mq6(MQ6_RX, MQ6_TX);

const unsigned long READ_INTERVAL_MS = 500;
const unsigned long RESPONSE_TIMEOUT_MS = 200;
const unsigned long SCREEN_INTERVAL_MS = 250;
const unsigned long LINK_TIMEOUT_MS = 3500;
const unsigned long ALARM_PERIOD_MS = 600;
const unsigned long ALARM_ON_MS = 300;

bool waitingForSensor = false;
bool hasSensorResult = false;
bool sensorOK = false;
bool hasJetsonPing = false;
bool alarmEnabled = false;
bool singleBeep = false;
bool buzzerOn = false;

uint16_t sensorRaw = 0;
uint8_t responseCount = 0;
uint8_t responseHi = 0;
uint8_t responseLo = 0;
uint32_t samples = 0;
unsigned long lastRequest = 0;
unsigned long requestStarted = 0;
unsigned long lastScreen = 0;
unsigned long lastPing = 0;
unsigned long alarmStarted = 0;
unsigned long beepStarted = 0;
unsigned long beepLength = 0;
char cmd[40];
uint8_t cmdLength = 0;
bool cmdTooLong = false;

bool jetsonOnline(unsigned long now) {
  return hasJetsonPing && (now - lastPing <= LINK_TIMEOUT_MS);
}

void sendReading(unsigned long now) {
  Serial.print("MQ6,");
  Serial.print(now);
  Serial.print(',');
  if (sensorOK) {
    Serial.print(sensorRaw);
    Serial.println(",OK");
  } else {
    Serial.println("-1,ERROR");
  }
}

void updateSensor(unsigned long now) {
  if (!waitingForSensor && now - lastRequest >= READ_INTERVAL_MS) {
    while (mq6.available()) mq6.read();
    mq6.write((uint8_t)0x41);
    waitingForSensor = true;
    requestStarted = now;
    lastRequest = now;
    responseCount = 0;
  }
  if (!waitingForSensor) return;

  while (mq6.available() && responseCount < 2) {
    uint8_t value = (uint8_t)mq6.read();
    if (responseCount == 0) responseHi = value;
    else responseLo = value;
    ++responseCount;
  }
  if (responseCount == 2) {
    uint16_t value = ((uint16_t)responseHi << 8) | responseLo;
    sensorOK = (value <= 1023);
    if (sensorOK) sensorRaw = value;
    hasSensorResult = true;
    waitingForSensor = false;
    ++samples;
    sendReading(now);
  } else if (now - requestStarted >= RESPONSE_TIMEOUT_MS) {
    sensorOK = false;
    hasSensorResult = true;
    waitingForSensor = false;
    ++samples;
    sendReading(now);
  }
}

void turnOffBuzzer() {
  alarmEnabled = false;
  singleBeep = false;
  noTone(BUZZER_PIN);
  buzzerOn = false;
}

void executeCommand(const char *line) {
  if (strcmp(line, "PING") == 0) {
    hasJetsonPing = true;
    lastPing = millis();
    Serial.println("PONG");
  } else if (strcmp(line, "ALARM,ON") == 0) {
    alarmEnabled = true;
    singleBeep = false;
    alarmStarted = millis();
    Serial.println("ACK,ALARM,ON");
  } else if (strcmp(line, "ALARM,OFF") == 0) {
    turnOffBuzzer();
    Serial.println("ACK,ALARM,OFF");
  } else if (strncmp(line, "BEEP,", 5) == 0) {
    char *end = NULL;
    long duration = strtol(line + 5, &end, 10);
    if (end == line + 5 || *end != '\0' || duration < 50 || duration > 2000) {
      Serial.println("ERR,BEEP_DURATION");
    } else if (alarmEnabled) {
      Serial.println("ERR,ALARM_ACTIVE");
    } else {
      singleBeep = true;
      beepStarted = millis();
      beepLength = (unsigned long)duration;
      Serial.println("ACK,BEEP");
    }
  } else {
    Serial.println("ERR,UNKNOWN_COMMAND");
  }
}

void updateJetsonCommands() {
  uint8_t handled = 0;
  while (Serial.available() && handled++ < 64) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (cmdTooLong) Serial.println("ERR,COMMAND_TOO_LONG");
      else if (cmdLength > 0) {
        cmd[cmdLength] = '\0';
        executeCommand(cmd);
      }
      cmdLength = 0;
      cmdTooLong = false;
    } else if (!cmdTooLong) {
      if (cmdLength < sizeof(cmd) - 1) cmd[cmdLength++] = ch;
      else cmdTooLong = true;
    }
  }
}

void updateBuzzer(unsigned long now) {
  bool play = false;
  if (alarmEnabled) {
    play = (now - alarmStarted) % ALARM_PERIOD_MS < ALARM_ON_MS;
  } else if (singleBeep) {
    if (now - beepStarted < beepLength) play = true;
    else singleBeep = false;
  }
  if (play != buzzerOn) {
    if (play) tone(BUZZER_PIN, 2200);
    else noTone(BUZZER_PIN);
    buzzerOn = play;
  }
}

void updateScreen(unsigned long now) {
  oled.clear();
  oled.text(0, 0, "MQ6 / JETSON");
  oled.text(1, 0, jetsonOnline(now) ? "JETSON: ONLINE" :
               (hasJetsonPing ? "JETSON: LOST" : "JETSON: WAIT"));
  oled.text(2, 0, !hasSensorResult ? "MQ6: WAIT" :
               (sensorOK ? "MQ6: OK" : "MQ6: ERROR"));
  if (sensorOK) oled.text(3, 0, "RAW ADC: %d", (int)sensorRaw);
  else oled.text(3, 0, "RAW ADC: ---");
  oled.text(4, 0, alarmEnabled ? "BUZZER: ALARM" :
               (singleBeep ? "BUZZER: BEEP" : "BUZZER: OFF"));
  oled.text(6, 0, "SAMPLES: %lu", (unsigned long)samples);
  oled.text(7, 0, "USB COM 115200");
  oled.show();
}

void setup() {
  Serial.begin(115200);
  mq6.begin(38400);
  pinMode(BUZZER_PIN, OUTPUT);
  turnOffBuzzer();
  oled.clear();
  oled.text(0, 0, "MQ6 SERIAL BRIDGE");
  oled.text(2, 0, "Starting...");
  oled.show();
  Serial.println("READY,POP32I_MQ6,V1");
}

void loop() {
  unsigned long now = millis();
  updateJetsonCommands();
  updateSensor(now);
  updateBuzzer(now);
  if (now - lastScreen >= SCREEN_INTERVAL_MS) {
    lastScreen = now;
    updateScreen(now);
  }
}
