#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <LiquidCrystal_I2C.h>

// ---------------- LCD ----------------
LiquidCrystal_I2C lcd(0x27,16,2);

// ---------------- GPS ----------------
SoftwareSerial gpsSerial(8,9); // RX, TX for GPS
TinyGPSPlus gps;
float latitude = 0.0, longitude = 0.0;
bool showLat = true;  // toggle for LCD display

// ---------------- MPU ----------------
const int MPU_ADDR = 0x68;
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;
unsigned long lastMPUPrint = 0;
const unsigned long mpuInterval = 1000; // print every 1s

// ---------------- Heart Rate ----------------
int pulsePin = A0;
int threshold = 550;
int pulseValue = 0; // raw sensor value
int BPM = 0;
int beatCount = 0;
unsigned long hrStartTime = 0;

// ---------------- Button ----------------
const int buttonPin = 7;
bool sosTriggered = false;
unsigned long lastButtonPress = 0;
const unsigned long buttonDebounce = 300; // 300 ms debounce

// ---------------- Timing ----------------
unsigned long lastLCDUpdate = 0;
const unsigned long lcdInterval = 1000; // update every 1 second

// ---------------- SIM800L ----------------
SoftwareSerial sim800l(2,3); // RX, TX for SIM800L
String phoneNumber = "+918310654801"; // <-- Replace with your number
unsigned long lastSMSSent = 0;
const unsigned long smsInterval = 10000; // prevent spam (10s gap)

// ---------------- Setup ----------------
void setup() {
  Serial.begin(9600);
  Wire.begin();
  gpsSerial.begin(9600);
  sim800l.begin(9600);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Women Safety Device");

  pinMode(buttonPin, INPUT_PULLUP);

  // MPU initialization
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up MPU-6050
  Wire.endTransmission(true);

  // SIM800L initialization
  delay(2000);
  sim800l.println("AT");
  delay(500);
  sim800l.println("AT+CMGF=1"); // text mode
  delay(500);

  delay(1000);
  Serial.println("System Ready");
}

// ---------------- Loop ----------------
void loop() {
  checkSOS();        // check panic button first (high priority)
  readGPS();         // non-blocking GPS read
  readMPU();         // non-blocking MPU read
  readHeartRate();   // read pulse

  if (millis() - lastLCDUpdate >= lcdInterval) {
    updateLCD();
    lastLCDUpdate = millis();
    showLat = !showLat;
  }
}

// ---------------- SOS Function ----------------
void checkSOS() {
  // Active LOW button
  if (digitalRead(buttonPin) == LOW && (millis() - lastButtonPress > buttonDebounce)) {
    lastButtonPress = millis();

    // Show SOS instantly
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("!!! SOS !!!");
    lcd.setCursor(0,1);
    lcd.print("Sending alert...");
    Serial.println("SOS ACTIVATED!");

    // Build SMS message
    String msg = "SOS ALERT!\n";
    if (gps.location.isValid()) {
      msg += "Lat: " + String(latitude,6) + "\n";
      msg += "Lon: " + String(longitude,6) + "\n";
      msg += "Map: https://maps.google.com/?q=" + String(latitude,6) + "," + String(longitude,6) + "\n";
    } else {
      msg += "GPS not fixed!\n";
    }
    msg += "Pulse: " + String(pulseValue);
    msg += "\nAccelX: " + String(accelX/100);

    sendSMS(phoneNumber, msg);

    sosTriggered = true;
  }
}

// ---------------- GPS Function ----------------
void readGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  }
}

// ---------------- MPU Function ----------------
void readMPU() {
  if (millis() - lastMPUPrint < mpuInterval) return;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
  int16_t temp = Wire.read() << 8 | Wire.read();
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();

  Serial.print("aX: "); Serial.print(accelX);
  Serial.print(" | aY: "); Serial.print(accelY);
  Serial.print(" | aZ: "); Serial.print(accelZ);
  Serial.print(" | gX: "); Serial.print(gyroX);
  Serial.print(" | gY: "); Serial.print(gyroY);
  Serial.print(" | gZ: "); Serial.println(gyroZ);

  lastMPUPrint = millis();
}

// ---------------- Heart Rate Function ----------------
void readHeartRate() {
  pulseValue = analogRead(pulsePin);
  Serial.print("Pulse Sensor: ");
  Serial.println(pulseValue);

  if (pulseValue > threshold) {
    beatCount++;
    delay(200); // short debounce
  }

  if (millis() - hrStartTime >= 10000) { // every 10s
    BPM = beatCount * 6;
    beatCount = 0;
    hrStartTime = millis();
  }
}

// ---------------- LCD Update Function ----------------
void updateLCD() {
  lcd.clear();

  // Row 0: GPS data
  lcd.setCursor(0,0);
  String lcdLine1;
  if (showLat) {
    lcd.print("Lat:");
    lcd.print(latitude, 4);
    lcdLine1 = "Lat:" + String(latitude,4);
  } else {
    lcd.print("Lon:");
    lcd.print(longitude, 4);
    lcdLine1 = "Lon:" + String(longitude,4);
  }

  // Row 1: Accel X + Pulse
  lcd.setCursor(0,1);
  lcd.print("aX:");
  lcd.print(accelX/100);
  lcd.print(" P:");
  lcd.print(pulseValue);

  String lcdLine2 = "aX:" + String(accelX/100) + " P:" + String(pulseValue);

  // --- Send LCD message as SMS (every smsInterval ms) ---
  if (millis() - lastSMSSent > smsInterval) {
    String fullMessage = lcdLine1 + "\n" + lcdLine2;
    sendSMS(phoneNumber, fullMessage);
    lastSMSSent = millis();
  }
}

// ---------------- SIM800L SMS Function ----------------
void sendSMS(String number, String text) {
  Serial.println("Sending SMS...");
  sim800l.print("AT+CMGS=\"");
  sim800l.print(number);
  sim800l.println("\"");
  delay(500);
  sim800l.print(text);
  sim800l.write(26); // CTRL+Z to send SMS
  delay(2000);
  Serial.println("SMS sent: " + text);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("SOS Sent!");
  lcd.setCursor(0,1);
  lcd.print("Stay calm...");
}
