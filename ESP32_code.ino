#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 
#define OLED_RESET -1 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Variables to hold the robot data
String robotDirection = "WAITING...";
int leftMotor = 0;
int rightMotor = 0;
String status = "[IDLE]";

int distL = 0;
int distM = 0;
int distR = 0;

//Setting up ULTRASONIC sensor
const int trigPinRIGHT = 13; //YELLOW
const int echoPinRIGHT = 12;

const int trigPinMIDDLE = 14; //GREEN
const int echoPinMIDDLE = 27;

const int trigPinLEFT = 26;
const int echoPinLEFT = 25; //RED

#define SOUND_SPEED 0.0343 

// Timer variable to track when last fired the sensors
unsigned long lastSensorTime = 0;

void setup() {
  // Serial for the Mac (Debugging)
  Serial.begin(115200);
  
  // Serial2 for listening to the MSP432 
  // RX is GPIO 16, TX is GPIO 17
  Serial2.begin(9600, SERIAL_8N1, 16, 17);   

  // Boot up the OLED on standard I2C pins (21 and 22)
  Wire.begin();
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("OLED failed to boot"));
    for(;;); 
  }

  //Set up ultrasonic sensors
  pinMode(trigPinRIGHT, OUTPUT); 
  pinMode(echoPinRIGHT, INPUT);  

  pinMode(trigPinMIDDLE, OUTPUT); 
  pinMode(echoPinMIDDLE, INPUT);  

  pinMode(trigPinLEFT, OUTPUT); 
  pinMode(echoPinLEFT, INPUT);

  // Draw the initial standby screen
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE); 
  updateScreen(); 
}

void loop() {

  // Continuos refersh of OLED module

  // Check if the MSP432 is sending a message over the wires
  if (Serial2.available()) {
    String incomingData = Serial2.readStringUntil('\n');
    incomingData.trim(); // Clean up any invisible characters
    
    // Print to Mac Serial Monitor just so you can see it working
    Serial.println("MSP432 Says: " + incomingData); 

    // EXPECTED FORMAT: "FORWARD,4000,4500"
    // Let's chop it up based on the commas!
    int firstComma = incomingData.indexOf(',');
    int secondComma = incomingData.indexOf(',', firstComma + 1);

    int bracketIndex = incomingData.indexOf(']');

    // If we found both commas, the message is formatted correctly
    if (firstComma > 0 && secondComma > 0) {
      robotDirection = incomingData.substring(0, firstComma);
      leftMotor = incomingData.substring(firstComma + 1, secondComma).toInt();
      rightMotor = incomingData.substring(secondComma + 1).toInt();   
    }
    if (bracketIndex >= 0 ) {
      status = incomingData;
    }
    //Redraw screen since data is updated
    updateScreen();
  }
  //Read ultrasonic sensors and send to MSP432
  if (millis() - lastSensorTime > 200) {
    fireSensorsAndSend();
    lastSensorTime = millis(); // Reset the stopwatch
  }
}

void fireSensorsAndSend() {
  long duration;

  ///// Ping LEFT ultrasonic sensor
  digitalWrite(trigPinLEFT, LOW); delayMicroseconds(2);
  //Set leftmost trigPin high for 10 microseconds to trigger the ultrasonic sensor
  // Delay to prevent cross interference of ultrasonic sensor (set up in an array of 3)
  digitalWrite(trigPinLEFT, HIGH); delayMicroseconds(10);
  digitalWrite(trigPinLEFT, LOW);
  // Read echo pulse duration in microseconds
  duration = pulseIn(echoPinLEFT, HIGH, 30000);
  distL = (duration * SOUND_SPEED) / 2;

  /*if (distanceCm == 0 || distanceCm > 400) {
    Serial.println("Out of range or sensor error!");
  } else {
    Serial.print("Distance for LEFT sensor: ");
    Serial.print(distanceCm);
    Serial.println(" cm");
  }*/
  delay(10);

  ////// Ping MIDDLE ultrasonic sensor
  digitalWrite(trigPinMIDDLE, LOW); delayMicroseconds(2);
  //Set middle trigPin high for 10 microseconds to trigger the ultrasonic sensor
  digitalWrite(trigPinMIDDLE, HIGH); delayMicroseconds(10);
  digitalWrite(trigPinMIDDLE, LOW);
  duration = pulseIn(echoPinMIDDLE, HIGH, 30000);
  //calculate distance
  distM = (duration * SOUND_SPEED) / 2;

  /*if (distanceCm == 0 || distanceCm > 400) {
    Serial.println("Out of range or sensor error!");
  } else {
    Serial.print("Distance for MIDDLE sensor: ");
    Serial.print(distanceCm);
    Serial.println(" cm");
  }*/
  delay(10);


   ////// Ping RIGHT ultrasonic sensor
  digitalWrite(trigPinRIGHT, LOW); delayMicroseconds(2);
  //Set rightmost trigPin high for 10 microseconds to trigger the ultrasonic sensor
  digitalWrite(trigPinRIGHT, HIGH); delayMicroseconds(10);
  digitalWrite(trigPinRIGHT, LOW);
  duration = pulseIn(echoPinRIGHT, HIGH, 30000);
  //calculate distance
  distR = (duration * SOUND_SPEED) / 2;

  updateScreen();

  /*if (distanceCm == 0 || distanceCm > 400) {
    Serial.println("Out of range or sensor error!");
  } else {
    Serial.print("Distance for RIGHT sensor: ");
    Serial.print(distanceCm);
    Serial.println(" cm");
  }
  Serial.println("RESTARTING SEQUENCE----------------");
  delay(250); */
  
  //Send data to MSP432
  char telemetryString[50];
  sprintf(telemetryString, "<DIST,%d,%d,%d>\n", distL, distM, distR);
  
  // Send over the TX wire to the MSP432
  Serial2.print(telemetryString);
  
  
  Serial.print("Sent to Brain: ");
  Serial.print(telemetryString);
}

// Custom function to draw on OLED
void updateScreen() {
  display.clearDisplay();
  
  // Draw Direction (Large Text)
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(robotDirection);

  // Draw Motor Power Stats (Smaller Text)
  display.setTextSize(1);
  display.setCursor(0, 25);
  display.print("Left  PWR: ");
  display.println(leftMotor);
  
  display.setCursor(0, 40);
  display.print("Right PWR: ");
  display.println(rightMotor);

  //Display Status of Robot (smaller text)
  display.setCursor(90, 55);
  display.println(status);

  //Display information about distance of obstacle
  int smallest = distL;

  if (smallest > distM) {
    smallest = distM;
  }

  if (smallest > distR) {
    smallest = distR;
  }

  display.setCursor(0, 55);
  display.print("[");
  display.print(smallest);
  display.print(" cm]");

  // Push to the physical screen
  display.display();
}
