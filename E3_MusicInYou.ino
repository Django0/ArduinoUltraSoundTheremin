#include "Volume.h" // Include the Volume library

Volume vol; // Plug your speaker into the default pin for your board type:
// https://github.com/connornishijima/arduino-volume#supported-pins


// Ultrasound
int inputPinDist = 3;     //ECHO pin
int outputPinDist = 2;    //TRIG pin

int TurnOnLEDNum = 2;     // which pin to blink for LED

int WinLEDPin = 12;       // Winning LED: Smoke on the Water
int startStrumPin = 10;   // Blue light from LED to give the user the start of sequence
#define NUM_OF_LEDS 4
int ArrayOfLEDPins[NUM_OF_LEDS] = {4, 6, 7, 8};
int strumPin = 11;        // Keep this pressed to play
int wahwahPin = 9;        // Continuous sound

#define DELAY_NOTES 1000 // seconds
#define LEN_EACH_NOTE 3

/*
  #define NUM_NOTES   12
  char ExpectedRiff[LEN_EACH_NOTE * NUM_NOTES] = "E2G2A2E2G2A2#A2E2G2A2G2E2";
  #define NUM_NOTES   3
  char ExpectedRiff[LEN_EACH_NOTE * NUM_NOTES] = "E2G2A2";
*/
#define NUM_NOTES   7
char ExpectedRiff[LEN_EACH_NOTE * NUM_NOTES] = "E3G3A3E3G3A3#A3"; // 0 3 5 0 3 6 5


char SmokeOnTheWaterRiff[LEN_EACH_NOTE * NUM_NOTES] = {0}; // GLOBAL VARIABLE

uint32_t ledBlink;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(WinLEDPin, OUTPUT);
  digitalWrite(WinLEDPin, HIGH);

  pinMode(startStrumPin, OUTPUT);
  digitalWrite(startStrumPin, LOW);

  for (int i = 0; i < NUM_OF_LEDS; i++)
  {
    pinMode(ArrayOfLEDPins[i], OUTPUT);
    digitalWrite(ArrayOfLEDPins[i], HIGH);
  }

  // Ultrasound
  pinMode(inputPinDist, INPUT);
  pinMode(outputPinDist, OUTPUT);

  Serial.begin(57600); //serial port to computer
  delay(1000);//while (!Serial); //wait for Serial to be available - remove this line after successful test run
  Serial.println("Startup");

  vol.begin();
  vol.tone(82, 255);  // 82 Hz, Note:E2
  vol.fadeOut(1000);  // Start a 1 s fade out
  vol.delay(1000);    // Wait for this fade to finish
  
  pinMode(strumPin, INPUT);
  pinMode(wahwahPin, INPUT);
  digitalWrite(wahwahPin, HIGH);  // To overcome loose wires
  digitalWrite(WinLEDPin, LOW);

}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void loop() {
  ledBlink = (ledBlink == HIGH) ? LOW : HIGH;
  digitalWrite(LED_BUILTIN, ledBlink);  // LED toggle

  // Ultrasound
  float distance = measureDistance();

  if (distance > 2) {
    //float freq = 330/(distance*0.01);   // 0.01 is due to centimeter resolution and 330 m/s is the speed of sound in vacuum.
    float freq = mapDistToFreq(distance);
    Serial.print("Distance = "); Serial.print(distance); Serial.print(" cm; Freq. = "); Serial.print(freq); Serial.println(" Hz");
    if (digitalRead(strumPin) == LOW) {
      gameOn();
    }
    if (digitalRead(wahwahPin) == LOW) {
      vol.tone(freq, 255);
    }
  }
  //vol.fadeOut(500);  // Start a 1 s fade out
  //vol.delay(500);    // Wait for this fade to finish

  /*
    byte volumes[4] = {255, 127, 12, 0};   // List of volumes: 100% Volume, 50% Volume, 5% Volume, 0% Volume
    for (int i = 0; i < 4; i++) { // Iterate through volume list one second at a time
    vol.tone(440, volumes[i]);
    vol.delay(1000);
    }
    vol.tone(880, 255); // 100% Volume

    vol.tone(82.4, 255); vol.fadeOut(1000); vol.delay(1000);
    vol.tone(98, 255); vol.fadeOut(1000); vol.delay(1000);
    vol.tone(440, 255); vol.fadeOut(1000); vol.delay(1000);
    vol.tone(880, 255); vol.fadeOut(1000); vol.delay(1000);

  */
  for (int i = 0; i < NUM_OF_LEDS; i++)
  {
    if (TurnOnLEDNum == ArrayOfLEDPins[i])
    {
      digitalWrite(TurnOnLEDNum, HIGH);
    }
    else
    {
      digitalWrite(ArrayOfLEDPins[i], LOW);
    }
  }
  //vol.delay(1000);

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
float measureDistance(void)
{
  digitalWrite(outputPinDist, HIGH); //Trigger ultrasonic detection
  delayMicroseconds(10);
  digitalWrite(outputPinDist, LOW);
  float distance = pulseIn(inputPinDist, HIGH); //Read ultrasonic reflection
  distance = distance / 58; //Calculate distance
  return (distance);
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void lightenUpAllLEDs(int highOrLow, int delayTime)
{
  for (int i = 0; i < NUM_OF_LEDS; i++)
  {
    digitalWrite(ArrayOfLEDPins[i], highOrLow);
    if (delayTime > 0)
    {
      vol.delay(delayTime);
    }
  }
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void gameOn(void)
{
  lightenUpAllLEDs(LOW, 0);
  lightenUpAllLEDs(HIGH, DELAY_NOTES);
  lightenUpAllLEDs(LOW, 0);

  while (1)
  {
    if (digitalRead(strumPin) == LOW)
    {
      memset(SmokeOnTheWaterRiff, 0, NUM_NOTES * sizeof(char));
      for (int i = 0; i < NUM_NOTES; i++)
      {
        float dist = measureDistance();
        float freq = mapDistToFreq(dist - 2); // HACK! the damn offset! why the bonk does the sensor act differently!
        if (0 == i) {
          digitalWrite(startStrumPin, HIGH);
        } else {
          digitalWrite(startStrumPin, LOW);
        }
        vol.tone(freq, 255);
        vol.fadeOut(DELAY_NOTES);
        vol.delay(DELAY_NOTES);
      }

      if (0 == strcmp(ExpectedRiff, SmokeOnTheWaterRiff))
      {
        lightenUpAllLEDs(HIGH, 0);
        digitalWrite(WinLEDPin, HIGH);
        Serial.println("Success");
        vol.delay(5000);
      }
      else
      {
        Serial.println("FAILED");
      }

    }
    else
    {
      return;
    }
  }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
float mapDistToFreq(float distance)
{
  float freq = 27.5; // All in Hz
  char str[] = "A0";

  TurnOnLEDNum = LED_BUILTIN;
  
  if ((36 > distance) && (distance >= 30)) {
    freq = 164.8; strcpy(str, "E3"); // E
    TurnOnLEDNum = ArrayOfLEDPins[0];
  } else if ((30 > distance) && (distance >= 26)) {
    freq = 174.6; strcpy(str, "F3"); // F

  } else if ((26 > distance) && (distance >= 22)) {
    freq = 185; strcpy(str, "F3#");   // F#

  } else if ((22 > distance) && (distance >= 17)) {
    freq = 196;  strcpy(str, "G3");  // G
    TurnOnLEDNum = ArrayOfLEDPins[1];
  } else if ((17 > distance) && (distance >= 12)) {
    freq = 207.7; strcpy(str, "G3#"); // G#

  } else if ((12 > distance) && (distance >= 7)) {
    freq = 220; strcpy(str, "A3");   // A
    TurnOnLEDNum = ArrayOfLEDPins[2];
  } else if ((7 > distance) && (distance >= 2)) {
    freq = 233.1; strcpy(str, "A3#"); // A#
    TurnOnLEDNum = ArrayOfLEDPins[3];
  }

  Serial.print("Note: ");
  Serial.print(str);
  Serial.print(" Dist: ");
  Serial.println(distance);
  
  strcat(SmokeOnTheWaterRiff, str);
  return (freq);
}

