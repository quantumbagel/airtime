#include <Arduino.h> // Maybe don't have this, maybe ness. for clion lib to run
#include <SoftwareSerial.h> //gps
#include <TinyGPS.h> //gps
#include <Servo.h> //motor control
#include <SPI.h> // dunno
#include <nRF24L01.h> //transeiver
#include <RF24.h> // transceiver
#include <Wire.h> // gyroscope/accel.
#define rxPin 3
#define txPin 2
#define topLeftMotorPin 9
#define topRightMotorPin 10
#define bottomLeftMotorPin 11
#define bottomRightMotorPin 12
const int stablePower[4] = {40, 40, 40, 40}; //TODO: ADJUST W/ TEST MUCH HIGHER THAN THIS LOL MOTORS DON'T EVEN SPIN UNTIL 90 LOL LOL LOL LOL LOL LOL LOL LOL
const float maxIncrease = 10/9; //TODO: should be adjusted
const float maxDecrease = 1/maxIncrease; //should not be adjusted
const int minPower = 20; //TODO: CHANGE W/ TEST 90 PLUS
const int maxPower = 180;
const int motorPins[4] = {topLeftMotorPin, topRightMotorPin, bottomLeftMotorPin, bottomRightMotorPin};
const float  forwardVector[4] = {maxDecrease, maxDecrease, maxIncrease, maxIncrease};
const float backwardVector[4] = {maxIncrease, maxIncrease, maxDecrease, maxDecrease};
const float leftVector[4] = {maxDecrease, maxIncrease, maxDecrease, maxIncrease};
const float rightVector[4] = {maxIncrease, maxDecrease, maxIncrease, maxDecrease};
const float upVector[4] = {maxIncrease, maxIncrease, maxIncrease, maxIncrease};
const float downVector[4] = {maxDecrease, maxDecrease, maxDecrease, maxDecrease};
const float cwVector[4] = {maxDecrease, maxIncrease, maxIncrease, maxDecrease};
const float ccwVector[4] = {maxIncrease, maxDecrease, maxDecrease, maxIncrease};
const byte address[6] = "57385";
struct joysticks {
    float oneX = 0;
    float oneY = 0;
    float twoX = 0;
    float twoY = 0;
};
float vectors[4];
float motorSpeeds[4];
float maxAndMin[2];
float starting_lat;
float starting_long;
float current_lat;
float current_long;
unsigned long fix_age;
Servo motors[4];
RF24 data(7, 8); //TODO: CE, CSN pins (change as needed)
TinyGPS gps;
SoftwareSerial nss(rxPin, txPin);
float motorValues[4];
bool systemsReady = false;
joysticks rcv;
float current_altitude;
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float accAngleX, accAngleY, accAngleZ;
float gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float accErrorX, accErrorY, accErrorZ;
float gyroErrorX, gyroErrorY, gyroErrorZ;
void update_gps_lat_long() {
    if (nss.available()) {
        int c = nss.read();
        if (gps.encode(c)) {
            gps.f_get_position(&current_lat, &current_long, &fix_age)
        }
    }
}

void update_altitude_with_bmp180() {
    Serial.println("lollol u forgot to make this :P");
    // TODO: implement lol
}

void update_gyroscope_stuff() {
    Serial.println("lollol u forgot to make this :P");
    // TODO: implement lol
}

void setup() {
    Serial.begin(9600);// what does this do? (init. the serial port, prob remove w/ sd card debug)
    // set up the rf24 receiver
    data.begin();
    data.openReadingPipe(0, address);
    data.setPALevel(rf24_pa_dbm_e(0));
    data.startListening();
    // attach motor pins
    for (int i = 0; i < 5; i++) {
        motors[i].attach(motorpins[i], 1000, 2000); // TODO: change
        // TODO: add takeoff method
    while (nss.available() == false) // loop until we have lat/long for startup
    {
        int c = nss.read();
        if (gps.encode(c))
        {

            gps.f_get_position(&starting_lat, &starting_long, &fix_age);
            if (fix_age == TinyGPS::GPS_INVALID_AGE) {
                continue;
            }
        }
    }
    systemsReady = true;
    }

}

void setupVector(float upDown, float forwardBackward, float leftRight, float turn) {
    // get rel. vector speeds
    for (int i = 0; i < 5; i++) {
        vectors[i] = 1;
        if (upDown > 0) {
            vectors[i] *= upVector[i];
        } else if (upDown < 0) {
            vectors[i] *= downVector[i];
        }
        if (forwardBackward > 0) {
            vectors[i] *= forwardVector[i];
        } else if (forwardBackward < 0) {
            vectors[i] *= backwardVector[i];
        }
        if (leftRight > 0) {
            vectors[i] *= rightVector[i];
        } else if (leftRight < 0) {
            vectors[i] *= leftVector[i];
        }
        if (turn > 0) {
            vectors[i] *= cwVector[i];
        } else if (turn < 0) {
            vectors[i] *= ccwVector[i];
        }
    }
    // get max / min of the vectors
    float maximum = vectors[0];
    float minimum = vectors[0];
    for (int i = 1; i < 5; i++) {
        if (maximum < vectors[i]) {
            maximum = vectors[i];
        }
        if (minimum > vectors[i]) {
            minimum = vectors[i];
        }
    }
    maxAndMin = {maximum, minimum};
}


void calculateMotorSpeeds() {
    maximum = maxAndMin[0];
    minimum = maxAndMin[1];
    for (int i=0; i < 5; i++) {
        if (vectors[i] > 1) {
            motorspeeds[i] = stablePower[i] + (vectors[i] - 1) / (maximum - 1) * (maxPower-stablePower[i]);
        } else if (vectors[i] < 1) {
            motorspeeds[i] = stablePower[i] -  (1 - vectors[i]) / (1 - minimum) * (stablePower[i]-minPower);
        } else {
            motorspeeds[i] = stablePower[i];
        }
    }
}
void loop() {
    if (data.available() and systemsReady) {
        data.read(&rcv, sizeof(joysticks));
        float x1 = rcv.oneX;
        float x2 = rcv.twoX;
        float y1 = rcv.oneY;
        float y2 = rcv.twoY;
        setupVector(y1, y2, x2, x1);
        calculateMotorSpeeds();
        for (int i = 0; i < 5; i++) {
            motors[i].write(motorSpeeds[i]);
        }
    }
}

