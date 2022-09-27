#include <Arduino.h> // Maybe don't have this, maybe ness. for clion lib to run
#include <SoftwareSerial.h> //gps
#include <TinyGPS.h> //gps
#include <Servo.h> //motor control
#include <SPI.h> // dunno
#include <nRF24L01.h> //transeiver
#include <RF24.h> // transceiver
#include <Adafruit_BMP085.h> // bmp180 lib
#include "MPU6050_6Axis_MotionApps20.h" // mpu6050 https://github.com/ElectronicCats/mpu6050
#include "I2Cdev.h" // mpu6050
#include "Wire.h" // mpu6050#include <SoftwareSerial.h> //gps
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
const int MPU = 0x68;
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
float previousTime, currentTime, elapsedTime;
Servo motors[4];
RF24 data(7, 8); //TODO: CE, CSN pins (change as needed)
TinyGPS gps;
MPU6050 mpu(MPU);
SoftwareSerial nss(rxPin, txPin);
float motorValues[4];
bool systemsReady = false;
bool droneInAir = false;
joysticks rcv;
float current_altitude;
struct gdata {
    struct acc {
        float x;
        float y;
        float z;
    };
    struct gyro {
        float x;
        float y;
        float z;
    };
};


void update_gps_lat_long() {
    if (nss.available()) {
        int c = nss.read();
        if (gps.encode(c)) {
            gps.f_get_position(&current_lat, &current_long, &fix_age)
        }
    }
}


//void update_altitude_with_bmp180() {
//    Serial.println("lollol u forgot to make this :P");
//    char status;
//    status = pressure.getPressure(P, T);
//    if (status != 0) {
//
//    }
//    // implement lol
//}


void update_gyroscope_stuff() {
    mpu.getMotion6(&gdata.acc.x, &gdata.acc.y, &gdata.acc.z, &gdata.gyro.x, &gdata.gyro.y, &gdata.gyro.z);
}



void setup() {
    Serial.begin(9600);// what does this do? (init. the serial port, prob remove w/ sd card debug)
    // set up the rf24 receiver
    data.begin();
    data.openReadingPipe(0, address);
    data.setPALevel(rf24_pa_dbm_e(0));
    data.startListening();
    pressure.begin();
    Wire.begin();
    mpu.initalize();
    // attach motor pins
    for (int i = 0; i < 5; i++) {
        motors[i].attach(motorpins[i], 1000, 2000); // TODO: change / make more sensitive
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


void takeOff() { // gonna need some form of launch algo (maybe 20 feet above groundish)

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
    if (data.available() and systemsReady) { // ensure we don't touch the motors UNTIL systems are ready AND we actually have data lol
        data.read(&rcv, sizeof(joysticks));
        float x1 = rcv.oneX;
        float x2 = rcv.twoX;
        float y1 = rcv.oneY;
        float y2 = rcv.twoY;
        if (x1 > 0.95 and y1 < -0.95 and x2 < -0.95, y2 < -0.95) { // if sticks in 'cross-eyed' pos. take off
            takeOff(); // take off
        } else {
            setupVector(y1, y2, x2, x1);
            calculateMotorSpeeds();
        }
        for (int i = 0; i < 5; i++) {
            motors[i].write(motorSpeeds[i]);
        }

    }
}

