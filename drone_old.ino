#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
int stablePower[4] = {40, 40, 40, 40}; // ADJUST W/ TEST
const float maxIncrease = 10/9; // should be adjusted
const float maxDecrease = 1/maxIncrease; //should not be adjusted
const int minPower = 20; // CHANGE W/ TEST
const int maxPower = 180;
int motorpins[4] = {1, 2, 3, 4}; // change
float const forwardvector[4] = {maxDecrease, maxDecrease, maxIncrease, maxIncrease};
float const backwardvector[4] = {maxIncrease, maxIncrease, maxDecrease, maxDecrease};
float const leftvector[4] = {maxDecrease, maxIncrease, maxDecrease, maxIncrease};
float const rightvector[4] = {maxIncrease, maxDecrease, maxIncrease, maxDecrease};
float const upvector[4] = {maxIncrease, maxIncrease, maxIncrease, maxIncrease};
float const downvector[4] = {maxDecrease, maxDecrease, maxDecrease, maxDecrease};
float const cwvector[4] = {maxDecrease, maxIncrease, maxIncrease, maxDecrease};
float const ccwvector[4] = {maxIncrease, maxDecrease, maxDecrease, maxIncrease};
const byte address[6] = "57385";
struct joysticks {
    float onex = 0;
    float oney = 0;
    float twox = 0;
    float twoy = 0;
};
float vectors[4];
float motorspeeds[4];
float starting_lat;
float starting_long;
unsigned long fix_age;
Servo motors[4];
RF24 data(7, 8); //CE, CSN pins (change as needed)
TinyGPS gps;
#define RXPIN 3
#define TXPIN 2
SoftwareSerial nss(RXPIN, TXPIN);
float motorValues[4];
joysticks rcv;

void setup() {
    Serial.begin(9600);// what does this do?
    data.begin();
    data.openReadingPipe(0, address);
    data.setPALevel(rf24_pa_dbm_e(0));
    data.startListening();
    for (int i = 0; i < 5; i++) {
        motors[i].attach(motorpins[i], 1000, 2000); // TODO: change
        // TODO: add takeoff method
        while (nss.available())
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
    }

}

void setupVector(float updown, float forwardbackward, float leftright, float turn) {
    for (int i =0;i < 5; i++) {
        vectors[i] = 1;
        if (updown > 0) {
            vectors[i] *= upvector[i];
        } else if (updown < 0) {
            vectors[i] *= downvector[i];
        }
        if (forwardbackward > 0) {
            vectors[i] *= forwardvector[i];
        } else if (forwardbackward < 0) {
            vectors[i] *= backwardvector[i];
        }
        if (leftright > 0) {
            vectors[i] *= rightvector[i];
        } else if (leftright < 0) {
            vectors[i] *= leftvector[i];
        }
        if (turn > 0) {
            vectors[i] *= cwvector[i];
        } else if (turn < 0) {
            vectors[i] *= ccwvector[i];
        }
    }
    float maximum = vectors[0];
    float minimum = vectors[0];
    for (int i=1; i < 5; i++) {
        if (maximum < vectors[i]) {
            maximum = vectors[i];
        }
        if (minimum > vectors[i]) {
            minimum = vectors[i];
        }
    }
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
    if (data.available()) {
        data.read(&rcv, sizeof(joysticks));
    }
    float x1 = rcv.onex;
    float x2 = rcv.twox;
    float y1 = rcv.oney;
    float y2 = rcv.twoy;
    setupVector(y1, y2, x2, x1);
    for (int i = 0; i < 5; i++) {
        motors[i].write(motorspeeds[i]);
    }
}
