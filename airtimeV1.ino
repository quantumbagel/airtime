#include <SoftwareSerial.h> //gps
#include <TinyGPS.h> //gps
#include <Servo.h> //motor control
#include <SPI.h> // dunno
#include <nRF24L01.h> //transeiver
#include <RF24.h> // transceiver
#include <Adafruit_BMP085.h> // bmp180
#include <MPU6050_6Axis_MotionApps20.h> // mpu6050 https://github.com/ElectronicCats/mpu6050
#include <I2Cdev.h> // mpu6050
#include <Wire.h> // mpu6050 / bmp180
const float maxIncrease = 1.1;
const float maxDecrease = 0.9;
struct vectors {
    const float forward[4] = {maxDecrease, maxDecrease, maxIncrease, maxIncrease};
    const float backward[4] = {maxIncrease, maxIncrease, maxDecrease, maxDecrease};
    const float left[4] = {maxDecrease, maxIncrease, maxDecrease, maxIncrease};
    const float right[4] = {maxIncrease, maxDecrease, maxIncrease, maxDecrease};
    const float up[4] = {maxIncrease, maxIncrease, maxIncrease, maxIncrease};
    const float down[4] = {maxDecrease, maxDecrease, maxDecrease, maxDecrease};
    const float cw[4] = {maxDecrease, maxIncrease, maxIncrease, maxDecrease};
    const float ccw[4] = {maxIncrease, maxDecrease, maxDecrease, maxIncrease};
};
struct wait {
    long currentMillis;
    // gps
    long lastGPSUpdate;
    int GPSWaitMillis = 1000;
    // gyro
    long lastGyroUpdate;
    int gyroWaitMillis = 500;
    // bmp180 doesn't need this
    long lastRF24Update;
    int RF24WaitMillis = 100;
};
struct gpsData {
    float takeoffLatitude;
    float takeoffLongitude;
    float currentLatitude;
    float currentLongitude;
    unsigned long fixAge;
    int rawNSSData;
};
struct gyroData {

};
struct bmpData {
    float groundPressure = 0.0;
    float lastPressure = 0.0;
    float lastAltitude = 0.0;
};
struct joysticksGeneric {
    int x1;
    int x2;
    int y1;
    int y2;
};
struct joystickPositions {
    float x1;
    float x2;
    float y1;
    float y2;
    float[4] vectors;
    float minimum;
    float maximum;
};
float altitude;
bool useMotors = false;
joysticksGeneric joysticks;
// sensor objects
SoftwareSerial nss(2, 3); // gps serial (rxpin, txpin)
TinyGPS gps; // gps object
RF24 radio(7, 8); // CE, CSN (transceiver)
Adafruit_BMP085 bmp;

void logLine(String line) {
    // TODO: implement
}
logLine("AIRTIME - A simple flight controller from quantumbagel");
// update functions
void updateGPS() {
    if nss.available() {
        gpsData.rawNSSData = nss.read();
        if (gps.encode(gpsData.rawNSSData)) {
            gps.f_get_position(&gpsData.currentLatitude, &gpsData.currentLongitude, &gpsData.fixAge);
            logLine("DEBUG: successfully got GPS data.");
            logLine("DEBUG: Lat: ".concat(gpsData.currentLatitude).concat(" / Long: ".concat(gpsData.currentLongitude)));
        } else {
            logLine("WARNING: Failed to encode GPS data!");
        }
    } else {
        logLine("DEBUG: No new GPS data.");
    }
}

float getAccuratePressure(int n) {
    for (int i=0; i < 3; i++) {
        bmp.readPressure();
    }
    float accuratePressure = 0.0;
    for (int i=0; i < n; i++) {
        accuratePressure += bmp.readPressure();
    }
    accuratePressure /= 20.0;
    return accuratePressure;
}

void updateAltitude() { // TODO: work on this lol
    bmpData.lastPressure = getAccuratePressure(3);
    bmpData.lastAltitude = bmp.readAltitude(bmpdata.groundPressure);
    logLine("DEBUG: altitude updated to ".concat(bmpData.lastAltitude));
}

void updateGyro() {
    // TODO: implement
}

void floatMapJoystick(int rawValue, float newValue) {
    newValue = ((rawValue / 1023.0) * 2.0) - 1.0;
}

void vectorConvert(float[4] vectorList, float scale) {
    float[4] moreVector = {};
    moreVector[0] = scale * vectorList[0];
    moreVector[1] = scale * vectorList[1];
    moreVector[2] = scale * vectorList[2];
    moreVector[3] = scale * vectorList[3];

}

void updateMotors() {
    // get rel. vector speeds
    logLine('updating motors...')
    float upDown = joystickPositions.y1;
    float leftRight = joystickPositions.x2;
    float forwardBackward = joystickPositions.y2;
    float turn = joystickPositions.x1;
    for (int i = 0; i < 5; i++) {
        joystickPositions.vectors[i] = 1;
        if (upDown > 0) {
            if (vectors.up[i] >= 1) {
                joystickPositions.vectors[i] *= 1+((vectors.up[i]-1) * upDown);
            } else {
                joystickPositions.vectors[i] *= 1-((1-vectors.up[i]) * upDown);
            }
        } else if (upDown < 0) {
            if (vectors.down[i] >= 1) {
                joystickPositions.vectors[i] *= 1+((vectors.down[i]-1) * -upDown);
            } else {
                joystickPositions.vectors[i] *= 1-((1-vectors.down[i]) * -upDown);
            }
        }
        if (forwardBackward > 0) {
            if (vectors.forward[i] >= 1) {
                joystickPositions.vectors[i] *= 1+((vectors.forward[i]-1) * forwardBackward);
            } else {
                joystickPositions.vectors[i] *= 1-((1-vectors.forward[i]) * forwardBackward);
            }
        } else if (forwardBackward < 0) {
            if (vectors.backward[i] >= 1) {
                joystickPositions.vectors[i] *= 1+((vectors.backward[i]-1) * -forwardBackward);
            } else {
                joystickPositions.vectors[i] *= 1-((1-vectors.backward[i]) * -forwardBackward);
            }
        }
        if (leftRight > 0) {
            if (vectors.left[i] >= 1) {
                joystickPositions.vectors[i] *= 1+((vectors.left[i]-1) * leftRight);
            } else {
                joystickPositions.vectors[i] *= 1-((1-vectors.left[i]) * leftRight);
            }
        } else if (leftRight < 0) {
            if (vectors.right[i] >= 1) {
                joystickPositions.vectors[i] *= 1+((vectors.right[i]-1) * -leftRight);
            } else {
                joystickPositions.vectors[i] *= 1-((1-vectors.right[i]) * -leftRight);
            }
        }
        if (turn > 0) {
            if (vectors.cw[i] >= 1) {
                joystickPositions.vectors[i] *= 1+((vectors.cw[i]-1) * turn);
            } else {
                joystickPositions.vectors[i] *= 1-((1-vectors.cw[i]) * turn);
            }
        } else if (turn < 0) {
            if (vectors.ccw[i] >= 1) {
                joystickPositions.vectors[i] *= 1+((vectors.ccw[i]-1) * -turn);
            } else {
                joystickPositions.vectors[i] *= 1-((1-vectors.ccw[i]) * -turn);
            }
        }
    }
    logLine("Raw vectors obtained.")

    // get max / min of the vectors
    joystickPositions.maximum = vectors[0];
    joystickPositions.minimum = vectors[0];
    for (int i = 1; i < 5; i++) {
        if (maximum < joystickPositions.vectors[i]) {
            maximum = joystickPositions.vectors[i];
        }
        if (minimum > joystickPositions.vectors[i]) {
            minimum = joystickPositions.vectors[i];
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
    // TODO: implement this wind correction algo.
    /* algorithm
     * super simple
     * if there is a direction in which I should NOT be moving
     * (up, down, turn left/right, left, right, forward, backward)
     * then add more vectors to counteract this (don't know how strong tho)
     */
    if (useMotors) {

    } else {
        logLine("motors not triggered.")
    }


}

void updateRF24() {
    if (radio.available()) {
        uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
        radio.read(&joysticks, bytes);
        floatMapJoystick(joysticks.x1, &joystickPositions.x1); // TODO: double-check that this actually works
        floatMapJoystick(joysticks.x2, &joystickPositions.x2);
        floatMapJoystick(joysticks.y1, &joystickPositions.y1);
        floatMapJoystick(joysticks.y2, &joystickPositions.y2);
        logLine("DEBUG: succesfully updated joysticks!");
        updateMotors();
    } else {
        logLine("DEBUG: No new joystick updates.");
    }
}

// setup
void setup() {
    if (!(radio.begin())) {
        logLine("FATAL: nRF24L01+ failed to initialize!!!");
        while(1) {}
    }
    if (!(bmp.begin())) {
        logLine("FATAL: BMP180 failed to initialize!!!");
        while(1) {}
    }
    bmpData.groundPressure = getAccuratePressure(20);
    radio.setPALevel(RF24_PA_LOW);
    radio.openWritingPipe("12345"); // address
    radio.stopListening();

}


// state machine loop :(
void loop() {
    wait.currentMillis = millis();
    if (wait.lastRF24Update - wait.currentMillis) > wait.RF24WaitMillis {
        logLine("DEBUG: Now checking for joystick updates!")
        updateRF24();
        wait.lastRF24Update = wait.currentMillis;
    };
    wait.currentMillis = millis();
    if (wait.lastGPSUpdate - wait.currentMillis) > wait.GPSWaitMillis {
        logLine("DEBUG: Now updating GPS!")
        updateGPS();
        wait.lastGPSUpdate = wait.currentMillis;
    };
    wait.currentMillis = millis();
    if (wait.lastGyroUpdate - wait.currentMillis) > wait.gyroWaitMillis {
        logLine("DEBUG: Now updating MPU6050!")
        updateGyro();
        wait.lastGyroUpdate = wait.currentMillis;
    };
}