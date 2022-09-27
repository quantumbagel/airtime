#include <SoftwareSerial.h> //gps
#include <TinyGPS.h> //gps
#include <Servo.h> //motor control
#include <SPI.h> // dunno
#include <nRF24L01.h> //transeiver
#include <RF24.h> // transceiver
#include <BMP180.h> // bmp180 lib https://github.com/enjoyneering/BMP180
#include "<MPU6050_6Axis_MotionApps20.h>" // mpu6050 https://github.com/ElectronicCats/mpu6050
#include "<I2Cdev.h>" // mpu6050
#include "<Wire.h>" // mpu6050 / bmp180
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
};
float altitude;
joysticksGeneric joysticks;
// sensor objects
SoftwareSerial nss(2, 3); // gps serial (rxpin, txpin)
TinyGPS gps; // gps object
RF24 radio(7, 8); // CE, CSN (transceiver)
BMP180 bmp(BMP180_ULTRAHIGHRES);
void logLine(String line) {
    // TODO: implement
}
logLine("AIRTIME - A simple flight controller from moc-buhtig");
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
void updateAltitude() { // TODO: work on this lol
    // https://www.mide.com/air-pressure-at-altitude-calculator
    float temperatureRatio = -35076.92308;
    float exponent = 0.1902632365;
    float pressure = bmp.getPressure();
    altitude = temperatureRatio * (pow(pressure/101325, exponent)-1)
}
void updateGyro() {
    // TODO: implement
}
void floatMapJoystick(int rawValue, float newValue) {
    newValue = ((rawValue / 1023.0) * 2.0) - 1.0;
}
void updateRF24() {
    if (radio.available()) {
        uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
        radio.read(&joysticks, bytes);
        floatMapJoystick(joysticks.x1, &joystickPositions.x1); // TODO: double-check that this actually works
        floatMapJoystick(joysticks.x2, &joystickPositions.x2);
        floatMapJoystick(joysticks.y1, &joystickPositions.y1);
        floatMapJoystick(joysticks.y2, &joystickPositions.y2);
        logLine("DEBUG: succesfully updated joysticks!")
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