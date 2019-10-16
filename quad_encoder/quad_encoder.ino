#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];

volatile long left_count = 0; // track # of left wheel movements, CCW is positive
volatile long right_count = 0; // track # of right wheel movements


void setup() {
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){2, 5}, SensorCount);
  
  Serial.begin(9600);
  
  // setup interrupts
  attachInterrupt(digitalPinToInterrupt(2),left_wheel_isr,CHANGE);
  attachInterrupt(digitalPinToInterrupt(3),right_wheel_isr,CHANGE);
}

// takes reading from qtr sensor, returns 1 if reading black, 0 if white
uint8_t black_or_white(int sensorValue) {
  if (sensorValue < 2300) {
    return 0b0;
  } else {
    return 0b1;
  }
}

// right wheel uses pins 3 and 4
void right_wheel_isr() {
    static int8_t lookup_table[] = {0,0,0,-1,0,0,1,0,0,1,0,0,-1,0,0,0};
    static uint8_t enc_val = 0;
    
    enc_val = enc_val << 2;

    uint8_t pin4 = black_or_white(sensorValues[2]);
    uint8_t pin3 = black_or_white(sensorValues[1]);
    enc_val = enc_val | ((pin4 << 1) | pin3); // puts current values fo pins 5 and 2 into enc_val
    right_count = right_count + lookup_table[enc_val & 0b1111];
}

// left wheel uses pins 2 and 5
void left_wheel_isr() {
    static int8_t lookup_table[] = {0,0,0,-1,0,0,1,0,0,1,0,0,-1,0,0,0};
    static uint8_t enc_val = 0;
    
    enc_val = enc_val << 2;

    uint8_t pin5 = black_or_white(sensorValues[3]);
    uint8_t pin2 = black_or_white(sensorValues[0]);
    enc_val = enc_val | ((pin5 << 1) | pin2); // puts current values fo pins 5 and 2 into enc_val
    left_count = left_count + lookup_table[enc_val & 0b1111];
}

void loop() {
  qtr.read(sensorValues);

  Serial.print(left_count);
  Serial.println();


  delay(100);
}
