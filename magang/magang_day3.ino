

#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>


#define pirPin 14

Adafruit_MPU6050 mpu;
Servo myservo1, myservo2, myservo3, myservo4, myservo5;

#define SERVO_PIN_1 17
#define SERVO_PIN_2 18
#define SERVO_PIN_3 2
#define SERVO_PIN_4 13
#define SERVO_PIN_5 26


void setup() {


  pinMode(pirPin, INPUT);
  Serial.begin(115200);
  myservo1.attach(SERVO_PIN_1);  
  myservo2.attach(SERVO_PIN_2);  
  myservo3.attach(SERVO_PIN_3);  
  myservo4.attach(SERVO_PIN_4);  
  myservo5.attach(SERVO_PIN_5);  
  
  //begin mpu6050
  if(!mpu.begin()){
    Serial.println("failed to fin 6050");
    while(1);
  } else{
    Serial.println("sensor found");
  }
  
}

void loop() {
  long duration, distance;
  // read data from sensor to determine roll pitch yaw
  //declare variables to hold sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  // calculate servo positions based ons ensor data since its still on acceleration data type
  float roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  float yaw = atan2(a.acceleration.z, a.acceleration.x) * 180.0 / PI;

  //define minimum and maximum angles for servos
  int rollAngles = map(roll, -90, 90, 0, 180);
  int pitchAngles = map(pitch, -90, 90, 0, 180);
  int yawAngles = map(yaw, -90, 90, 0, 180);


  //servo 1 dan 2 (roll) experience rotaion to NEGATIVE when system ROLLS (x) to POSITIVE
  int pirState = digitalRead(pirPin);
  
  // read PIR sensor state to dteermine if there is motion
  if(pirState == HIGH){

    myservo1.write(45);
    myservo2.write(45);
    myservo3.write(45);
    myservo4.write(45);
    myservo5.write(45);

    // delay 1s and move back to neutral position
    delay(1000);
    myservo1.write(90);
    myservo2.write(90);
    myservo3.write(90);
    myservo4.write(90);
    myservo5.write(90);

  }
  else{
    // if roll changes, move servo 1 and 2 negatively
    myservo1.write(180 - rollAngles); 
    myservo2.write(180 - rollAngles);

    // if pitch changes, move servo 3 and 4 positively
    myservo3.write(pitchAngles );
    myservo4.write(pitchAngles);

    // if yaw changes to positive direction, move servo 5 according to the direction
    if(a.acceleration.z > 0) {//if yaw is poistive
      myservo5.write(yawAngles );
       // after 1s delay, return servo 5 to neutral position without being affected by sensor
       delay(1000);
       myservo5.write(180-yawAngles);
    }
    else{
      // if yaw changes to negative direction, move servo 5 to negative direction
       myservo5.write(180 - yawAngles);
       delay(1000);
       myservo5.write(90);

    }

  // Wait a bit before scanning again

}

delay(500);

}