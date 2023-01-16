// Forward kinematics code
// Vyankatesh Ashtekar, IIT Kanpur, July 2022

// Burn the code on Arduino Uno board
// PWM pin no. 1 and 3 are used

// Open serial monitor
// theta1,theta2 <Enter>



#include <Servo.h>
#include<math.h>

Servo myservo1, myservo2;

void setup() {
  Serial.begin(9600);
  // PWM pin no. 3 for motor 1
  myservo1.attach(3);
  // PWM pin no. 5 for motor 2
  myservo2.attach(5);

}

void loop() {

  if (Serial.available() > 0)
  {
    String theta1degree = Serial.readStringUntil(',');
    Serial.read();
    String theta2degree = Serial.readStringUntil('\0');

    int theta1d = theta1degree.toInt();
    int theta2d = theta2degree.toInt();
    // 180 deg - argument because the motors are mounted upside down
    // so CW becomes CCW and vice versa
    // 90 deg offset becuase we want to have both elbow up an down solution demo in inverse kinematics
    int theta1s = 180 - (theta1d + 90);
    int theta2s = 180 - (theta2d + 90);

    if (abs(theta1d) > 90 || abs(theta2d) > 90)
    {
      Serial.print("theta1 is ");
      Serial.println(theta1d);
      Serial.print("theta2 is ");
      Serial.println(theta2d);
      Serial.println("Servo angles should be between [-90deg, +90deg]");
      Serial.println("Either of the thetas is out of range");      
    }
    else {
      
      Serial.println("Sending angles to servo motors...");
      Serial.print("theta1 is ");
      Serial.println(theta1d);
      Serial.print("theta2 is ");
      Serial.println(theta2d);

      myservo1.write(theta1s);
      myservo2.write(theta2s);

    }
  }

}
