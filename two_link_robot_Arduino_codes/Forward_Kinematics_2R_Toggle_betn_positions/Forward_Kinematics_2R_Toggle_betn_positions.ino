// Forward kinematics toggle between positions A and B
// Vyankatesh Ashtekar, IIT Kanpur, July 2022

// Burn the code on Arduino Uno board
// PWM pin no. 1 and 3 are used

// Open serial monitor
// Type this in serial monitor: theta1A,theta1B,theta2A,theta2B <Press Enter>
// This means robot will toggle between configurations (theta1A,theta2A) and (theta1B,theta2B)

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
    String theta1Adegree = Serial.readStringUntil(',');
    String theta1Bdegree = Serial.readStringUntil(',');
    String theta2Adegree = Serial.readStringUntil(',');
    String theta2Bdegree = Serial.readStringUntil('\0');
    Serial.read();
    int theta1Ad = theta1Adegree.toInt();
    int theta1Bd = theta1Bdegree.toInt();
    int theta2Ad = theta2Adegree.toInt();
    int theta2Bd = theta2Bdegree.toInt();
    Serial.print("Position A: ");
    Serial.print("theta1A is ");
    Serial.print(theta1Ad);
    Serial.print(" theta1B is ");
    Serial.print(theta1Bd);
    Serial.println();

    Serial.print("Position B: ");
    Serial.print("theta2A is ");
    Serial.print(theta2Ad);
    Serial.print(" theta2B is ");
    Serial.print(theta2Bd);
    Serial.println();

    //    Serial.println("Enter angles for Position B");
    //
    //    String theta1Bdegree = Serial.readStringUntil(',');
    //    Serial.read();
    //    String theta2Bdegree = Serial.readStringUntil('\0');
    //    int theta1Bd = theta1Bdegree.toInt();
    //    int theta2Bd = theta2Bdegree.toInt();
    //    Serial.print("theta1B is ");
    //    Serial.println(theta1Bd);
    //    Serial.print("theta2B is ");
    //    Serial.println(theta2Bd);


    // 180 deg - argument because the motors are mounted upside down
    // so CW becomes CCW and vice versa
    // 90 deg offset becuase we want to have both elbow up an down solution demo in inverse kinematics
    int theta1As = 180 - (theta1Ad + 90);
    int theta2As = 180 - (theta2Ad + 90);
    int theta1Bs = 180 - (theta1Bd + 90);
    int theta2Bs = 180 - (theta2Bd + 90);
    if (abs(theta1Ad) > 90 || abs(theta2Ad) > 90 || abs(theta1Bd) > 90 || abs(theta2Bd) > 90)
    {
      Serial.println("Servo angles should be between [-90deg, +90deg]");
      Serial.println("Either of the thetas is out of range");
    }
    else {
      //      String stopper = Serial.readStringUntil('\0');
      //      int stopp = stopper.toInt();
      int n = 3;
      int idx = 0;
      Serial.println("Sending angles to servo motors...");
      while (idx < n) {
        idx++;
        myservo1.write(theta1As);
        myservo2.write(theta2As);
        delay(1000);
        myservo1.write(theta1Bs);
        myservo2.write(theta2Bs);
        delay(1000);

      }
      myservo1.write(90);
      myservo2.write(90);
    }
  }

}
