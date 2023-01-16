// Inverse kinematics code
// Vyankatesh Ashtekar, IIT Kanpur, July 2022

// Burn the code on Arduino Uno board
// PWM pin no. 1 and 3 are used

// Open serial monitor
// enter x and y in millimeters
// 1 means elbow up solution
// -1 means elbow down solution
// x,y,"1" or "-1" <Enter>



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

  float l1 = 0.200;
  float l2 = 0.200;

  if (Serial.available() > 0)
  {
    String xstr = Serial.readStringUntil(',');
    Serial.read();
    String ystr = Serial.readStringUntil(',');
    Serial.read();
    String soltype = Serial.readStringUntil('\0');

    float x = float(xstr.toInt()) / 1000;
    float y = float(ystr.toInt()) / 1000;
    int sol = soltype.toInt();

    Serial.print("X is ");
    Serial.println(x);
    Serial.print("Y is ");
    Serial.println(y);

    //***********Solution process********************

    float d = sqrt(x * x + y * y);
    //    Serial.print("d is ");
    //    Serial.println(d);
    // Inverse kinematics computations-->
    float psi = atan2(y, x);
    //    Serial.print("ArcTan2(y,x) is ");
    //    Serial.println(psi);

    float costh2 = (d * d - l1 * l1 - l2 * l2) / (2 * l1 * l2) ;

    if (abs(costh2) > 1)
    {
      Serial.println("The point is outside workspace (assuming no limits on joint angles)");
    }
    else
    {
      float Pi = 3.14159;
      float theta2 = acos(costh2);
      Serial.print("theta2 solutions (in degrees) are +- ");
      Serial.println(180 * theta2 / Pi);

      if (sol == 1)
      {
        Serial.println("Presenting elbow up solution");
        theta2 = -theta2;
      }
      else if (sol == -1)
      {
        Serial.println("Presenting elbow down solution");
      }

      float sinth2 = sin(theta2);
      float a11 = l1 + l2 * costh2;
      float a12 = -1.0 * l2 * sinth2;
      float a21 = -1.0 * a12;
      float a22 = a11;
      float det = a22 * a11 - a12 * a21;
      float detc1 = -(y * a12 - x * a22);
      float dets1 = (y * a11 - a21 * x);
      float c1 = detc1 / det;
      float s1 = dets1 / det;

      float theta1 = atan2(s1, c1);

      int theta1s = int(180 * theta1 / Pi);
      int theta2s = int(180 * theta2 / Pi);
      Serial.print("theta1 (in degrees) is ");
      Serial.println(theta1s);
      Serial.print("theta2 (in degrees) is  ");
      Serial.println(theta2s);

      if (abs(theta1s) > 90 || abs(theta2s) > 90)
      {
        Serial.println("Point inside geometrical workspace, but outside joint angle limits");
      }
      else
      {
        // mapping to actual servos-- offset and direction flip
        theta1s = 180 - (theta1s + 90);
        theta2s = 180 - (theta2s + 90);
        delay(2000);
        myservo1.write(theta1s);
        myservo2.write(theta2s);
      }
    }
  }
}
