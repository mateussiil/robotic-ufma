#include <stdlib.h>
#include <math.h>
#include <Servo.h>

#define L1 7 // Length of the first link (in cm)
#define L2 8 // Length of the second link

#define pinJoint1 9
#define pinJoint2 10

Servo joint1;
Servo joint2;

// Variables
double x = 0.0, y = 0.0; // Coordinates of the end effector
double theta1 = 0.0, theta2 = 0.0; // Angles of the joints
double beta = 0.0, psi = 0.0;

// Function prototypes
void inverseKinematics(double x, double y);
double radToDeg(double rad);
void move(double angle1, double angle2);
double readSerialInput();

void setup() {
  Serial.begin(115200);

  joint1.attach(pinJoint1);
  joint2.attach(pinJoint2);

  joint1.write(90);
  joint2.write(90);
}

void loop() {
  Serial.flush();

  // Get the coordinates of the end effector
  Serial.println("+---------- Enter the coordinates of the end effector ----------+");
  Serial.print("x = ");
  while (Serial.available() == 0) {}
  x = readSerialInput();
  Serial.println(x);
  
  Serial.print("y = ");
  while (Serial.available() == 0) {}
  y = readSerialInput();
  Serial.println(y);

  Serial.print("\n+---------- Calculating angles to the coordinates: (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.println(") ----------+");

  // Calculate the angles of the joints
  inverseKinematics(x, y);

  // Print the angles of the joints
  Serial.println("\n> Angles of the joints:");
  Serial.print("theta1 = ");
  Serial.println(theta1);
  Serial.print("theta2 = ");
  Serial.println(theta2);
  Serial.println();

  // Move joints
  Serial.println("> Moving...");
  move(theta1, theta2);
}

void inverseKinematics(double x, double y) {
  // Calculate the angles of the joints

  // Getting theta2
  theta2 = acos( ( pow(x, 2) + pow(y, 2) - pow(L1, 2) - pow(L2, 2) ) / ( 2 * L1 * L2  ) );

  // Getting theta1
  beta = atan2(y, x);
  psi = acos( ( pow(x, 2) + pow(y, 2) + pow(L1, 2) - pow(L2, 2) ) / ( 2 * L1 * sqrt( pow(x, 2) + pow(y, 2) ) ) );
  if( theta2 < 0 )
    theta1 = beta + psi;
  else if ( theta2 > 0)
    theta1 = beta - psi;

  // Convert the angles from radians to degrees
  theta1 = radToDeg(theta1);
  theta2 = radToDeg(theta2);
  Serial.println(theta2);
}

double radToDeg(double rad) {
  // Convert the angles from radians to degrees
  return rad * 180 / M_PI;
}

void move(double angle1, double angle2) {
  delay(1000);
  joint1.write(angle1);
  delay(1000);
  joint2.write(angle2);
}

double readSerialInput() {
  String s;
  double num;
  if(Serial.available() > 0) {
    while (Serial.available() > 0) {
      char c = Serial.read();
      if(c != '\n') {
        s += c;
      }
      delay(10);
    }
  }
  num = s.toDouble();
  return num;
}