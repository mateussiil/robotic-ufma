#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define L1 8 // Length of the first link (in cm)
#define L2 8 // Length of the second link
#define L3 8 // Length of the third link

double x = 0.0, y = 0.0, phi = 0.0; // Coordinates of the end effector

double x1 = 0.0, y_1 = 0.0; // x' e y'

double theta1 = 0.0, theta2 = 0.0, theta3 = 0.0; // Angles of the joints
double cosTheta2 = 0.0, sinTheta2 = 0.0;
double k1 = 0.0, k2 = 0.0;

// Function prototypes
void inverseKinematics(double x, double y, double phi);
double radToDeg(double rad);

int main()
{
    // Get the coordinates of the end effector
    printf("Enter the coordinates of the end effector:\n");
    printf("x = ");
    scanf("%lf", &x);
    printf("y = ");
    scanf("%lf", &y);
    printf("phi = ");
    scanf("%lf", &phi);

    // Calculate the angles of the joints
    inverseKinematics(x, y, phi);

    // Print the angles of the joints
    printf("\nAngles of the joints:\n");
    printf("theta1 = %.2lf\ntheta2 = %.2lf\ntheta3 = %.2lf\n", theta1, theta2, theta3);

    return 0;
}

void inverseKinematics(double x, double y, double phi)
{
    // Calculate the angles of the joints

    // Getting phi
    phi = phi * M_PI / 180;

    // Getting x' and y'
    x1 = x - L3 * cos(phi);
    y_1 = y - L3 * sin(phi);

    // Getting theta2
    cosTheta2 = (pow(x1, 2) + pow(y_1, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2);
    sinTheta2 = sqrt(1 - pow(cosTheta2, 2));
    theta2 = atan2(sinTheta2, cosTheta2);

    // Getting theta1
    k1 = L1 + L2 * cosTheta2;
    k2 = L2 * sinTheta2;
    theta1 = atan2(k1 * y_1 - k2 * x1, k1 * x1 + k2 * y_1);

    // Getting theta3
    theta3 = phi - theta1 - theta2;

    // Convert the angles from radians to degrees
    theta1 = radToDeg(theta1);
    theta2 = radToDeg(theta2);
    theta3 = radToDeg(theta3);
}

double radToDeg(double rad)
{
    // Convert the angles from radians to degrees
    return rad * 180 / M_PI;
}