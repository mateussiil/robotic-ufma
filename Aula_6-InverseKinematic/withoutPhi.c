#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define L1 8 // Length of the first link (in cm)
#define L2 8 // Length of the second link
#define L3 8 // Length of the third link

double x = 0.0, y = 0.0, phi = 0.0; // Coordinates of the end effector

double x1 = 0.0, y_1 = 0.0; // x' e y'

double theta1 = 0.0, theta2 = 0.0; // Angles of the joints
double cosTheta2 = 0.0, sinTheta2 = 0.0;
double k1 = 0.0, k2 = 0.0;

// Function prototypes
void inverseKinematics(double x, double y);
void forwardKinematics(double theta1, double theta2);
double radToDeg(double rad);
double degToRad(double degrees);

int main()
{
    // Get the coordinates of the end effector
    printf("Enter the coordinates of the end effector:\n");
    // printf("x = ");
    // scanf("%lf", &x);
    // printf("y = ");
    // scanf("%lf", &y);

    x = 5;
    y = 5;
    // Calculate the angles of the joints
    inverseKinematics(x, y);

    // Print the angles of the joints
    printf("\nAngles of the joints:\n");
    printf("theta1 = %.2lf\ntheta2 = %.2lf\n", theta1, theta2);

    forwardKinematics(theta1, theta2);

    // Print the angles of the joints
    printf("\nAngles of the joints:\n");
    printf("x = %.2lf\ny = %.2lf\n", x, y);


    return 0;
}

void inverseKinematics(double x, double y)
{
    // Calculate the angles of the joints

    // Getting phi in radians
    // Getting x' and y'
    double x1 = x;
    double y1 = y;

    // Getting theta2
    double cosTheta2 = (pow(x1, 2) + pow(y1, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2);
    double sinTheta2 = sqrt(1 - pow(cosTheta2, 2));
    theta2 = atan2(sinTheta2, cosTheta2);

    // Getting theta1
    double k1 = L1 + L2 * cosTheta2;
    double k2 = L2 * sinTheta2;
    theta1 = atan2(k1 * y1 - k2 * x1, k1 * x1 + k2 * y1);

    theta1 = radToDeg(theta1);
    theta2 = radToDeg(theta2);
}

void forwardKinematics(double theta1, double theta2)
{
    // Calculate the coordinates of the endpoint

    // Convert the angles from degrees to radians
    theta1 = degToRad(theta1);
    theta2 = degToRad(theta2);

    // Calculate x and y
    x = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    y = L1 * sin(theta1) + L2 * sin(theta1 + theta2);
}


double degToRad(double degrees)
{
    return degrees * M_PI / 180;
}

double radToDeg(double rad)
{
    // Convert the angles from radians to degrees
    return rad * 180 / M_PI;
}