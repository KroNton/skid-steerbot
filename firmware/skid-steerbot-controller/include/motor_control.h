#include <Arduino.h>
#include "CytronMotorDriver.h"

// Pin definitions for motor control
const int motorFL1 = 2;       // Front-left motor positive
const int motorFL2 = 15;        // Front-left motor negative


const int motorBL1 = 14;       // Back-left motor positive
const int motorBL2 = 27;        // Back-left motor negative



const int motorBR1 = 4;       // Back-right motor positive
const int motorBR2 = 5;       // Back-right motor negative


const int motorFR1 = 19;       // Front-right motor positive
const int motorFR2 = 18;       // Front-right motor negative


CytronMD motor_frontleft(PWM_DIR, motorFL1,motorFL2);   //  PWM 1 = motorFL1, DIR 1 = motorFL2.
CytronMD motor_frontright(PWM_DIR,motorFR1,motorFR2);   //  PWM 1 = motorFR1, DIR 1 = motorFR2 .

CytronMD motor_backleft(PWM_DIR,motorBL1,motorBL2);   // PWM 1 = motorBL1, DIR 1 = motorBL2.
CytronMD motor_backright(PWM_DIR,motorBR1,motorBR2);   // PWM 1 = motorBR1, DIR 1 = motorBR2.


double w_fl;
double w_fr;
double w_rl;
double w_rr;

double Vx = 0.0;
double Vy = 0.0;
double Wz = 0.0;

// double Vx_ = 0;
// double Vy_ = 0;
// double Wz_ = 0;

double pwm_fl = 0;
double pwm_fr = 0;
double pwm_rl = 0;
double pwm_rr = 0;

void motorsSetup(){

  // Set motor control pins as outputs
  pinMode(motorFL1, OUTPUT);
  pinMode(motorFL2, OUTPUT);


  pinMode(motorFR1, OUTPUT);
  pinMode(motorFR2, OUTPUT);


  pinMode(motorBL1, OUTPUT);
  pinMode(motorBL2, OUTPUT);


  pinMode(motorBR1, OUTPUT);
  pinMode(motorBR2, OUTPUT);



}



// Function to move the robot forward
void straightAhead(double v_fl, double v_fr, double v_rl, double v_rr)
{
    // Set enable pins HIGH to enable the motors


    digitalWrite(motorFL1, HIGH);
    digitalWrite(motorFL2, LOW);

    digitalWrite(motorFR1, HIGH);
    digitalWrite(motorFR2, LOW);

    digitalWrite(motorBL1, HIGH);
    digitalWrite(motorBL2, LOW);

    digitalWrite(motorBR1, HIGH);
    digitalWrite(motorBR2, LOW);
}
void straightBack(double v_fl, double v_fr, double v_rl, double v_rr)
{
    // Set enable pins HIGH to enable the motors


    digitalWrite(motorFL1, LOW);
    digitalWrite(motorFL2, HIGH);

    digitalWrite(motorFR1, LOW);
    digitalWrite(motorFR2, HIGH);

    digitalWrite(motorBL1, LOW);
    digitalWrite(motorBL2, HIGH);

    digitalWrite(motorBR1, LOW);
    digitalWrite(motorBR2, HIGH);
}

void sideWay(double v_fl, double v_fr, double v_rl, double v_rr)
{
    // Set enable pins HIGH to enable the motors


    digitalWrite(motorFL1, HIGH);
    digitalWrite(motorFL2, LOW);

    digitalWrite(motorFR1, LOW);
    digitalWrite(motorFR2, HIGH);

    digitalWrite(motorBL1, LOW);
    digitalWrite(motorBL2, HIGH);

    digitalWrite(motorBR1, HIGH);
    digitalWrite(motorBR2, LOW);
}


void sideWayInv(double v_fl, double v_fr, double v_rl, double v_rr)
{
    // Set enable pins HIGH to enable the motors


    digitalWrite(motorFL1, LOW);
    digitalWrite(motorFL2, HIGH);

    digitalWrite(motorFR1, HIGH);
    digitalWrite(motorFR2, LOW);

    digitalWrite(motorBL1, HIGH);
    digitalWrite(motorBL2, LOW);

    digitalWrite(motorBR1, LOW);
    digitalWrite(motorBR2, HIGH);
}

void turnRound(double v_fl, double v_fr, double v_rl, double v_rr)
{
    // Set enable pins HIGH to enable the motors


    digitalWrite(motorFL1, HIGH);
    digitalWrite(motorFL2, LOW);

    digitalWrite(motorFR1, LOW);
    digitalWrite(motorFR2, HIGH);

    digitalWrite(motorBL1, HIGH);
    digitalWrite(motorBL2, LOW);

    digitalWrite(motorBR1, LOW);
    digitalWrite(motorBR2, HIGH);
}
void turnRoundInv(double v_fl, double v_fr, double v_rl, double v_rr)
{
    // Set enable pins HIGH to enable the motors

    digitalWrite(motorFL1, LOW);
    digitalWrite(motorFL2, HIGH);

    digitalWrite(motorFR1, HIGH);
    digitalWrite(motorFR2, LOW);

    digitalWrite(motorBL1, LOW);
    digitalWrite(motorBL2, HIGH);

    digitalWrite(motorBR1, HIGH);
    digitalWrite(motorBR2, LOW);
}

void OFF()
{
    // Set enable pins HIGH to enable the motors

}