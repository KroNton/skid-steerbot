#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <motor_control.h>
#include <mpu6050.h>

#define COMMAND_RATE 5 // hz

double radius = 0.020;

double ly = 0.18;
double lx = 0.27;

double lower_speed_limit = -10;
double upper_speed_limit = 10;

double lower_pwm_limit = -255;
double upper_pwm_limit = 255;

static unsigned long prev_control_time = 0;
static unsigned long now = 0;
static unsigned long lastTime = 0;

ros::NodeHandle nh;
ros::Publisher imu_pub("imu_raw", &floatArrayMsg);

void commandCallback(const geometry_msgs::Twist &cmd_msg)
{

  Vx = cmd_msg.linear.x;
  Vy = cmd_msg.linear.y;
  Wz = cmd_msg.angular.z;
}

void FourMecanumKinematic()
{
  // forward kinematics of a Four Mecanum Wheeled Mobile Robot
  w_fl = (1 / radius) * (Vx - Vy - (lx + ly) * Wz);
  w_fr = (1 / radius) * (Vx + Vy + (lx + ly) * Wz);
  w_rl = (1 / radius) * (Vx + Vy - (lx + ly) * Wz);
  w_rr = (1 / radius) * (Vx - Vy + (lx + ly) * Wz);

  // inverse kinematics of a Four Mecanum Wheeled Mobile Robot
  // Vx_ = (w_fl + w_fr + w_rl + w_rr) * (radius / 4);
  // Vy_ = (-w_fl + w_fr + w_rl - w_rr) * (radius / 4);
  // Wz_ = (-w_fl + w_fr - w_rl + w_rr) * (radius / (4 * (lx + ly)));

  pwm_fl = map(w_fl, lower_speed_limit, upper_speed_limit, lower_pwm_limit, upper_pwm_limit);
  pwm_fr = map(w_fr, lower_speed_limit, upper_speed_limit, lower_pwm_limit, upper_pwm_limit);
  pwm_rl = map(w_rl, lower_speed_limit, upper_speed_limit, lower_pwm_limit, upper_pwm_limit);
  pwm_rr = map(w_rr, lower_speed_limit, upper_speed_limit, lower_pwm_limit, upper_pwm_limit);

  // Serial.print("front left W: ");
  // Serial.println(w_fl);

  // Serial.print("front left pwm: ");
  // Serial.println(pwm_fl);
 motor_frontleft.setSpeed( pwm_fl);   // PWM 1A = Pin 10, PWM 1B = Pin 11.
 motor_frontright.setSpeed(pwm_fr);   // PWM 2A = Pin 8, PWM 2B = Pin 9.

 motor_backleft.setSpeed(pwm_rl);   // PWM 1A = Pin 10, PWM 1B = Pin 11.
 motor_backright.setSpeed(pwm_rr);   // PWM 2A = Pin 8, PWM 2B = Pin 9.

  // if (Vx > 0)
  // {
  //   straightAhead(pwm_fl, pwm_fr, pwm_rl, pwm_rr);
  // }

  // else if (Wz > 0)
  // {
  //   turnRound(pwm_fl, pwm_fr, pwm_rl, pwm_rr);
  // }
  // else if (Wz < 0)
  // {
  //   turnRoundInv(pwm_fl, pwm_fr, pwm_rl, pwm_rr);
  // }
  // else if (Vx < 0)
  // {
  //   straightBack(pwm_fl, pwm_fr, pwm_rl, pwm_rr);
  // }

  // else
  // {
  //   OFF();
  // }
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &commandCallback);

void setup()
{

  // Inizialize the ROS node on the Arduino
  nh.initNode();
  motorsSetup();
  // Serial.begin(9600);
  // Inform ROS that this node will subscribe to messages on a given topic
  nh.subscribe(sub);
  nh.advertise(imu_pub);
  setupIMU();
}

void loop()
{

  // now = millis();
  // int timeChange = (now - lastTime);
  // if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
  // {
    FourMecanumKinematic();
    publishIMU();

    // lastTime = now;
  // }

  imu_pub.publish(&floatArrayMsg);
  nh.spinOnce();

  delay(1);

  // delay(2000);  // Wait for 2 seconds
}
