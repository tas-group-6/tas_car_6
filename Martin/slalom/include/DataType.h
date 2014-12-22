#pragma once

#define RAD_TO_DEG 180.0/M_PI // 360 / 2Pi
#define DEG_TO_RAD M_PI/180.0 // 360 / 2Pi


#define DEG_TO_SERVO_SIGNAL_STEER(p) { (p) * 500 / 30 + 1500 }


#define SAFE_DELETE( p ) { if( (p) ) { delete (p); (p) = 0; } }


/// Simple sturcture to throw custom exceptions with some message
struct ErrorType
{
  std::string msg;

  ErrorType( std::string s) : msg(s) {}

  void what() { printf("%s\n", msg.c_str()); }
};


/// This structure represents a full instruction for the ardiuno board
struct  ServoInstructionType
{
    /// Time when this instruction was recorded. In milliseconds. 64-bits are rquired for the std::chrono functions
    unsigned long long int t;
    int velocity;   ///< pwm value for velocity
    int steer;      ///< pwm value for steering angle
};
