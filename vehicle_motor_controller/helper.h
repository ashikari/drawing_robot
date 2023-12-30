#ifndef HELPER_H
#define HELPER_H
//Motor Structure
class motorClass
{
  private:
    int pwmPin;
    int dirPin;
    unsigned long currentTime = 0;
    unsigned long prevTime = 0;
    float dt = 0; 
    float Kpv = 0;
    float Kdv = 0;
    float Kiv = 0;
    signed long encodercountPrev = 0;
    float errorVel = 0;
    float errorVelPrev = 0;
    float integratedVelError = 0;
    float pCommand = 0;
    float dCommand = 0;
    float iCommand = 0;
    int currentCommand = 0;
    float MotorVel;
    float desiredMotorVel = 0;
    float avgMotorVel;
  public:
    void inputpwmPin(int a);
    void inputdirPin(int a);
    void inputKpv(float a);
    void inputKdv(float a);
    void inputKiv(float a);
    void setMotorVel(float a);
    signed long encodercount = 0;
    int openLoopController(void);
    void storeOldVals(void);
    float motor_velocity_calc(void);
    void calc_t(void);
    float proportional_control(void);
    float derivative_control(void);
    float integral_control(void);
    int closedLoopController(void);
};

//Constants
const int MAX_PWM = 255;
const int MIN_PWM = 0;
//Staff constants
const int   FREQ = 2000;
const float PERIOD = 0.0005;
const float PERIOD_MICROS = (PERIOD * 1000000.0);
const int   SERIAL_FREQ = 100;  //Should be at least 10 times greater than sin wave frequency
const float SERIAL_PERIOD = 0.01;
const float SERIAL_PERIOD_MICROS = 10000.0;

//Staff functions
void stopIfFault(void);
void DoSerialSend(void);

#endif //HELPER_H
