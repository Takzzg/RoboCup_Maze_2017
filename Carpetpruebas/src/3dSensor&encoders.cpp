#include "mbed.h"
#include "Adafruit_BNO055.h"
#include "MotorDC.h"
#include "QEI.h"//---
MotorDC M_izq(PTD3, PTC9, PTB23);  //pwma, ain1, ain2---- M2
MotorDC M_der(PTA1, PTB9, PTC1);  //pwmb, bin1, bin2---- M1
DigitalOut stby(PTC8,1);

//DigitalOut l(LED_GREEN);

#define BNO055_SAMPLERATE_DELAY_MS (100)
#define vel_motores 0.7f
#define enc_der_chana PTB11
#define enc_der_chanb PTC11


#define enc_izq_chana PTB10
#define enc_izq_chanb PTB2 //antes ptb20
#define PULSES_PER_REVOLUTION 334

QEI leftQei(enc_izq_chana, enc_izq_chanb, NC, PULSES_PER_REVOLUTION);  //chanA, chanB, index, ppr
QEI rightQei(enc_der_chana, enc_der_chanb, NC, PULSES_PER_REVOLUTION); //chanB, chanA, index, ppr

int st;
I2C  i2c(PTE25,PTE24);
Adafruit_BNO055 bno = Adafruit_BNO055(55,0x28,&i2c);
Serial pc(USBTX,USBRX);

Ticker update_speed_motors;

float speed_mizq = 0;
float speed_mder = 0;

void updateSpeedMotors(){
  M_izq = speed_mizq;
  M_der = speed_mder;
}

imu::Quaternion quat;
imu::Vector<3> vector_euler;
imu::Vector<3> vector_acc;
imu::Vector<3> vector_euler_init;

void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  pc.printf("\n");
  pc.printf("System Status: %X\n",system_status);
  pc.printf("Self Test: %X\n",self_test_results);
  pc.printf("System Error: %X\n",system_error);
  wait_ms(500);
}
/*
saves the initial value of 3d sensor
*/
void init_value_3d_sensor(){
  //wait_ms(1000);
  quat = bno.getQuat();
  vector_euler_init = quat.toEuler();
  vector_euler_init.toDegrees();
  wait_ms(1000);
}

void turn_90(int direction){
 init_value_3d_sensor();
 float eje_x = 0;
 float eje_y = 0;
 float eje_z = 0;
 float grados = 86;

 speed_mizq = 0.7f * direction;
 speed_mder = -0.7f * direction;

 while(fabs(eje_x) < grados){
    quat = bno.getQuat();
    vector_euler = quat.toEuler();
    vector_euler.toDegrees();

    pc.printf("X inicial: %.1f \t",vector_euler_init.x());
    pc.printf("X calc: %.1f \t", eje_x);
    pc.printf("\t X actual: %.1f \n",vector_euler.x());

    eje_x = vector_euler.x() - vector_euler_init.x();
    if(eje_x > 180) eje_x -= 360;
    if(eje_x < -180) eje_x += 360;
  }
  pc.printf("Ya giré %.1f grados",grados);
  speed_mizq = 0;
  speed_mder = 0;
  wait_ms(500);
}
void turn_left90(){
  turn_90(-1);
}
void turn_right90(){
  turn_90(1);
}
/*
  direction = -1; Turn_left90
  direction = 1; turn_right90
*/
void check_Bno(){
  if(!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections
    pc.printf("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  wait(1.0f);
  pc.printf("Iniciando...\n");
  displaySensorStatus();
  Adafruit_BNO055::adafruit_bno055_rev_info_t info;
  bno.setExtCrystalUse(true);
  bno.getRevInfo(&info);
 }


/**
direction = 1 move forward
direction = -1 move backward
**/
void moveTile(int direction){

  leftQei.reset();
  rightQei.reset();

  int leftPulses  = 0;    //How far the left wheel has travelled.
  int rightPulses = 0;    //How far the right wheel has travelled.
  //int a = 1;
  int dist_izq = 1305; //Number of pulses to travel.
  int dist_der = 1343;

  speed_mizq = 0.7f * direction;
  speed_mder = 0.7f * direction;
  wait_us(1);

  while (abs(leftPulses) <= dist_izq || abs(rightPulses) <= dist_der){
    if(abs(leftPulses) <= dist_izq){
      leftPulses  = leftQei.getPulses();
      pc.printf("encoder_izq: %d \n", leftPulses);
    } else {
      speed_mizq = 0;
      wait_us(1);
    }
    if(abs(rightPulses) <= dist_der){
      rightPulses = rightQei.getPulses();
      pc.printf("encoder_der: %d \n", rightPulses);
    } else {
      speed_mder = 0;
      wait_us(1);
    }
  }
  //me aseguro de frenar
  speed_mizq = 0;
  speed_mder = 0;
  leftQei.reset();
  rightQei.reset();
  wait_ms(500);
}
void moveTileForward(){
  moveTile(1);
}

void moveTileBackward(){
  moveTile(-1);
}

int main() {
  pc.baud(115200);
  update_speed_motors.attach_us(&updateSpeedMotors,500.0f); //500us
  check_Bno();
  while(1){
    //examples about how to call our functions
    moveTileForward();
    //turn_left90();
    //turn_right90();
    //moveTileBackward();
  }
}
