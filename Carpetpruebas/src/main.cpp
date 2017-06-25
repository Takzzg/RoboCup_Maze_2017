#include "mbed.h"
#include "hcsr04.h"
#include "MotorDC.h"
#include "QEI.h"//---
MotorDC M_izq(PTD3, PTC9, PTB23);  //pwma, ain1, ain2---- M2
MotorDC M_der(PTA1, PTB9, PTC1);  //pwmb, bin1, bin2---- M1
DigitalOut stby(PTC8,1);

#define enc_der_chana PTB11
#define enc_der_chanb PTC11


#define enc_izq_chana PTB10
#define enc_izq_chanb PTB2 //antes ptb20
#define PULSES_PER_REVOLUTION 334

QEI leftQei(enc_izq_chana, enc_izq_chanb, NC, PULSES_PER_REVOLUTION);  //chanA, chanB, index, ppr
QEI rightQei(enc_der_chana, enc_der_chanb, NC, PULSES_PER_REVOLUTION); //chanB, chanA, index, ppr

Ticker update_speed_motors;

float speed_mizq = 0;
float speed_mder = 0;

void updateSpeedMotors(){
  M_izq = speed_mizq;
  M_der = speed_mder;
}

#define TRIGGER PTB3
PinName Echo [8] ={
  PTC5,PTC7,PTC0,PTC3,PTC2,PTA2,PTB19,PTB18
};
/*
0 = PTC5
1 = PTC7
2 = PTC0
3 = PTC3
4 = PTC2
5 = PTA2
6 = PTB19
7 = PTB18
*/
HCSR04 sensor1(TRIGGER,PTC5);
HCSR04 sensor2(TRIGGER,PTC7);
HCSR04 sensor3(TRIGGER,PTC0);
HCSR04 sensor4(TRIGGER,PTC3);
HCSR04 sensor5(TRIGGER,PTC2);
HCSR04 sensor6(TRIGGER,PTA2);
HCSR04 sensor7(TRIGGER,PTB19);
HCSR04 sensor8(TRIGGER,PTB18);

I2C i2c(PTE25,PTE24);

Serial pc(USBTX,USBRX);

int ult_del_izq , ult_izq_del, ult_izq_tras, ult_tras_izq, ult_tras_der, ult_der_tras, ult_der_del, ult_del_der;
int cero,uno,dos,tres,cuatro,cinco,seis,siete;

void correccion()
{
  wait_ms(25);
    uno = sensor2.distance();
    dos = sensor3.distance();
    wait_ms(25);
    cinco = sensor6.distance();
    wait_ms(25);
    seis = sensor7.distance();
    wait_ms(25);
    pc.printf("Entre a corregir \n");
    if (uno < 20 && dos < 20) {
        while (abs(uno - dos) != 0) {
            uno = sensor2.distance();
            wait_ms(25);
            dos = sensor3.distance();
            wait_ms(25);
            //Giro_izq(0.5f * abs(uno - dos) / (uno - dos))
            speed_mder = 0;
            speed_mizq = -0.5f * abs(uno - dos) / (uno - dos);
            pc.printf("A \n");
        }
        speed_mizq = 0;
        speed_mder = 0;
        wait_ms(50);
        //FRENO(0.05f);
    } else if (seis < 20 && cinco < 20) {
        while (abs(seis - cinco) != 0) {
            cinco = sensor6.distance();
            wait_ms(25);
            seis = sensor7.distance();
            wait_ms(25);
            speed_mizq = 0;
            speed_mder = -0.5f * abs(seis - cinco) / (seis - cinco);
            pc.printf("B \n");
            //Giro_izq(-0.5 * abs(seis - cinco) / (seis - cinco))
        }
        speed_mizq = 0;
        speed_mder = 0;
        wait_ms(25);
        //FRENO(0.05f);
    }
    if (uno < 4 && uno != 0 && dos < 10) {
        uno = sensor2.distance();
        wait_ms(25);
        dos = sensor3.distance();
        wait_ms(25);
        speed_mizq = 0;
        speed_mder = -0.5f;
        //Giro_izq(-0.5);
        while (uno <= dos) {
            uno = sensor2.distance();
            wait_ms(25);
            dos = sensor3.distance();
            wait_ms(25);
            speed_mizq = 0;
            speed_mder = -0.5f;
            pc.printf("C \n");
            //Giro_izq(-0.5);
        }
        speed_mizq = 0;
        speed_mder = 0;
        wait_ms(25);
        //Giro_izq(0.5); comentado originalmente
        while (uno > dos) {
            uno = sensor2.distance();
            wait_ms(25);
            dos = sensor3.distance();
            wait_ms(25);
            speed_mizq = -0.5f;
            speed_mder = 0;
            pc.printf("D \n");
            //Giro_izq(0.5);
        }
        speed_mizq = 0;
        speed_mder = 0;
        wait_ms(25);
        //FRENO(0.05f);
    } else if (seis < 4 && seis != 0 && cinco < 10) {
        cinco = sensor6.distance();
        wait_ms(25);
        seis = sensor7.distance();
        wait_ms(25);
        //Serial.println("Muy cerca de pared derecha, debo acomodar");
        //Giro_izq(0.5);
        while (seis <= cinco) {
            cinco = sensor6.distance();
            wait_ms(25);
            seis = sensor7.distance();
            wait_ms(25);
            speed_mder = 0.5f;
            speed_mizq = 0;
            pc.printf("E \n");
            //Giro_izq(-0.5);
        }
        speed_mizq = 0;
        speed_mder = 0;
        wait_ms(50);
        //FRENO(0.05f);
        //Giro_izq(-0.5); comentado originalmente
        while (seis > cinco) {
            cinco = sensor6.distance();
            wait_ms(25);
            seis = sensor7.distance();
            wait_ms(25);
            speed_mder = -0.5f;
            speed_mizq = 0;
            //Giro_izq(-0.5);
            pc.printf("F \n");
        }
        speed_mder = 0;
        speed_mizq = 0;
    }
}

void moveTile(int direction){

  leftQei.reset();
  rightQei.reset();

  int leftPulses  = 0;    //How far the left wheel has travelled.
  int rightPulses = 0;    //How far the right wheel has travelled.
  //int a = 1;
  int dist_izq = 1205; //Number of pulses to travel.
  int dist_der = 1243;

  speed_mizq = 0.7f * direction;
  speed_mder = 0.7f * direction;
  wait_us(1);

  while (abs(leftPulses) <= dist_izq || abs(rightPulses) <= dist_der){
    if(abs(leftPulses) <= dist_izq){
      leftPulses  = leftQei.getPulses();

    } else {
      speed_mizq = 0;
      wait_us(1);
    }
    if(abs(rightPulses) <= dist_der){
      rightPulses = rightQei.getPulses();

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

int main(){
  pc.baud(115200);
  update_speed_motors.attach_us(&updateSpeedMotors,500.0f); //500us
  pc.printf("Holo \n");
  while(1){
    /*ult_del_izq = sensor1.distance();
    wait_ms(50);
    ult_izq_del = sensor2.distance();
    wait_ms(50);
    ult_izq_tras = sensor3.distance();
    wait_ms(50);
    ult_tras_izq = sensor4.distance();
    wait_ms(50);
    ult_tras_der = sensor5.distance();
    wait_ms(50);
    ult_der_tras = sensor6.distance();
    wait_ms(50);
    ult_der_del = sensor7.distance();
    wait_ms(50);
    ult_del_der = sensor8.distance();
    wait_ms(50);

    pc.printf("0 : %d \t 7: %d \n",ult_del_izq, ult_del_der);
    pc.printf("1: %d \t 2: %d \n",ult_izq_del, ult_izq_tras);
    pc.printf("3: %d \t 4: %d \n",ult_tras_izq, ult_tras_der);
    pc.printf("5: %d \t 6: %d \n",ult_der_tras, ult_der_del);
    wait(1);*/
    moveTileForward();
    wait_ms(500);
    correccion();
    wait_ms(200);

  }

}
