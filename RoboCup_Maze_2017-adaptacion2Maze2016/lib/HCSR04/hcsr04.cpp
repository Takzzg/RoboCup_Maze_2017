#include "hcsr04.h"
#include "mbed.h"
//*HCSR04.cpp
HCSR04::HCSR04(PinName t, PinName e) : trig(t), echo(e) {}
//HOLA LINDO;
long HCSR04::echo_duration() {
  timer.reset();  //reset timer
  trig=0;   // trigger low
  wait_us(2); //  wait
  trig=1;   //  trigger high
  wait_us(10);
  trig=0;  // trigger low
  while(!echo); // start pulseIN
  timer.start();
  while(echo);
  timer.stop();
  return timer.read_us();
}

long HCSR04::distance(){ //return distance in cm
  duration = echo_duration();
  distance_cm = (duration/2)/29.1  ;
  return distance_cm;
}

long HCSR04::distance_mm(){
  duration = echo_duration();
  distancemm = (duration/2)/2.91;
  return distancemm;
}
