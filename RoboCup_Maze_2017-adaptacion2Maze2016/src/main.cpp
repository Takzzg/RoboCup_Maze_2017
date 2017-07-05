#include "mbed.h"
#include "Adafruit_BNO055.h"
#include "MotorDC.h"
#include "hcsr04.h"
#include "QEI.h"//---
//#include "mlx90614.h"

int dist_1;
int dist_2;

/*
DigitalOut mandar90(PTE20);
DigitalIn giro_menos(PTE22);
DigitalIn rampa_arriba(PTC7);
DigitalIn rampa_abajo(PTC6);
InterruptIn pin_vict_der(PTA4);
//InterruptIn pin_vict_izq (D5);*/

///variables mlx
/*
I2C i2c(PTE0, PTE1);   //sda,scl
MLX90614 s_calor_izq(&i2c, 0xB0);
MLX90614 s_calor_der(&i2c);
float Medida_ini_izq = 0;
float Medida_ini_der = 0;

float temp_izq;
float temp_der;
bool vict_izq = false;
int vict_der = 0;
bool pin_sube_rampa = false;
bool pin_baja_rampa = false;
//variables del encoder
int diferencia = 0;
int encoderPOS = 0;
int encoderALast = 0;
int n = 1;
//DigitalIn encoderA(PTD3);
//DigitalIn encoderB(PTA12);*/
AnalogIn cny70(A0);
bool vict_izq = false;
bool vict_der = false;
float lectura;
bool detenido = false;
//variables motores
MotorDC M_izq(PTD3, PTC9, PTB23);  //pwma, ain1, ain2---- M2
MotorDC M_der(PTA1, PTB9, PTC1);  //pwmb, bin1, bin2---- M1
DigitalOut stby(PTC8,1);

#define enc_der_chana PTB11
#define enc_der_chanb PTC11


#define enc_izq_chana PTB10
#define enc_izq_chanb PTB2 //antes ptb20
#define PULSES_PER_REVOLUTION 334
#define BNO055_SAMPLERATE_DELAY_MS (100)

QEI leftQei(enc_izq_chana, enc_izq_chanb, NC, PULSES_PER_REVOLUTION);  //chanA, chanB, index, ppr
QEI rightQei(enc_der_chana, enc_der_chanb, NC, PULSES_PER_REVOLUTION); //chanB, chanA, index, ppr

I2C i2c(PTE25,PTE24);
Adafruit_BNO055 bno = Adafruit_BNO055(55,0x28,&i2c);
imu::Quaternion quat;
imu::Vector<3> vector_euler;
imu::Vector<3> vector_acc;
imu::Vector<3> vector_euler_init;

Ticker update_speed_motors;

float speed_mizq = 0;
float speed_mder = 0;

void updateSpeedMotors(){
  M_izq = speed_mizq;
  M_der = speed_mder;
}

#define TRIGGER PTB3

/*#define vel(i,d) motor_izq.speed(i);motor_der.speed(d);
#define giro(i,d) motor_izq.speed(i);motor_der.speed(d);
#define Giro_izq(i) motor_izq.speed(-1*i);motor_der.speed(0);
#define FRENO(a) motor_izq.speed(0),motor_der.speed(0), wait(a);

//D12 TRIGGER D11 ECHO
//HCSR04 sensor(D12, D11);
//HCSR04 ultra_n(PTC10, PTC11);//original
HCSR04 ultra_o(PTE23, PTB20);
HCSR04 ultra_e(PTB19, PTB18);
HCSR04 ultra_n(PTC10, PTC11);
HCSR04 ultra_se(PTE31, PTE19);
HCSR04 ultra_so(PTE18, PTE17);*/

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

Serial pc(USBTX, USBRX);


int ult_del_izq , ult_izq_del, ult_izq_tras, ult_tras_izq, ult_tras_der, ult_der_tras, ult_der_del, ult_del_der;
int cero,uno,dos,tres,cuatro,cinco,seis,siete;



//SLCD slcd;
#define est_add_info 0
#define est_calcular_dest 1
#define est_recorrer_camino 2
#define est_fin_lab 3

#define sub_calculo_giro 0
#define sub_realizo_giro 1
#define sub_lectura 2
#define sub_negro 3
#define sub_blanco 4

#define point_limit 64
int estado = 0;
int sub_estado = 0;
/*DigitalOut led_2(LED2);
DigitalOut led(LED1);
TSIAnalogSlider tsi(PTB16, PTB17, 100);*/
//unsigned long timer = millis();
bool timer_on = false;
int dir = 0;
int pinchila = 1;
int dist_n, dist_o, dist_e, dist_d, dist_se, dist_so;

/*int LEDS[4] = { 40, 34, 36, 38 };
PinName Trigger[4] = { PTC10, PTB19, PTB19, PTE23 //PTC10,PTB19,PTB19,PTE20 original
};

PinName Echo[4] = { PTC11, PTB18, PTB18, PTB20 //PTC11,PTB18,PTB18,PTE23 original
};*/

//Cálculo para giro (dirección actual - dirección buscada + 4)%4
int GIROS[4] = { 0, -1, 2, 1 };
/*
 0  no gira
 -1 gira hacia la izquierda
 1 gira hacia la derecha
 2 giro de 180.
 */
 void displaySensorStatus(void)
 {
   /* Get the system status values (mostly for debugging purposes) */
   uint8_t system_status, self_test_results, system_error;
   system_status = self_test_results = system_error = 0;
   bno.getSystemStatus(&system_status, &self_test_results, &system_error);

   /* Display the results in the Serial Monitor */
   //pc.printf("\n");
   //pc.printf("System Status: %X\n",system_status);
   //pc.printf("Self Test: %X\n",self_test_results);
   //pc.printf("System Error: %X\n",system_error);
   wait_ms(500);
 }
 void init_value_3d_sensor(){
   //wait_ms(1000);
   quat = bno.getQuat();
   vector_euler_init = quat.toEuler();
   vector_euler_init.toDegrees();
   //wait_ms(1000);
 }

 void turn_90(int direction){
  init_value_3d_sensor();

  float eje_x = 0;
  float eje_y = 0;
  float eje_z = 0;
  float grados = 86;

  speed_mizq = 0.8f * direction;
  speed_mder = -0.8f * direction;

  while(fabs(eje_x) < grados){
     quat = bno.getQuat();
     vector_euler = quat.toEuler();
     vector_euler.toDegrees();

     /*//pc.printf("X inicial: %.1f \t",vector_euler_init.x());
     //pc.printf("X calc: %.1f \t", eje_x);
     //pc.printf("\t X actual: %.1f \n",vector_euler.x());
     */
     eje_x = vector_euler.x() - vector_euler_init.x();
     if(eje_x > 180) eje_x -= 360;
     if(eje_x < -180) eje_x += 360;
   }
   //pc.printf("Ya giré %.1f grados",grados);
   speed_mizq = 0;
   speed_mder = 0;
   wait_us(1);
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
     //pc.printf("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
     while(1);
   }
   wait(1.0f);
   //pc.printf("Iniciando...\n");
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
	  int dist_izq = 1205; //Number of pulses to travel.
	  int dist_der = 1293;

	  speed_mizq = 0.8f * direction;
	  speed_mder = 0.8f * direction;
	  wait_us(1);

	  while (abs(leftPulses) <= dist_izq || abs(rightPulses) <= dist_der){
      /*cero = sensor1.distance();
      wait_ms(25);
      if(cero <= 5 ){
        leftPulses = dist_izq;
        rightPulses = dist_der;
      }*/
      if(abs(leftPulses) <= dist_izq){
	      leftPulses  = leftQei.getPulses();
	      //pc.printf("encoder_izq: %d \n", leftPulses);
	    } else {
	      speed_mizq = 0;
	      wait_us(1);
	    }
	    if(abs(rightPulses) <= dist_der){
	      rightPulses = rightQei.getPulses();
	      //pc.printf("encoder_der: %d \n", rightPulses);
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
	  //wait_ms(100);
	}
	void moveTileForward(){
	  moveTile(1);
	}

	void moveTileBackward(){
	  moveTile(-1);
	}

struct comb_bool {
	bool a;
	bool b;
};

struct pcardinales {
	bool n;
	bool e;
	bool s;
	bool o;
};

struct point {
	int X;
	int Y;
	int Z;
};

struct navpoint {
	int X;
	int Y;
	int Z;
	bool conexiones[6];
	bool f_nav;
	int pond;
	bool f_negro;
	bool f_check;
	bool hay_vict;
	bool is_rampa;
};

struct str_rampa {
  int pbX;
  int pbY;
  int pbZ;
  int paX;
  int paY;
  int paZ;
  int dir_sub;
  bool f_rampa;
}rampa;

struct mi_robot {
	int X;
	int Y;
	int Z;
	//boolean conexiones[6];
	int pos_destino;
	int mi_dir;
	int pos_mc;
	int micamino[point_limit];
	int giro;
} masche;

int cant_puntos = 0;
struct navpoint pnavegados[point_limit];

//Devuelve un point con las coordenadas del punto que se encuentra en la dirección dir desde el punto (X, Y)
struct point armar_punto(int X, int Y, int Z, int dir) {
	struct point punto;
	if (dir == 0) {
		Y++;
	} else if (dir == 1) {
		X++;
	} else if (dir == 2) {
		Y--;
	} else if (dir == 3) {
		X--;
	} else if (dir == 4) {
		Z--;
	} else {
		Z++;
	}
	punto.X = X;
	punto.Y = Y;
	punto.Z = Z;
	return punto;
}

//Busca un punto en el arrego de puntos existentes y devuelve la posición en la que este se encuentra.
//Si no existe devuelve el valor cant_puntos
//Busca un punto en el arrego de puntos existentes y devuelve la posición en la que este se encuentra.
//Si no existe devuelve el valor cant_puntos
int buscar_pospunto(int X, int Y, int Z) {
	int pos = cant_puntos;
	for (int i = 0; i < cant_puntos; i++) {
		if (pnavegados[i].X == X && pnavegados[i].Y == Y
				&& pnavegados[i].Z == Z) {
			return i;
		}
	}
	return pos;
}

void new_point(int n, int e, int s, int o, bool f_check, bool p_rampa) {
	//Se guarda el punto en el que se encuentra actualmente masche
	int pos_m = buscar_pospunto(masche.X, masche.Y, masche.Z);
	if (pos_m == 0) {
		pnavegados[pos_m].X = masche.X;
		pnavegados[pos_m].Y = masche.Y;
		pnavegados[pos_m].Z = masche.Z;
		pnavegados[pos_m].hay_vict = false;
		pnavegados[pos_m].conexiones[4] = false;
		pnavegados[pos_m].conexiones[5] = false;
		cant_puntos++;
	}
	if (n > 25 || n == 0) {
		/*//Serial.println("1");
		 //Serial.print("Dist n: ");//Serial.println(dist_n);
		 */
		pnavegados[pos_m].conexiones[0] = true;
	} else {
		pnavegados[pos_m].conexiones[0] = false;
	}
	if (e > 25 || e == 0) {
		/*//Serial.println("2");
		 //Serial.print("Dist e: ");//Serial.println(dist_e);
		 */
		pnavegados[pos_m].conexiones[1] = true;
	} else {
		pnavegados[pos_m].conexiones[1] = false;
	}
	if (s > 25 || s == 0) {
		////Serial.println("3");
		pnavegados[pos_m].conexiones[2] = true;
	} else {
		pnavegados[pos_m].conexiones[2] = false;
	}
	if (o > 25 || o == 0) {
		/*//Serial.println("4");
		 //Serial.print("Dist o: ");//Serial.println(dist_o);
		 */
		pnavegados[pos_m].conexiones[3] = true;
	} else {
		pnavegados[pos_m].conexiones[3] = false;
	}
	pnavegados[pos_m].f_nav = true;
	pnavegados[pos_m].f_check = f_check;
	pnavegados[pos_m].is_rampa = p_rampa;
	//Se debe ver en todas direcciones los puntos conectados y verificar si existen (fueron guardados)
	struct point punto;
	for (int i = 0; i < 4; i++) {
		punto = armar_punto(masche.X, masche.Y, masche.Z, i);
		int pos = buscar_pospunto(punto.X, punto.Y, punto.Z);

		//Serial.print("punto: ");
		//Serial.print(" ( ");
		//Serial.print(punto.X);
		//Serial.print(" , ");
		//Serial.print(punto.Y);
		//Serial.print(" , ");
		//Serial.print(punto.Z);
		//Serial.println(" )");

		//Si el punto no fue guardado y en la dirección correspondiente hay conexión lo guarda.
		if (pos == cant_puntos && pnavegados[pos_m].conexiones[i]) {
			//Serial.println("el punto no existe y hay conexion");
			//Serial.print("pos: ");
			//Serial.println(pos);
			pnavegados[pos].X = punto.X;
			pnavegados[pos].Y = punto.Y;
			pnavegados[pos].Z = punto.Z;
			pnavegados[pos].hay_vict = false;
			pnavegados[pos].pond = point_limit;
			pnavegados[pos].f_negro = false;
			pnavegados[pos].f_nav = false;
			pnavegados[pos].conexiones[4] = false;
			pnavegados[pos].conexiones[5] = false;
			cant_puntos++;
		}
	}
}
//Devuelve la distancia DX + DY entre dos puntos A y B
int dist_AB(int AX, int AY, int AZ, int BX, int BY, int BZ) {
	return abs(AX - BX) + abs(AY - BY) + abs(AZ - BZ);
}

//Devuelve la dirección que se debe seguir para llegar de un punto A a un punto B
//Si devuelve 4 entonces los puntos tienen una distancia mayor a 1
//Si devuelve 5 entonces los puntos tienen las mismas coordenadas
int armar_dir(int AX, int AY, int AZ, int BX, int BY, int BZ) {
	int dist = dist_AB(AX, AY, AZ, BX, BY, BZ);
	if (dist == 1) {
		if (AX < BX) {
			return 1;
		} else if (AY < BY) {
			return 0;
		} else if (AY > BY) {
			return 2;
		} else if (AX > BX) {
			return 3;
		} else if (AZ < BZ) {
			return 5;
		} else {
			return 4;
		}
	} else if (dist == 0) {
		return 6;
	} else {
		return 7;
	}
}

//Al finalizar la ejecución de esta función todos los puntos terminan ponderados de acuerdo a la cantidad mínima de puntos que se deben atravesar para llegar a un punto objetivo.
//El punto objetivo se encontrará en la posición pos.

//Devuelve un boolean = 1 si hay conexión entre dos puntos A, de posición pos_ini, y B, de posición pos_fin, existentes.
//Devuelve un boolean = 1 si hay conexión entre dos puntos A, de posición pos_ini, y B, de posición pos_fin, existentes.
bool bool_conect(int pos_ini, int pos_fin) {
	int dir = armar_dir(pnavegados[pos_ini].X, pnavegados[pos_ini].Y,
			pnavegados[pos_ini].Z, pnavegados[pos_fin].X, pnavegados[pos_fin].Y,
			pnavegados[pos_fin].Z);

	//Si el punto fue navegado se conoce las direcciones que se puede seguir desde este
	//y si existe una dirección para ir de un punto a otro se devuelve el valor de verdad de la conexión
	if (dir < 6 && pnavegados[pos_ini].f_nav && !pnavegados[pos_ini].f_negro
			&& !pnavegados[pos_fin].f_negro) {
		return pnavegados[pos_ini].conexiones[dir];
	} else {
		return false;
	}
}

//Al finalizar la ejecución de esta función todos los puntos terminan ponderados de acuerdo a la cantidad mínima de puntos que se deben atravesar para llegar a un punto objetivo.
//El punto objetivo se encontrará en la posición pos.
void ponderar(int pos) {
	bool cambios = false;

	//Inicializo las poderaciones
	for (int i = 0; i < cant_puntos; i++) {
		pnavegados[i].pond = cant_puntos;
	}
	int pond_act = 0;
	int X = pnavegados[pos].X;
	int Y = pnavegados[pos].Y;
	int Z = pnavegados[pos].Z;
	pnavegados[pos].pond = pond_act;

	//Ponderizo todos los puntos.
	do {
		cambios = false;
		for (int i = 0; i < cant_puntos; i++) {
			if (pnavegados[i].pond == pond_act) {
				for (int j = 0; j < cant_puntos; j++) {
					if (bool_conect(i, j) && pnavegados[j].pond > pond_act) {
						cambios = true;
						pnavegados[j].pond = pond_act + 1;
					}
				}
			}
		}
		pond_act++;
	} while (cambios);
}

//Realizar función para ponderar puntos hasta encontrar destino.
//Realizar función para elegir destino con ciertas características.

//Esta función devuelve la posición (en el arreglo de puntos guardados) del punto sin navegar más cercano.
//En caso de no haber más puntos sin navegar devuelve el punto de inicio del laberinto.
int new_destino() {
	ponderar(buscar_pospunto(masche.X, masche.Y, masche.Z));
	int dist = cant_puntos;
	int pos = 0;
	for (int i = 0; i < cant_puntos; i++) {
		if (!pnavegados[i].f_nav && pnavegados[i].pond <= dist
				&& !pnavegados[i].f_negro) {
			dist = pnavegados[i].pond;
			pos = i;
		}
	}
	masche.pos_destino = pos;
	return pos;
}

//Realizar función para armar camino con destino al punto navegado en la posición pos_fin del arreglo correspondiente.
void armar_camino(int pos_fin) {
	int pond = pnavegados[pos_fin].pond;
	int pos = pos_fin;    //, pos_p;//, dist_dir_p, dir_post;
	//int dir_act = mi_dir;
	//int dist_dir;
	bool found;

	//Se llena el arreglo de mi_camino con las direcciones a seguir.
	struct point punto;
	while (pond > 0) {
		found = false;
		for (int i = 0; i < cant_puntos && !found; i++) {
			if (bool_conect(i, pos)
					&& pnavegados[pos].pond - pnavegados[i].pond == 1) {
				found = true;
				pond--;
				masche.micamino[pond] = armar_dir(pnavegados[i].X,
						pnavegados[i].Y, pnavegados[i].Z, pnavegados[pos].X,
						pnavegados[pos].Y, pnavegados[pos].Z);
				pos = i;
			}
		}
	}
}

void correccion(){
    cero = sensor1.distance();
    wait_ms(25);
    siete = sensor8.distance();
    wait_ms(25);
    tres = sensor4.distance();
    wait_ms(25);
    cuatro = sensor5.distance();
    wait_ms(25);
    if(cero < 15 && siete < 15){
      //pc.printf("Distancia para ALEJARME O ACERCARME\n");
      while(cero != 5){
        cero = sensor1.distance();
        wait_ms(25);
        siete = sensor8.distance();
        wait_ms(25);
        if(cero > 5){
          speed_mizq = 0.4f;
          speed_mder = 0.4f;
          wait_us(1);
        }
        else if(cero < 5){
          speed_mizq = -0.4f;
          speed_mder = -0.4f;
          wait_us(1);
        }
      }
      speed_mder = 0;
      speed_mizq = 0;
      wait_us(1);
      do{
        cero = sensor1.distance();
        wait_ms(25);
        siete = sensor8.distance();
        wait_ms(25);
        if(cero < siete)
             speed_mder = 0.4f;
        else
          speed_mder = -0.4f;
        wait_us(1);
      }while(cero != siete);
    }
    else if(tres < 15 && cuatro < 15){
      //pc.printf("Distancia para ALEJARME O ACERCARME de atras\n");
      while(tres != 5){
        tres = sensor4.distance();
        wait_ms(25);
        cuatro = sensor5.distance();
        wait_ms(25);
        if(tres > 5){
          speed_mizq = -0.4f;
          speed_mder = -0.4f;
          wait_us(1);
        }
        else if(tres < 5){
          speed_mizq = 0.4f;
          speed_mder = 0.4f;
          wait_us(1);
        }
      }
      speed_mder = 0;
      speed_mizq = 0;
      wait_us(1);
      do{
        tres = sensor4.distance();
        wait_ms(25);
        cuatro = sensor5.distance();
        wait_ms(25);
        if(tres < cuatro)
          speed_mizq = 0.4f;
        else
          speed_mizq = -0.4f;
        wait_us(1);
      }while(tres != cuatro);
    }
    speed_mder = 0;
    speed_mizq = 0;
    wait_us(1);

    uno = sensor2.distance();
    wait_ms(25);
    dos = sensor3.distance();
    wait_ms(25);
    cinco = sensor6.distance();
    wait_ms(25);
    seis = sensor7.distance();
    wait_ms(25);
    //pc.printf("Entre a corregir \n");
    if (uno < 20 && dos < 20) {
        speed_mder = 0;
        while (uno - dos != 0) {
            uno = sensor2.distance();
            wait_ms(25);
            dos = sensor3.distance();
            wait_ms(25);
            speed_mizq = -0.4f * abs(uno - dos) / (uno - dos);
            //pc.printf("A \n");
        }
        speed_mizq = 0;
        wait_us(1);
    } else if (seis < 20 && cinco < 20) {
        while (seis - cinco != 0) {
            cinco = sensor6.distance();
            wait_ms(25);
            seis = sensor7.distance();
            wait_ms(25);
            speed_mder = -0.4f * abs(seis - cinco) / (seis - cinco);
            //pc.printf("B \n");
        }
        speed_mder = 0;
        wait_us(1);
    }
}

void impr_ultras(){
  while(true){
    cero = sensor1.distance_mm();
    wait_ms(20);
    uno = sensor2.distance_mm();
    wait_ms(20);
    dos = sensor3.distance_mm();
    wait_ms(20);
    tres = sensor4.distance_mm();
    wait_ms(20);
    cuatro = sensor5.distance_mm();
    wait_ms(20);
    cinco = sensor6.distance_mm();
    wait_ms(20);
    seis = sensor7.distance_mm();
    wait_ms(20);
    siete = sensor8.distance_mm();
    wait_ms(20);
    pc.printf("\ndist_mm\t0 = %d\t1 = %d\t 2 = %d\t 3 = %d\t 4 = %d\t 5 = %d\t 6 = %d\t 7 = %d\t 8 = %d", cero,uno,dos,tres,cuatro,cinco,seis,siete);
    wait_ms(300);
  }
}

void correccion_maxi(){
  speed_mizq = 0;
  speed_mder = 0;
  uno = sensor2.distance_mm();
  wait_ms(25);
  dos = sensor3.distance_mm();
  wait_ms(25);
  seis = sensor7.distance_mm();
  wait_ms(25);
  cinco = sensor6.distance_mm();
  wait_ms(25);
  if((uno < 50 || dos < 50) && uno < dos + 2){
    speed_mder = -0.5f;
    do{
      uno = sensor2.distance_mm();
      wait_ms(25);
      dos = sensor3.distance_mm();
      wait_ms(25);
    }while(uno < dos + 2);
    speed_mder = 0.4f;
    do{
      uno = sensor2.distance_mm();
      wait_ms(25);
      dos = sensor3.distance_mm();
      wait_ms(25);
    }while(uno> dos + 2);
    speed_mder = 0;
  }
  else if((seis < 50 || cinco < 50) && seis < cinco + 2 ){
    speed_mizq = -0.5f;
    do{
      seis = sensor7.distance_mm();
      wait_ms(25);
      cinco = sensor6.distance_mm();
      wait_ms(25);
    }while(seis < cinco + 2);
    speed_mizq = 0.4f;
    do{
      seis = sensor7.distance_mm();
      wait_ms(25);
      cinco = sensor6.distance_mm();
      wait_ms(25);
    }while(seis > cinco + 2);
    speed_mizq = 0;
  }

  cero = sensor1.distance();
  wait_ms(25);
  tres = sensor4.distance();
  wait_ms(25);
  if(cero < 20){
    while(cero != 4){
      if(cero > 4){
        speed_mizq = 0.4f;
        speed_mder = 0.4f;
      }
      else if(cero < 4){
        speed_mizq = -0.4f;
        speed_mder = -0.4f;
      }
      cero = sensor1.distance();
      wait_ms(25);
    }
    speed_mizq = 0;
    speed_mder = 0;
    do{
      cero = sensor1.distance();
      wait_ms(25);
      siete = sensor8.distance();
      wait_ms(25);
      seis = sensor7.distance();
      wait_ms(25);
      uno = sensor2.distance();
      wait_ms(25);
      if(seis < 7){
        if(cero < siete){
          speed_mizq = -0.4f;
        }
        else if (siete < cero){
          speed_mizq = 0.4f;
        }
      }
      else{// if(uno < 7){
        if(cero < siete){
          speed_mder = 0.4f;
        }
        else if (siete < cero){
          speed_mder = -0.4f;
        }
      }
    }while(cero != siete);
  }
  else if(tres < 20){
    while(tres != 5){
      if(tres > 5){
        speed_mizq = -0.4f;
        speed_mder = -0.4f;
      }
      else if(tres < 5){
        speed_mizq = 0.4f;
        speed_mder = 0.4f;
      }
      tres = sensor4.distance();
      wait_ms(25);
    }
    speed_mizq = 0;
    speed_mder = 0;
    do{
      tres = sensor4.distance();
      wait_ms(25);
      cuatro = sensor5.distance();
      wait_ms(25);
      cinco = sensor6.distance();
      wait_ms(25);
      dos = sensor3.distance();
      wait_ms(25);
      if(cinco < 7){
        if(tres < cuatro){
          speed_mizq = 0.4f;
        }
        else if (cuatro < tres){
          speed_mizq = -0.4f;
        }
      }
      else{// if(dos < 7){
        if(tres < cuatro){
          speed_mder = -0.4f;
        }
        else if (cuatro < tres){
          speed_mder = 0.4f;
        }
      }
    }while(tres != cuatro);
  }

  tres = sensor4.distance();
  wait_ms(25);
  cuatro = sensor5.distance();
  wait_ms(25);
  /*if(tres < 200 && tres > 50){
    speed_mizq = 0.4f;
    speed_mder = 0.4f;
    do{
      tres = sensor4.distance_mm();
      wait_ms(25);
    }while(tres > 50);
  }*/
}

void hay_vict_der() {
	vict_der = 1;
}

void hay_vict_izq() {
	vict_izq = 1;
}

/*void ENCODER(int valor, float speed, bool ver_neg) {
	int encoderPOS = 0;
	int encoderALast = 0;
	int n = 1;
	DigitalIn encoderA(PTD3);
	DigitalIn encoderB(PTA12);
	while (encoderPOS < valor) {
		//pc.printf("CNY70: %f\n", cny70.read());
		float leyendo = cny70.read();
		if (leyendo < 0.02f && ver_neg) {   ///abs(lectura - 0.5)
			vel(0, 0);
			//pc.printf("Retorno\n");
			detenido = true;
			diferencia = encoderPOS;
			//pc.printf("Alcance a avanzar: %d\n", diferencia);
			encoderPOS = valor;

		} else {
			detenido = false;
		}
		//mlx();
		n = encoderA;
		wait(0.003f);
		if (encoderALast == 0 && n == 1) {
			if (encoderB == 1) {
				encoderPOS++;
			} else {
				encoderPOS--;
			}
			//pc.printf("%d \n", encoderPOS);
		}
		encoderALast = n;
		wait(0.003f);
		if (encoderPOS == valor) {
			vel(0, 0);
			wait(1);
			//vel(0.4f,0.4f);
			//encoderPOS = 0;
		} else {
			vel(speed, speed);
		}

	}
	//vel(-speed,-speed);
	//wait(0.05f);
	vel(0, 0);

}*/

int main() {
	pc.baud(115200);
	update_speed_motors.attach_us(&updateSpeedMotors,500.0f); //500us
  check_Bno();
  bool a = true;
	//mandar90 = 0;
	//giro_menos = 0;
  int rampa_arriba = 0;
  int rampa_abajo = 0;
	//led_2 = 0;

	//pin_vict_der.rise(&hay_vict_der);
	//pin_vict_izq.rise(&hay_vict_izq);
	rampa.f_rampa = false;
	estado = est_add_info;
	//pc.printf("Laberinto \n");
	masche.X = 0;
	masche.Y = 0;
	masche.pos_mc = 0;
  wait(1);
	while (true) {
		if (estado == est_add_info) {
			//led = 0;
			//pc.printf("Estado est_add_info\n");

			int dist[4] = { 0, 0, 0, 0 };
			if (masche.mi_dir == 4) {
				dir = (rampa.dir_sub + 2) % 4;
			} else if (masche.mi_dir == 5) {
				dir = rampa.dir_sub;
			} else {
				dir = masche.mi_dir;
			}

			int mi_dir = dir;
      cero = sensor1.distance();
      wait_ms(25);
      uno = sensor2.distance();
      wait_ms(25);
      dos = sensor3.distance();
      wait_ms(25);
      tres = sensor4.distance();
      wait_ms(25);
      cuatro = sensor5.distance();
      wait_ms(25);
      cinco = sensor6.distance();
      wait_ms(25);
      seis = sensor7.distance();
      wait_ms(25);
      siete = sensor8.distance();
      wait_ms(25);
      int dist_n [] = {(cero+siete)/2,(cinco+seis)/2,(cuatro+tres)/2,(dos+uno)/2};
      for (int i = 0; i < 4; i++) {
				//pc.printf("i : %d \n", i);

				//HCSR04 ultra(Trigger[i], Echo[i]);
				//dist_n = ultra.distance();
				//wait_ms(50);
				//pc.printf("Dir = %d ---->, Dist = %d \n", i, dist_n[i]);
        //wait_ms(10);
				dir = (i + mi_dir) % 4;
        dist[dir] = dist_n[i];
        /*if (i == 2) {
					if ((masche.X == rampa.paX && masche.Y == rampa.paY
							&& masche.Z == rampa.paZ)
							|| (masche.X == rampa.pbX && masche.Y == rampa.pbY
									&& masche.Z == rampa.pbZ)) {
						if (masche.X != 0 || masche.Y != 0 || masche.Z != 0) {
							dist[dir] = 1;
						} else {
							dist[dir] = 26;
						}
					} else {
						dist[dir] = 26;
					}
				} else {
					dist[dir] = dist_n;
				}*/
			}
			int direc[3] = { (0 + masche.mi_dir) % 4, (1 + masche.mi_dir) % 4,
					(3 + masche.mi_dir) % 4 };
			bool p_rampa = false;
			//while(tsi.readPercentage() < 0.7f);
			//wait(0.5f);
			/*if (!rampa.f_rampa && dist[direc[0]] <= 25 && dist[direc[0]] != 0) {
			 if (((dist[direc[1]] > 25 || dist[direc[1]] == 0)
			 && dist[direc[2]] <= 25 && dist[direc[2]] != 0)
			 || ((dist[direc[2]] > 25 || dist[direc[2]] == 0)
			 && dist[direc[1]] <= 25 && dist[direc[1]] != 0)) {
			 p_rampa = true;
			 //pc.printf("Posible rampa\n");
			 ///*digitalWrite(led, HIGH);
			 //delay(300);
			 //digitalWrite(led, LOW);
			 FRENO(0.3f);
			 }
			 }*/
			/*bool check = false;
			 if (analogRead(pin_ldr_ch) > PLATEADO) {
			 check = true;
			 pos_check = buscar_pospunto(masche.X, masche.Y, masche.Z);///guarda pos del ultimo check
			 Serial.print("pos en X: ");
			 Serial.println(masche.X);
			 Serial.print("pos en Y: ");
			 Serial.println(masche.Y);
			 Serial.print("pos en Z: ");
			 Serial.println(masche.Z);
			 for(int i = 0; i < cant_puntos; i++) {
			 pnavegados[i] = false;
			 }
			 for (int u = 0; u < 2; u++) {
			 digitalWrite(led, HIGH);
			 delay(50);
			 digitalWrite(led, LOW);
			 delay(50);
			 }
			 pausa(500);
			 }*/
			new_point(dist[0], dist[1], dist[2], dist[3], false, p_rampa);
			//pc.printf("Cantidad de puntos (new) = %d \n", cant_puntos);
      //wait_ms(10);
      //Serial.println(cant_puntos);
			for (int i = 0; i < cant_puntos; i++) {
				//pc.printf("( %d, %d, %d)\n", pnavegados[i].X, pnavegados[i].Y,pnavegados[i].Z);
        //wait_ms(500);
        /*Serial.print("( ");
				 Serial.print(pnavegados[i].X);
				 Serial.print(" , ");
				 Serial.print(pnavegados[i].Y);
				 Serial.println(" )");*/
			}
			estado = est_calcular_dest;
		}    //llave de estado add info
		if (estado == est_calcular_dest) {

			//pc.printf("Estado calcular destino \n");
			//lectura = cny70.read();
			//pc.printf("Lectura: %f\n", lectura);
			new_destino();
			//pc.printf("pos destino: %d, // ( %d, %d, %d)\n", masche.pos_destino,pnavegados[masche.pos_destino].X,pnavegados[masche.pos_destino].Y,pnavegados[masche.pos_destino].Z);
      //wait(2);
			if (!pnavegados[masche.pos_destino].f_nav
					|| (pnavegados[masche.pos_destino].X == 0
							&& pnavegados[masche.pos_destino].Y == 0
							&& pnavegados[masche.pos_destino].Z == 0)) {
				masche.pos_mc = 0;
				armar_camino(masche.pos_destino);
				estado = est_recorrer_camino;
				sub_estado = sub_calculo_giro;
			}
			for (int i = 0; i < cant_puntos; i++) {
				//pc.printf("(%d,%d , %d)//// negro: %d \n ", pnavegados[i].X,pnavegados[i].Y, pnavegados[i].Z,pnavegados[i].f_negro);
			}
		}    //llave est_calcular_dest
		if (estado == est_recorrer_camino) {

			int dir = masche.micamino[masche.pos_mc];
			if (sub_estado == sub_calculo_giro) {
				if (masche.mi_dir == 4) {
					dir = (rampa.dir_sub + 2) % 4;
				} else if (masche.mi_dir == 5) {
					dir = rampa.dir_sub;
				} else {
					dir = masche.mi_dir;
				}

				sub_estado = sub_realizo_giro;

			}
			if (sub_estado == sub_realizo_giro) {
				//pc.printf("pond = %d \n", pnavegados[masche.pos_destino].pond);
				int dir = masche.micamino[masche.pos_mc];
				int mi_dir;
				if (dir == 4) {
					dir = (rampa.dir_sub + 2) % 4;
				} else if (dir == 5) {
					dir = rampa.dir_sub;
				}
				if (masche.mi_dir == 4) {
					//dir = rampa.dir_sub;
					mi_dir = (rampa.dir_sub + 2) % 4;
				} else if (masche.mi_dir == 5) {
					//dir = (rampa.dir_sub + 2) % 4;;
					mi_dir = rampa.dir_sub;
				} else {
					mi_dir = masche.mi_dir;
				}
				masche.giro = GIROS[(mi_dir - dir + 4) % 4];
				int velocidad = masche.giro / abs(masche.giro);
				if (masche.giro == 1) {
          //pc.printf("Giro Der\n");
          turn_right90();
				} else if (masche.giro == -1) {
					//pc.printf("Giro Izq\n");
          turn_left90();
				} else if (masche.giro == 2) {
					//pc.printf("Giro 180\n");
          turn_right90();
          turn_right90();
				}
        /*for (int i = 0; i < abs(masche.giro); i++) {
				  speed_mder = 0;
          speed_mizq = 0;
          wait_ms(600);
					int pos_p = buscar_pospunto(masche.X, masche.Y, masche.Z);


					if (!pnavegados[pos_p].hay_vict) {
						if (vict_izq) {

              pnavegados[pos_p].hay_vict = true;
							vict_izq = 0;
						}
						if (vict_der) {

              pnavegados[pos_p].hay_vict = true;
							vict_der = 0;
						}
					} else {
						vict_izq = 0;
						vict_der = 0;
					}

				}*/
				//pc.printf("Deje de girar\n");
				masche.mi_dir = masche.micamino[masche.pos_mc];
				bool subir_bajar = false;
				//wait(0.5f);
				if (masche.mi_dir > 3) {
					subir_bajar = true;
				}
				if (masche.giro == 2 || masche.giro == 1 || masche.giro == -1) {
					//pc.printf("Correccion\n");
					//correccion();
          correccion_maxi();
					//pc.printf("Deje de corregir\n");
				} else if (masche.giro == 0) {
          speed_mder = 0;
          speed_mizq = 0;
          wait_us(1);
          //FRENO(0.01);
				}
				//digitalWrite(led, LOW);
				//bool detenido;
				//float vel = 0.5;
				do {
					if (subir_bajar) {
						//bool detener_r = false;
						//Acá va el código que hace que se acomode en la rampa.
						subir_bajar = false;
					} else if (!rampa.f_rampa
							&& pnavegados[buscar_pospunto(masche.X, masche.Y,
									masche.Z)].is_rampa
							&& masche.pos_mc + 1
									== pnavegados[masche.pos_destino].pond) {
						//pc.printf("Avanzo para saber si hay rampa\n");
						//ENCODER(38, 0.6, true); original
						//ENCODER(38, 0.6, true);
            moveTileForward();
						//Si no hay inclinación
            //Acá va el código que hace que detectemos la rampa. Temporalmente está con variables enteras
						if (rampa_arriba == 0 && rampa_abajo == 0) {
							//pc.printf("No hay rampa\n");
							pnavegados[masche.pos_destino].is_rampa = false;
						} else {
							rampa.f_rampa = true;
							subir_bajar = true;
							//si hay inclinación hacia abajo
							if (rampa_abajo == 1) {
								//pc.printf("Hay rampa abajo\n");
								int pos = buscar_pospunto(masche.X, masche.Y,
										masche.Z);
								rampa.paX = masche.X;
								rampa.paY = masche.Y;
								rampa.paZ = masche.Z; //Guardar posición actual en (paX, paY, paZ)

								rampa.pbX = masche.X;
								rampa.pbY = masche.Y;
								rampa.pbZ = masche.Z - 1; //Guardar posición actual en (pbX, pbY, pbZ)
								pnavegados[masche.pos_destino].X = masche.X;
								pnavegados[masche.pos_destino].Y = masche.Y;
								pnavegados[masche.pos_destino].Z = masche.Z - 1; //Guardar la posición del punto de abajo
								rampa.dir_sub = (masche.mi_dir + 2) % 4;
								pnavegados[pos].conexiones[4] = true; //Guardar conexión hacia abajo en punto actual.
								pnavegados[pos].conexiones[masche.mi_dir] =
										false;
								pnavegados[masche.pos_destino].conexiones[5] =
										true;
								masche.micamino[masche.pos_mc] = 4;
							} else {
								int pos = buscar_pospunto(masche.X, masche.Y,
										masche.Z);
								//pc.printf("Rampa arriba\n");
								rampa.pbX = masche.X;
								rampa.pbY = masche.Y;
								rampa.pbZ = masche.Z; //Guardar posición actual en (pbX, pbY, pbZ)

								rampa.paX = masche.X;
								rampa.paY = masche.Y;
								rampa.paZ = masche.Z + 1;
								pnavegados[masche.pos_destino].X = masche.X;
								pnavegados[masche.pos_destino].Y = masche.Y;
								pnavegados[masche.pos_destino].Z = masche.Z + 1; //Guardar la posición del punto de arriba
								rampa.dir_sub = masche.mi_dir;
								pnavegados[pos].conexiones[5] = true; //Guardar conexión hacia abajo en punto actual.
								pnavegados[pos].conexiones[masche.mi_dir] =
										false;
								pnavegados[masche.pos_destino].conexiones[4] =
										true;
								masche.micamino[masche.pos_mc] = 5;
							}
							masche.mi_dir = masche.micamino[masche.pos_mc];
						}
					} else {
						//ENCODER(38, 0.6, true); original
            //pc.printf("Avanzo\n");
            moveTileForward();
            //ENCODER(38, 0.6, true);
					}
				} while (subir_bajar);

				//obstaculo aca

				//pc.printf("Correccion\n");
				//correccion();
        correccion_maxi();
				//pc.printf("Deje de corregir\n");
				if (detenido) {
					pnavegados[masche.pos_destino].f_negro = true;
          moveTileBackward();
          //ENCODER(diferencia, -0.5, false);
					estado = est_calcular_dest;
				} else {

					//dist_n = ultra_n.distance();

          /*if (dist_n < 20) {
						while (dist_n != 6) {
							dist_n = ultra_n.distance();
							wait(0.1f);
							if (dist_n > 6) {
								vel(0.4f, 0.4f);
							} else if (dist_n < 6) {
								vel(-0.4f, -0.4f);
							}
						}
					}
					vel(0, 0);*/

					masche.pos_mc++;

					if (masche.mi_dir == 0) {
						masche.Y++;
					} else if (masche.mi_dir == 1) {
						masche.X++;
					} else if (masche.mi_dir == 2) {
						masche.Y--;
					} else if (masche.mi_dir == 3) {
						masche.X--;
					} else if (masche.mi_dir == 4) {
						masche.Z--;
					} else {
						masche.Z++;
					}

					int pos_p = buscar_pospunto(masche.X, masche.Y, masche.Z);

					if (!pnavegados[pos_p].hay_vict) {
						if (vict_izq) {
              //pc.printf("Victima a la izq\n");
							pnavegados[pos_p].hay_vict = true;
							vict_izq = 0;
						}
						if (vict_der) {
              //pc.printf("Victima a la der\n");
							pnavegados[pos_p].hay_vict = true;
							vict_der = 0;
						}
					} else {
            vict_izq = 0;
            vict_der = 0;
					}
					if (masche.pos_mc < pnavegados[masche.pos_destino].pond) {
						sub_estado = sub_calculo_giro;
					} else {
						if (masche.pos_destino) {
							estado = est_add_info;
						} else {
							estado = est_fin_lab;
							//pc.printf("Fin Laberinto, Vamo lo 3,14 bes \n");
							//Serial.println("Fin laberinto");
							////Serial.println("VAMOS LA PUTA MADRE!");
						}
					}
				}
			}
		}
		while (estado == est_fin_lab){

        DigitalOut(LED1,a);
        wait_ms(400);
        a = !a;
        wait_ms(400);
    }
		//led = 1;

  }
}                            //llave int main
