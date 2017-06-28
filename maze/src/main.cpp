/*
Bueno, esto es lo que hay.
Voy a dejar una lista de cosas que hay que hacer
con este código para empezar a probar el robot.

Las librerías no están.
Hay que migrar lo que está hecho con los motores,
los giros, los avances y retrocesos.
También la función loadData requiere información
de los sensores.

*incluir librerías
*agregar todo lo que se necesita para manejo de motores
  *rellenar las funciones de motores(a partir de línea 125)

*agregar todo lo que se necesita para lectura de sensores
  *rellenar la función loadData(a partir de línea 496)

*añadir una función que avance por una rampa hasta llegar a una baldosa plana.
*/
 //------------------------------------------ INITIALIZATION ------------------------------------------------
#include <mbed.h>
#include <stdlib.h>
#include <Adafruit_BNO055.h>
#include "MotorDC.h"
#include "hcsr04.h"
#include "QEI.h"
#include <vector>

imu::Quaternion quat;
imu::Vector<3> vector_euler;
imu::Vector<3> vector_acc;
imu::Vector<3> vector_euler_init;

MotorDC M_izq(PTD3, PTC9, PTB23);  //pwma, ain1, ain2---- M2
MotorDC M_der(PTA1, PTB9, PTC1);  //pwmb, bin1, bin2---- M1
DigitalOut stby(PTC8,1);

#define BNO055_SAMPLERATE_DELAY_MS (100)
#define vel_motores 0.7f
#define enc_der_chana PTB11
#define enc_der_chanb PTC11
#define enc_izq_chana PTB10
#define enc_izq_chanb PTB2 //antes ptb20
#define PULSES_PER_REVOLUTION 334

#define TRIGGER PTB3
PinName Echo [8] ={ PTC5,PTC7,PTC0,PTC3,PTC2,PTA2,PTB19,PTB18 };
HCSR04 sensor1(TRIGGER,PTC5);
HCSR04 sensor2(TRIGGER,PTC7);
HCSR04 sensor3(TRIGGER,PTC0);
HCSR04 sensor4(TRIGGER,PTC3);
HCSR04 sensor5(TRIGGER,PTC2);
HCSR04 sensor6(TRIGGER,PTA2);
HCSR04 sensor7(TRIGGER,PTB19);
HCSR04 sensor8(TRIGGER,PTB18);

//I2C i2c(PTE25,PTE24);

//Serial pc(USBTX,USBRX);

int ult_del_izq , ult_izq_del, ult_izq_tras, ult_tras_izq, ult_tras_der, ult_der_tras, ult_der_del, ult_del_der;
int cero,uno,dos,tres,cuatro,cinco,seis,siete;

QEI leftQei(enc_izq_chana, enc_izq_chanb, NC, PULSES_PER_REVOLUTION);  //chanA, chanB, index, ppr
QEI rightQei(enc_der_chana, enc_der_chanb, NC, PULSES_PER_REVOLUTION); //chanB, chanA, index, ppr

int st;
I2C  i2c(PTE25,PTE24);
Adafruit_BNO055 bno = Adafruit_BNO055(55,0x28,&i2c);
Serial pc(USBTX,USBRX);

float speed_mizq = 0;
float speed_mder = 0;

Ticker update_speed_motors;

bool finishedFloor = false; //this floor has been explored entirely
bool ignore = false;
char dir = 'N'; //points in which direction the robot is facing. it can be N, E, W or S. Always starts facing N
int x = 0, y = 0, z = 0; //robot starts in the middle of the first floor.
int lastFloor = 0;
int quit = 0;
 //------------------------------------------ INITIALIZATION ------------------------------------------------
 //---------------------------------------------- CELLS ------------------------------------------------
 typedef struct Cell {
   bool north; //sets all cells' variables to false (= blank cell)
   bool south;
   bool east;
   bool west;
   bool checkpoint;
   bool start;
   bool black;
   bool visited;
   bool exit;
   bool out;
   char victimStatus;
   vector<char> instructions;
   int weight; //sets a ceiling for the weight to compare to
   unsigned int x, y, z;
   int linkedFloor;
   int linkedX;
   int linkedY;
 }Cell;
/**
 * [initCells resets all variables to default for a new cell]
 * @param targ [target cell]
 * @param Z    [Z-axis position]
 * @param Y    [Y-axis position]
 * @param X    [X-axis position]
 */
 void initCells(Cell *targ, int Z, int Y, int X) {
   targ->z = Z;
   targ->y = Y;
   targ->x = X;
   targ->black = false;
   targ->exit = false;
   targ->north = false;
   targ->south = false;
   targ->west = false;
   targ->east = false;
   targ->linkedFloor = 0;
   targ->out = false;
   targ->weight = 999;
   targ->visited = false;
   targ->checkpoint = false;
   targ->start = false;
   targ->victimStatus = 'F';
   targ->instructions.clear();
 }
/**
 * [assignCells copies the information from one cell to another]
 * @param to   [cell the info is being copied to]
 * @param from [cell the info is being copied from]
 */
 void assignCells(Cell *to, Cell from){
   to->black = from.black;
   to->exit = from.exit;
   to->north = from.north;
   to->south = from.south;
   to->west = from.west;
   to->east = from.east;
   to->linkedFloor = from.linkedFloor;
   to->out = from.out;
   to->weight =  from.weight;
   to->visited = from.visited;
   to->checkpoint = from.checkpoint;
   to->start = from.start;
   to->victimStatus =  from.victimStatus;
   to->instructions = from.instructions;
   to->linkedX = from.linkedX;
   to->linkedY = from.linkedY;
 }
 //---------------------------------------------- CELLS ------------------------------------------------
 //--------------------------------------------- MOVEMENT ------------------------------------------------
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
     }
     else if (seis < 4 && seis != 0 && cinco < 10) {
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
/**
 * [displaySensorStatus does what the title says]
 */
void displaySensorStatus(void){
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
/**
 * [updateSpeedMotors prevents motor from randomly getting stuck]
 */
void updateSpeedMotors(){
  M_izq = speed_mizq;
  M_der = speed_mder;
}
/**
 * [init_value_3d_sensor reads the an initial value for the x-axis  to determinate wich direction the robot was facing]
 */
void init_value_3d_sensor(){
  //wait_ms(1000);
  quat = bno.getQuat();
  vector_euler_init = quat.toEuler();
  vector_euler_init.toDegrees();
  wait_ms(1000);
}
/**
 * [turn_90 makes a 90 degree turn]
 * @param direction [sets the orientation of the motors]
 */
void turn_90(int direction){
 init_value_3d_sensor();
 float eje_x = 0;
 //float eje_y = 0;
 //float eje_z = 0;
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
/**
 * [moveTile makes the robot advance or go backwards one tile]
 * @param direction [sets the orientation of the motors]
 */
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
    }
    else {
      speed_mizq = 0;
      wait_us(1);
    }
    if(abs(rightPulses) <= dist_der){
      rightPulses = rightQei.getPulses();
    }
    else {
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
/**
 * [moveRobot checks if the robot is facing the way it should to continue exploring the maze]
 * @param destination [is the target orientation in wich the robot will finish facing]
 */
 void moveRobot(char destination) {
   bool reverse = false; //instead of turning 180°, we can go back 30cm and keep the direction
   if (destination != dir) {//we have to turn or go backwards
     switch(dir) {
     case 'N':
       switch (destination) {
       case 'E':
         turn_90(1);
         x++;
         break;
       case 'W':
         turn_90(-1);
         x--;
         break;
       case 'S':
         moveTile(-1);
         reverse = true;
         y++;
       }
       break;

     case 'E':
       switch (destination) {
       case 'N':
         turn_90(-1);
         y--;
         break;
       case 'S':
         turn_90(1);
         y++;
         break;
       case 'W':
         moveTile(-1);
         reverse = true;
         x--;
       }
       break;

     case 'S':
       switch (destination) {
       case 'N':
         moveTile(-1);
         reverse = true;
         y--;
         break;
       case 'E':
         turn_90(-1);
         x++;
         break;
       case 'W':
         turn_90(1);
         x--;
       }
       break;

     case 'W':
       switch (destination) {
       case 'N':
         turn_90(1);
         y--;
         break;
       case 'E':
         moveTile(-1);
         reverse = true;
         x++;
         break;
       case 'S':
         turn_90(-1);
         y++;
       }
     }
     if (!reverse) dir = destination;
   }
   if(!reverse) moveTile(1);
   correccion();
 }
 //--------------------------------------------- MOVEMENT ------------------------------------------------
 vector<vector<vector<Cell> > > arena;
 //----------------------------------------------- ARENA ------------------------------------------------
/**
 * [addLayer increases the size of the arena when needed]
 * @param axis [in which direction the arena is going to grow]
 */
void addLayer(char axis) {
 if (axis == 'x'){
   arena.at(z).at(y).resize(arena.at(z).at(y).size() + 1);
   initCells(&arena.at(z).at(y).at(arena.at(z).at(y).size() - 1), z, y, arena.at(z).at(y).size() - 1);
 }
 else if (axis == 'y'){
   arena.at(z).resize(arena.at(z).size() + 1);//make it one unit taller
   arena.at(z).at(arena.at(z).size() - 1).resize(x + 1);//make the last layer as wide as necessary
   for(unsigned int i = 0; i < x + 1; i++){
     initCells(&arena.at(z).at(arena.at(z).size() - 1).at(i), z, arena.at(z).size() - 1, i);
   }
 }
 else if (axis == 'z'){
   arena.resize(arena.size() + 1);
   arena.at(arena.size() - 1).resize(1);
   arena.at(arena.size() - 1).at(0).resize(1);
   initCells(&arena.at(arena.size() - 1).at(0).at(0), arena.size() - 1, 0, 0);
 }
}
 /**
  * [addLayer increments a layer by one unit, making it wider]
  * @param axis [direction in which the arena grows]
  * @param Y    [the index of the layer to increment]
  */
void addLayer(char axis, int Y){
  arena.at(z).at(Y).resize(arena.at(z).at(Y).size() + 1);
  initCells(&arena.at(z).at(Y).at(arena.at(z).at(Y).size() - 1), z, Y, arena.at(z).at(Y).size() - 1);
}
/**
 * [shift moves the tiles down or to the right]
 * @param axis [in wich direction shifts the arena]
 */
void shift(char axis) {
 if (axis == 'x') {//add a layer to the left
   x++;//The robot doesn't move, but its whole map does.
   for(unsigned int i = 0; i < arena.at(z).size(); i++){//for every line
     addLayer('x', i);//we increment the width
   }
   for(unsigned int i = 0; i < arena.at(z).size(); i++){//for every line
     for(int j = arena.at(z).at(i).size() - 1; j > 0; j--){//for every cell in the line, starting from the right
       assignCells(&arena.at(z).at(i).at(j), arena.at(z).at(i).at(j-1));//we copy the contents of the cell to the left
     }
     initCells(&arena.at(z).at(i).at(0), z, i, 0);//we blank the first cell of each line(the leftmost, now unknown area of the arena)
   }
 }
 else if (axis == 'y') {
   y++;//the robot doesn't move, but its whole map does.
   addLayer('y');//add a layer to the bottom

   for(int i = arena.at(z).size() - 1; i > 0; i--){//for every line, starting from the bottom.
     arena.at(z).at(i) = arena.at(z).at(i-1);//we assign it the line above it.(this kind of messes with their coordinates by copying them)
   }
   for(unsigned int i = 0; i < arena.at(z).size(); i++){//for every line
     for(unsigned int j = 0; j < arena.at(z).at(i).size(); j++){//for every Cell in the line
       arena.at(z).at(i).at(j).y++;//we fix the Y coordinate
     }
   }
   arena.at(z).at(0).resize(x + 1);//and make it wide enough to reach the robot's x position(the robot is moving north/up)
   for(unsigned int j = 0; j <= x; j++){//for every Cell in the top line
     initCells(&arena.at(z).at(0).at(j), z, 0, j);//we blank it and give it its coordinates.
   }
 }
}
 //----------------------------------------------- ARENA ------------------------------------------------
 //---------------------------------------------- DIJKSTRA ------------------------------------------------
 Cell lastCheckpoint; //The last checkpoint visited
 vector<Cell> history;


/**
 * [check checks if the cell has no unexplored paths]
 * @param  y [y-axis position of the current cell being checked]
 * @param  x [x-axis position of the current cell being checked]
 * @return   [returns true if there are unexplored paths, else returns false]
 */
bool check(int y, int x) { //returns true if at least one neighbour cell isn't explored and there's no wall between the robot and it
 if (!arena.at(z).at(y).at(x).north)//it has a wall on its north side
   if (!arena.at(z).at(y - 1).at(x).visited) return true;//the tile above it has not been visited

 if (!arena.at(z).at(y).at(x).south)
   if (!arena.at(z).at(y + 1).at(x).visited) return true;

 if (!arena.at(z).at(y).at(x).east)
   if (!arena.at(z).at(y).at(x + 1).visited) return true;

 if (!arena.at(z).at(y).at(x).west)
   if (!arena.at(z).at(y).at(x - 1).visited) return true;

 return false;//the tile doesn't have unvisited neighbours
}
/**
 * [end chacks if the whole room has been explored]
 * @return [true if all check cases where false, else returns true]
 */
bool end() { //checks if the whole room has been visited
 for(unsigned int i = 0; i < arena.at(z).size(); i++){//for every line
   for(unsigned int j = 0; j < arena.at(z).at(i).size(); j++){//for every Cell
     if(arena.at(z).at(i).at(j).visited && !arena.at(z).at(i).at(j).black)//if it has been visited and it isn't black(black tiles may have an unvisited neighbour but it may be onli accessible from the black tile)
       if(check(i, j)) return false;//at least this cell has an unvisited neighbour, can't go back yet.
   }
 }
 return true;//we checked every single cell and didn't find unvisited tiles.
}
/**
 * [follow tells the robot which is the best path between two points]
 * @param target [target cell the robot is heading to]
 */
 void follow(Cell target) {
   for(unsigned int i = 0; i < target.instructions.size(); i++)//we iterate throug our target's set of instructions
     moveRobot(target.instructions.at(i));//and turn or move towards it
 }
/**
 * [addOption adds an optional tile to the list of posible tiles to find the best path between two points]
 * @param  a           [a character describing the direction we need to follow to get from compareFrom to compareTo]
 * @param  compareFrom [the tile whose point of view we're using]
 * @param  compareTo   [the tile we go to from compareFrom]
 * @param  addWeight   [the difference in weight between the tiles]
 * @return             [the modified compareTo we'll add to our array of options]
 */
 Cell addOption(char a, Cell compareFrom, Cell compareTo, int addWeight) {
   compareTo.weight = compareFrom.weight + addWeight; //adds the weight
   compareTo.instructions = compareFrom.instructions;//to get to compareTo, we first need to get to compareFrom
   compareFrom.instructions.push_back(a);//the last step from compareFrom to compareTo
   arena.at(compareTo.z).at(compareTo.y).at(compareTo.x) = compareTo;//we update the map
   return compareTo;//we return the modified tile
 }
/**
 * [search scans the neighbour cells to the current to add them as posible paths]
 * @param compareFrom [current cell, constantly updated]
 */
 void search(Cell compareFrom) {
   Cell compareTo; //cell that's going to be visited from compareFrom
   vector<Cell> options(0);//a list of tiles we can get to and might lead to a new tile
   arena.at(z).at(compareFrom.y).at(compareFrom.x).weight = 0; //current cell's cost to get to is 0
   compareFrom.weight = 0;
   finishedFloor = end();//if end() returns true, it means that we have to find an exit or the starting tile, because there are no more new tiles on this floor.
   int weightGain = 1;//the standard weight difference between two tiles, the difference is bigger if we need to make a turn.

   while ((!check(compareFrom.y, compareFrom.x) && !finishedFloor) || (finishedFloor && ((!compareFrom.start && z == 0) || (!compareFrom.exit && z != 0)))) { //until it starts comparing from a cell with unvisited neighbours or a starting tile
     arena.at(z).at(compareFrom.y).at(compareFrom.x).out = true;//we mark it as considered
     //we can add up to 4 options from every tile to our list
     if (!compareFrom.north) { //if there's no north wall
       if (!arena.at(z).at(compareFrom.y - 1).at(compareFrom.x).out && !(arena.at(z).at(compareFrom.y - 1).at(compareFrom.x).black && arena.at(z).at(compareFrom.y - 1).at(compareFrom.x).visited)) { //if the north cell hasn't been explored
         compareTo = arena.at(z).at(compareFrom.y - 1).at(compareFrom.x); //it's used to compare
         if ((('N' != compareFrom.instructions.at(compareFrom.instructions.size() - 1)  || 'S' != compareFrom.instructions.at(compareFrom.instructions.size() - 1)) && compareFrom.weight != 0) || (compareFrom.weight == 0 && ('N' != dir || 'S' != dir)))weightGain++; //Add weight if a turn is involved
         if (compareFrom.weight+weightGain < compareTo.weight) //if traveling from the actual place there is better than from the last place
           options.push_back(addOption('N', compareFrom, compareTo, weightGain));//we modify it and add it as an option.

       }
     }

     if (!compareFrom.east && compareFrom.x != arena.at(z).at(y).size()-1) { //if there's no east wall
       if (!arena.at(z).at(compareFrom.y).at(compareFrom.x + 1).out && !(arena.at(z).at(compareFrom.y).at(compareFrom.x + 1).black && arena.at(z).at(compareFrom.y).at(compareFrom.x + 1).visited)) { //if the north cell hasn't been explored
         compareTo = arena.at(z).at(compareFrom.y).at(compareFrom.x + 1); //it's used to compare
         if ((('E' != compareFrom.instructions.at(compareFrom.instructions.size() - 1)  || 'W' != compareFrom.instructions.at(compareFrom.instructions.size() - 1)) && compareFrom.weight != 0) || (compareFrom.weight == 0 && ('E' != dir || 'W' != dir)))weightGain++; //Add weight if a turn is involved
           if (compareFrom.weight+weightGain < compareTo.weight) //if traveling from the actual place there is better than from the last place
             options.push_back(addOption('E', compareFrom, compareTo, weightGain)); //we modify it and add it as an option.

       }
     }

     if (!compareFrom.south && compareFrom.y != arena.at(z).size()-1) { //if there's no south wall
       if (!arena.at(z).at(compareFrom.y + 1).at(compareFrom.x).out && !(arena.at(z).at(compareFrom.y + 1).at(compareFrom.x).black && arena.at(z).at(compareFrom.y + 1).at(compareFrom.x).visited)) { //if the north cell hasn't been explored
         compareTo = arena.at(z).at(compareFrom.y + 1).at(compareFrom.x); //it's used to compare
         if ((('S' != compareFrom.instructions.at(compareFrom.instructions.size() - 1)  || 'N' != compareFrom.instructions.at(compareFrom.instructions.size() - 1)) && compareFrom.weight != 0) || (compareFrom.weight == 0 && ('S' != dir || 'N' != dir)))weightGain++; //Add weight if a turn is involved
         if (compareFrom.weight+weightGain < compareTo.weight) //if traveling from the actual place there is better than from the last place
           options.push_back(addOption('S', compareFrom, compareTo, weightGain));//we modify it and add it as an option.

       }
     }

     if (!compareFrom.west) { //if there's no west wall
       if (!arena.at(z).at(compareFrom.y).at(compareFrom.x - 1).out && !(arena.at(z).at(compareFrom.y).at(compareFrom.x - 1).black && arena.at(z).at(compareFrom.y).at(compareFrom.x - 1).visited)) { //if the north cell hasn't been explored
         compareTo = arena.at(z).at(compareFrom.y).at(compareFrom.x - 1); //it's used to compare
         if ((('W' != compareFrom.instructions.at(compareFrom.instructions.size() - 1)  || 'E' != compareFrom.instructions.at(compareFrom.instructions.size() - 1)) && compareFrom.weight != 0) || (compareFrom.weight == 0 && ('E' != dir || 'W' != dir)))weightGain++; //Add weight if a turn is involved
         if (compareFrom.weight+weightGain < compareTo.weight) //if traveling from the actual place there is better than from the last place
           options.push_back(addOption('W', compareFrom, compareTo, weightGain));//we modify it and add it as an option

       }
     }
     int bestWeight = 9999;//a weight to beat
     for (unsigned int i = 0; i < options.size(); i++){//for every cell in our list of possible options
       if(arena.at(options.at(i).z).at(options.at(i).y).at(options.at(i).x).weight < bestWeight && !arena.at(options.at(i).z).at(options.at(i).y).at(options.at(i).x).out){//if we haven't checked its neighbours and it's the best weight yet
         bestWeight = arena.at(options.at(i).z).at(options.at(i).y).at(options.at(i).x).weight;//we update the weight to beat.
       }
     }
     for (unsigned int i = 0; i < options.size(); i++){
       if(!arena.at(options.at(i).z).at(options.at(i).y).at(options.at(i).x).out && arena.at(options.at(i).z).at(options.at(i).y).at(options.at(i).x).weight == bestWeight){//if it's one of the nearest tiles and we haven't checked its neighbours
         compareFrom = arena.at(options.at(i).z).at(options.at(i).y).at(options.at(i).x);//next time we'll compareFrom there.
         break;
       }
     }
     weightGain = 1;//we need to reset the weightGain or it'll continue to grow.
   }

   follow(compareFrom);//we broke out of the while and the robot has to get to its target

   for (unsigned int i = 0; i < arena.at(z).size(); i++) {//for every line of the map
     for (unsigned int j = 0; j < arena.at(z).at(y).size(); j++) {//for every cell in the line we reset its dijkstra related values
       arena.at(z).at(i).at(j).weight = 9999;//its weight
       arena.at(z).at(i).at(j).out = false;//its mark
       arena.at(z).at(i).at(j).instructions.clear();//its instructions.
     }
   }
   options.clear();//we clear our list of options
   ignore = true;//idk
 }
/**
 * [init makes a turn to the left, depending on which direction the robot is facing to]
 */
void init() {
 char one_direction[5] = {'N', 'W', 'S', 'E', 'N'};//we list all possible directions counter-clockwise.
 for (int i = 0; i < 4; i++){//for each of the first 4 directions
   if (dir == one_direction[i]){//if that's the one the robot is facing
     dir = one_direction[i + 1];//the next direction in is left to the current one
     turn_90(-1);//the robot turns to the left
     return;//we don't want to keep changing the direction
   }
 }
}
/**
 * [loadData reads the properties of each cell]
 */
void loadData() {
  Cell loadCell;//we'll make modivications in here
  loadCell.visited = true;//we mark it as visited
  /*if (cny70 < ???)*/loadCell.black = false;
  /*if (cny70 > ???)*/loadCell.checkpoint = false;
  int us0 = sensor1.distance();
  wait(.25f);
  int us1 = sensor2.distance();
  wait(.25f);
  int us2 = sensor3.distance();
  wait(.25f);
  int us3 = sensor4.distance();
  wait(.25f);
  int us4 = sensor5.distance();
  wait(.25f);
  int us5 = sensor6.distance();
  wait(.25f);
  int us6 = sensor7.distance();
  wait(.25f);
  int us7 = sensor8.distance();
  wait(.25f);
  int usArray[8] = {us0,us1,us2,us3,us4,us5,us6,us7};//the values of every ultrasonic sensor in centimeters
  int vDir = 0;
  switch(dir){
    case 'S':
      vDir = 4;
      break;
    case 'W':
      vDir = 6;
      break;
    case 'E':
      vDir = 2;
  }
  loadCell.north = usArray[(0 + vDir) % 8] < 8 && usArray[(7 + vDir) % 8];
  loadCell.south = usArray[(3 + vDir) % 8] < 8 && usArray[(4 + vDir) % 8];
  loadCell.east = usArray[(5 + vDir) % 8] < 8 && usArray[(6 + vDir) % 8];
  loadCell.west = usArray[(1 + vDir) % 8] < 8 && usArray[(2 + vDir) % 8];
  /*if (inclinación ???)*/loadCell.exit = false;
  assignCells(&arena.at(z).at(y).at(x), loadCell);//we apply the changes made to loadCell into the map
}
/**
 * [changeDir makes the robot face the way it has to]
 */
void changeDir() { //changes robot's direction
 bool f = false;
 switch (dir) {
 case 'N': //it's going north
   if (!arena.at(z).at(y).at(x).east) { //if the east cell isn't explored and has no wall in between
     if (!arena.at(z).at(y).at(x + 1).visited) {
       moveRobot('E');//we turn and go ahead
       ignore = true;//idk
       f = true;
     }
   }
   if (!f) { //else if the north cell hasn't been explored and has no wall between
     if (arena.at(z).at(y).at(x).north) init();
     else if (arena.at(z).at(y - 1).at(x).visited)init();
   }
   break;

 case 'W': //it's going left
   if (!arena.at(z).at(y).at(x).north) { //if the north cell isn't explored and has no wall in between
     if (!arena.at(z).at(y - 1).at(x).visited) {
       moveRobot('N');//we turn and move ahead
       ignore = true;//idk
       f = true;
     }
   }
   if (!f) { //else if the west cell hasn't been explored and has no wall between
     if (arena.at(z).at(y).at(x).west) init();
     else if (arena.at(z).at(y).at(x - 1).visited)init();
   }
   break;

 case 'S': //it's going down
   if (!arena.at(z).at(y).at(x).west) { //if the north cell isn't explored and has no wall in between
     if (!arena.at(z).at(y).at(x - 1).visited) {
       moveRobot('W');//we turn and go ahead
       ignore = true;//idk
       f = true;
     }
   }
   if (!f) { //else if the south cell hasn't been explored and has no wall between
     if (arena.at(z).at(y).at(x).south) init();
     else if (arena.at(z).at(y + 1).at(x).visited)init();
   }
   break;

 default: //it's going right
   if (!arena.at(z).at(y).at(x).south) { //if the north cell isn't explored and has no wall in between
     if (!arena.at(z).at(y + 1).at(x).visited) {
       moveRobot('S');//we turn and go ahead
       ignore = true;//idk
       f = true;
     }
   }
   if (!f) { //else if the east cell hasn't been explored and has no wall between
     if (arena.at(z).at(y).at(x).east) init();
     else if (arena.at(z).at(y).at(x + 1).visited)init();
   }
 }
}
/**
 * [run tells the robot what to do next]
 */
void run() {
  moveRobot(dir);//we go ahead
    loadData();//we update this tile's info in the map according to the maze
    if (!arena.at(z).at(y).at(x).checkpoint) { //if it's not a checkpoint, it's added to the history of visited cells
      history.push_back(arena.at(z).at(y).at(x));
    }
    else { //if it is a checkpoint, sets it as the last visited checkpoint and resets the history
      lastCheckpoint = arena.at(z).at(y).at(x);
      history.clear();
    }
    if (arena.at(z).at(y).at(x).black) { //if it finds a black cell, it'll go back
      moveTile(-1);
      switch(dir){
        case 'W':
          x++;
          break;
        case 'E':
          x--;
          break;
        case 'N':
          y++;
          break;
        case 'S':
          y--;
      }
    }
    else if (arena.at(z).at(y).at(x).exit) { //if it finds an exit
      arena.at(z).at(y).at(x).linkedFloor = ++lastFloor;//we increment the last floor and set it as this tile's ramp's destination floor
      addLayer('z');//we add a layer to the bottom of the map
      arena.at(lastFloor).at(y).at(x).linkedFloor = z;//this new, higher tile will have the current floor as its ramp's destination floor
      arena.at(lastFloor).at(y).at(x).linkedX = x;//we write in the tile above that our current coordinates are//TODO: UPDATE THIS IN THE MAP SHIFTS
      arena.at(lastFloor).at(y).at(x).linkedY = y;//
      //[INSERTE FUNCIÓN PARA SUBIR RAMPAS AQUÍ] //
      x = 0;//next floor's starting tile will be (0;0)
      y = 0;
      z = lastFloor;//we update the robot's coordinates
      ignore = true;//idk
      arena.at(z).at(y).at(x).visited = true;//we mark that new floor's new tile as visited
    }
}
/**
 * [expand actually expands the arena when it has to]
 */
 void expand(){
   if(x == 0){//we're on the left edge
     if(!arena.at(z).at(y).at(x).west)//and there's no left wall
       shift('x');//we shift the map
   }
   else if(x == arena.at(z).at(y).size() - 1){//we're on the right edge of this line
     if(!arena.at(z).at(y).at(x).east)//and there's no right wall
       addLayer('x');//we add a unit to this line
   }
   if(y == 0){//we're on the top of the map
     if(!arena.at(z).at(y).at(x).north)//and there's no wall to the north
       shift('y');//we shift the map
   }
   else if(y == arena.at(z).size() - 1){//we're on the bottom of the map
     if(!arena.at(z).at(y).at(x).south)//and there's no bottom wall
       addLayer('y');//we add a new line to the bottom of the map
   }
 }
/**
 * [explore description]
 */
 void explore() {
   loadData();//we take info from the sensors and load it into this tile in the map
   expand();//we expand the map if necessary
   ignore = false;//idk
   finishedFloor = false;//we reset this variable, it weill be set to true if the robot gets stuck, calls search() and search() calls end() and end() returns true. whew!
   if (!check(y, x)) { //if the robot is stuck
     if (!arena.at(z).at(y).at(x).exit) search(arena.at(z).at(y).at(x)); //if it's stuck on a visited ramp and the room isn't finished it'll continue exploring the room
     else if (!finishedFloor) search(arena.at(z).at(y).at(x)); //got stuck on a visited ramp, but there are still some tiles left unvisited.
     else {//we're back on the ramp after completing this floor
       //[PLEASE INSERTE FUNCIÓN PARA SUBIR O BAJAR UNA RAMPA AQUÍ PLEASEEEEE] //
       z = arena.at(z).at(y).at(x).linkedFloor;//we change the coordinates to the coordinates of the tile on the other end of the ramp
       x = arena.at(z).at(y).at(x).linkedX;
       y = arena.at(z).at(y).at(x).linkedY;
       ignore = true;//idk
       //arena.at(z).at(y).at(x).visited = true;
       arena.at(z).at(y).at(x).exit = false;//we delete the exit marker on this tile
       finishedFloor = false;//we are on a different floor so we reset this flag.
     }
   }
   else {//the robot is not stuck
     changeDir();//if needed, we'll turn
     if (!ignore) run();//and move forward
   }
 }
//------------------------------------------ ENDS DIJKSTRA ------------------------------------------------
int main() {
  update_speed_motors.attach_us(&updateSpeedMotors,500.0f); //500us
  moveTile(1);
  arena.resize(1);//we make the arena one floor high
  arena.at(0).resize(1); //one line
  arena.at(0).at(0).resize(1);//one column
  initCells(&arena.at(0).at(0).at(0), 0, 0, 0);//we set its coordinates
  arena.at(0).at(0).at(0).start = true;//and mark the only known tile as the start
  wait(1.5f);
  while (1) {
    explore();//we explore the maze
  }
}
