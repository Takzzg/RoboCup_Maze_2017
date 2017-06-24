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
#include "QEI.h"

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
  int dist_izq = 1305; //Number of pulses to travel.
  int dist_der = 1343;

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
 * [turn_left90 makes a 90 degree turn to the left]
 */
void turn_left90(){
  turn_90(-1);
}
/**
 * [turn_right90 makes a 90 degree turn to the right]
 */
void turn_right90(){
  turn_90(1);
}
/**
 * [moveTileForward makes the robot advance one tile]
 */
void moveTileForward(){
  moveTile(1);
}
/**
 * [moveTileBackward makes the robot go backwards one tile]
 */
void moveTileBackward(){
  moveTile(-1);
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
         turn_right();
         x++;
         break;
       case 'W':
         turn_left();
         x--;
         break;
       case 'S':
         moveTileBackward();
         reverse = true;
         y++;
       }
       break;
 
     case 'E':
       switch (destination) {
       case 'N':
         turn_left();
         y--;
         break;
       case 'S':
         turn_right();
         y++;
         break;
       case 'W':
         moveTileBackward();
         reverse = true;
         x--;
       }
       break;
 
     case 'S':
       switch (destination) {
       case 'N':
         moveTileBackward();
         reverse = true;
         y--;
         break;
       case 'E':
         turn_left();
         x++;
         break;
       case 'W':
         turn_right();
         x--;
       }
       break;
 
     case 'W':
       switch (destination) {
       case 'N':
         turn_right();
         y--;
         break;
       case 'E':
         moveTileBackward();
         reverse = true;
         x++;
         break;
       case 'S':
         turn_left();
         y++;
       }
     }
     if (!reverse) dir = destination;
   }
   if(!reverse) moveTileForward();
 }
 //--------------------------------------------- MOVEMENT ------------------------------------------------
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
   for(unsigned int i = 0; i < arena.at(z).size(); i++){
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
  * [addLayer increments the arena a preseted size (?)]
  * @param axis [direction in which the arena grows]
  * @param Y    [amount of cells the arena grows (?)]
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
   x++;
   for(unsigned int i = 0; i < arena.at(z).size(); i++){
     addLayer('x', i);
   }
   for(unsigned int i = 0; i < arena.at(z).size(); i++){
     for(int j = arena.at(z).at(i).size() - 1; j > 0; j--){
       assignCells(&arena.at(z).at(i).at(j), arena.at(z).at(i).at(j-1));
     }
   }
   for(unsigned int i = 0; i < arena.at(z).size(); i++){
     initCells(&arena.at(z).at(i).at(0), z, i, 0);
   }
 }
 else if (axis == 'y') {
   y++;
   addLayer('y');

   for(int i = arena.at(z).size() - 1; i > 0; i--){
     arena.at(z).at(i) = arena.at(z).at(i-1);
   }
   for(unsigned int i = 0; i < arena.at(z).size(); i++){
     for(unsigned int j = 0; j < arena.at(z).at(i).size(); j++){
       arena.at(z).at(i).at(j).y++;//we fix the Y coordinate
     }
   }
   arena.at(z).at(0).clear();
   arena.at(z).at(0).resize(x + 1);
   for(unsigned int i = 0; i < x + 1; i++){
     initCells(&arena.at(z).at(arena.at(z).size() - 1).at(i), z, arena.at(z).size() - 1, i);
   }
 }
}
 //----------------------------------------------- ARENA ------------------------------------------------
 //---------------------------------------------- DIJKSTRA ------------------------------------------------
 Cell lastCheckpoint; //The last checkpoint visited
 vector<Cell> history;
 vector<vector<vector<Cell> > > arena;

/**
 * [check checks if the cell has no unexplored paths]
 * @param  y [y-axis position of the current cell being checked]
 * @param  x [x-axis position of the current cell being checked]
 * @return   [returns true if there are unexplored paths, else returns false]
 */
bool check(int y, int x) { //returns true if at least one neighbour cell isn't explored and there's no wall between the robot and it
 if (!arena.at(z).at(y).at(x).north)
   if (!arena.at(z).at(y - 1).at(x).visited) return true;
 
 if (!arena.at(z).at(y).at(x).south)
   if (!arena.at(z).at(y + 1).at(x).visited) return true;
 
 if (!arena.at(z).at(y).at(x).east)
   if (!arena.at(z).at(y).at(x + 1).visited) return true;
 
 if (!arena.at(z).at(y).at(x).west)
   if (!arena.at(z).at(y).at(x - 1).visited) return true;
 
 return false;
}
/**
 * [end chacks if the whole room has been explored]
 * @return [true if all check cases where false, else returns true]
 */
bool end() { //checks if the whole room has been visited
 for(unsigned int i = 0; i < arena.at(z).size(); i++){
   for(unsigned int j = 0; j < arena.at(z).at(i).size(); j++){
     if(arena.at(z).at(i).at(j).visited && !arena.at(z).at(i).at(j).black)
       if(check(i, j)) return false;
   }
 }
 return true;
}
/**
 * [follow tells the robot which is the best path between two points]
 * @param target [target cell the robot is heading to]
 */
 void follow(Cell target) {
   for(unsigned int i = 0; i < target.instructions.size(); i++)
     moveRobot(target.instructions.at(i));
 }
/**
 * [addOption adds an optional tile to the list of posible tiles to find the best path between two points]
 * @param  a           [description]
 * @param  compareFrom [description]
 * @param  compareTo   [description]
 * @param  addWeight   [description]
 * @return             [description]
 */
 Cell addOption(char a, Cell compareFrom, Cell compareTo, int addWeight) {
   compareTo.weight = compareFrom.weight + addWeight; //changes its weight
   compareTo.instructions = compareFrom.instructions;
   compareFrom.instructions.push_back(a);
   arena.at(compareTo.z).at(compareTo.y).at(compareTo.x) = compareTo;
   return compareTo;
 }
/**
 * [search scans the neighbour cells to the current to add them as posible paths]
 * @param compareFrom [current cell, constantly updated]
 */
 void search(Cell compareFrom) {
   Cell compareTo; //cell that's going to be visited from compareFrom
   vector<Cell> options(0);
   arena.at(z).at(compareFrom.y).at(compareFrom.x).weight = 0; //current cell's value is 0
   compareFrom.weight = 0;
   finishedFloor = end();
   int weightGain = 1;
 
   while ((!check(compareFrom.y, compareFrom.x) && !finishedFloor) || (finishedFloor && ((!compareFrom.start && z == 0) || (!compareFrom.exit && z != 0)))) { //until it starts comparing from a cell with unvisited neighbours
     arena.at(z).at(compareFrom.y).at(compareFrom.x).out = true;
     if (!compareFrom.north) { //if there's no north wall
       if (!arena.at(z).at(compareFrom.y - 1).at(compareFrom.x).out && !(arena.at(z).at(compareFrom.y - 1).at(compareFrom.x).black && arena.at(z).at(compareFrom.y - 1).at(compareFrom.x).visited)) { //if the north cell hasn't been explored
         compareTo = arena.at(z).at(compareFrom.y - 1).at(compareFrom.x); //it's used to compare
         if ((('N' != compareFrom.instructions.at(compareFrom.instructions.size() - 1)  || 'S' != compareFrom.instructions.at(compareFrom.instructions.size() - 1)) && compareFrom.weight != 0) || (compareFrom.weight == 0 && ('N' != dir || 'S' != dir)))weightGain++; //Add weight if a turn is involved
           if (compareFrom.weight+weightGain < compareTo.weight) //if traveling from the actual place there is better than from the last place
             options.push_back(addOption('N', compareFrom, compareTo, weightGain));
 
       }
     }
 
     if (!compareFrom.east && compareFrom.x != arena.at(z).at(y).size()-1) { //if there's no east wall
       if (!arena.at(z).at(compareFrom.y).at(compareFrom.x + 1).out && !(arena.at(z).at(compareFrom.y).at(compareFrom.x + 1).black && arena.at(z).at(compareFrom.y).at(compareFrom.x + 1).visited)) { //if the north cell hasn't been explored
         compareTo = arena.at(z).at(compareFrom.y).at(compareFrom.x + 1); //it's used to compare
         if ((('E' != compareFrom.instructions.at(compareFrom.instructions.size() - 1)  || 'W' != compareFrom.instructions.at(compareFrom.instructions.size() - 1)) && compareFrom.weight != 0) || (compareFrom.weight == 0 && ('E' != dir || 'W' != dir)))weightGain++; //Add weight if a turn is involved
           if (compareFrom.weight+weightGain < compareTo.weight) //if traveling from the actual place there is better than from the last place
             options.push_back(addOption('E', compareFrom, compareTo, weightGain));
 
       }
     }
 
     if (!compareFrom.south && compareFrom.y != arena.at(z).size()-1) { //if there's no south wall
       if (!arena.at(z).at(compareFrom.y + 1).at(compareFrom.x).out && !(arena.at(z).at(compareFrom.y + 1).at(compareFrom.x).black && arena.at(z).at(compareFrom.y + 1).at(compareFrom.x).visited)) { //if the north cell hasn't been explored
         compareTo = arena.at(z).at(compareFrom.y + 1).at(compareFrom.x); //it's used to compare
         if ((('S' != compareFrom.instructions.at(compareFrom.instructions.size() - 1)  || 'N' != compareFrom.instructions.at(compareFrom.instructions.size() - 1)) && compareFrom.weight != 0) || (compareFrom.weight == 0 && ('S' != dir || 'N' != dir)))weightGain++; //Add weight if a turn is involved
         if (compareFrom.weight+weightGain < compareTo.weight) //if traveling from the actual place there is better than from the last place
           options.push_back(addOption('S', compareFrom, compareTo, weightGain));
 
       }
     }
 
     if (!compareFrom.west) { //if there's no west wall
       if (!arena.at(z).at(compareFrom.y).at(compareFrom.x - 1).out && !(arena.at(z).at(compareFrom.y).at(compareFrom.x - 1).black && arena.at(z).at(compareFrom.y).at(compareFrom.x - 1).visited)) { //if the north cell hasn't been explored
         compareTo = arena.at(z).at(compareFrom.y).at(compareFrom.x - 1); //it's used to compare
         if ((('W' != compareFrom.instructions.at(compareFrom.instructions.size() - 1)  || 'E' != compareFrom.instructions.at(compareFrom.instructions.size() - 1)) && compareFrom.weight != 0) || (compareFrom.weight == 0 && ('E' != dir || 'W' != dir)))weightGain++; //Add weight if a turn is involved
         if (compareFrom.weight+weightGain < compareTo.weight) //if traveling from the actual place there is better than from the last place
           options.push_back(addOption('W', compareFrom, compareTo, weightGain));
 
       }
     }
     int bestWeight = 9999;
     for (unsigned int i = 0; i < options.size(); i++){
       if(arena.at(options.at(i).z).at(options.at(i).y).at(options.at(i).x).weight < bestWeight && !arena.at(options.at(i).z).at(options.at(i).y).at(options.at(i).x).out){
         bestWeight = arena.at(options.at(i).z).at(options.at(i).y).at(options.at(i).x).weight;
       }
     }
     for (unsigned int i = 0; i < options.size(); i++){
       if(!arena.at(options.at(i).z).at(options.at(i).y).at(options.at(i).x).out && arena.at(options.at(i).z).at(options.at(i).y).at(options.at(i).x).weight == bestWeight){
         compareFrom = arena.at(options.at(i).z).at(options.at(i).y).at(options.at(i).x);
         break;
       }
     }
   }
 
   follow(compareFrom);
 
   for (unsigned int i = 0; i < arena.at(z).size(); i++) {
     for (unsigned int j = 0; j < arena.at(z).at(y).size(); j++) {
       arena.at(z).at(i).at(j).weight = 9999;
       arena.at(z).at(i).at(j).out = false;
       arena.at(z).at(i).at(j).instructions.clear();
     }
   }
   options.clear();
   ignore = true;
 }
/**
 * [init makes a turn to the left, depending on which dir is facing to]
 */
void init() {
 char one_direction[5] = {'N', 'W', 'S', 'E', 'N'};
 for (int i = 0; i < 4; i++){
   if (dir == one_direction[i]){
     dir = one_direction[i + 1];
     return;
   }
 }
}
/**
 * [loadData reads the properties of each cell]
 */
void loadData() {
  struct Cell loadCell;
  loadCell.visited = true;
  /*if (cny70 < ???)*/loadCell.black = false;
  /*if (cny70 > ???)*/loadCell.checkpoint = false;
  int usArray[8];
  for(int i = 0; i < 8; i++){
    usArray[i] = ;
    wait(.25f);
  }
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

  //IMPORTANTE: izquierda, derecha, adelante, atrás, son relativos a la arena, no al robot. Así que, por ejemplo, si la variable dir == 'S', todo sería al revés, los sensores de adelante serían los de atrás y viceversa, lo mismo con los costados.
  //queda la información de las víctimas y otras cosas. Para testear lo único que importa es que funcione la detección de paredes.
  assignCells(&arena[getIndex(z, y, x)], loadCell);
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
       moveRobot('E');
       ignore = true;
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
       moveRobot('N');
       ignore = true;
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
       moveRobot('W');
       ignore = true;
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
       moveRobot('S');
       ignore = true;
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
  moveRobot(dir);
    loadData();
    //arena[getIndex(z, y, x)].visited = true; //sets the current cell as visited
    if (!arena.at(z).at(y).at(x).checkpoint) { //if it's not a checkpoint, it's added to the history of visited cells
      history.push_back(arena.at(z).at(y).at(x));
    }
    else { //if it is a checkpoint, sets it as the last visited checkpoint and resets the history
      lastCheckpoint = arena.at(z).at(y).at(x);
      history.clear();
    }
    if (arena.at(z).at(y).at(x).black) { //if it finds a black cell, it'll go back
      moveTileBackward();
    }
    else if (arena.at(z).at(y).at(x).exit) { //if it finds an exit
      arena.at(z).at(y).at(x).linkedFloor = ++lastFloor;
      addLayer('z');
      arena.at(lastFloor).at(y).at(x).linkedFloor = z;
      arena.at(lastFloor).at(y).at(x).linkedX = x;
      arena.at(lastFloor).at(y).at(x).linkedY = y;
      //[INSERTE FUNCIÓN PARA SUBIR RAMPAS AQUÍ] //
      x = 0;
      y = 0;
      z = lastFloor;
      ignore = true;
      arena.at(z).at(y).at(x).visited = true;
    }
}
/**
 * [expand actually expands the arena when it has to]
 */
 void expand(){
   if(x == 0){
     if(!arena.at(z).at(y).at(x).west)
       shift('x');
   }
   else if(x == arena.at(z).at(y).size() - 1){
     if(!arena.at(z).at(y).at(x).east)
       addLayer('x');
   }
   if(y == 0){
     if(!arena.at(z).at(y).at(x).north)
       shift('y');
   }
   else if(y == arena.at(z).size() - 1){
     if(!arena.at(z).at(y).at(x).south)
       addLayer('y');
   }
 }
/**
 * [explore description]
 */
 void explore() {
   loadData();
   expand();
   ignore = false;
   finishedFloor = false ;
   if (!check(y, x)) { //if the robot is stuck
     if (!arena.at(z).at(y).at(x).exit) search(arena.at(z).at(y).at(x)); //if it's stuck on a visited ramp and the room isn't finished it'll continue exploring the room
     else if (!finishedFloor) search(arena.at(z).at(y).at(x)); //got stuck on a visited ramp, but there are still some tiles left unvisited.
     else {
       //[PLEASE INSERTE FUNCIÓN PARA SUBIR O BAJAR UNA RAMPA AQUÍ PLEASEEEEE] //
       z = arena.at(z).at(y).at(x).linkedFloor;
       x = arena.at(z).at(y).at(x).linkedX;
       y = arena.at(z).at(y).at(x).linkedY;
       ignore = true;
       arena.at(z).at(y).at(x).visited = true;
       arena.at(z).at(y).at(x).exit = false;
       finishedFloor = false;
     }
   }
   else {
     changeDir();
     if (!ignore) run();
   }
 }
//------------------------------------------ ENDS DIJKSTRA ------------------------------------------------
int main() {
  arena.resize(1);
  arena.at(0).resize(1);
  arena.at(0).at(0).resize(1);
  initCells(&arena.at(0).at(0).at(0), 0, 0, 0);
  arena.at(0).at(0).at(0).start = true;
  while (1) {
    explore();
  }
}
