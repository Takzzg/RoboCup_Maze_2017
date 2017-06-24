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

int ySize = 1; //maximum expected width of the arena
int xSize = 1; //maximum expected height of the arena
int zSize = 1; //maximum expected amount of floors
bool finishedFloor = false; //this floor has been explored entirely
bool ignore = false;
char dir = 'N'; //points in which direction the robot is facing. it can be N, E, W or S. Always starts facing N
int x = 0, y = 0, z = 0; //robot starts in the middle of the first floor.
int lastFloor = 0;
int quit = 0;
 //------------------------------------------ INITIALIZATION ------------------------------------------------
 //---------------------------------------------- CELLS ------------------------------------------------
  struct Cell {
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
  char *instructions;
  int instructionsSize;
  int weight; //sets a ceiling for the weight to compare to
  int x, y, z;
  int linkedFloor;
};

void initCells(struct Cell *targ, int Z, int Y, int X) {
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
  targ->instructionsSize = 0;
  targ->visited = false;
  targ->checkpoint = false;
  targ->start = false;
  targ->victimStatus = 'F';
  targ->instructions = new char[0];
}

void assignCells(struct Cell *to, struct Cell from){
  to->black = from.black;
  to->exit = from.exit;
  to->north = from.north;
  to->south = from.south;
  to->west = from.west;
  to->east = from.east;
  to->linkedFloor = from.linkedFloor;
  to->out = from.out;
  to->weight =  from.weight;
  to->instructionsSize = from.instructionsSize;
  to->visited = from.visited;
  to->checkpoint = from.checkpoint;
  to->start = from.start;
  to->victimStatus =  from.victimStatus;
  for(int i = 0; i < from.instructionsSize; i++)
    to->instructions[i] = from.instructions[i];
}
 //---------------------------------------------- CELLS ------------------------------------------------
 //--------------------------------------------- MOVEMENT ------------------------------------------------
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

void updateSpeedMotors(){
  M_izq = speed_mizq;
  M_der = speed_mder;
}

int getIndex(int Z, int Y, int X, int ySum, int xSum) {
  return X + Y * (xSize + xSum) + Z * (xSize + xSum) * (ySize + ySum);
}

int getIndex(int Z, int Y, int X) {
  return X + Y * xSize + Z * xSize * ySize;
}

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

void turn_left90(){
  turn_90(-1);
}

void turn_right90(){
  turn_90(1);
}

void moveTileForward(){
  moveTile(1);
}

void moveTileBackward(){
  moveTile(-1);
}

void moveRobot(char destination) {
  bool reverse = false; //instead of turning 180°, we can go back 30cm and keep the direction

  if (destination != dir) {//we have to turn or go backwards
    switch(dir) {
    case 'N':
      switch (destination) {
      case 'E':
        turn_right90();
        x++;
        break;
      case 'W':
        turn_left90();
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
        turn_left90();
        y--;
        break;
      case 'S':
        turn_right90();
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
        turn_left90();
        x++;
        break;
      case 'W':
        turn_right90();
        x--;
      }
      break;

    case 'W':
      switch (destination) {
      case 'N':
        turn_right90();
        y--;
        break;
      case 'E':
        moveTileBackward();
        reverse = true;
        x++;
        break;
      case 'S':
        turn_left90();
        y++;
      }
    }
    if (!reverse) dir = destination;
  }
  if(!reverse) moveTileForward();
}
 //--------------------------------------------- MOVEMENT ------------------------------------------------
 //----------------------------------------------- ARENA ------------------------------------------------
struct Cell lastCheckpoint; //The last checkpoint visited
int historySize = 1; //keeps track of the amount of tiles in the array
struct Cell *history = new struct Cell[historySize];
struct Cell *arena = new struct Cell[zSize * ySize * xSize];

struct Cell addOption(char a, struct Cell compareFrom, struct Cell compareTo, int addWeight) {
  compareTo.weight = compareFrom.weight + addWeight; //changes its weight
  compareTo.instructionsSize = compareFrom.instructionsSize + 1;
  compareTo.instructions = new char[1 + compareFrom.instructionsSize];
  for (int i  = 0; i < compareFrom.instructionsSize; i++) {
    compareTo.instructions[i] = compareFrom.instructions[i];
  }
  compareTo.instructions[compareTo.instructionsSize-1] = a; //an extra instruction to get to compareTo is added
  arena[getIndex(compareTo.z, compareTo.y, compareTo.x)] = compareTo; //the matrix is updated with the newest information
  return compareTo;
}

void addLayer(char axis) {
  if (axis == 'x') { //we have to add a layer to the right of the arena
    struct Cell *tempArena = new struct Cell[(xSize + 1) * ySize * zSize]; //we create another matrix which is one unit wider.
    for (int h = 0; h < zSize; h++) {
      for (int i = 0; i < ySize; i++) {
        for (int j = 0; j < xSize+1; j++) {
          initCells(&tempArena[getIndex(h, i, j, 0, 1)], h, i, j);//we initialize the new cells, this gives them their coordinates and deletes resitual data.
        }
      }
    }
    for (int h = 0; h < zSize; h++) { //for every layer (z axis)
      for (int i = 0; i < ySize; i++) { //for every line in the arena
        for (int j  = 0; j < xSize; j++) { //for every column in the arena
          tempArena[getIndex(h, i, j, 0, 1)] = arena[getIndex(h, i, j)]; //we copy from arena to tempArena.
        }
      }
    }
    arena = new struct Cell[ySize * ++xSize * zSize]; //we resize the arena and increment the width.
    for (int  i = 0; i < ySize * xSize * zSize; i++) {
      arena[i] = tempArena[i];
    }
    delete[] tempArena;
  }
  else if (axis == 'y') { //we have to add a layer to the bottom of the arena
    struct Cell *tempArena = new struct Cell[(ySize + 1) * xSize * zSize];
    for (int h = 0; h < zSize; h++) {
      for (int i = 0; i < ySize+1; i++) {
        for (int j = 0; j < xSize; j++) {
          initCells(&tempArena[getIndex(h, i, j, 1, 0)], h, i, j);//we initialize the new cells, this gives them their coordinates and deletes resitual data.
        }
      }
    }
    for (int h = 0; h < zSize; h++) { //for every layer (z axis)
      for (int i = 0; i < ySize; i++) { //for every line in the arena
        for (int j  = 0; j < xSize; j++) { //for every column in the arena
          tempArena[getIndex(h, i, j, 1, 0)] = arena[getIndex(h, i, j)]; //we copy from arena to tempArena.
        }
      }
    }
    arena = new struct Cell[++ySize * xSize * zSize];//we resize the arena and increment the height.
    for (int  i = 0; i < ySize * xSize * zSize; i++) {
      arena[i] = tempArena[i];//we copy from tempArena to arena
    }
    delete[] tempArena;
  }
  else if (axis == 'z'){
    struct Cell *tempArena = new struct Cell[(zSize + 1) * xSize * ySize];
    for(int i = 0; i < zSize * xSize * ySize; i++){
      tempArena[i] = arena[i];//We copy all the previous floors to tempArena. The newest floor is not initialized.
    }
    for (int i = 0; i < ySize; i++) {
      for (int j = 0; j < xSize; j++) {
        initCells(&tempArena[getIndex(zSize, i, j)], zSize, i, j);//We initialize the latest floor.
      }
    }
    arena = new struct Cell[xSize * ySize * ++zSize];//we resize the arena and increment the depth.
    for(int i = 0; i < zSize * xSize * ySize; i++){
      arena[i] = tempArena[i];//we copy from tempArena to arena
    }
    delete[] tempArena;
  }
}

void shift(char axis) {
  if (axis == 'x') {//add a layer to the left
    x++;
    addLayer('x');//add a layer to the right, then
    for (int h = 0; h < zSize; h++) {
      for (int i = 0; i < ySize; i++) {
        for (int j = xSize-1; j > 0; j--) {//for every cell, from right to left
          assignCells(&arena[getIndex(h, i, j)], arena[getIndex(h, i, j - 1)]);//we copy ALMOST(not the coordinates please!) all the data from the cell to the left of it.
        }
      }
    }
    for (int h = 0; h < zSize; h++) {
      for (int i = 0; i < ySize; i++) {
        initCells(&arena[getIndex(h, i, 0, 0, 1)], h, i, 0);//then initialize(we wipe it) the first column because it has old data which is now to the right.
      }
    }
  }
  else if (axis == 'y') {
    y++;
    addLayer('y');
    for (int h = 0; h < zSize; h++) {
      for (int i = ySize-1; i > 0; i--) {
        for (int j = 0; j < xSize; j++) {
          assignCells(&arena[getIndex(h, i, j)], arena[getIndex(h, i - 1, j)]);
        }
      }
    }
    for (int h = 0; h < zSize; h++) {
      for (int j = 0; j < xSize; j++) {
          initCells(&arena[getIndex(h, 0, j, 0, 1)], h, 0, j);
      }
    }
  }
}
 //----------------------------------------------- ARENA ------------------------------------------------
 //---------------------------------------------- DIJKSTRA ------------------------------------------------
bool check(int y, int x) { //returns true if at least one neighbour cell isn't explored and there's no wall between the robot and it
  if (!arena[getIndex(z, y, x)].north)
    if (!arena[getIndex(z, y - 1, x)].visited) return true;

  if (!arena[getIndex(z, y, x)].south)
    if (!arena[getIndex(z, y + 1, x)].visited) return true;

  if (!arena[getIndex(z, y, x)].east)
    if (!arena[getIndex(z, y, x + 1)].visited) return true;

  if (!arena[getIndex(z, y, x)].west)
    if (!arena[getIndex(z, y, x-1)].visited) return true;

  return false;
}

bool end() { //checks if the whole room has been visited
  for (int i = 0; i < ySize; i++) {
    for (int j = 0; j < xSize; j++) {
      if (arena[getIndex(z, i, j)].visited && !arena[getIndex(z, i, j)].black)
        if (check(i, j)) return false;
    }
  }
  return true;
}

void follow(struct Cell target) {
  for (int i = 0; i < target.instructionsSize; i++) {
    moveRobot(target.instructions[i]);
  }
}

void search(struct Cell compareFrom) {
  struct Cell compareTo; //cell that's going to be visited from compareFrom
  int optionsSize = 0;
  struct Cell *options = new struct Cell[optionsSize];
  options = new struct Cell[0]; //array which stores the cells that later can be used to compare from
  arena[getIndex(z, compareFrom.y, compareFrom.x)].weight = 0; //current cell's value is 0
  compareFrom.weight = 0;
  finishedFloor = end();
  int weightGain = 1;

  while ((!check(compareFrom.y, compareFrom.x) && !finishedFloor) || (finishedFloor && ((!compareFrom.start && z == 0) || (!compareFrom.exit && z != 0)))) { //until it starts comparing from a cell with unvisited neighbours
    arena[getIndex(z, compareFrom.y, compareFrom.x)].out = true; //the cell is marked on the matrix

    if (!compareFrom.north) { //if there's no north wall
      if (!arena[getIndex(z, compareFrom.y-1, compareFrom.x)].out && !(arena[getIndex(z, compareFrom.y-1, compareFrom.x)].black && arena[getIndex(z, compareFrom.y-1, compareFrom.x)].visited)) { //if the north cell hasn't been explored
        compareTo = arena[getIndex(z, compareFrom.y-1, compareFrom.x)]; //it's used to compare
        if ((('N' != compareFrom.instructions[compareFrom.instructionsSize - 1]  || 'S' != compareFrom.instructions[compareFrom.instructionsSize - 1]) && compareFrom.weight != 0) || (compareFrom.weight == 0 && ('N' != dir || 'S' != dir)))weightGain++; //Add weight if a turn is involved
        if (compareFrom.weight+weightGain < compareTo.weight) { //if traveling from the actual place there is better than from the last place
          struct Cell *tempOptions = new struct Cell[++optionsSize];
          for (int i  = 0; i < optionsSize-1; i++) {
            tempOptions[i] = options[i];
          }
          options = new struct Cell[optionsSize];
          for (int i  = 0; i < optionsSize-1; i++) {
            options[i] = tempOptions[i];
          }
          delete[] tempOptions;

          options[optionsSize] = addOption('N', compareFrom, compareTo, weightGain);
        }
      }
    }

    if (!compareFrom.east && compareFrom.x != xSize-1) { //if there's no east wall
      if (!arena[getIndex(z, compareFrom.y, compareFrom.x+ 1)].out && !(arena[getIndex(z, compareFrom.y, compareFrom.x+ 1)].black && arena[getIndex(z, compareFrom.y, compareFrom.x+ 1)].visited)) { //the right cell hasn't been explored yet
        compareTo =  arena[getIndex(z, compareFrom.y, compareFrom.x+ 1)]; //used to comapre
        if ((('E' != compareFrom.instructions[compareFrom.instructionsSize - 1]  || 'W' != compareFrom.instructions[compareFrom.instructionsSize - 1]) && compareFrom.weight != 0) || (compareFrom.weight == 0 && ('E' != dir || 'W' != dir)))weightGain++; //Add weight if a turn is involved
        if (compareFrom.weight+weightGain < compareTo.weight) { //if traveling from the actual place there is better than from the last place
          struct Cell *tempOptions = new struct Cell[++optionsSize];
          for (int i  = 0; i < optionsSize-1; i++) {
            tempOptions[i] = options[i];
          }
          options = new struct Cell[optionsSize];
          for (int i  = 0; i < optionsSize-1; i++) {
            options[i] = tempOptions[i];
          }
          delete[] tempOptions;

          options[optionsSize] = addOption('E', compareFrom, compareTo, weightGain);
        }
      }
    }

    if (!compareFrom.south && compareFrom.y != ySize-1) { //if there's no south wall
      if (!arena[getIndex(z, compareFrom.y+ 1, compareFrom.x)].out && !(arena[getIndex(z, compareFrom.y+ 1, compareFrom.x)].black && arena[getIndex(z, compareFrom.y+ 1, compareFrom.x)].visited)) { //if the south cell hasn't been explored yet
        compareTo = arena[getIndex(z, compareFrom.y+ 1, compareFrom.x)]; //used to compare
        if ((('S' != compareFrom.instructions[compareFrom.instructionsSize - 1]  || 'N' != compareFrom.instructions[compareFrom.instructionsSize - 1]) && compareFrom.weight != 0) || (compareFrom.weight == 0 && ('S' != dir || 'N' != dir)))weightGain++; //Add weight if a turn is involved
        if (compareFrom.weight+weightGain < compareTo.weight) { //if traveling from the actual place there is better than from the last place
          struct Cell *tempOptions = new struct Cell[++optionsSize];
          for (int i  = 0; i < optionsSize-1; i++) {
            tempOptions[i] = options[i];
          }
          options = new struct Cell[optionsSize];
          for (int i  = 0; i < optionsSize-1; i++) {
            options[i] = tempOptions[i];
          }
          delete[] tempOptions;

          options[optionsSize] = addOption('S', compareFrom, compareTo, weightGain);
        }
      }
    }

    if (!compareFrom.west) { //if there's no west wall
      if (!arena[getIndex(z, compareFrom.y, compareFrom.x-1)].out && !(arena[getIndex(z, compareFrom.y, compareFrom.x-1)].black && arena[getIndex(z, compareFrom.y, compareFrom.x-1)].visited)) { //if the west cell hasn't been explored yet
        compareTo = arena[getIndex(z, compareFrom.y, compareFrom.x-1)]; //used to compareSe usa para comparar
        if ((('W' != compareFrom.instructions[compareFrom.instructionsSize - 1]  || 'E' != compareFrom.instructions[compareFrom.instructionsSize - 1]) && compareFrom.weight != 0) || (compareFrom.weight == 0 && ('E' != dir || 'W' != dir)))weightGain++; //Add weight if a turn is involved
        if (compareFrom.weight+weightGain < compareTo.weight) { //if traveling from the actual place there is better than from the last place
          struct Cell *tempOptions = new struct Cell[++optionsSize];
          for (int i  = 0; i < optionsSize-1; i++) {
            tempOptions[i] = options[i];
          }
          options = new struct Cell[optionsSize];
          for (int i  = 0; i < optionsSize-1; i++) {
            options[i] = tempOptions[i];
          }
          delete[] tempOptions;

          options[optionsSize] = addOption('W', compareFrom, compareTo, weightGain);
        }
      }
    }

    int bestWeight = 9999;
    for (int i = 0; i < optionsSize; i++) { //reads the array to find the lower cost cells
      if (arena[getIndex(z, options[i].y, options[i].x)].weight < bestWeight && !arena[getIndex(z, options[i].y, options[i].x)].out) {
        bestWeight = arena[getIndex(z, options[i].y, options[i].x)].weight;
      }
    }

    for (int i = 0; i < optionsSize; i++) {
      if (!arena[getIndex(z, options[i].y, options[i].x)].out && arena[getIndex(z, options[i].y, options[i].x)].weight == bestWeight) {
        compareFrom = arena[getIndex(z, options[i].y, options[i].x)];
        break;
      }
    }
  }

  follow(compareFrom);

  for (int i = 0; i < ySize; i++) {
    for (int j = 0; j < xSize; j++) {
      arena[getIndex(z, i, j)].weight = 9999;
      arena[getIndex(z, i, j)].out = false;
      arena[getIndex(z, i, j)].instructionsSize = 0;
      delete[] arena[getIndex(z, i, j)].instructions;
    }
  }
  ignore = true;
  delete[] options;
}

void init() {
  char one_direction[5] = {'N', 'W', 'S', 'E', 'N'};
  for (int i = 0; i < 4; i++)
    if (dir == one_direction[i]){ dir = one_direction[i + 1];
      return;
    }
}

void loadData() {
  struct Cell loadCell;
  loadCell.visited = true;
  /*if (cny70 < ???)*/loadCell.black = true;
  /*if (cny70 > ???)*/loadCell.checkpoint = true;
  /*if (sensores ultrasonido frontales detectan pared)*/loadCell.north = true;
  /*if (sensores ultrasonido derechos detectan pared)*/loadCell.east = true;
  /*if (sensores ultrasonido traseros detectan pared)*/loadCell.south = true;
  /*if (sensores ultrasonido izquierdos detectan pared)*/loadCell.west = true;
  /*if (inclinación ???)*/loadCell.exit = true;

  //IMPORTANTE: izquierda, derecha, adelante, atrás, son relativos a la arena, no al robot. Así que, por ejemplo, si la variable dir == 'S', todo sería al revés, los sensores de adelante serían los de atrás y viceversa, lo mismo con los costados.
  //queda la información de las víctimas y otras cosas. Para testear lo único que importa es que funcione la detección de paredes.
  assignCells(&arena[getIndex(z, y, x)], loadCell);
}

void changeDir() { //changes robot's direction
  bool f = false;
  switch (dir) {
  case 'N': //it's going north
    if (!arena[getIndex(z, y, x)].east) { //if the east cell isn't explored and has no wall in between
      if (!arena[getIndex(z, y, x+ 1)].visited) {
        moveRobot('E');
        ignore = true;
        f = true;
      }
    }
    if (!f) { //else if the north cell hasn't been explored and has no wall between
      if (arena[getIndex(z, y, x)].north) init();
      else if (arena[getIndex(z, y-1, x)].visited)init();
    }
    break;

  case 'W': //it's going left
    if (!arena[getIndex(z, y, x)].north) { //if the north cell isn't explored and has no wall in between
      if (!arena[getIndex(z, y-1, x)].visited) {
        moveRobot('N');
        ignore = true;
        f = true;
      }
    }
    if (!f) { //else if the west cell hasn't been explored and has no wall between
      if (arena[getIndex(z, y, x)].west) init();
      else if (arena[getIndex(z, y, x-1)].visited)init();
    }
    break;

  case 'S': //it's going down
    if (!arena[getIndex(z, y, x)].west) { //if the west cell isn't explored and has no wall in between
      if (!arena[getIndex(z, y, x-1)].visited) {
        moveRobot('W');
        ignore = true;
        f = true;
      }
    }
    if (!f) { //else if the south cell hasn't been explored and has no wall between
      if (arena[getIndex(z, y, x)].south) init();
      else if (arena[getIndex(z, y+ 1, x)].visited)init();
    }
    break;

  default: //it's going right
    if (!arena[getIndex(z, y, x)].south) { //if the south cell isn't explored and has no wall in between
      if (!arena[getIndex(z, y+ 1, x)].visited) {
        moveRobot('S');
        ignore = true;
        f = true;
      }
    }
    if (!f) { //else if the east cell hasn't been explored and has no wall between
      if (arena[getIndex(z, y, x)].east) init();
      else if (arena[getIndex(z, y, x+ 1)].visited)init();
    }
  }
}

void run() {
  moveRobot(dir);
    loadData();
    //arena[getIndex(z, y, x)].visited = true; //sets the current cell as visited
    if (!arena[getIndex(z, y, x)].checkpoint) { //if it's not a checkpoint, it's added to the history of visited cells
      struct Cell *tempHistory = new struct Cell[++historySize];
      for (int i = 0; i < historySize-1; i++) {
        tempHistory[i] = history[i];
      }
      history = new struct Cell[historySize];
      for (int i = 0; i < historySize; i++) {
        history[i] = tempHistory[i];
      }
      delete[] tempHistory;
      history[historySize] = arena[getIndex(z, y, x)];
      historySize++;
    }
    else { //if it is a checkpoint, sets it as the last visited checkpoint and resets the history
      delete[] history;
      historySize = 0;
      lastCheckpoint = arena[getIndex(z, y, x)];
    }
    if (arena[getIndex(z, y, x)].black) { //if it finds a black cell, it'll go back
      moveTileBackward();
    }
    else if (arena[getIndex(z, y, x)].exit) { //if it finds an exit
      arena[getIndex(z, y, x)].linkedFloor = ++lastFloor;
      addLayer('z');
      arena[getIndex(lastFloor, y, x)].linkedFloor = z;
      //[INSERTE FUNCIÓN PARA SUBIR RAMPAS AQUÍ] //
      z = lastFloor;
      ignore = true;
      arena[getIndex(z, y, x)].visited = true;
     }
}

void expand(){
  if(x == 0){
    if(!arena[getIndex(z, y, x)].west)
      shift('x');
  }
  else if(x == xSize - 1){
    if(!arena[getIndex(z, y, x)].east)
      addLayer('x');
  }
  if(y == 0){
    if(!arena[getIndex(z, y, x)].north)
      shift('y');
  }
  else if(y == ySize - 1){
    if(!arena[getIndex(z, y, x)].south)
      addLayer('y');
  }
}

void explore() {
  loadData();
  expand();
  ignore = false;
  finishedFloor = false ;
  if (!check(y, x)) { //if the robot is stuck
    if (!arena[getIndex(z, y, x)].exit) search(arena[getIndex(z, y, x)]); //if it's stuck on a visited ramp and the room isn't finished it'll continue exploring the room
    else if (!finishedFloor) search(arena[getIndex(z, y, x)]); //got stuck on a visited ramp, but there are still some tiles left unvisited.
    else {
      //[PLEASE INSERTE FUNCIÓN PARA SUBIR O BAJAR UNA RAMPA AQUÍ PLEASEEEEE] //
      z = arena[getIndex(z, y, x)].linkedFloor;
      ignore = true;
      arena[getIndex(z, y, x)].visited = true; //sets the cell as visited
      arena[getIndex(z, y, x)].exit = false;
      finishedFloor = false;
    }
  }
  else {
    changeDir();
    if (!ignore) run();
  }
//---------------------------------------------- DIJKSTRA ------------------------------------------------
int main() {
  arena[0].start = true;
  while (1) {
    explore();
  }
}
