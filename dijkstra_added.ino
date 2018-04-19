//***********************************************************
//
//  main.cpp
//  RoboCar Project
//
//  Created by Mohammad and Mike.
//  Copyright © 2018 Mohammad Shahili, Mike Nourian. All rights reserved.
//
#include <SPI.h>  
#include <Pixy.h>
#include <limits.h>
// This is the main Pixy object 
Pixy pixy;
#define RIGHT 6
#define LEFT 5
#define STRAIGHT 7
#define BACKWARD 8
// connect motor controller pins to Arduino digital pins
// motor one (left)
int enA = 10;
int in1 = 9;
int in2 = 8;
// motor two (right)
int enB = 5;
int in3 = 7;
int in4 = 6;
int redYellowCC = 10; //might be changed later
int blueSig = 3;


int intersection = 0;  //initialize the location of the pointer to the first element in Ar, should go from 0 to (numNodes... -1)
bool firstRun=true;

int leftRightDirectionArray [100] = { -1};
int arraySize = 0;
int staticTurn = STRAIGHT ;

//Globals
int NUM_NODES = 12;

//end of globals

#define NORTH 1
#define EAST 2
#define SOUTH 3
#define WEST 4

struct Point {
    bool inSPT;
    int pointID;
    int distance; //record the distance of all the points
    Point * prev;
    int northNeighbor;
    int rightNeightbor;
    int leftNeighbor;
    int southNeighbor;
};

//for knowing the directions such as turnleft, right and ..., we need this to know the relative directions
void initialize_points_for_all_nodes(Point * points){
    for (int i = 0; i <  NUM_NODES; i ++){
        points[i].northNeighbor = i + 4;
        points[i].southNeighbor = i -4;
        points[i].leftNeighbor = i -1;
        points[i].rightNeightbor = i+1;
        if (i >= 8 & i <= 11){
            points[i].northNeighbor = -1;
        }
        if (i >=0 && i <= 3){
            points[i].southNeighbor = -1;
        }
        if (i %4 == 0){
            points[i].leftNeighbor= -1;
        }
        if (i %4 == 3){
            points[i].rightNeightbor = -1;
        }
    }
}
int findShortestDistanceInGraph(Point * points){
    int min= INT_MAX;
    int shortestPathPt = 0;
    for (int i = 0; i < NUM_NODES; i++){
//        int distance = points[i].distance;
//        bool inSpt = points[i].inSPT;
        if (points[i].inSPT == false && points[i].distance < min){
            shortestPathPt = i;
            min = points[i].distance;
        }
    }
    return shortestPathPt;
}
int numberOfNodesToBeTraced = 0;

void getNumNodesToBeTraced(Point * points,int size, int * nodes, int source, int dest){
    int j = dest;
    int i = size - 1;
    while (j != source && j >=0 && i >= 0){
        nodes[i] = points[j].pointID;
        j = points[j].prev->pointID;
        i --;
    }
    nodes[0] = source;
}

int getDirection (Point * points, int firstPoint, int secondPoint){
    //use norht
    if (points[firstPoint].leftNeighbor == points[secondPoint].pointID){
        return WEST;
    }
    if (points[firstPoint].rightNeightbor == points[secondPoint].pointID){
        return EAST;
    }
    if (points[firstPoint].northNeighbor == points[secondPoint].pointID){
        return NORTH;
    }
    if (points[firstPoint].southNeighbor == points[secondPoint].pointID){
        return SOUTH;
    }
    return -1;
}


//directionarray will be returned
void backtrace(Point * points, int * nodes, int size, int * directionArray, int directionArraySize){
    for (int i = 0; i < directionArraySize; i ++){
        int direction = getDirection(points, nodes[i], nodes[i+1]);
        if (direction == -1){fprintf(stderr, "error in backtrace");}
        directionArray[i] = direction;
    }
}

int getDirs (int initialDirectionOfRobot, int intendedDirection){
    int dir = STRAIGHT;
    if (initialDirectionOfRobot == NORTH) {
        if (intendedDirection == WEST){
            dir = LEFT;
        }
        else if (intendedDirection == EAST){
            dir = RIGHT;
        }
        else if (intendedDirection == NORTH){
            dir = STRAIGHT;
        }
        else if (intendedDirection == SOUTH){
            dir = BACKWARD;
        }
    }
    
    if (initialDirectionOfRobot == WEST){
        if (intendedDirection == WEST){
            dir = STRAIGHT;
        }
        if (intendedDirection == NORTH){
            dir = RIGHT;
        }
        if (intendedDirection == SOUTH){
            dir = LEFT;
        }
        if (intendedDirection == EAST){
            dir = BACKWARD;
        }
    }
    if (initialDirectionOfRobot == SOUTH){
        if (intendedDirection == WEST){
            dir = RIGHT;
        }
        if (intendedDirection == NORTH){
            dir = BACKWARD;
        }
        if (intendedDirection == EAST){
            dir = LEFT;
        }
        else if (intendedDirection == SOUTH){
            dir = STRAIGHT;
        }
    }
    if (initialDirectionOfRobot == EAST){
        if (intendedDirection == WEST){
            dir = BACKWARD;
        }
        if (intendedDirection == NORTH){
            dir = LEFT;
        }
        if (intendedDirection == EAST){
            dir = STRAIGHT;
        }
        else if (intendedDirection == SOUTH){
            dir = RIGHT;
        }
    }
    initialDirectionOfRobot = intendedDirection;
    return dir;
}

//points start at 0 -->
void shortestPath(int graph[12][12], int source, int dest, int initialDirection){
    //create an array of points so that we can keep track of previous points for backtrace
    Point points [NUM_NODES];
    for (int i = 0; i < NUM_NODES; i++){
        points[i].pointID = i;
        points[i].distance = INT_MAX; //set all the distances initially to INFINITY
        points[i].prev = NULL;
        points[i].inSPT = false;
    }
    initialize_points_for_all_nodes(points);
    points[source].distance = 0; //set the distance equal to 0 for the source
    
    for (int i = 0; i < NUM_NODES && points[dest].inSPT == false; i++){
        //take the shortest distance in the graph and update its neighbors, using the matrix
        int minPt = findShortestDistanceInGraph(points);
        //set as in inSPT
        points[minPt].inSPT = true;
        //update the neighbors
        //u is the min pt
        for (int j = 0; j < NUM_NODES; j++){
            if (points[j].inSPT == false && graph[minPt][j] && points[minPt].distance != INT_MAX && points[minPt].distance+ graph[minPt][j]< points[j].distance){
                //set the previous pointer
                points[j].prev = &points[minPt];
                points[j].distance = points[minPt].distance + graph[j][minPt];
            }
        }
    }
    //now we have the the shortest path and we can backtrace
    int j = dest;
    
    //I dont want to use STL like queue or vector, since I am running on Arduino
    int count = 0;
    while (j != source){
        count++;
        j = points[j].prev->pointID;
    }
    count ++; //to include the source
    //create an array of size count
    int nodes [count];
    //this function modifies the nodes so now nodes has src -> p1 -> ... -> dest
    getNumNodesToBeTraced(points, count, nodes, source, dest);
    int directionsArray [count-1];
    //backtrace(points, nodes, count, directionsArray, count-1);
    for (int i = 0; i < count-1; i ++){
        int direction = getDirection(points, nodes[i], nodes[i+1]);
        if (direction == -1){fprintf(stderr, "error in backtrace");}
        fprintf(stderr, "%d, direction, %d\n", i,direction);
        directionsArray[i] = direction;
    }
    //get left, right, straight with initial oriention
    int initialDir = directionsArray[0];//the direction of robot when moving
    int leftRightStraightDirs [count -1];
    arraySize = count -1;
  for (int i = 0; i < count - 1; i++) {
    leftRightStraightDirs[i] = getDirs(initialDir, directionsArray[i]);
    leftRightDirectionArray[i] = getDirs(initialDir, directionsArray[i]);
    initialDir = directionsArray[i];//this is the intended direction
  }
    
    //USAGE 1: use leftRightStraightDirs with size == count -1 to get an array of directions
    
    //USAGE 2:
    //Use this for the stationaryTurn
    //left RightStraight firstMove would be to go straight
    int stationaryTurn = getDirs(initialDirection, directionsArray[0]);
    staticTurn = stationaryTurn;
    fprintf(stderr, "stationatyTurn = %d\n", stationaryTurn);
}

//*********************************************************************************************
  //Dijkstra finish
  void setup()
{
  Serial.begin(9600);
  Serial.print("Starting...\n");
  pixy.init();
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
    int graph[12][12] = {
        {0,2,0,0,1,0,0,0,0,0,0,0},//d
        {2,0,1,0,0,2,0,0,0,0,0,0},
        {0,1,0,6,0,0,4,0,0,0,0,0},//done
        {0,0,6,0,0,0,0,1,0,0,0,0}, //done
        {1,0,0,0,0,3,0,0,1,0,0,0}, //4done d
        {0,2,0,0,3,0,3,0,0,1,0,0}, //5doned
        {0,0,4,0,0,3,0,1,0,0,1,0}, //6 done d
        {0,0,0,1,0,0,1,0,0,0,0,2}, //7 done d
        {0,0,0,0,1,0,0,0,0,1,0,0}, //8 done d
        {0,0,0,0,0,1,0,0,1,0,2,0}, //9 done
        {0,0,0,0,0,0,1,0,0,2,0,1}, //10
        {0,0,0,0,0,0,0,2,0,0,1,0}, //11
    };

//      int graph[12][12] = {
//        {0,2,0,0,0,0,0,0,0,0,0,0},
//        {2,0,3,0,0,0,0,0,0,0,0,0},
//        {0,3,0,5,0,0,6,0,0,0,0,},
//        {0,0,5,0,0,0,0,7,0,0,0,0},
//        {0,0,0,0,0,0,0,0,0,0,0,0},
//        {0,0,0,0,0,0,0,0,0,0,0,0},
//        {0,0,1,0,0,0,0,0,0,0,2,0},
//        {0,0,0,7,0,0,0,0,0,0,0,4},
//        {0,0,0,0,0,0,0,0,0,0,0,},
//        {0,0,0,0,0,0,0,0,0,0,0,0},
//        {0,0,0,0,0,0,2,0,0,0,0,3},
//        {0,0,0,0,0,0,0,4,0,0,3,0},
//    };
    //void shortestPath(int graph[12][12], int source, int dest, int initialDirection){

    shortestPath(graph, 0, 3, EAST); //change the size and intial points
}

void demoOne(int pos)   //go left
{
  int rateTimesHun = (float(150)/pos) * 70; //from 100, from 150
 
  if (rateTimesHun > 255)
  {
    rateTimesHun = 254;
  }
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enA, 60); //from 85
  // turn on motor B (right)
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enB, rateTimesHun);  //100

}
void demoTwo(int pos)   //go right
{  
  int rateTimesHun = (float(pos)/150) * 60; //from 85, from 150
  if (rateTimesHun > 255)
  {
    rateTimesHun = 254;
  }
  // turn on motor A (left)
  digitalWrite(in1, HIGH); 
  digitalWrite(in2, LOW);
  // set speed to 85 out of possible range 0~255
  analogWrite(enA, rateTimesHun); //85
  // turn on motor B (right
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enB, 70); //from 100 
}
void turn(int direction){
  //right and left
  //direction = 6 => right
  //direction = 5 => left
  //direction = 7 => go straight
  switch (direction){
    case RIGHT:
    turnRight();
      break;
    case LEFT:
    turnLeft();
      break;
    case STRAIGHT: 
    goStraight();
      break;
  }
}

void turnLeft()
{
  digitalWrite(in1, LOW); 
  digitalWrite(in2, HIGH);
  // set speed to 85 out of possible range 0~255
  analogWrite(enA, 85); 
  // turn on motor B (right
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  // set speed to 90 out of possible range 0~255
  analogWrite(enB, 90); //from 100 
}
void turnRight()
{
  digitalWrite(in1, HIGH); 
  digitalWrite(in2, LOW);
  // set speed to 85 out of possible range 0~255
  analogWrite(enA, 85); 
  // turn on motor B (right
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  // set speed to 90 out of possible range 0~255
  analogWrite(enB, 90); 
}

void stop(){
  //  // now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void goStraight()  //Probably won't use! (if it has to go straight, then just skip in the switch statement 
{
  // this function will go straight for 5 seconds
  // turn on motor A
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
//  // set speed to 200 out of possible range 0~255
  analogWrite(enA, 60);
  // turn on motor B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enB, 70);

}
void loop()
{
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[60]; 

  // grab blocks!
  blocks = pixy.getBlocks();
  if (firstRun == true)
  {
          delay(1000);
          switch(staticTurn)
          {
          case RIGHT:
              turn (RIGHT);  //pre-initialized to go straight after stopping. we set this later to what Dijkstra says
              delay (350);  // turns left for 350 miliseconds, this can change based in the power of the battery
              stop();       // stops for 1 second after turning left
              delay(1000);
              intersection++;
          break;
          case LEFT:
              turn (LEFT);  //pre-initialized to go straight after stopping. we set this later to what Dijkstra says
              delay (350);  // turns left for 350 milisecond, this can change based in the power of the battery
              stop();       // stops for 1 second after turning left
              delay(1000);
              intersection++;
          break;
          case STRAIGHT:
          break;
          }
          //intersection++;
          firstRun = false;
  }
  // If there are detect blocks, print them!
  
  if (blocks)
  {
    for (i = 0; i< blocks;i++)
      {        
        if (intersection<arraySize+1 && pixy.blocks[i].signature == redYellowCC  && (pixy.blocks[i].x)<190 && (pixy.blocks[i].x)>110 && (abs(int (pixy.blocks[i].angle)) <40 || (abs(int (pixy.blocks[i].angle)) >150)&& (abs(int (pixy.blocks[i].angle)) <200)))
        {
            if (pixy.blocks[i].x > 145 && pixy.blocks[i].x < 155)  //change the middle to 160
            {
              //Serial.print("into if statement\n");
              goStraight();
            }
    
            if (pixy.blocks[i].x <= 145)  //go left, change from 145
            {
              demoOne(pixy.blocks[i].x);
            }
           else if (pixy.blocks[i].x >= 155)    //go right, change from 155
           {
             demoTwo(pixy.blocks[i].x);
           }
        }
        else if (pixy.blocks[i].signature == blueSig && pixy.blocks[i].y >= 190)
        {
          if (intersection<arraySize+1) //if we still have directions (note that 5 is the number of directions in Ar to be executed)
          {
             int dir = leftRightDirectionArray[intersection];  // 0--> tright, 1--> left, 2--> straight
             if(dir!=STRAIGHT)
             {
                delay(400);   //delays for 400 miliseconds after seeing the blue sign (y>=150) and then stops --> to get on the actual intersection
                stop();
                delay (500); //waits for half a second after stoping 
                //now we have to detect which direction we are supposed  to go to.
                turn (dir);  //pre-initialized to go straight after stopping. we set this later to what Dijkstra says
                delay (350);  // turns left for 350 miliseconds
                stop();       // stops for 0.5 second after turning left
                delay(500);
             }
             else
             {
                delay(500);
             }
             intersection++;
          }
          else
          {
             intersection++;
             stop();
             delay (10000); //waits for 1 seconds after stoping
          } 
        }
    }// end for
  }
}

