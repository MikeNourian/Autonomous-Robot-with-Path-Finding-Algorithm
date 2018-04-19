//
//  main.cpp
//  Dijksta's for Arduino
//
//  Created by Mike Nourian on 3/4/18.
//  Copyright © 2018 Mike Nourian. All rights reserved.
//

#include <iostream>

//Globals
int NUM_NODES = 12;


//end of globals

#define NORTH 1
#define EAST 2
#define SOUTH 3
#define WEST 4


#define LEFT 5
#define RIGHT 6
#define STRAIGHT 7
#define BACKWARD 8

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
            dir = LEFT;
        }
        if (intendedDirection == NORTH){
            dir = BACKWARD;
        }
        if (intendedDirection == EAST){
            dir = RIGHT;
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
    for (int i = 0; i < count; i++){
        fprintf(stdout, "%d,", nodes[i]);
    }
    fprintf(stdout, "\n");
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
    
    for (int i = 0; i < count - 1; i++){
        leftRightStraightDirs[i] = getDirs(initialDir, directionsArray[i]);
        initialDir = directionsArray[i];//this is the intended direction
        fprintf(stderr, "left,right,straight: %d\n", leftRightStraightDirs[i]);
    }
    
    //USAGE 1: use leftRightStraightDirs with size == count -1 to get an array of directions
    
    
    
    //USAGE 2:
    //Use this for the stationaryTurn
    //left RightStraight firstMove would be to go straight
    int stationaryTurn = getDirs(initialDirection, directionsArray[0]);
    fprintf(stderr, "stationatyTurn = %d\n", stationaryTurn);
    if (stationaryTurn == STRAIGHT){
        //no need to do a stationaryTurn
    }
    else if (stationaryTurn == LEFT){
        //stationaryTurn by 90 to the left
    }
    else if (stationaryTurn ==  RIGHT){
        //stationaryTurn by 90 to the right
    }
    else if (stationaryTurn == BACKWARD){
        //stationaryTurn by 180
    }
    
    
    
    //
//    int dirs [count];
//    dirs [0] = initialDirection;
}



/*
 #define NORTH 1
 #define EAST 2
 #define SOUTH 3
 #define WEST 4
 */



//#define LEFT
//#define NORTH
//#define WEST
//#define SOUTH


int main(int argc, const char * argv[]) {
    
    //0 represents that there is no edge between the nodes
//    int graph[16][16] = {
//        {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
//        {1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
//        {0,1,0,1,0,0,40,0,0,0,0,0,0,0,0,0},
//        {0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0},
//        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
//        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
//        {0,0,40,0,0,0,0,0,0,0,50,0,0,0,0,0},
//        {0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0},
//        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
//        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
//        {0,0,0,0,0,0,50,0,0,0,0,0,0,0,20,0},
//        {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1},
//        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
//        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
//        {0,0,0,0,0,0,0,0,0,0,20,0,0,0,0,5},
//        {0,0,0,0,0,0,0,0,0,0,0,1,0,0,5,0},
//    };
    int graph[12][12] = {
        {0,2,0,0,1,0,0,0,0,0,0,0},
        {2,0,1,0,0,2,0,0,0,0,0,0},
        {0,1,0,6,0,0,4,0,0,0,0,0},//done
        {0,0,6,0,0,0,0,1,0,0,0,0}, //done
        {1,0,0,0,3,0,0,0,1,0,0,0}, //done
        {0,2,0,0,3,0,3,0,0,1,0,0}, //done
        {0,0,1,0,0,3,0,1,0,0,1,0}, //6 done
        {0,0,0,7,0,0,1,0,0,0,0,2}, //7 done
        {0,0,0,0,1,0,0,0,0,1,0,0}, //8 done
        {0,0,0,0,0,1,0,0,1,0,2,0}, //9 done
        {0,0,0,0,0,0,1,0,0,2,0,1}, //10
        {0,0,0,0,0,0,0,2,0,0,1,0}, //11
    };
    

    
    //void shortestPath(int graph[12][12], int source, int dest, int initialDirection){

    shortestPath(graph, 0, 3, EAST); //change the size and intial points

    
}
