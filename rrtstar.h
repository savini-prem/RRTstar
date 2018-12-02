#ifndef __RRTSTAR_H__
#define __RRTSTAR_H__

typedef struct _vertex_t{ // needs to have loc, parent, & weight
	float loc[2];
	struct _vertex_t* parent;
	float cost;
} vertex_t;

typedef struct _array_t{ // holds an array of pointers & length of array 
  vertex_t** arr;
  int len;
} array_t;

void printPath(vertex_t* current);
void printVertex(vertex_t* current);
void printArray(array_t* current);
vertex_t* newVertex(void);
array_t* newArray(void);
void addToArray(array_t* result, vertex_t* element);
void randLoc(vertex_t* result);
int collisionFreeMult(vertex_t* v, float obst[][5][2], int num_obstacles, int obstacle_size);
int collisionFree(float a[], float obst[][2], int obstacle_size);
float isLeft(float a[], float b[], float c[]);
float calcDistance(float a[], float b[]);
vertex_t* steer(vertex_t* rand, vertex_t* nearest);
float calcRadius(int count, int num_robots);
vertex_t* findNearest(array_t* endpts, vertex_t* rand);
array_t* findNear(array_t* endpts, vertex_t* new_steer, int count, int num_robots);
int contains(array_t* result, vertex_t* current);
float det(float a[], float b[]);
int obstacleFree(float a[], float b[], float obst[][2], int obstacle_size);
int obstacleFreeMult(vertex_t* nearest, vertex_t* new_steer, float obst[][5][2], int num_obstacles, int obstacle_sizes);
float calcCost(vertex_t* nearest, vertex_t* new_steer);
vertex_t* findMinCost(array_t* near);
void extend(array_t* endpts, vertex_t* new_steer);
void rewire(array_t* near, vertex_t* new_steer);
void freeVertices(vertex_t* current);
void freeArray(array_t* current);

#endif 