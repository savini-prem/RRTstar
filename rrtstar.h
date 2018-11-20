#ifndef __RRTSTAR_H__
#define __RRTSTAR_H__

// LOL UPDATE THIS WITH ALL THE NEW FUNCTIONS

typedef struct _vertex_t{ //needs to have loc, parent, and weight
	float loc[2];
	struct _vertex_t* parent;
	float cost;
} vertex_t;

typedef struct _vertexArray_t{ //an instance of this will eventually be the path
	int length;
} vertexArray_t;

void printPath(vertex_t* current);
void printVertex(vertex_t* current);
vertex_t* newVertex(void);
void randLoc(vertex_t* result);
int collisionFreeMult(vertex_t* v, float obst[][5][2], int num_obstacles, int obstacle_size);
int collisionFree(float a[], float obst[][2], int obstacle_size);
float isLeft(float a[], float b[], float c[]);
float calcDistance(float a[], float b[]);
vertex_t* steer(vertex_t* rand, vertex_t* nearest);
float calcRadius(int num_v);
vertex_t* findNearest(vertex_t* current, vertex_t* rand);
int findNear(vertex_t* current, vertex_t* new_steer, int count);
float det(float a[], float b[]);
int obstacleFree(float a[], float b[], float obst[][2], int obstacle_size);
int obstacleFreeMult(vertex_t* nearest, vertex_t* new_steer, float obst[][5][2], int num_obstacles, int obstacle_sizes);
float calcCost(vertex_t* nearest, vertex_t* new_steer);
void freeVertices(vertex_t* current);



#endif 





