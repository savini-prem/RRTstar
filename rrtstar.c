#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "rrtstar.h"

#define NUM_ROBOTS 1
#define TRUE 1
#define FALSE 0

// print current tree
void printPath(vertex_t* current){
	printf("[%f,%f]\n",current->loc[0],current->loc[1]);
	while(current->parent!=NULL){
		current = current->parent;
		printf("[%f,%f]\n",current->loc[0],current->loc[1]);
	}
}

// for debugging 
void printVertex(vertex_t* current){
	printf("[%f,%f]\n",current->loc[0],current->loc[1]);
}

// create new vertex and initialize 
vertex_t* newVertex(void){
	vertex_t* result = (vertex_t*)malloc(sizeof(vertex_t));
	result->loc[0] = 0.0;
	result->loc[1] = 0.0;
	result->parent=NULL;
	result->cost=0.0;
	return result;
}

// choose random location and assign to vertex
void randLoc(vertex_t* result){ //add obstacle check to this 
	result->loc[0] = ((float) rand()) / ((float) RAND_MAX);
    result->loc[1] = ((float) rand()) / ((float) RAND_MAX);
}

// returns TRUE if vertex does not collide with ALL obstacles
int collisionFreeMult(vertex_t* v, float obst[][5][2], int num_obstacles, int obstacle_size){
	float a[] = {v->loc[0],v->loc[1]};

	int result = TRUE;
	for(int i=0; i<num_obstacles; i++){
		if(!collisionFree(a,obst[i],obstacle_size)){
			result = FALSE;
			break;
		}
	}
	return result;
}

// returns TRUE if vertex does not collide with one obstacle
int collisionFree(float a[], float obst[][2], int obstacle_size){
	// pseudo-code from: http://geomalgorithms.com/a03-_inclusion.html#cn_PnPoly()
	int counter = 0;

	for(int i=0; i<obstacle_size; i++){
		if(isLeft(obst[i],obst[i+1],a)==0) return 0;
		if(obst[i][1]<=a[1]){
			if(obst[i+1][1]>a[1]){
				if(isLeft(obst[i],obst[i+1],a)>0) ++counter;
			}
		}
		else{
			if(obst[i+1][1]<=a[1]){
				if(isLeft(obst[i],obst[i+1],a)<0) --counter;
			}
		}
	}
	return !counter;
}

// for collisionFree
float isLeft(float a[], float b[], float c[]){
	return ( (b[0]-a[0])*(c[1]-a[1]) - (c[0]-a[0])*(b[1]-a[1]) );
}

// calculate distance between two vertex locations 
float calcDistance(float a[], float b[]){
	float result = sqrt( pow(a[0]-b[0],2) + pow(a[1]-b[1],2) );
	return result;
}

// changes location of rand vertex 
// new location ensures distance between nearest and new rand < nu
// new location ensures distance between old rand and new rand < old rand and nearest
// i.e that new rand lies on the line between old rand and nearest, a distance of nu away from nearest
vertex_t* steer(vertex_t* rand, vertex_t* nearest){ 
	// calculate nu
	float nu = 0.25*NUM_ROBOTS; //because environment is 1x1

	// calculate direction vector 
	float dv[] = {rand->loc[0]-nearest->loc[0],rand->loc[1]-nearest->loc[1]};
	
	// calculate distance between neareast & rand
	float a = calcDistance(nearest->loc,rand->loc);

	// scale direction vector by nu
	float w[] = {(nu/a)*dv[0],(nu/a)*dv[1]};

	// find new rand 
	rand->loc[0] = nearest->loc[0]+w[0];
	rand->loc[1] = nearest->loc[1]+w[1];

	/*float old_rand[] = {rand->loc[0],rand->loc[1]};
	float dist = calcDistance(nearest->loc,rand->loc);
	
	while(1){
		randLoc(rand);
		if( calcDistance(nearest->loc,rand->loc)< nu && calcDistance(old_rand,rand->loc)<dist){
			break;
		}
	}*/

	return rand;
}

// calculates radius used by near 
// CONFIRM WITH XUSHENG THAT dim = 2*NUM_ROBOTS AND THAT mu = 1
float calcRadius(int count, int num_robots){ 
	float nu = 0.25*num_robots; //because environment is 1x1
	float gamma = ceil(4*pow((1/M_PI),1/(2*num_robots)));
	float radius = fmin( ( gamma*pow( log(count)/count , 1/(2*num_robots) ) ), nu );
	return radius;
}

// finds vertex in tree that is closest to the random vertex
vertex_t* findNearest(vertex_t* current, vertex_t* rand){
	int dist = calcDistance(current->loc,rand->loc);
	vertex_t* result = current;

	while(current->parent!=NULL){
		current = current->parent;
		int dist1 = calcDistance(current->loc,rand->loc);

		if(dist1<dist){
			dist = dist1;
			vertex_t* result = current;
		}
	}
	return result;
}

// should: find ALL vertices in tree that are a max radial distance from new_steer 
// this version: returns TRUE if within max radial distance from new_steer
int findNear(vertex_t* current, vertex_t* new_steer, int count, int num_robots){
	float dist = calcDistance(current->loc,new_steer->loc);
	float radius = calcRadius(count, num_robots);
	//printf("radius: %f\n",radius);

	if(dist<radius) return TRUE;
	return FALSE;
}


// for obstacle free, calculates determinant
float det(float a[], float b[]){
	float result = a[0]*b[1] - a[1]*b[0];
	return result; 
}

// returns TRUE if the line between two vertices doesn't intersect one obstacle
int obstacleFree(float a[], float b[], float obst[][2], int obstacle_size){
	// pseudo-code from: http://geomalgorithms.com/a13-_intersect-4.html
	float tE = 0.0;
	float tL = 1.0;
	float t, N, D;
	float dS[] = {b[0]-a[0], b[1]-a[1]};

	for(int i = 0; i<obstacle_size; i++){
		float edge[] = {obst[i+1][0]-obst[i][0], obst[i+1][1]-obst[i][1]};
		float intermediate[] = {a[0]-obst[i][0], a[1]-obst[i][1]};
		N = det(edge,intermediate);
		D = -det(edge,dS);
		t = N/D;
		
		if(fabs(D)==0.0){
			if(N<0) return TRUE;
			else continue;
		}

		if(D<0){
			if(t>tE){
				tE = t;
				if(tE>tL) return TRUE;
			}
		}

		if(D>0){
			if(t<tL){
				tL = t;
				if(tL<tE)return TRUE;
			}
		}
	}
	return FALSE;
}

// returns TRUE if the line between two vertices doesn't intersect ALL obstacles
int obstacleFreeMult(vertex_t* nearest, vertex_t* new_steer, float obst[][5][2], int num_obstacles, int obstacle_size){
	// start and end points of line segment 
	float a[] = {nearest->loc[0],nearest->loc[1]};
	float b[] = {new_steer->loc[0],new_steer->loc[1]};

	int result = TRUE;
	for(int i=0; i<num_obstacles; i++){
		if(!obstacleFree(a,b,obst[i],obstacle_size)){
			result = FALSE;
			break;
		}
	}
	return result;
}

// calculates cost of adding vertex (note that C=1)
float calcCost(vertex_t* nearest, vertex_t* new_steer){
	return calcDistance(nearest->loc,new_steer->loc) + nearest->cost;
}

// free heap memory
void freeVertices(vertex_t* current){
	vertex_t* parentVertex = current->parent;
	free(current);
	while(parentVertex!=NULL){
		current = parentVertex;//maybe problem
		parentVertex = parentVertex->parent;
		free(current);
	}
}