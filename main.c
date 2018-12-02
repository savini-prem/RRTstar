#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "rrtstar.h"

#define NUM_ROBOTS 1
#define NUM_RUNS 100
#define NUM_OBSTACLES 2
#define OBSTACLE_SIZE 4
#define NUM_GOALS 1
#define GOAL_SIZE 3

/* LIST OF FLAWS : ) 
      1. collisionFree means something diff?
      2. obstacles hard coded (ADD INPUT FILES WITH OBSTACLES AND GOAL REGIONS)
      3. tree searches not efficient
      6. fix free
*/

int main(int argc, char ** argv){
  // hardcoded for now
  // obstacles: n+1 vertices (listed on counterclockwise order)
  float obst[2][5][2] = {{{0.3, 0.2}, {0.3, 0.0}, {0.7, 0.0}, {0.7, 0.2}, {0.3, 0.2}},
                         {{0.4, 1.0}, {0.4, 0.7}, {0.6, 0.7}, {0.6, 1.0}, {0.4, 1.0}}};
  // goals: n+1 vertices (listed on counterclockwise order)
  float goal[1][5][2] = {{{0.3, 0.5}, {0.3, 0.3}, {0.5, 0.3}, {0.3, 0.5}, {0.3, 0.3}}};
  //float goal[1][5][2] = {{{0.1, 0.9}, {0.1, 0.7}, {0.3, 0.7}, {0.1, 0.9}, {0.1, 0.7}}};                     

  // initialize endpoint array
  array_t* endpts = newArray();

  // start count of vertices in tree
  int count = 0;

  // create starting vertex with random location
  vertex_t* init = newVertex();
  while(1){
    randLoc(init); 
    if(collisionFreeMult(init,obst,NUM_OBSTACLES,OBSTACLE_SIZE)) break;
  }
  //printf("init is ");
  //printVertex(init);

  // increase count
  // add init to endpoint array
  count++;
  endpts->arr = (vertex_t**)malloc(sizeof(vertex_t*)); 
  endpts->arr[0] = init;                          
  endpts->len++;

  // iterate NUMBER_RUNS times or until path is found 
  for(int i=0; i<NUM_RUNS; i++){
    // create new vertex with random location
    vertex_t* rand = newVertex(); 
    while(1){
      randLoc(rand); 
      if(collisionFreeMult(rand,obst,NUM_OBSTACLES,OBSTACLE_SIZE)) break;
    }
    //printf("Rand is ");
    //printVertex(rand);

    // initialize pointer to the nearest node
    vertex_t* nearest = findNearest(endpts,rand);
    //printf("Nearest is ");
    //printVertex(nearest);

    // create (actually replace rand) new node using steer
    vertex_t* new_steer = steer(rand, nearest);
    //printf("Steer is ");
    //printVertex(new_steer);

    // check if no obstacles in path between nearest and new_steer
    if(obstacleFreeMult(nearest, new_steer, obst, NUM_OBSTACLES, OBSTACLE_SIZE)){
      
      // increment counter
      count++;

      // find ALL vertices in tree that are within a max radial distance from new_steer 
      array_t* near = findNear(endpts,new_steer,count,NUM_ROBOTS);
      //printf("Near is: ");
      //printArray(near);

      // find vertex in near that yields min cost path 
      vertex_t* min = findMinCost(near);
      //printf("Min is: ");
      //printVertex(min);

      // connect new_steer to min & calc cost of new_steer
      //printf("Min is ");
      //printVertex(min);
      new_steer->parent = min; 
      new_steer->cost = calcCost(min, new_steer);
      //printf("cost of steer is %f\n",new_steer->cost);

      
      // extend
      extend(endpts, new_steer);

      //printf("endpts are: ");
      //printArray(endpts);

      // rewire (future: include while loop in function??)
      rewire(near,new_steer); 

      // break if vertex in goal region
      if(!collisionFreeMult(new_steer,goal,NUM_GOALS,GOAL_SIZE)) break;

      // free allocated memory in near
      //freeArray(near);
    }
  }
  
  // print all paths
  printf("\nAll Paths:\n");
  for(int i=0; i<endpts->len; i++){
    printf("Path %d:\n",i);
    printPath(endpts->arr[i]);
  }
  printf("\n");

  // print all paths that reach goal region & add to array
  printf("All Feasible Paths:\n");
  array_t* feasible = (array_t*)malloc(sizeof(array_t));
  feasible->arr = NULL;
  feasible->len = 0; 
  for(int i=0; i<endpts->len; i++){
    if(!collisionFreeMult(endpts->arr[i],goal,NUM_GOALS,GOAL_SIZE)){
      printf("Path %d:\n",feasible->len);
      printPath(endpts->arr[i]);

      addToArray(feasible,endpts->arr[i]);
    }
  }
  printf("\n");

  // print least-cost path of paths that reach goal region
  printf("Least-Cost Path:\n");
  vertex_t* result = findMinCost(feasible);
  printPath(result);
  printf("\n");

  // free all allocated memory (one day run in valgrind to see if works lol)
  /*for(int i=0; i<endpts->len; i++){ 
    freeVertices(endpts->arr[i]);
  }
  freeArray(endpts);
  freeArray(feasible);*/
} 