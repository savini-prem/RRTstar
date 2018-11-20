#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "rrtstar.h"

#define NUM_RUNS 100
#define NUM_OBSTACLES 2
#define OBSTACLE_SIZE 4
#define NUM_GOALS 1
#define GOAL_SIZE 3

/* LIST OF FLAWS : ) 
      1. collisionFree means something diff?
      2. obstacles hard coded (ADD INPUT FILES WITH OBSTACLES AND GOAL REGIONS)
      3. steer doesnt actually minimize, just checks
      4. radius calculation may be slightly wrong
      5. near algorithm is slightly altered 
      6. abbreviated cost function is just distance function (i.e. c=1)
      7. when sampling random point, can it be contained within any of the target regions? 
      8. rewire??? 
*/

int main(int argc, char ** argv){
  // hardcoded for now
  // obstacles: n+1 vertices (listed on counterclockwise order)
  float obst[2][5][2] = {{{0.3, 0.2}, {0.3, 0.0}, {0.7, 0.0}, {0.7, 0.2}, {0.3, 0.2}},
                         {{0.4, 1.0}, {0.4, 0.7}, {0.6, 0.7}, {0.6, 1.0}, {0.4, 1.0}}};
  // goals: n+1 vertices (listed on counterclockwise order)
  float goal[1][5][2] = {{{0.3, 0.5}, {0.3, 0.3}, {0.5, 0.3}, {0.3, 0.5}, {0.3, 0.3}}};
  //float goal[1][5][2] = {{{0.1, 0.9}, {0.1, 0.7}, {0.3, 0.7}, {0.1, 0.9}, {0.1, 0.7}}};                     

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

  // initialize pointer to current vertex in tree
  vertex_t* current = init;
  count++;

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
    vertex_t* nearest = findNearest(current,rand);
    //printf("Nearest is ");
    //printVertex(nearest);

    // create (actually replace rand) new node using steer
    vertex_t* new_steer = steer(rand, nearest);
    //printf("Steer is ");
    //printVertex(new_steer);

    // check if no obstacles in path between nearest and new_steer
    if(obstacleFreeMult(nearest, new_steer, obst, NUM_OBSTACLES, OBSTACLE_SIZE)){
      
      // calculate cost of adding new_steer & increment counter
      new_steer->cost = calcCost(nearest, new_steer);
      count++;
      //printf("cost of steer is %f\n",new_steer->cost);

      // create new vertex pointer xmin, which is equal to nearest
      // note that now min->cost = cmin 
      vertex_t* min = nearest;
      //printf("cost of nearest is %f\n",nearest->cost);

      // find vertices in tree closest to new_steer within a ball of radius r 
      vertex_t* ptr = current;
      while(ptr->parent!=NULL){
        if(findNear(ptr,new_steer,count)){
          // for each vertex in near check if the cost is less than cmin, 
          // if so this vertex becomes xmin
          if(ptr->cost < min->cost) min=ptr;
        }
        ptr = ptr->parent;
      }

      // connect new_steer to xmin 
      //printf("Min is ");
      //printVertex(min);
      new_steer->parent = min;
      current = new_steer;

      // rewire

      // break if vertex in goal region
      if(!collisionFreeMult(current,goal,NUM_GOALS,GOAL_SIZE)) break;
    }
  }
  printPath(current);
  freeVertices(current);
}