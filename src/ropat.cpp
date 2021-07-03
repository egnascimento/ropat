#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <limits.h>
#include <stdio.h>
#include <float.h>
#include <math.h>

// Number of vertices in the cost_matrix
#define VertexNumber 30
int route_plan[VertexNumber]={-1};
int route_index = 0;

// Array of vertexes coordinates
int vertex_list[VertexNumber][2] = {
  {	  -2	,	136	},   // Tractor area
  {	  -2	,	38	},   // 1
  {	  -2	,	102	},   // 2
  {	 -68	,	38	},   // 3
  {	 -68	,	102	},   // 4
  {	 -68	,	136	},   // 5
  {	 -84	,	38	},   // 6
  {	 -84	,	102	},   // 7
  {	 -84	,	136	},   // 8
  {	-100	,	38	}, // 9
  {	-100	,	102	}, // 10
  {	-100	,	136	}, // 11
  {	-116	,	38	}, // 12
  {	-116	,	102	}, // 13
  {	-116	,	136	}, // 14
  {	-132	,	38	}, // 15
  {	-132	,	102	}, // 16
  {	-132	,	136	}, // 17
  {	-148	,	38	}, // 18
  {	-148	,	102	}, // 19
  {	-148	,	136	}, // 20
  {	-164	,	38	}, // 21
  {	-164	,	102	}, // 22
  {	-164	,	136	}, // 23
  {	-180	,	38	}, // 24
  {	-180	,	102	}, // 25
  {	-180	,	136	}, // 26
  {	-196	,	38	}, // 27
  {	-196	,	102	}, // 28
  {	-196	,	136	}, // 29
};

/* Costs matrix based on distance */
int cost_matrix[VertexNumber][VertexNumber] = { 
      //   T   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29
        {	00,	00,	34,	00,	74,	66,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	},
        {	00,	00,	64,	66,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	},
        {	34,	64,	00,	00,	66,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	},
        {	00,	66,	00,	00,	64,	00,	16,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	},
        {	74,	00,	66,	64,	00,	34,	00,	16,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	},
        {	66,	00,	00,	00,	34,	00,	00,	00,	16,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	},
        {	00,	00,	00,	16,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	},
        {	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	},
        {	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	},
        {	00,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	},
        {	00,	00,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	},
        {	00,	00,	00,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	},
        {	00,	00,	00,	00,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	},
        {	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	},
        {	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	},
        {	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	16,	00,	00,	00,	64,	00,	16,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	},
        {	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	16,	00,	64,	00,	00,	00,	16,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	},
        {	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	00,	00,	00,	00,	},
        {	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	00,	00,	00,	},
        {	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	00,	00,	},
        {	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	00,	},
        {	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	},
        {	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	},
        {	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	16,	00,	00,	00,	},
        {	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	16,	00,	00,	},
        {	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	16,	00,	},
        {	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	16,	00,	00,	00,	00,	00,	16,	},
        {	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	16,	00,	00,	00,	64,	00,	},
        {	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	00,	16,	00,	64,	00,	34,	},
};


/**
 * Find the closest vertex to the provided point
 * 
 * @param x X coordinate of the point
 * @param y Y coordinate of the point
 * @return Number of the closest vertex
 */
int closest_vertex(float x, float y)
{
  float euclidean_dist = 0;
  float closest_dist = FLT_MAX;
  int closest_vertex = 0;

  for(int v=0; v<VertexNumber; v++)
  {
    euclidean_dist=sqrt(pow(x-vertex_list[v][0], 2) + pow(y-vertex_list[v][1],2));
    if(euclidean_dist < closest_dist)
    {
      closest_dist = euclidean_dist;
      closest_vertex = v;
    }
  }
  return closest_vertex;
}




typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * A utility function to find the vertex with minimum distance value, from
 * the set of vertices not yet included in shortest path tree.
 * 
 * @param dist Distance array
 * @param sptSet Set of already marked vertices
 * @return The vertex with the minimal distance
 */
int minDistance(int dist[], bool sptSet[])
{
    // Initialize min value
    int min = INT_MAX, min_index;
  
    for (int v = 0; v < VertexNumber; v++)
        if (sptSet[v] == false && dist[v] <= min)
        {
            min = dist[v], min_index = v;
        }
  
    return min_index;
}

/**
 * Function to print shortest
 * path from source to j
 * using parent array
 * 
 * @param parent Array of vertexes to follow
 * @param j Index of array
 */
void printPath(int parent[], int j)
{
       
    // Base Case : If j is source
    if (parent[j] == - 1)
        return;
   
    printPath(parent, parent[j]);
   
    route_plan[route_index++] = j;
    printf("%d ", j);
    fflush(stdout);
}

/**
 *  A utility function to print 
 * the constructed distance
 * array
 * 
 *  @param dist Array with distances
 *  @param n 
 *  @param parent Array of vertexes to follow
 */
void printSolution(int dist[], int n, 
                      int parent[])
{
    int src = 0;
    printf("Vertex\t Distance\tPath");
    for (int i = 1; i < VertexNumber; i++)
    {
        printf("\n%d -> %d \t\t %d\t\t%d ",
                      src+1, i+1, dist[i], src+1);
        printPath(parent, i);
    }
}

/**
 *  Funtion that implements Dijkstra's
 *  single source shortest path
 *  algorithm for a cost_matrix represented
 *  using adjacency matrix representation
 */
void dijkstra(int cost_matrix[VertexNumber][VertexNumber], int src, int dst)
{
    fflush(stdout);
    // The output array. dist[i]
    // will hold the shortest
    // distance from src to i
    int dist[VertexNumber]; 
   
    // sptSet[i] will true if vertex
    // i is included / in shortest
    // path tree or shortest distance 
    // from src to i is finalized
    bool sptSet[VertexNumber];
   
    // Parent array to store
    // shortest path tree
    int parent[VertexNumber];
   
    // Initialize all distances as 
    // INFINITE and stpSet[] as false
    for (int i = 0; i < VertexNumber; i++)
    {
        parent[0] = -1;
        dist[i] = INT_MAX;
        sptSet[i] = false;
    }
   
    // Distance of source vertex 
    // from itself is always 0
    dist[src] = 0;
   
    // Find shortest path
    // for all vertices
    for (int count = 0; count < VertexNumber - 1; count++)
    {
        // Pick the minimum distance
        // vertex from the set of
        // vertices not yet processed. 
        // u is always equal to src
        // in first iteration.
        int u = minDistance(dist, sptSet);
   
        // Mark the picked vertex 
        // as processed
        sptSet[u] = true;
   
        // Update dist value of the 
        // adjacent vertices of the
        // picked vertex.
        for (int v = 0; v < VertexNumber; v++)
   
            // Update dist[v] only if is
            // not in sptSet, there is
            // an edge from u to v, and 
            // total weight of path from
            // src to v through u is smaller
            // than current value of
            // dist[v]
            if (!sptSet[v] && cost_matrix[u][v] &&
                dist[u] + cost_matrix[u][v] < dist[v])
            {
                parent[v] = u;
                dist[v] = dist[u] + cost_matrix[u][v];
            } 
    }
   
    // print the constructed
    // distance array
    //printSolution(dist, VertexNumber, parent);
    fflush(stdout);
  
    route_index = 0;
    printPath(parent, dst);
    
    printf("\r\n\r\n");
    fflush(stdout);
}

/**
 * Entry point
 * 
 * @param argc Number of arguments
 * @param argv Pointer to the arguments
 * @return Zero if successfull or the error code
 */
int main(int argc, char** argv){
  ros::init(argc, argv, "ROPAT - Routing Optimization Platform for Autonotomous Tractors");

  int harvester_position[2] = {-165, 110};
  int dst=0;

  dst = closest_vertex(harvester_position[0], harvester_position[1]);
  printf("\r\nYour destination vertex before path plan to harvester is: %d\r\n", dst);
  printf("\r\nTractor should follow from this origin in vertex T a route following the points:");
  fflush(stdout);
  
  dijkstra(cost_matrix, 0, dst);
  
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;
  
  for(int n=0; n<VertexNumber; n++)
  {
    // If reached the end, exit for loop
    if(route_plan[n] == -1)
    {
      break;
    }

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 1.0;
    goal.target_pose.pose.position.y = 1.0;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal to next node");
    ac.sendGoal(goal);

    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Vertex reached successfully!");
    else
      ROS_INFO("Fail to reach the vertex."); // Todo: fail handling in case does not reach vertex

  }

  return 0;
}