#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <limits.h>
#include <stdio.h>
#include <float.h>
#include <math.h>

// Number of vertices in the graph
#define VertexNumber 30

int vertex_list[VertexNumber][2] = {
  {	-2	,	136	},
  {	-2	,	38	},
  {	-2	,	102	},
  {	-68	,	38	},
  {	-68	,	102	},
  {	-68	,	136	},
  {	-84	,	38	},
  {	-84	,	102	},
  {	-84	,	136	},
  {	-100	,	38	},
  {	-100	,	102	},
  {	-100	,	136	},
  {	-116	,	38	},
  {	-116	,	102	},
  {	-116	,	136	},
  {	-132	,	38	},
  {	-132	,	102	},
  {	-132	,	136	},
  {	-148	,	38	},
  {	-148	,	102	},
  {	-148	,	136	},
  {	-164	,	38	},
  {	-164	,	102	},
  {	-164	,	136	},
  {	-180	,	38	},
  {	-180	,	102	},
  {	-180	,	136	},
  {	-196	,	38	},
  {	-196	,	102	},
  {	-196	,	136	},
};

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

/* Let us create the example graph discussed above */
int graph[VertexNumber][VertexNumber] = { 
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


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
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

// Function to print shortest
// path from source to j
// using parent array
void printPath(int parent[], int j)
{
       
    // Base Case : If j is source
    if (parent[j] == - 1)
        return;
   
    printPath(parent, parent[j]);
   
    printf("%d ", j);
    fflush(stdout);
}

// A utility function to print 
// the constructed distance
// array
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

// Funtion that implements Dijkstra's
// single source shortest path
// algorithm for a graph represented
// using adjacency matrix representation
void dijkstra(int graph[VertexNumber][VertexNumber], int src, int dst)
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
            if (!sptSet[v] && graph[u][v] &&
                dist[u] + graph[u][v] < dist[v])
            {
                parent[v] = u;
                dist[v] = dist[u] + graph[u][v];
            } 
    }
   
    // print the constructed
    // distance array
    //printSolution(dist, VertexNumber, parent);
    fflush(stdout);
    
    
    printPath(parent, dst);
    
    printf("\r\n\r\n");
    fflush(stdout);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "ROPAT - Routing Optimization Platform for Autonotomous Tractors");

  int harvester_position[2] = {-165, 110};
  int dst=0;

  dst = closest_vertex(harvester_position[0], harvester_position[1]);
  printf("\r\nYour destination vertex before path plan to harvester is: %d\r\n", dst);
  printf("\r\nTractor should follow from this origin in vertex T a route following the points:");
  fflush(stdout);
  
  dijkstra(graph, 0, dst);
  

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

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
    ROS_INFO("Fail to reach the vertex.");

  return 0;
}