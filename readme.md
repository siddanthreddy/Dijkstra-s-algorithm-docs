github : 

#### Constants and Includes
```cpp
#define MAXV 1000

#include <iostream>
#include <limits>

using namespace std;
```
- **MAXV**: Defines the maximum number of vertices in the graph.
- **Includes**: Includes the necessary headers for input/output operations and numeric limits.

#### EdgeNode Class
```cpp
class EdgeNode {
public:
    int key;
    int weight;
    EdgeNode *next;
    EdgeNode(int, int);
};

EdgeNode::EdgeNode(int key, int weight) {
    this->key = key;
    this->weight = weight;
    this->next = NULL;
}
```
- **EdgeNode**: Represents an edge in the graph.
- **key**: Destination vertex of the edge.
- **weight**: Weight of the edge.
- **next**: Pointer to the next edge node (for linked list implementation).
- **EdgeNode(int key, int weight)**: Constructor to initialize an edge with a destination vertex and weight.

#### Graph Class
```cpp
class Graph {
    bool directed;
public:
    EdgeNode *edges[MAXV + 1];
    Graph(bool);
    ~Graph();
    void insert_edge(int, int, int, bool);
    void print();
};

Graph::Graph(bool directed) {
    this->directed = directed;
    for (int i = 1; i < (MAXV + 1); i++) {
        this->edges[i] = NULL;
    }
}

Graph::~Graph() {
    // TODO: Implement destructor to clean up allocated memory
}

void Graph::insert_edge(int x, int y, int weight, bool directed) {
    if (x > 0 && x < (MAXV + 1) && y > 0 && y < (MAXV + 1)) {
        EdgeNode *edge = new EdgeNode(y, weight);
        edge->next = this->edges[x];
        this->edges[x] = edge;
        if (!directed) {
            insert_edge(y, x, weight, true);
        }
    }
}

void Graph::print() {
    for (int v = 1; v < (MAXV + 1); v++) {
        if (this->edges[v] != NULL) {
            cout << "Vertex " << v << " has neighbors: " << endl;
            EdgeNode *curr = this->edges[v];
            while (curr != NULL) {
                cout << curr->key << " ";
                curr = curr->next;
            }
            cout << endl;
        }
    }
}
```
- **Graph**: Represents a graph with a specified number of vertices and edges.
- **directed**: Boolean indicating if the graph is directed.
- **edges[MAXV + 1]**: Array of pointers to EdgeNode, representing the adjacency list.
- **Graph(bool directed)**: Constructor to initialize the graph and set the directed flag.
- **~Graph()**: Destructor (currently a placeholder, needs implementation).
- **void insert_edge(int x, int y, int weight, bool directed)**: Inserts an edge into the graph.
- **void print()**: Prints the adjacency list representation of the graph.

#### Utility Functions
```cpp
void init_vars(bool discovered[], int distance[], int parent[]) {
    for (int i = 1; i < (MAXV + 1); i++) {
        discovered[i] = false;
        distance[i] = numeric_limits<int>::max();
        parent[i] = -1;
    }
}

void dijkstra_shortest_path(Graph *g, int parent[], int distance[], int start) {
    bool discovered[MAXV + 1];
    EdgeNode *curr;
    int v_curr;
    int v_neighbor;
    int weight;
    int smallest_dist;

    init_vars(discovered, distance, parent);

    distance[start] = 0;
    v_curr = start;

    while (discovered[v_curr] == false) {
        discovered[v_curr] = true;
        curr = g->edges[v_curr];

        while (curr != NULL) {
            v_neighbor = curr->key;
            weight = curr->weight;

            if ((distance[v_curr] + weight) < distance[v_neighbor]) {
                distance[v_neighbor] = distance[v_curr] + weight;
                parent[v_neighbor] = v_curr;
            }
            curr = curr->next;
        }

        smallest_dist = numeric_limits<int>::max();
        for (int i = 1; i < (MAXV + 1); i++) {
            if (!discovered[i] && (distance[i] < smallest_dist)) {
                v_curr = i;
                smallest_dist = distance[i];
            }
        }
    }
}

void print_shortest_path(int v, int parent[]) {
    if (v > 0 && v < (MAXV + 1) && parent[v] != -1) {
        print_shortest_path(parent[v], parent);
        cout << v << " ";
    }
}

void print_distances(int start, int distance[]) {
    for (int i = 1; i < (MAXV + 1); i++) {
        if (distance[i] != numeric_limits<int>::max()) {
            cout << "Shortest distance from " << start << " to " << i << " is: " << distance[i] << endl;
        }
    }
}
```
- **init_vars**: Initializes the `discovered`, `distance`, and `parent` arrays.
  - **discovered[]**: Boolean array indicating if a vertex has been visited.
  - **distance[]**: Array storing the shortest distance from the start vertex.
  - **parent[]**: Array storing the parent of each vertex in the shortest path tree.
- **dijkstra_shortest_path**: Implements Dijkstra's algorithm to find the shortest path from the start vertex.
  - **Graph *g**: Pointer to the graph.
  - **int parent[]**: Array to store the parent of each vertex.
  - **int distance[]**: Array to store the shortest distances.
  - **int start**: The start vertex.
  - Uses a priority queue to extract the vertex with the smallest distance and updates the distances to its neighbors.
- **print_shortest_path**: Recursively prints the shortest path from the start vertex to a given vertex.
  - **int v**: The destination vertex.
  - **int parent[]**: Array storing the parent of each vertex.
- **print_distances**: Prints the shortest distances from the start vertex to all other vertices.
  - **int start**: The start vertex.
  - **int distance[]**: Array storing the shortest distances.

#### Main Function
```cpp
int main() {
    Graph *g = new Graph(false);
    int parent[MAXV + 1];
    int distance[MAXV + 1];
    int start = 1;

    g->insert_edge(1, 2, 4, false);
    g->insert_edge(1, 3, 1, false);
    g->insert_edge(3, 2, 1, false);
    g->insert_edge(3, 4, 5, false);
    g->insert_edge(2, 4, 3, false);
    g->insert_edge(2, 5, 1, false);
    g->insert_edge(4, 5, 2, false);

    dijkstra_shortest_path(g, parent, distance, start);

    // Print the shortest path from vertex 1 to 5
    cout << "Shortest path from 1 to 5: ";
    print_shortest_path(5, parent);
    cout << endl;

    print_distances(start, distance);

    delete g;
    return 0;
}
```
- **Graph *g = new Graph(false)**: Creates a new undirected graph.
- **int parent[MAXV + 1]**: Array to store the parent of each vertex.
- **int distance[MAXV + 1]**: Array to store the shortest distances.
- **int start = 1**: Sets the start vertex to 1.
- **g->insert_edge(..., ..., ..., false)**: Inserts edges into the graph.
- **dijkstra_shortest_path(g, parent, distance, start)**: Calls Dijkstra's algorithm to find the shortest paths.
- **print_shortest_path(5, parent)**: Prints the shortest path from vertex 1 to 5.
- **print_distances(start, distance)**: Prints the shortest distances from the start vertex to all other vertices.
- **delete g**: Deletes the graph object to free memory.