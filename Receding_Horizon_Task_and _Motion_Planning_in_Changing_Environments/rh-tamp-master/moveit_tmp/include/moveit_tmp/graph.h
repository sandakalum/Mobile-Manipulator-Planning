#ifndef GRAPH_H
#define GRAPH_H

class Graph
{
public:
  Graph(int V);
  void addEdge(int v, int w); // to add an edge to graph
  bool isCyclic();            // returns true if there is a cycle in this graph

private:
  int V;               // No. of vertices

  std::list<int>* adj; // Pointer to an array containing adjacency lists
  bool isCyclicUtil(int v, bool visited[], bool* rs); // used by isCyclic()
};

#endif // GRAPH_H
