#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <limits>
#include <algorithm>
#include <fstream>
#include <unordered_map>
#include <vector>
#include <queue>
#include <stack>

using namespace std;

// estrutura das arestas
typedef struct
{
    int tail, head;
    int edge_weight = 0;
} graph_data;

// estrutura das arestas na lista de adjacencia
typedef struct
{
    int destination;
    int weight = 0;
} edge;

class Graph
{
public:
    Graph(const char *output_path, int n, int m, bool directed, bool edge_weighted, bool node_weighted, vector<graph_data> &edges);
    ~Graph();
    void print_graph();
    int get_graph_degree();
    void get_node_degree(int s);

    // funcionalidades
    vector<int> direct_transitive_closure(int s);
    vector<int> indirect_transitive_closure(int s);
    float local_clustering_coefficient(int s);
    float global__clustering_coefficient();
    vector<int> dijkstra(int s, int r);
    vector<int> floyd(int s, int r);
    vector<graph_data> mst_Prim_vertex_induced(vector<int> &vertices);
    vector<graph_data> min_spanning_tree_Prim();
    vector<graph_data> mst_Kruskal_vertex_induced(vector<int> &vertices);
    vector<graph_data> min_spanning_tree_Kruskal();
    vector<graph_data> dfs(int s);

private:
    // estrutura dos vertices e suas listas de adjacencia
    // int = label do vertice --> vector<edge> = sua lista de adjacencia
    unordered_map<int, vector<edge>> adj;
    const char *output_path;
    // n = numero de vertices do grafo
    // m = numero de arestas do grafo
    int n, m;
    int graph_degree;
    bool directed;
    bool edge_weighted;
    bool node_weighted;

    // vector auxiliar com todas as arestas do grafo
    vector<graph_data> edges_s;
    // flag para ordenar o vector apenas uma vez
    bool is_edges_sorted = false;

    Graph *aux_g;
    bool exist_aux_g = false;

    void calc_graph_degree();
    bool dfs_indirect(int s, int t);
    bool is_connected(int s);
    int triangles_at_vertex(int s);
    Graph *vertex_induced(vector<int> &vertices);
};

#endif // GRAPH_H