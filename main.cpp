#include <iostream>
#include <stddef.h>
#include <stdlib.h>
#include <string>
#include "Graph.h"
#include "Reading.h"
#include "Timer.h"
#include "Menu.h"

using namespace std;

int main(int argc, char **argv)
{
    if (argc != 6)
    {
        cout << "Erro na leitura dos comandos" << endl;
        exit(0);
    }

    for (int i = 0; i < argc; ++i)
    {
        if (argv[i] == NULL)
        {
            cout << "Erro na leitura dos comandos: algum argumento nulo" << endl;
            exit(0);
        }
    }

    const char *current_exec_name = argv[0];
    const char *input_path = argv[1];
    const char *output_path = argv[2];
    int directed, edge_weighted, vertex_weighted;
    directed = atoi(argv[3]);
    edge_weighted = atoi(argv[4]);
    vertex_weighted = atoi(argv[5]);

    int n, m;
    Reading reading;
    vector<graph_data> edges = reading.graph_input(input_path, edge_weighted, m, n);
    Graph *graph = new Graph(output_path, n, m, directed, edge_weighted, vertex_weighted, edges);

    Menu menu(graph, directed, edge_weighted, vertex_weighted);
    menu.application();

    return 0;
}
