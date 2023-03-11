#ifndef MENU_H
#define MENU_H

#include <string>
#include "Timer.h"
#include "Graph.h"

using namespace std;

class Menu
{
public:
    // Menu recebe o grafo e a informacao se ele eh direcionado
    Menu(Graph *g, bool directed, bool edge_weighted, bool node_weighted);
    ~Menu();
    void application();
    // printa os itens do menu
    void main_menu();
    // salta linhas para simular limpar tela
    void clear_screen();
    // funcoes auxiliares para printar os resultados das funcionalidades
    void print_vector_v(vector<int> &result);
    void print_vector_e(vector<graph_data> &result);

private:
    Graph *graph;
    // vector que armazena o resultado de funcionalidades que retornam um vector<int>
    vector<int> result_v;
    // vector que armazena o resultado de funcionalidades que retornam um vector<graph_data>
    vector<graph_data> result_e;
    // variavel de selecao de funcionalidades
    int control;
    bool directed;
    bool edge_weighted;
    bool node_weighted;
};

#endif // MENU_H
