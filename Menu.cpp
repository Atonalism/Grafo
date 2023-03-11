#include "Menu.h"

using namespace std;

Menu::Menu(Graph *g, bool directed, bool edge_weighted, bool node_weighted)
{
    this->graph = g;
    this->directed = directed;
    this->edge_weighted = edge_weighted;
    this->node_weighted = node_weighted;
}

Menu::~Menu() {}

void Menu::main_menu()
{
    cout << "Teoria dos Grafos -- Trabalho Parte 1" << endl;
    cout << endl;
    cout << "1. fecho transitivo direto" << endl;
    cout << "2. fecho transitivo indireto" << endl;
    cout << "3. agrupamento local" << endl;
    cout << "4. agrupamento global" << endl;
    cout << "5. caminho minimo (Djkstra)" << endl;
    cout << "6. caminho minimo (Floyd)" << endl;
    cout << "7. arvore geradora minima (Prim)" << endl;
    cout << "8. arvore geradora minima (Kruskal)" << endl;
    cout << "9. caminhamento em profundidade (DFS)" << endl;
    cout << "0. sair" << endl;
    cout << endl;
}

void Menu::clear_screen()
{
    cout << string(100, '\n');
}

void Menu::print_vector_v(vector<int> &result)
{
    if (!result.empty())
    {
        cout << "{ ";
        for (const int &n : result)
            cout << n << " ";
        cout << "}\n";
    }
    else
        cout << "{ }\n";
}

void Menu::print_vector_e(vector<graph_data> &result)
{
    if (result.empty())
        return;

    int soma = 0;
    for (const graph_data &gd : result)
    {
        cout << "(" << gd.tail << ", " << gd.head << ") peso: " << gd.edge_weight << endl;
        soma += gd.edge_weight;
    }
    cout << "Peso total = " << soma << endl;
}

// Na application que esta a estrutura de Switch
// onde sera controlado as acoes do usuario que
// esta utilizando a aplicacao e as informacoes
// mostradas em tela
void Menu::application()
{
    int i, n, s, t;
    float r;
    vector<int> vertices;

    do
    {
        i = 0;
        clear_screen();

        main_menu();
        cout << "Selecione uma funcionalidade:" << endl;

        cin >> control;
        switch (control)
        {
        case 1:
            clear_screen();

            if (directed)
            {
                do
                {
                    i++;

                    clear_screen();
                    cout << "Fecho transitivo direto" << endl;
                    cout << "digite o id de um vertice:" << endl;
                    cin >> s;
                    cout << endl;

                    clear_screen();
                    cout << "Resultado para o fecho transitivo direto do vertice " << s << endl;
                    {
                        Timer timer;
                        result_v = graph->direct_transitive_closure(s);
                    }
                    print_vector_v(result_v);

                    cout << endl;
                    cout << "0 para sair, 1 para repetir." << endl;
                    cin >> n;
                } while (n != 0);
            }
            else
            {
                cout << "O grafo nao eh direcionado! 0 para voltar." << endl;
                cin >> n;
                if (n == 0)
                    break;
            }
            break;

        case 2:
            clear_screen();

            if (directed)
            {
                do
                {
                    i++;

                    clear_screen();

                    cout << "Fecho transitivo indireto" << endl;
                    cout << "digite o id de um vertice:" << endl;
                    cin >> s;
                    cout << endl;

                    clear_screen();
                    cout << "Resultado para o fecho transitivo indireto do vertice " << s << endl;
                    {
                        Timer timer;
                        result_v = graph->indirect_transitive_closure(s);
                    }
                    print_vector_v(result_v);

                    cout << endl;
                    cout << "0 para sair, 1 para repetir." << endl;
                    cin >> n;

                } while (n != 0);
            }
            else
            {
                cout << "O grafo nao eh direcionado! 0 para voltar." << endl;
                cin >> n;
                if (n == 0)
                    break;
            }
            break;

        case 3:
            clear_screen();

            do
            {
                clear_screen();

                cout << "Agrupamento local" << endl;
                cout << "digite o id de um vertice:" << endl;
                cin >> s;
                cout << endl;

                clear_screen();
                {
                    Timer timer;
                    r = graph->local_clustering_coefficient(s);
                }
                cout << "Resultado para o agrupamento local do vertice " << s << ": " << r << endl;

                cout << endl;
                cout << "0 para sair, 1 para repetir." << endl;
                cin >> n;

            } while (n != 0);

            break;

        case 4:
            clear_screen();

            do
            {
                clear_screen();
                {
                    Timer timer;
                    r = graph->global__clustering_coefficient();
                }
                cout << "Resultado para o agrupamento global do grafo: " << r << endl;

                cout << endl;
                cout << "0 para sair." << endl;
                cin >> n;

            } while (n != 0);
            break;

        case 5:
            clear_screen();

            if (!edge_weighted)
            {
                cout << "O grafo nao eh ponderado nas arestas! 0 para voltar." << endl;
                cin >> n;
                if (n == 0)
                    break;
            }

            do
            {
                i++;
                clear_screen();

                cout << "Caminho minimo (Djkstra)" << endl;
                cout << "digite o id de dois vertices:" << endl;
                cin >> s >> t;
                cout << endl;

                clear_screen();
                cout << "Resultado do algoritmo de Dijkstra para caminho minimo com os vertices " << s << " e " << t << endl
                     << endl;
                {
                    Timer timer;
                    result_v = graph->dijkstra(s, t);
                }
                print_vector_v(result_v);

                cout << endl;
                cout << "0 para sair, 1 para repetir." << endl;
                cin >> n;
            } while (n != 0);

            break;
        case 6:
            clear_screen();

            if (!edge_weighted)
            {
                cout << "O grafo nao eh ponderado nas arestas! 0 para voltar." << endl;
                cin >> n;
                if (n == 0)
                    break;
            }

            do
            {
                i++;

                clear_screen();

                cout << "Caminho minimo (Floyd)" << endl;
                cout << "digite o id de dois vertices:" << endl;
                cin >> s >> t;
                cout << endl;

                clear_screen();
                cout << "Resultado do algoritmo de Floyd para caminho minimo com os vertices " << s << " e " << t << endl
                     << endl;
                {
                    Timer timer;
                    result_v = graph->floyd(s, t);
                }
                print_vector_v(result_v);

                cout << endl;
                cout << "0 para sair, 1 para repetir." << endl;
                cin >> n;
            } while (n != 0);
            break;
        case 7:
            clear_screen();

            if (!edge_weighted)
            {
                cout << "O grafo nao eh ponderado nas arestas! 0 para voltar." << endl;
                cin >> n;
                if (n == 0)
                    break;
            }

            do
            {
                i++;

                if (!vertices.empty())
                {
                    vertices.clear();
                }
                clear_screen();

                cout << "Arvore geradora minima (Prim)" << endl;
                cout << "Digite o tamanho do subconjunto de vertices: " << endl;
                cin >> s;
                cout << endl;
                cout << "Digite os " << s << " elementos do subconjunto: " << endl;
                for (int i = 0; i < s; i++)
                {
                    cin >> t;
                    vertices.push_back(t);
                }

                clear_screen();
                cout << "Resultado do algoritmo de Prim para AGM com o subconjunto" << endl;
                print_vector_v(vertices);
                cout << endl;
                {
                    Timer timer;
                    result_e = graph->mst_Prim_vertex_induced(vertices);
                }
                print_vector_e(result_e);

                cout << endl;
                cout << "0 para sair, 1 para repetir." << endl;
                cin >> n;
            } while (n != 0);

            break;
        case 8:
            clear_screen();

            if (!edge_weighted)
            {
                cout << "O grafo nao eh ponderado nas arestas! 0 para voltar." << endl;
                cin >> n;
                if (n == 0)
                    break;
            }

            do
            {
                i++;
                if (!vertices.empty())
                {
                    vertices.clear();
                }
                clear_screen();

                cout << "Arvore geradora minima (Kruskal)" << endl;
                cout << "Digite o tamanho do subconjunto de vertices: " << endl;
                cin >> s;
                cout << endl;
                cout << "Digite os " << s << " elementos do subconjunto: " << endl;
                for (int i = 0; i < s; i++)
                {
                    cin >> t;
                    vertices.push_back(t);
                }

                clear_screen();
                cout << "Resultado do algoritmo de Kruskal para AGM com o subconjunto" << endl;
                print_vector_v(vertices);
                cout << endl;
                {
                    Timer timer;
                    result_e = graph->mst_Kruskal_vertex_induced(vertices);
                }
                print_vector_e(result_e);

                cout << endl;
                cout << "0 para sair, 1 para repetir." << endl;
                cin >> n;
            } while (n != 0);

            break;
        case 9:
            clear_screen();

            do
            {
                i++;
                clear_screen();

                cout << "Caminhamento em profundidade (DFS)" << endl;
                cout << "digite o id de um vertice:" << endl;
                cin >> s;
                cout << endl;

                clear_screen();
                cout << "Resultado da DFS com o vertice inicial " << s << endl
                     << endl;
                {
                    Timer timer;
                    result_e = graph->dfs(s);
                }
                // print_vector_e(result_e);
                cout << endl;

                cout << "0 para sair, 1 para repetir." << endl;
                cin >> n;
            } while (n != 0);

            break;
        case 0:
            exit(1);
            break;
        default:
            break;
        }
    } while (control != 0);
}