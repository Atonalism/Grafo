#include "Graph.h"
#define INF std::numeric_limits<int>::max()

typedef pair<int, int> w_n; // par peso_vertice

// inicializacao do grafo e construcao das listas de adjacencias
// entrada:
// (caminho do arquivo de saida, numero de vertices, numero de arestas, direcionado?,
//  peso nas arestas?, peso nos vertices?, conjunto de arestas)
Graph::Graph(const char *output_path, int n, int m, bool directed, bool edge_weighted, bool node_weighted, vector<graph_data> &edges)
{
    // inicializacao
    this->output_path = output_path;
    this->n = n;
    this->m = m;
    this->directed = directed;
    this->edge_weighted = edge_weighted;
    this->node_weighted = node_weighted;
    this->edges_s = edges;

    // contador de vertices adicionados no grafo
    int counter = 0;

    // laco que percorre todas as arestas do arquivo de entrada
    for (int i = 0; i < m; i++)
    {
        // dados da aresta
        int tail = edges[i].tail;
        int head = edges[i].head;
        int weight = edges[i].edge_weight;

        // nova aresta para a lista de adjacencia do vertice tail (tail, head)
        edge new_edge = {head, weight};

        // se o vertice tail nao esta incluido
        if (adj.count(tail) == 0)
        {
            // insira-o e inicialize sua lista de adjacencia
            adj.insert(make_pair(tail, vector<edge>()));
            counter++; // o contador de vertices adicionados eh incrementado

            // se a aresta nao existe na lista de adjacencia de tail
            if (none_of(begin(adj[tail]), end(adj[tail]), [&head](const edge &e)
                        { return e.destination == head; }))
            {
                adj[tail].push_back(new_edge); // a aresta eh adicionada
            }
        }
        else
        {
            // caso o vertice tail ja esteja incluido no grafo
            // vamos direto para a verificacao se a aresta nao existe na lista de adjacencia de tail
            if (none_of(begin(adj[tail]), end(adj[tail]), [&head](const edge &e)
                        { return e.destination == head; }))
            {
                adj[tail].push_back(new_edge); // a aresta eh adicionada
            }
        }

        if (adj.count(head) == 0)
        {
            // Como sabemos que o vertice head existe no grafo
            // aproveitamos para inseri-lo
            adj.insert(make_pair(head, vector<edge>()));
            counter++;
        }

        // se o grafo nao eh direcionado
        if (!directed)
        {

            // fazemos a aresta de volta (head, tail)
            new_edge = {tail, weight};

            // se a aresta nao existe na lista de adjacencia de head
            if (none_of(begin(adj[head]), end(adj[head]), [&tail](const edge &e)
                        { return e.destination == tail; }))
            {
                adj[head].push_back(new_edge); // a aresta eh adicionada
            }
        }
    }

    // se o numero de vertices incluidos no grafo for menor
    // que o N informado, sabemos que eles são isolados e o
    // incluimos com uma label arbitraria
    if (counter < n)
    {
        int aux1 = n - counter;
        int aux2 = INF;

        for (int i = 0; i < aux1; i++)
        {
            adj.insert(make_pair(aux2, vector<edge>()));
            aux2 = aux2 - 1;
        }
    }

    // calculo do grau do grafo
    calc_graph_degree();
}

Graph::~Graph()
{
    adj.clear();
    adj.~unordered_map();
}

void Graph::print_graph()
{
    // percorrer todos os vertices e suas listas de adjacencias
    // it.first = label do vertice atual
    // it.second = lista de adjacencia do vertice atual
    for (const auto &it : adj)
    {
        cout << it.first << ": ";
        for (const edge &e : it.second)
        {
            cout << e.destination << "(" << e.weight << ") ";
        }
        cout << endl;
    }
}

void Graph::calc_graph_degree()
{
    if (edges_s.empty())
    {
        graph_degree = 0;
        return;
    }

    // o grau do vertice eh facilmente obtido
    // chamando a funcao size() na sua lista de adjacencia
    int greater = 0;
    for (const auto &it : adj)
        if (it.second.size() > greater)
            greater = it.second.size();

    graph_degree = greater;
}

int Graph::get_graph_degree()
{
    return graph_degree;
}

void Graph::get_node_degree(int s)
{
    // os vertices de saida sao aqueles que estao na lista de adjacencia do vertice
    // out eh o resultado para a consulta em grafos nao direcionados e resultado parcial
    // em grafos direcionados
    int out = adj[s].size();
    if (directed)
    {
        // se eh direcionado precisamos calcular os vertices de entrada
        int in = 0;

        // para isso percorremos todas as listas de adjacencias
        // e procuramos se o vertice em questao eh alcancando pelos outros
        for (const auto &it : adj)
        {
            if (any_of(begin(it.second), end(it.second), [&s](const edge &e)
                       { return e.destination == s; }))
                --in;
        }
        cout << "indegree(" << s << "): " << in << "\noutdegree(" << s << "): " << out << endl;
    }
    else
        cout << "d(" << s << "): " << out << endl;
}

// modificacao da DFS onde eh retornado verdadeiro se
// existe um caminho de s até t, falso caso contrario
bool Graph::dfs_indirect(int s, int t)
{
    unordered_map<int, bool> explored; // vertices explorados

    // inicia todos os vertices como nao explorados
    for (const auto &it : adj)
        explored[it.first] = false;

    // pilha para simular a recursao
    // inicia-se a pilha com o vertice inicial s
    stack<int> S;
    S.push(s);

    // enquanto a pilha nao estiver vazia
    while (!S.empty())
    {
        // desempilhamos um vertice u
        int u = S.top();
        S.pop();

        // se ele ainda nao foi explorado
        if (!explored[u])
        {
            // marque-o como explorado
            explored[u] = true;

            // se s alcanca t, entao s faz parte do conjunto transitivo indireto de t
            if (explored[t])
                return true;

            // para cada vertice v adjacente a u
            // se v nao foi explorado, empilhe-o
            for (const edge &e : adj[u])
            {
                int v = e.destination;
                if (!explored[v])
                    S.push(v);
            }
        }
    }

    // se saiu do while, entao s nao faz parte do conjunto transitivo indireto de t
    return false;
}

// a) fecho transitivo direto

// usa-se uma DFS para encontrar todos os vertices alcancados por s
vector<int> Graph::direct_transitive_closure(int s)
{
    // vector com os vertices alcancados por s
    vector<int> result;

    unordered_map<int, bool> explored; // vertices explorados

    // inicia todos os vertices como nao explorados
    for (const auto &it : adj)
        explored[it.first] = false;

    // pilha para simular a recursao
    // inicia-se a pilha com o vertice inicial s
    stack<int> S;
    S.push(s);

    // enquanto a pilha nao estiver vazia
    while (!S.empty())
    {
        // desempilhamos um vertice u
        int u = S.top();
        S.pop();

        // se ele ainda nao foi explorado
        if (!explored[u])
        {
            explored[u] = true; // marque o vertice atual como explorado

            // o vertice u eh alcancado por s
            // entao eh colocado no vector resultado
            if (u != s)
                result.push_back(u);

            // para cada vertice v adjacente a u
            // se v nao foi explorado, empilhe-o
            for (const edge &e : adj[u])
            {
                int v = e.destination;
                if (!explored[v])
                    S.push(v);
            }
        }
    }

    return result;
}

// b) fecho transitivo indireto

vector<int> Graph::indirect_transitive_closure(int s)
{
    vector<int> result;

    // Para cada vertice do grafo, rodamos uma DFS para verificar se
    // ele alcanca o vertice selecionado
    for (const auto &it : adj)
        if (dfs_indirect(it.first, s) && it.first != s)
            result.push_back(it.first);

    return result;
}

// c) agrupamento local do vertice s

// calcula o numero de triangulos de um vertice,
// ou seja, quantos ciclos diferentes de tamanho 3 ele participa
int Graph::triangles_at_vertex(int s)
{
    int counter = 0;

    // para cada vertice e adjacente de s
    for (const edge &e : adj[s])
    {
        // para cada vertice f adjacente de e
        for (const edge &f : adj[e.destination])
        {
            if (f.destination != s)
            {
                // para cada vertice g adjacente de e
                for (const edge &g : adj[f.destination])
                {
                    // se g eh igual a s
                    // entao temos o caminho fechado <s, e, f, s>
                    if (g.destination == s)
                    {
                        counter++;
                        break;
                    }
                }
            }
        }
    }

    return counter / 2;
}

float Graph::local_clustering_coefficient(int s)
{
    int d; // grau do vertice s
    int p; // numero de triangulos do vertice s

    if (directed)
    {
        if (!exist_aux_g)
        {
            aux_g = new Graph(output_path, n, m, false, edge_weighted, node_weighted, edges_s);
            exist_aux_g = true;
        }

        d = aux_g->adj[s].size();
        p = aux_g->triangles_at_vertex(s);

        float aux = (float)(2 * p) / (d * (d - 1));

        // retorna a porcentagem de vertices vizinhos de s que sao conectados entre si
        return aux / 2.0f;
    }
    else
    {
        d = adj[s].size();
        p = triangles_at_vertex(s);

        // retorna a porcentagem de vertices vizinhos de s que sao conectados entre si
        return (float)(2 * p) / (d * (d - 1));
    }
}

// d) agrupamento global do grafo
float Graph::global__clustering_coefficient()
{
    float sum = 0;
    // soma os coeficientes de agrupamento local de todos os vertices e retorna a media para o grafo
    for (const auto &it : adj)
        sum += local_clustering_coefficient(it.first);

    return (float)sum / n;
}

// e) caminho minimo entre dois vertices usando o algoritmo de Dijkstra
vector<int> Graph::dijkstra(int s, int r)
{
    // vector para armazenar o caminho minimo entre s e r
    vector<int> path;
    path.push_back(s);

    unordered_map<int, int> distance; // armazena a distancia de todo vertice ate s
    unordered_map<int, int> parent;   // armazena o pai de um vertice

    // inicializacao
    for (const auto &it : adj)
    {
        distance[it.first] = INF; // todos os vertices sao inicializados como tendo distancia infinita de s
        parent[it.first] = -1;    // todos os vertices sao inicializados com pai indefinido
    }

    // distancia de s para ele mesmo eh 0
    distance[s] = 0;

    // fila de prioridade que simula uma heap minima, ou seja, ela insere de forma ordenada
    // em ASC pelo first do par w_n, onde first: distancia de s | second: id do vertice
    priority_queue<w_n, vector<w_n>, greater<w_n>> Q;
    Q.push(make_pair(0, s));

    while (!Q.empty())
    {
        // extraia o vertice u da fila de prioridade
        int u = Q.top().second;
        Q.pop();

        // para cada vertice v adjacente de u
        for (const edge &e : adj[u])
        {
            int v = e.destination;

            // Se houver menor caminho para v atraves de u
            if (distance[v] > distance[u] + e.weight)
            {
                // Atualiza distancia de v
                distance[v] = distance[u] + e.weight;
                parent[v] = u;

                Q.push(make_pair(distance[v], v));
            }
        }
    }

    // secao auxiliar para construir o caminho minimo de s ate r
    // dado o resultado do algoritmo de Dijkstra

    // constroi os pares (pai, filho) passando por
    // toda a hierarquia até chegar em s que tem pai -1
    vector<pair<int, int>> parent_path;
    while (parent[r] != -1)
    {
        parent_path.push_back(make_pair(parent[r], r));
        r = parent[r];
    }

    int aux_dis, soma = 0; // aux_dis vai receber os pesos de cada aresta, soma o peso total
    int t, h;              // t = pai, h = filho

    // como parent_path foi construido ao contrario (r até s) e nos
    // queremos (s até r), passamos pelo vector do ultimo elemento ao primeiro
    for (int i = parent_path.size() - 1; i >= 0; i--)
    {
        t = parent_path[i].first;
        h = parent_path[i].second;
        aux_dis = distance[h] - distance[t];
        soma += aux_dis;
        cout << "(" << t << ", " << h << ") Peso: " << aux_dis << "\n";
        path.push_back(h);
    }
    cout << "Peso total: " << soma << endl
         << endl;

    return path;
}

// f) caminho minimo entre dois vertices usando o algoritmo de Floyd
vector<int> Graph::floyd(int s, int r)
{
    // vector para armazenar o caminho minimo entre s e r
    vector<int> path;
    path.push_back(s);

    // conversao dos labels dos vertices para indices de vetores
    int i = 0;
    unordered_map<int, int> index;
    for (const auto &it : adj)
    {
        index[it.first] = i;
        i++;
    }

    vector<vector<int>> dist(n, vector<int>(n)); // matriz de distancia
    vector<vector<int>> next(n, vector<int>(n)); // matriz de caminho

    // inicializacao das matrizes
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (i == j)
                dist[i][j] = 0; // caminho de um vertice ate ele mesmo eh 0
            else
                dist[i][j] = INF; // caminho de um vertice aos outro eh inicializado como infinito

            next[i][j] = -1; // todos os proximos vertices do caminho sao inicializados como indefinidos
        }
    }

    // inicializacao da matrizes com os dados do grafo
    // para cada aresta do grafo
    for (const graph_data &gd : edges_s)
    {
        int i = index[gd.tail];
        int j = index[gd.head];

        // peso da distancia entre tail (i) e head (j)
        dist[i][j] = gd.edge_weight;
        next[i][j] = gd.head;

        if (!directed)
        {
            dist[j][i] = gd.edge_weight;
            next[j][i] = gd.tail;
        }
    }

    for (int k = 0; k < n; k++)
    {
        // faça todos os vertices como origem, um por um
        for (int i = 0; i < n; i++)
        {
            // faça todos os vertices como destino para a origem atual
            for (int j = 0; j < n; j++)
            {
                if (dist[i][k] == INF || dist[k][j] == INF)
                    continue;

                // se o vertice k esta no menor caminho de i para j
                // entao a dist[i][j] eh atualizada
                if (dist[i][j] > dist[i][k] + dist[k][j])
                {
                    dist[i][j] = dist[i][k] + dist[k][j];
                    next[i][j] = next[i][k];
                }
            }
        }
    }

    // secao auxiliar para construir o caminho minimo de s ate r
    // dado o resultado do algoritmo de Floyd

    vector<pair<int, int>> parent_path;
    while (s != r)
    {
        parent_path.push_back(make_pair(s, next[index[s]][index[r]]));
        s = next[index[s]][index[r]];
    }

    int soma = 0;
    for (const auto &it : parent_path)
    {
        soma += dist[index[it.first]][index[it.second]];
        cout << "(" << it.first << ", " << it.second << ") Peso: " << dist[index[it.first]][index[it.second]] << "\n";
        path.push_back(it.second);
    }
    cout << "Peso total: " << soma << endl;

    return path;
}

// Grafo vértice induzido

// funcao que recebe um vector de vertices e retorna o grafo vertice induzido por esse vector
Graph *Graph::vertex_induced(vector<int> &vertices)
{
    // novo numero de vertice do grafo
    int new_n = vertices.size();
    int new_m;

    // vector para armazenar todas a arestas do grafo vertice induzido
    vector<graph_data> gd;

    // para cada vertice v do vector
    for (const int &v : vertices)
    {
        // para cada vertice adjacente de v
        for (const edge &e : adj[v])
        {
            // se o vertice adjacente esta no vector
            if (find(begin(vertices), end(vertices), e.destination) != end(vertices))
            {
                // a aresta (v, e.destination) vai fazer parte do grafo vertice induzido
                graph_data new_data = {v, e.destination, e.weight};
                gd.push_back(new_data);
            }
        }
    }

    // novo numero de arestas
    new_m = gd.size();

    Graph *induced = new Graph(output_path, new_n, new_m, directed, edge_weighted, node_weighted, gd);

    return induced;
}

bool Graph::is_connected(int s)
{
    unordered_map<int, bool> explored; // vertices explorados

    // inicia todos os vertices como nao explorados
    for (const auto &it : adj)
        explored[it.first] = false;

    // pilha para simular a recursao
    // inicia-se a pilha com o vertice inicial s
    stack<int> S;
    S.push(s);

    // enquanto a pilha nao estiver vazia
    while (!S.empty())
    {
        // desempilhamos um vertice u
        int u = S.top();
        S.pop();

        // se ele ainda nao foi explorado
        if (!explored[u])
        {
            explored[u] = true; // marque-o como explorado

            // para cada vertice v adjacente a u
            // se v nao foi explorado, empilhe-o
            for (const edge &e : adj[u])
            {
                int v = e.destination;
                if (!explored[v])
                    S.push(v);
            }
        }
    }

    // se no fim da DFS algum vertice nao ter sido explorado
    // entao o grafo eh desconexo
    for (const auto &it : adj)
        if (explored[it.first] == false)
            return false;

    return true;
}

// g) arvore Geradora Minima sobre o subgrafo vertice-induzido pelo subconjunto de vertices X usando o algoritmo de Prim

vector<graph_data> Graph::mst_Prim_vertex_induced(vector<int> &vertices)
{
    Graph *induced = vertex_induced(vertices);

    return induced->min_spanning_tree_Prim();
}

vector<graph_data> Graph::min_spanning_tree_Prim()
{
    // verificando se o grafo eh conexo ou nao
    int aux;
    for (const auto &it : adj)
    {
        aux = it.first;
        break;
    }

    if (!is_connected(aux))
    {
        cout << "O grafo eh desconexo!" << endl;
        return {};
    }

    // arestas da solucao
    vector<graph_data> mst;

    unordered_map<int, int> parent, key; // pai e peso
    unordered_map<int, bool> in_mst;     // se o vertice faz parte da solucao

    // inicializacao
    for (const auto &it : adj)
    {
        key[it.first] = INF;      // peso infinto para todos
        parent[it.first] = -1;    // pai indefinido para todos
        in_mst[it.first] = false; // nenhum faz parte da solucao
    }

    // vertice de origem arbitrario
    int r = edges_s[0].tail;
    // fila de prioridade em ordem crescente do par (peso, vertice)
    // eh ordenada pelo peso
    priority_queue<w_n, vector<w_n>, greater<w_n>> Q;
    Q.push(make_pair(0, r)); // inicia a fila de prioridade com o vertice de origem e peso 0

    ofstream output;
    output.open(output_path);

    if (output.is_open())
    {
        output << "// g) arvore Geradora Minima sobre o subgrafo vertice-induzido pelo subconjunto de vertices X usando o algoritmo de Prim\n\n";
        if (directed)
            output << "digraph G {\n";
        else
            output << "strict graph G {\n";

        // enquanto a fila de prioridade nao eh vazia
        while (!Q.empty())
        {
            // o primeiro elemento da fila de prioridade
            // eh o que possui menor peso
            w_n element = Q.top();
            Q.pop();
            int u = element.second;     // vertice u de menor peso
            int weight = element.first; // peso minimo

            // se o vertice u nao esta na solucao
            if (!in_mst[u])
            {
                // inclua-o na solucao
                in_mst[u] = true;

                // se ele nao eh o vertice de origem (unico que a ficar com pai -1)
                // podemos montar a aresta incluida na solucao
                if (parent[u] != -1)
                {
                    if (directed)
                        output << "    " << parent[u] << " -> " << u << " [label=\" " << weight << "\"];\n";
                    else
                        output << "    " << parent[u] << " -- " << u << " [label=\" " << weight << "\"];\n";

                    // a aresta (pai de u, u, peso) eh adiconada na solucao
                    graph_data data = {parent[u], u, weight};
                    mst.push_back(data);
                }

                // para cada vertice v adjacente de u
                for (const edge &e : adj[u])
                {
                    int v = e.destination;
                    // se v nao esta na solucao
                    if (!in_mst[v])
                    {
                        // colocamos o vertice v e o peso da aresta (u, v)
                        // na fila de prioridade
                        Q.push(make_pair(e.weight, v));

                        // se o peso de (u, v) eh menor que o peso atual de v
                        if (key[v] >= e.weight)
                        {
                            // atualizamos o pai de v
                            parent[v] = u;
                            // atualizamos o peso de v
                            key[v] = e.weight;
                        }
                    }
                }
            }
        }

        output << "}\n";
    }
    else
    {
        cout << "Falha ao abrir o arquivo de saida" << endl;
        exit(1);
    }

    return mst;
}

// h) arvore Geradora Minima sobre o subgrafo vertice-induzido pelo subconjunto de vertices X usando o algoritmo de Kruskal

vector<graph_data> Graph::mst_Kruskal_vertex_induced(vector<int> &vertices)
{
    Graph *induced = vertex_induced(vertices);

    return induced->min_spanning_tree_Kruskal();
}

// Como cada aresta E = (v, w) é considerada, a estrutura identifica os componentes
// conexos contendo v e w. Se esses componentes diferirem, então não há caminho de v e w
// e, portanto, a aresta E deve ser incluída; mas se os componentes forem os mesmos, então
// já existe um caminho v-w nas arestas e, portanto, E deve ser omitido. No caso de e ser
// incluído, a estrutura funde os componentes de v e w em um único novo componente.
struct union_find
{
    unordered_map<int, int> parent, rank;

    void make_set(int v)
    {
        parent[v] = v;
        rank[v] = 0;
    }

    int find_set(int v)
    {
        if (v == parent[v])
            return v;
        return parent[v] = find_set(parent[v]);
    }

    void union_sets(int a, int b)
    {
        a = find_set(a);
        b = find_set(b);

        if (a != b)
        {
            if (rank[a] < rank[b])
                swap(a, b);
            parent[b] = a;
            if (rank[a] == rank[b])
                rank[a]++;
        }
    }
};

vector<graph_data> Graph::min_spanning_tree_Kruskal()
{

    // verificando se o grafo eh desconexo
    int aux;
    for (const auto &it : adj)
    {
        aux = it.first;
        break;
    }

    if (!is_connected(aux))
    {
        cout << "O grafo eh desconexo!" << endl;
        return {};
    }

    // aresta da solucao
    vector<graph_data> mst;

    // custo total da mst
    int cost = 0;
    // estrutura auxiliar union-find
    union_find uf;

    // inicializa cada vertice como fazendo parte de componentes conexas diferentes entre si
    for (const auto &it : adj)
        uf.make_set(it.first);

    // ordena os pesos das arestas em ordem crescente
    if (!is_edges_sorted)
    {
        sort(begin(edges_s), end(edges_s),
             [](const graph_data &e1, const graph_data &e2)
             {
                 return e1.edge_weight < e2.edge_weight;
             });

        is_edges_sorted = true;
    }

    ofstream output;
    output.open(output_path);

    if (output.is_open())
    {
        output << "// h) arvore Geradora Minima sobre o subgrafo vertice-induzido pelo subconjunto de vertices X usando o algoritmo de Kruskal\n\n";
        if (directed)
            output << "digraph H {\n";
        else
            output << "strict graph H {\n";

        // para cada aresta ordenada do grafo
        for (const graph_data &gd : edges_s)
        {
            // se as duas extremidades da aresta nao fazem parte da mesma componente conexa
            if (uf.find_set(gd.head) != uf.find_set(gd.tail))
            {
                cost += gd.edge_weight;
                mst.push_back(gd);               // a aresta eh adicionada na solucao
                uf.union_sets(gd.head, gd.tail); // e as duas componentes conexas sao fundidas em uma

                if (directed)
                    output << "    " << gd.tail << " -> " << gd.head << " [label=\"" << gd.edge_weight << "\"];\n";
                else
                    output << "    " << gd.tail << " -- " << gd.head << " [label=\"" << gd.edge_weight << "\"];\n";
            }
        }

        output << "}\n";
    }
    else
    {
        cout << "Falha ao abrir o arquivo de saida" << endl;
        exit(1);
    }

    return mst;
}

// i) Arvore dada pela busca em profundidade de um vertice inicial s

vector<graph_data> Graph::dfs(int s)
{
    vector<graph_data> result; // arestas da árvore dada pela DFS
    vector<int> traversal;     // travessia
    traversal.push_back(s);

    unordered_map<int, bool> explored; // vertices explorados
    unordered_map<int, int> parent;    // pais dos vertices

    int time = 0;
    unordered_map<int, int> start_time; // quando o vertice foi explorado
    unordered_map<int, int> end_time;   // quando todos os seus adjacentes já foram incluidos na pilha de recursao

    // inicia todos os vertices como nao explorados
    // todos os vertices com pai desconhecido
    // e os tempos zerados
    for (const auto &it : adj)
    {
        explored[it.first] = false;
        parent[it.first] = -1;
        start_time[it.first] = 0;
        end_time[it.first] = 0;
    }

    // pilha para simular a recursao
    // inicia-se a pilha com o vertice inicial s
    stack<int> S;
    S.push(s);

    ofstream output;
    output.open(output_path);

    if (output.is_open())
    {
        output << "// i) Arvore dada pela busca em profundidade de um vertice inicial s\n\n";
        if (directed)
            output << "digraph I {\n";
        else
            output << "graph I {\n";

        // enquanto a pilha nao estiver vazia
        while (!S.empty())
        {
            // desempilhamos um vertice u
            int u = S.top();
            S.pop();

            // se ele ainda nao foi explorado
            if (!explored[u])
            {
                explored[u] = true;   // marque-o como explorado
                start_time[u] = time; // marque o tempo atual como o de u
                time++;               // incrementa o tempo

                if (u != s)
                {
                    if (directed)
                        output << "    " << parent[u] << " -> " << u << ";\n";
                    else
                        output << "    " << parent[u] << " -- " << u << ";\n";

                    // uma aresta da arvore
                    graph_data gd = {parent[u], u, 0};
                    result.push_back(gd);

                    traversal.push_back(u);
                    cout << "(" << parent[u] << ", " << u << ")" << endl;
                }

                // para cada vertice v adjacente a u
                // se v nao foi explorado, empilhe-o
                for (const edge &e : adj[u])
                {
                    int v = e.destination;
                    if (!explored[v])
                    {
                        S.push(v);
                        parent[v] = u;
                    }
                    // se o vertice u eh percorrido apos o vertice v,
                    // a aresta (u, v) eh uma aresta de retorno
                    else if (start_time[u] > start_time[v] && end_time[u] < end_time[v])
                    {
                        cout << "Aresta de retorno: (" << u << ", " << v << ")" << endl;
                        if (directed)
                            output << "    " << u << " -> " << v << " [style=dotted];\n";
                        else
                            output << "    " << u << " -- " << v << " [style=dotted];\n";
                    }

                    end_time[u] = time; // tempo atual como o final de u
                    time++;             // incrementa o tempo
                }
            }
        }
        output << "}\n";
    }
    else
    {
        cout << "Falha ao abrir o arquivo de saida" << endl;
        exit(1);
    }

    // printando a travessia no grafo pela DFS
    cout << "\n{ ";
    for (const int &n : traversal)
        cout << n << " ";
    cout << "}" << endl;

    return result;
}