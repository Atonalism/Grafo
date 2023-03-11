#include "Reading.h"

Reading::Reading() {}
Reading::~Reading() {}

vector<graph_data> Reading::graph_input(const char *path, bool e_weighted, int &m, int &n)
{
    vector<graph_data> edges;
    ifstream file;
    string line;
    char delimiter = ' ';
    int counter;
    file.open(path);
    if (file.is_open())
    {
        getline(file, line);
        n = atoi(line.c_str());

        counter = 0;
        while (getline(file, line))
        {
            istringstream iss(line);
            graph_data new_edge;
            while (getline(iss, line, delimiter))
            {
                switch (counter)
                {
                case 0:
                    new_edge.tail = atoi(line.c_str());
                    ++counter;
                    break;

                case 1:
                    new_edge.head = atoi(line.c_str());
                    ++counter;
                    break;

                case 2:
                    if (e_weighted)
                        new_edge.edge_weight = atoi(line.c_str());
                    ++counter;
                    break;
                }
            }
            counter = 0;
            edges.push_back(new_edge);
        }
    }
    else
    {
        cout << "Falha ao abrir o arquivo de entrada" << endl;
        exit(0);
    }
    file.close();

    m = edges.size();

    return edges;
}
