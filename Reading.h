#ifndef READING_H
#define READING_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "Graph.h"

using namespace std;

class Reading
{
public:
    Reading();
    ~Reading();
    vector<graph_data> graph_input(const char *path, bool e_weighted, int &m, int &n);
};

#endif // READING_H
