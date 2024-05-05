#include <iostream>
#include <limits.h>
#include <string.h>
#include <queue>
#include <C:/opencv/opencv/build/include/opencv2/opencv.hpp>


using namespace std;
using namespace cv;

// ���������� ������ � �����
#define V 6

// ������� ��� ���������� ������������ �������� �� ����
int min(int a, int b) {
    return (a < b) ? a : b;
}

// ����� ���� � ���������� ���� � �������������� BFS
bool bfs(int rGraph[V][V], int s, int t, int parent[]) {
    bool visited[V];
    memset(visited, 0, sizeof(visited));

    queue <int> q;
    q.push(s);
    visited[s] = true;
    parent[s] = -1;

    while (!q.empty()) {
        int u = q.front();
        q.pop();

        for (int v = 0; v < V; v++) {
            if (visited[v] == false && rGraph[u][v] > 0) {
                q.push(v);
                parent[v] = u;
                visited[v] = true;
            }
        }
    }

    return (visited[t] == true);
}

// ������� ��� ���������� ������������� ������ � ����� � �������������� ��������� �����-����������
int fordFulkerson(int graph[V][V], int source, int sink) {
    int u, v;

    // ������� ��������� ����� ����� ��� �������� ���������� ����
    int rGraph[V][V];
    for (u = 0; u < V; u++)
        for (v = 0; v < V; v++)
            rGraph[u][v] = graph[u][v];

    int parent[V];// ������ ���� BFS � ���������� ����
    int max_flow = 0; // �������������� �����
    // ����� ���� � ���������� ���� � �������������� BFS
    while (bfs(rGraph, source, sink, parent)) {
        // ������� ����������� ���������� ����������� ����� ����
        int path_flow = INT_MAX;
        for (v = sink; v != source; v = parent[v]) {
            u = parent[v];
            path_flow = min(path_flow, rGraph[u][v]);
        }

        // ��������� ����������� ���������� ����
        for (v = sink; v != source; v = parent[v]) {
            u = parent[v];
            rGraph[u][v] -= path_flow;
            rGraph[v][u] += path_flow;
        }
        // ����������� �������������� �����
        max_flow += path_flow;
    }

    return max_flow;
}
// ������� ��� ���������� ����� � ����
void addEdge(int graph[V][V], int u, int v, int capacity) {
    graph[u][v] = capacity;
}

int main() {
    setlocale(LC_ALL, "rus");

    int graph[V][V] = { {0, 16, 13, 0, 0, 0},
                        {0, 0, 20, 12, 0, 0},
                        {0, 4, 0, 0, 14, 0},
                        {0, 0, 9, 0, 0, 20},
                        {0, 0, 0, 7, 0, 4},
                        {0, 0, 0, 0, 0, 0}
    };

    int source = 0, sink = 5;
    int max_flow = fordFulkerson(graph, source, sink);

    cout << "������������ �����: " << max_flow << endl;


    return 0;
}
