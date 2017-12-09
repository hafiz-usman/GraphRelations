// GraphRelations.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <iostream>
#include <vector>
#include <unordered_set>
#include <stack>
#include <queue>
#include <map>
#include <assert.h>

using namespace std;

class Graph
{
public:
    Graph(int numVertices, bool isDirected) :
        _V(numVertices),
        _E(0),
        _directedGraph(isDirected)
    {
        _adjList.resize(numVertices);
    }
    int getVertexCount()
    {
        return _V;
    }
    int getEdgeCount()
    {
        return _E;
    }
    void addEdge(int v, int w)
    {
        if (_adjList[v].find(w) == _adjList[v].end())
        {
            _adjList[v].insert(w);
            if (_directedGraph == false)
            {
                _adjList[w].insert(v);
            }
            _E++;
        }
    }
    unordered_set<int> getAdjacencyList(int v)
    {
        return _adjList[v];
    }
    bool isDirected()
    {
        return _directedGraph;
    }
    void printGraph()
    {
        assert(_V == _adjList.size());

        cout << "PRINT Graph (V=" << _V << ",E=" << _E << (_directedGraph ? "," : ",un") << "directed):" << endl;
        for (int i = 0; i < _V; i++)
        {
            cout << i << ": ";
            for (auto t = _adjList[i].begin(); t != _adjList[i].end(); t++)
            {
                cout << *t << " ";
            }
            cout << endl;
        }
        //cout << endl;
    }

private:
    vector<unordered_set<int>> _adjList;
    const int _V;
    const bool _directedGraph;
    int _E;
};

class DFS
{
public:
    vector<int> TryDFS(Graph& g, int s)
    {
        _visited.clear();
        _visited.resize(g.getVertexCount(), false);

        vector<int> dfsPath;
        _dfs(g, s, dfsPath);
        return dfsPath;
    }

private:
    vector<bool> _visited;

    void _dfs(Graph& g, int v, vector<int>& dfsPath)
    {
        _visited[v] = true;
        dfsPath.push_back(v);

        auto adj = g.getAdjacencyList(v);
        for (auto t = adj.begin(); t != adj.end(); t++)
        {
            if (_visited[*t] == false)
            {
                _dfs(g, *t, dfsPath);
            }
        }
    }
};

class BFS
{
public:
    vector<int> TryBFS(Graph& g, int v)
    {
        _visited.clear();
        _visited.resize(g.getVertexCount(), false);

        vector<int> bfsPath;
        queue<int> q;

        // IMPORTANT: mark as visited before pushing into queue
        _visited[v] = true;
        bfsPath.push_back(v);
        q.push(v);
        while (q.empty() == false)
        {
            v = q.front();
            q.pop();

            auto adj = g.getAdjacencyList(v);
            for (auto t = adj.begin(); t != adj.end(); t++)
            {
                if (_visited[*t] == false)
                {
                    // IMPORTANT: mark as visited before pushing into queue
                    _visited[*t] = true;
                    bfsPath.push_back(*t);
                    q.push(*t);
                }
            }
        }
        return bfsPath;
    }

private:
    vector<bool> _visited;
};

class ConnectedComponents
{
public:
    map<int, vector<int>> TryConnectedComponents(Graph& g)
    {
        _visited.clear();
        _visited.resize(g.getVertexCount(), false);
        _connectedComponentCount = 0;

        map<int, vector<int>> connectedComponents;
        vector<int> dfsPath;
        for (int v = 0; v < g.getVertexCount(); v++)
        {
            if (_visited[v] == false)
            {
                dfsPath.clear();
                _connectedComponentCount++;
                _dfs(g, v, dfsPath);
                connectedComponents.insert(pair<int, vector<int>>(_connectedComponentCount, dfsPath));
            }
        }
        return connectedComponents;
    }

private:
    vector<bool> _visited;
    int _connectedComponentCount;

    void _dfs(Graph& g, int v, vector<int>& dfsPath)
    {
        _visited[v] = true;
        dfsPath.push_back(v);

        auto adj = g.getAdjacencyList(v);
        for (auto t = adj.begin(); t != adj.end(); t++)
        {
            if (_visited[*t] == false)
            {
                _dfs(g, *t, dfsPath);
            }
        }
    }
};

class PathTo
{
public:
    vector<int> TryPathTo(Graph& g, int s, int t)
    {
        _visited.clear();
        _visited.resize(g.getVertexCount(), false);

        // do a dfs however during this dfs also construct the parentof list
        vector<int> dfsPath;
        vector<int> parentOf(g.getVertexCount(), -1);
        _dfs(g, s, dfsPath, parentOf);

        stack<int> pathFrom;
        // DONT forget to push the destination vertex
        pathFrom.push(t);
        int temp = t;
        while (parentOf[temp] != s && parentOf[temp] != -1)
        {
            pathFrom.push(parentOf[temp]);
            temp = parentOf[temp];
        }

        vector<int> pathTo;
        if (parentOf[temp] == -1)
        {
            // NO path from s to t
            return pathTo;
        }
        // DONT forget to push the source vertex
        pathFrom.push(s);
        while (pathFrom.empty() == false)
        {
            pathTo.push_back(pathFrom.top());
            pathFrom.pop();
        }
        return pathTo;
    }

private:
    vector<int> _visited;

    void _dfs(Graph& g, int v, vector<int>& dfsPath, vector<int>& parentOf)
    {
        _visited[v] = true;
        dfsPath.push_back(v);

        auto adj = g.getAdjacencyList(v);
        for (auto t = adj.begin(); t != adj.end(); t++)
        {
            if (_visited[*t] == false)
            {
                // IMPORTANT: Note this one change from the standard DFS code!
                parentOf[*t] = v;

                _dfs(g, *t, dfsPath, parentOf);
            }
        }
    }

};

class ReverseGraph
{
public:
    Graph TryReverseGraph(Graph g)
    {
        Graph g2(g.getVertexCount(), g.isDirected());
        for (int v = 0; v < g.getVertexCount(); v++)
        {
            auto adj = g.getAdjacencyList(v);
            for (auto t = adj.begin(); t != adj.end(); t++)
            {
                g2.addEdge(*t, v);
            }
        }
        return g2;
    }
};

class Bipartite
{
public:
    bool TryBipartite(Graph& g, vector<int>& red, vector<int>& blue)
    {
        _visited.clear();
        _visited.resize(g.getVertexCount(), false);

        _isBipartite = true; // assume we are bipartite for now
        _color.clear();
        _color.resize(g.getVertexCount(), false);

        for (int v = 0; v < g.getVertexCount(); v++)
        {
            if (_visited[v] == false)
            {
                _dfs(g, v);
            }
        }

        red.resize(0);
        blue.resize(0);
        if (_isBipartite == false)
        {
            return _isBipartite;
        }
        for (int v = 0; v < g.getVertexCount(); v++)
        {
            // let _color == true be red and otherwise blue
            if (_color[v])
            {
                red.push_back(v);
            }
            else
            {
                blue.push_back(v);
            }
        }
        return _isBipartite;
    }

private:
    vector<bool> _visited;
    vector<bool> _color;
    bool _isBipartite;

    void _dfs(Graph& g, int s)
    {
        // early exit
        if (_isBipartite == false)
        {
            return;
        }

        _visited[s] = true;

        auto adj = g.getAdjacencyList(s);
        for (auto t = adj.begin(); t != adj.end(); t++)
        {
            if (_visited[*t] == false)
            {
                _color[*t] = !_color[s];
                _dfs(g, *t);
            }
            // else we've seen this node before so it MUST have a different color else it's not bipartite
            else if (_color[*t] == _color[s])
            {
                _isBipartite = false;
                return;
            }
        }
    }
};

class BasicCycleDetection
{
public:
    bool TryCycleDetection(Graph& g)
    {
        _visited.clear();
        _visited.resize(g.getVertexCount(), false);

        bool cycleDetected = false;
        for (int v = 0; v < g.getVertexCount(); v++)
        {
            if (_visited[v] == false)
            {
                // setting parent of v to v is fine when we start a dfs on a new connected component
                cycleDetected = _dfs(g, v, v);
            }

            if (cycleDetected)
            {
                break; // no need to check further!
            }
        }

        return cycleDetected;
    }

private:
    vector<bool>_visited;

    bool _dfs(Graph& g, int s, int parentOfS)
    {
        bool cycleDetected = false;
        _visited[s] = true;

        auto adj = g.getAdjacencyList(s);
        for (auto t = adj.begin(); t != adj.end(); t++)
        {
            if (_visited[*t] == false)
            {
                cycleDetected = _dfs(g, *t, s);
            }
            else if (*t != parentOfS)
            {
                /*
                - We land here:
                1) when *t is marked. AND
                2) It wasn't marked by current for-loop.
                */
                cycleDetected = true;
            }
            if (cycleDetected)
            {
                // quick exit
                return cycleDetected;
            }
        }
        return cycleDetected;
    }
};

class CyclePath
{
public:
    vector<vector<int>> TryCyclePath(Graph& g)
    {
        _visited.clear();
        _visited.resize(g.getVertexCount(), false);
        _onStack.clear();
        _onStack.resize(g.getVertexCount(), false);

        vector<vector<int>> result;
        bool cycleDetected = false;
        for (int v = 0; v < g.getVertexCount(); v++)
        {
            stack<int> nodesOnStack;
            cycleDetected = false;
            if (_visited[v] == false)
            {
                _dfs(g, v, nodesOnStack, cycleDetected);
            }
            if (cycleDetected)
            {
                assert(nodesOnStack.size() > 0);
                vector<int> temp;
                while (nodesOnStack.empty() == false)
                {
                    temp.push_back(nodesOnStack.top());
                    nodesOnStack.pop();
                }
                reverse(temp.begin(), temp.end());
                result.push_back(temp);
            }
            else
            {
                assert(nodesOnStack.size() == 0);
            }
        }
        return result;
    }

private:
    vector<bool> _visited;
    vector<bool> _onStack;

    // instead of passing around the nodesOnStack, you could also use the _parentOf approach to get the full cycle path
    void _dfs(Graph& g, int s, stack<int>& nodesOnStack, bool& cycleDetected)
    {
        // early exit. needed??
        if (cycleDetected)
        {
            return;
        }

        _visited[s] = true;
        _onStack[s] = true;
        nodesOnStack.push(s);

        auto adj = g.getAdjacencyList(s);
        for (auto t = adj.begin(); t != adj.end(); t++)
        {
            if (_visited[*t] == false)
            {
                _dfs(g, *t, nodesOnStack, cycleDetected);
            }
            else if (_onStack[*t])
            {
                // we have a cycle, bail!
                // However to complete the cycle, we need to push this last node on to our temp stack
                cycleDetected = true;
                nodesOnStack.push(*t);
            }

            if (cycleDetected)
            {
                return;
            }
        }
        _onStack[s] = false;
        nodesOnStack.pop();
    }
};

class TopologicalSort
{
public:
    bool TryTopologicalSort(Graph& g, vector<stack<int>>& topologicalSortOrder)
    {
        // TOPOLOGICAL sort can only be performed on a DAG. So first confirmed that!
        _visited.clear();
        _visited.resize(g.getVertexCount(), false);

        bool cycleDetected = _detectCycle(g);

        // we could've also done generated the reverse post order traversal stack during the _detectCycle dfs but we want to be explicit here
        vector<stack<int>> reversePostOrderTraversalNodes;
        if (cycleDetected == false)
        {
            // NOTE: there can be many topological sort orders for a given DAG!

            _visited.clear();
            _visited.resize(g.getVertexCount(), false);
            for (int v = 0; v < g.getVertexCount(); v++)
            {
                if (_visited[v] == false)
                {
                    // do a separate topological sort on each disconnected component
                    stack<int> temp;
                    _dfsInReversePostOrderTraversal(g, v, temp);
                    reversePostOrderTraversalNodes.push_back(temp);
                }
            }

            topologicalSortOrder = reversePostOrderTraversalNodes;
        }

        // if cycle exists, no topological sort can be performed
        return !cycleDetected;
    }

private:
    vector<bool> _visited;

    bool _detectCycle(Graph& g)
    {
        bool cycleDetected = false;
        for (int v = 0; v < g.getVertexCount(); v++)
        {
            if (_visited[v] == false)
            {
                _dfs(g, v, v, cycleDetected);
            }

            if (cycleDetected)
            {
                break;
            }
        }
        return cycleDetected;
    }

    void _dfs(Graph& g, int s, int parentOfS, bool& cycleDetected)
    {
        if (cycleDetected)
        {
            return;
        }
        _visited[s] = true;

        auto adj = g.getAdjacencyList(s);
        for (auto t = adj.begin(); t != adj.end(); t++)
        {
            if (_visited[*t] == false)
            {
                _dfs(g, *t, s, cycleDetected);
            }
            else if (*t != parentOfS)
            {
                // cycle detected
                cycleDetected = true;
            }

            if (cycleDetected)
            {
                return;
            }
        }
    }

    void _dfsInReversePostOrderTraversal(Graph& g, int s, stack<int>& reversePostOrderNodes)
    {
        //assumes no cycles are in the graph!
        _visited[s] = true;

        auto adj = g.getAdjacencyList(s);
        for (auto t = adj.begin(); t != adj.end(); t++)
        {
            if (_visited[*t] == false)
            {
                _dfsInReversePostOrderTraversal(g, *t, reversePostOrderNodes);
            }
        }
        reversePostOrderNodes.push(s);
    }
};

class StronglyConnectedComponents
{
public:
    bool TryStronglyConnectedComponents(Graph& g)
    {
        if (g.isDirected() == false)
        {
            cout << "Strongly connected components should really be tested on directed graphs!" << endl; //It is easy for undirected graph, we can just do a BFS and DFS starting from any vertex. If BFS or DFS visits all vertices, then the given undirected graph is connected
        }

        // NOTE: The idea is, if every node can be reached from a vertex v, and every node can reach v, then the graph is strongly connected!!!
        // Kosaraju’s Algorithm

        int startVertex = 0;

        // 1. Start a dfs from any vertex. Remember the vertext you started from!
        _visited.clear();
        _visited.resize(g.getVertexCount(), false);
        _dfs(g, startVertex);

        // 2. all vertices should have been visited
        for (int i = 0; i < _visited.size(); i++)
        {
            if (_visited[i] == false)
            {
                return false;
            }
        }

        // 3. reverse the graph
        ReverseGraph r;
        Graph rg = r.TryReverseGraph(g);

        // 4. Perform dfs on reversed Graph starting from the same vertex as before!
        _visited.clear();
        _visited.resize(rg.getVertexCount(), false);
        _dfs(rg, startVertex);

        // 6. all vertices should still be reachable
        for (int i = 0; i < _visited.size(); i++)
        {
            if (_visited[i] == false)
            {
                return false;
            }
        }

        return true;
    }
private:
    vector<bool> _visited;

    void _dfs(Graph& g, int s)
    {
        _visited[s] = true;

        auto adj = g.getAdjacencyList(s);
        for (auto t = adj.begin(); t != adj.end(); ++t)
        {
            if (_visited[*t] == false)
            {
                _dfs(g, *t);
            }
        }
    }
};

Graph createGraph1(bool isDirected)
{
    Graph g(13, isDirected);
    g.addEdge(0, 1);
    g.addEdge(0, 2);
    g.addEdge(0, 5);
    g.addEdge(0, 6);
    g.addEdge(3, 4);
    g.addEdge(3, 5);
    g.addEdge(4, 6);
    g.addEdge(5, 4);
    g.addEdge(7, 8);
    g.addEdge(9, 10);
    g.addEdge(9, 11);
    g.addEdge(9, 12);
    g.addEdge(11, 12);
    return g;
}

Graph createGraph2(bool isDirected, bool makeBipartite)
{
    Graph g(11, isDirected);
    g.addEdge(0, 1);
    g.addEdge(1, 2);
    if (makeBipartite == false)
    {
        g.addEdge(2, 0); // adding this edge will make this graph NOT be bipartite
    }
    g.addEdge(3, 4);
    g.addEdge(5, 6);
    g.addEdge(7, 8);
    g.addEdge(8, 9);
    g.addEdge(9, 10);
    g.addEdge(10, 7);
    return g;
}

Graph createGraph3(bool isDirected, int numCyclesToCreate)
{
    Graph g(10, isDirected);
    g.addEdge(0, 1);
    g.addEdge(1, 2);
    g.addEdge(2, 3);
    g.addEdge(2, 4);
    g.addEdge(2, 5);
    if (numCyclesToCreate-- > 0)
    {
        g.addEdge(2, 0);
    }
    g.addEdge(5, 6);
    g.addEdge(7, 8);
    g.addEdge(8, 9);
    if (numCyclesToCreate-- > 0)
    {
        g.addEdge(9, 7);
    }
    return g;
}

Graph createGraph4()
{
    Graph g(5, true);

    g.addEdge(0, 1);
    g.addEdge(1, 2);
    g.addEdge(2, 3);
    g.addEdge(3, 0);
    g.addEdge(2, 4);
    g.addEdge(4, 2);

    return g;
}

Graph createGraph5()
{
    Graph g(4, true);

    g.addEdge(0, 1);
    g.addEdge(1, 2);
    g.addEdge(2, 3);

    return g;
}

void testDFS(Graph& g)
{
    cout << "DFS: " << endl;
    g.printGraph();
    for (int v = 0; v < g.getVertexCount(); v++)
    {
        DFS dfs;
        vector<int> d = dfs.TryDFS(g, v);
        cout << v << ": ";
        for (int j = 0; j < d.size(); j++)
        {
            cout << d[j] << " ";
        }
        cout << endl;
    }
}

void testBFS(Graph& g)
{
    cout << "BFS: " << endl;
    g.printGraph();
    for (int v = 0; v < g.getVertexCount(); v++)
    {
        BFS bfs;
        vector<int> b = bfs.TryBFS(g, v);
        cout << v << ": ";
        for (int j = 0; j < b.size(); j++)
        {
            cout << b[j] << " ";
        }
        cout << endl;
    }
}

void testConnectedComponents(Graph& g)
{
    cout << "Connected Components:" << endl;
    g.printGraph();
    ConnectedComponents cc;
    map<int, vector<int>> connectedVerticesList = cc.TryConnectedComponents(g);
    for (auto t = connectedVerticesList.begin(); t != connectedVerticesList.end(); t++)
    {
        cout << t->first << ": ";
        for (int i = 0; i < t->second.size(); i++)
        {
            cout << t->second[i] << " ";
        }
        cout << endl;
    }
}

void testPathTo(Graph& g)
{
    cout << "Path To:" << endl;
    g.printGraph();

    PathTo p;
    int s = -1;
    int t = -1;
    vector<int> pathTo;

    s = 0;
    t = 4;
    pathTo.clear();
    pathTo = p.TryPathTo(g, s, t);
    cout << "s=" << s << ",t=" << t << ": ";
    for (int i = 0; i < pathTo.size(); i++)
    {
        cout << pathTo[i] << " ";
    }
    cout << endl;

    s = 1;
    t = 8;
    pathTo.clear();
    pathTo = p.TryPathTo(g, s, t);
    cout << "s=" << s << ",t=" << t << ": ";
    for (int i = 0; i < pathTo.size(); i++)
    {
        cout << pathTo[i] << " ";
    }
    cout << endl;

    s = 7;
    t = 8;
    pathTo.clear();
    pathTo = p.TryPathTo(g, s, t);
    cout << "s=" << s << ",t=" << t << ": ";
    for (int i = 0; i < pathTo.size(); i++)
    {
        cout << pathTo[i] << " ";
    }
    cout << endl;
}

void testReverseGraph(Graph& g)
{
    cout << "Reverse Graph:" << endl;
    g.printGraph();

    ReverseGraph rg;
    Graph reverseGraph = rg.TryReverseGraph(g);
    cout << "Original Graph:" << endl;
    g.printGraph();
    cout << "Reversed Graph: " << endl;
    reverseGraph.printGraph();
}

void testBipartite(Graph& g)
{
    cout << "Bipartite Graph:" << endl;
    g.printGraph();

    vector<int> red;
    vector<int> blue;
    Bipartite bp;
    bool isBipartite = bp.TryBipartite(g, red, blue);
    if (isBipartite == false)
    {
        cout << "Graph is NOT bipartite" << endl;
    }
    else
    {
        cout << "Graph IS bipartite" << endl;
        cout << "Red: " << endl;
        for (int i = 0; i < red.size(); i++)
        {
            cout << red[i] << " ";
        }
        cout << endl;
        cout << "Blue: " << endl;
        for (int i = 0; i < blue.size(); i++)
        {
            cout << blue[i] << " ";
        }
        cout << endl;

    }
}

void testBasicCycleDetection(Graph& g)
{
    cout << "Cycle Detection:" << endl;
    g.printGraph();

    BasicCycleDetection cd;
    bool hasCycle = cd.TryCycleDetection(g);
    cout << "Graph cycle detection: " << hasCycle << endl;
}

void testCyclePath(Graph& g)
{
    cout << "Cycle Path:" << endl;
    g.printGraph();

    CyclePath cp;
    auto cycles = cp.TryCyclePath(g);
    if (cycles.size() == 0)
    {
        cout << "NO cycle detected" << endl;;
    }
    else
    {
        cout << "Cycle(s) detected!" << endl;
        for (int i = 0; i < cycles.size(); i++)
        {
            cout << "Cycle path " << i << ": ";
            for (int j = 0; j < cycles[i].size(); j++)
            {
                cout << cycles[i][j] << " ";
            }
            cout << endl;
        }
    }
}

void testTopologicalSort(Graph& g)
{
    cout << "Topological Sort:" << endl;
    g.printGraph();

    TopologicalSort ts;
    vector<stack<int>> topoSort;
    bool topSortExists = ts.TryTopologicalSort(g, topoSort);
    if (topSortExists)
    {
        cout << "Sort order (per disconnected component): " << endl;
        for (int i = 0; i < topoSort.size(); i++)
        {
            cout << "DC " << i << ": ";
            while (topoSort[i].empty() == false)
            {
                cout << topoSort[i].top() << " ";
                topoSort[i].pop();
            }
            cout << endl;
        }
        cout << endl;
    }
    else
    {
        cout << "Topological Sort DOES NOT exist!" << endl;
    }
}

void testStronglyConnectedComponents(Graph& g)
{
    cout << "Strongly Connected Components:" << endl;
    g.printGraph();
    StronglyConnectedComponents scc;
    bool result = scc.TryStronglyConnectedComponents(g);
    cout << "Is Strongly Connected Component: " << result << endl;
}

int main()
{
    Graph g1 = createGraph1(false);
    testDFS(g1);
    testBFS(g1);
    testConnectedComponents(g1);
    testPathTo(g1);

    Graph g2 = createGraph1(true);
    testReverseGraph(g2);

    Graph g3a = createGraph2(false, true);
    testBipartite(g3a);
    Graph g3b = createGraph2(false, false);
    testBipartite(g3b);

    Graph g4 = createGraph3(true, 0);
    testBasicCycleDetection(g4);
    Graph g5 = createGraph3(true, 1);
    testBasicCycleDetection(g5);
    Graph g6 = createGraph3(true, 2);
    testBasicCycleDetection(g6);

    testCyclePath(g4);
    testCyclePath(g5);
    testCyclePath(g6);

    testTopologicalSort(g4);
    testTopologicalSort(g5);
    testTopologicalSort(g6);

    Graph g7 = createGraph4();
    Graph g8 = createGraph5();
    testStronglyConnectedComponents(g7);
    testStronglyConnectedComponents(g8);


    return 0;
}

