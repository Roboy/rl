/*
    BSD 3-Clause License

    Copyright (c) 2017, Roboy
            All rights reserved.

    Redistribution and use in source and binary forms, with or without
            modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
            IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
            FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
            DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
            SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
            CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    author: Simon Trendel ( simon.trendel@tum.de ), 2018
    description: Class implementing a simple tree
*/

#pragma once
#include <vector>
#include <map>
#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>

using std::map;
using std::vector;
using std::cout;
using std::endl;
using std::string;

namespace rl {
    namespace kin {
        struct Vertex;
        struct Edge;

        struct Vertex {
            uint index;
            string name;
            vector<Edge> edges;
            vector<Vertex *> parents;
        };
        struct Edge {
            Vertex *to;
        };

        class Tree {
        public:
            Tree() {
            }

            Tree(uint numberVertices) {
                for (uint i = 1; i <= numberVertices; i++) {
                    addVertex();
                }
            }

            void children(uint of, vector<Vertex*> &children) {
                if (indexVertex.find(of) != indexVertex.end()) {
                    Vertex *v = indexVertex.at(of);
                    cout << "vertex " << of << " children: \n";
                    for (auto edge:v->edges) {
                        if (edge.to == v)
                            cout << "none";
                        else {
                            cout << edge.to->index << " ";
                            children.push_back(edge.to);
                        }
                    }
                    cout << endl;
                } else {
                    cout << "ERROR: vertex " << of << " does not exist in graph" << endl;
                }
            }

            void children(string of, vector<Vertex*> &children) {
                if (nameVertex.find(of) != nameVertex.end()) {
                    Vertex *v = nameVertex.at(of);
                    cout << "vertex " << of << " children: \n";
                    for (auto edge:v->edges) {
                        if (edge.to == v)
                            cout << "none";
                        else {
                            cout << edge.to->name << " ";
                            children.push_back(edge.to);
                        }
                    }
                    cout << endl;
                } else {
                    cout << "ERROR: vertex " << of << " does not exist in graph" << endl;
                }
            }

            void parents(uint of, vector<Vertex*> &parents) {
                if (indexVertex.find(of) != indexVertex.end()) {
                    Vertex *v = indexVertex.at(of);
                    cout << "vertex " << of << " parents: ";
                    for (auto parent:v->parents) {
                        cout << parent->index << " ";
                    }
                    cout << endl;
                    parents = v->parents;
                } else {
                    cout << "ERROR: vertex " << of << " does not exist in graph" << endl;
                }
            }

            void parents(string of, vector<Vertex*> &parents) {
                if (nameVertex.find(of) != nameVertex.end()) {
                    Vertex *v = nameVertex.at(of);
                    cout << "vertex " << of << " parents: ";
                    for (auto parent:v->parents) {
                        cout << parent->name << " ";
                    }
                    cout << endl;
                    parents = v->parents;
                } else {
                    cout << "ERROR: vertex " << of << " does not exist in graph" << endl;
                }
            }

            void traverse(Vertex *from) {
                if (from->edges.empty())
                    return;
                for (auto edge:from->edges) {
                    cout << "(" << from->index << " -> " << edge.to->index << ")" << endl;
                    traverse(edge.to);
                }
            }

            template<typename F>
            void traverse(Vertex *from, F f) {
                if (from == from->edges[0].to)
                    return;
                for (auto edge:from->edges) {
                    f(from);
                    traverse(edge.to, f);
                }
            }

            void addEdge(uint from, uint to) {
                if (indexVertex.find(from) != indexVertex.end() && indexVertex.find(to) != indexVertex.end()) {
                    Vertex *v0 = indexVertex[from], *v1 = indexVertex[to];
                    Edge edge;
                    edge.to = v1;
                    totalNrEdges++;
                    v0->edges.push_back(edge);
                    v1->parents.push_back(v0);
                } else {
                    cout << "ERROR: vertex " << from << " or " << to << " does not exist in graph" << endl;
                }
            }

            void addEdge(string from, string to) {
                if (nameVertex.find(from) != nameVertex.end() && nameVertex.find(to) != nameVertex.end()) {
                    Vertex *v0 = nameVertex[from], *v1 = nameVertex[to];
                    Edge edge;
                    edge.to = v1;
                    totalNrEdges++;
                    v0->edges.push_back(edge);
                    v1->parents.push_back(v0);
                } else {
                    cout << "ERROR: vertex " << from << " or " << to << " does not exist in graph" << endl;
                }
            }

            void addVertex() {
                Vertex *v = new Vertex;
                v->index = vertices.size();
                vertices.push_back(v);
                indexVertex[v->index] = vertices[v->index];
                char str[100];
                sprintf(str,"vertex%d",v->index);
                nameVertex[str] = vertices[v->index];
            }

            void addVertex(string name) {
                Vertex *v = new Vertex;
                v->index = vertices.size();
                v->name = name;
                vertices.push_back(v);
                indexVertex[v->index] = vertices[v->index];
                nameVertex[name] = vertices[v->index];
            }

            void removeEdge(uint from, uint to) {
                if (indexVertex.find(from) != indexVertex.end() && indexVertex.find(to) != indexVertex.end()) {
                    Vertex *v0 = indexVertex.at(from), *v1 = indexVertex.at(to);
                    totalNrEdges--;
                    Edge edge;
                    vector<Edge>::iterator edge2delete;
                    edge.to = v1;
                    if (checkIfEdgeExists(v0->edges, edge, edge2delete))
                        v0->edges.erase(edge2delete);
                    for (auto it = v1->parents.begin(); it != v1->parents.end(); ++it) {
                        if ((*it)->index == v0->index) {
                            v1->parents.erase(it);
                            return;
                        }
                    }
                } else {
                    cout << "ERROR: vertex " << from << " or " << to << " does not exist in graph" << endl;
                }
            }

            inline bool checkIfEdgeExists(vector<Edge> &a, Edge &b, vector<Edge>::iterator &edge) {
                for (auto it = a.begin(); it != a.end(); ++it) {
                    if (it->to->index == b.to->index) {
                        edge = it;
                        return true;
                    }
                }
                return false;
            }

            void mergeVertices(uint v, uint w) {
                if (indexVertex.find(v) != indexVertex.end() && indexVertex.find(w) != indexVertex.end()) {
                    Vertex *v0 = indexVertex.at(v), *v1 = indexVertex.at(w);
                    // delete all edges in v0 to v1 or itself
                    vector<Edge> edges2delete;
                    for (auto edge:v0->edges) {
                        if (edge.to == v1 || edge.to == v0) {
                            edges2delete.push_back(edge);
                        }
                    }
                    vector<Edge>::iterator edge2delete;
                    for (auto edge:edges2delete) {
                        if (checkIfEdgeExists(v0->edges, edge, edge2delete))
                            v0->edges.erase(edge2delete);
                    }
                    // delete all edges in v1 to v1 or v0 and adjust v0 as parent
                    edges2delete.clear();
                    for (auto edge:v1->edges) {
                        if (edge.to == v1 || edge.to == v0) {
                            edges2delete.push_back(edge);
                        } else {
                            for (auto it = edge.to->parents.begin(); it != edge.to->parents.end(); ++it) {
                                if ((*it) == v1)
                                    (*it) = v0;
                            }
                        }
                    }
                    edge2delete;
                    for (auto edge:edges2delete) {
                        if (checkIfEdgeExists(v1->edges, edge, edge2delete))
                            v1->edges.erase(edge2delete);
                    }
                    // copy the remaining edges to v0
                    v0->edges.insert(v0->edges.end(), v1->edges.begin(), v1->edges.end());
                    // adjust parent edges to v0
                    for (auto parent:v1->parents) {
                        for (auto it = parent->edges.begin(); it != parent->edges.end(); ++it) {
                            if ((*it).to == v1)
                                (*it).to = v0;
                        }
                    }
                    // copy the parents of v1 to v0
                    v0->parents.insert(v0->parents.end(), v1->parents.begin(), v1->parents.end());
                    // erase vertex w
                    {
                        auto it = nameVertex.find(indexVertex[w]->name);
                        nameVertex.erase(it);
                    }
                    {
                        auto it = indexVertex.find(w);
                        indexVertex.erase(it);
                    }
                    for (auto it = vertices.begin(); it != vertices.end(); ++it) {
                        if ((*it)->index == w) {
                            // clear the data
                            delete (*it);
                            vertices.erase(it);
                            return;
                        }
                    }
                } else {
                    cout << "ERROR: vertex " << v << " or " << w << " does not exist in graph" << endl;
                }
            }

            Vertex *operator[](uint i) {
                auto it = indexVertex.find(i);
                if (it != indexVertex.end()) {
                    return it->second;
                } else {
                    cout << "ERROR: vertex " << i << " does not exist in graph" << endl;
                    return nullptr;
                }
            }

            Vertex *operator[](string name) {
                auto it = nameVertex.find(name);
                if (it != nameVertex.end()) {
                    return it->second;
                } else {
                    cout << "ERROR: vertex " << name << " does not exist in graph" << endl;
                    return nullptr;
                }
            }

            void graphviz(const char *file = "graph.png") {
                std::stringstream str;
                str << "Graph g{\n";
                for (auto it:vertices) {
                    str << it->name << ";\n";
                    for (auto edge:it->edges) {
                        str << it->name << "--" << edge.to->name << ";\n";
                    }
                }
                str << "}" << endl;
                char cmd[2000];
                sprintf(cmd, "echo \"%s\"|dot -Tpng -o%s", str.str().c_str(), file);
                system(cmd);
            }

        private:
            map<uint, Vertex *> indexVertex;
            map<string, Vertex *> nameVertex;
            vector<Vertex *> vertices;
            uint totalNrEdges = 0;
        };
    }
}