#include "edge.h"


Edge::Edge(std::string startNode, std::string endNode, double valueEdge):
    startNode(startNode),
    endNode(endNode),
    valueEdge(valueEdge)
{}

double Edge::value(){
    return valueEdge;
}

std::string Edge::startName(){
    return startNode;
}

std::string Edge::endName(){
    return endNode;
}
