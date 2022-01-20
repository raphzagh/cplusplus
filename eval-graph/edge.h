#include <iostream>
#include <cmath>
#include <string>
#include <vector>

//Création de la classe arête dont les attributs sont le nom du sommet de départ, le nom du sommet d'arrivée et la valeur de l'arête
//Cette classe va nous permettre de mettre en place une représentation en mémoire de graphe orienté et dont les arêtes sont pondérés.

class Edge {
    std::string startNode; //nom du sommet de départ
    std::string endNode; //nom du sommet d'arrivée
    double valueEdge; //valeur de l'arête

    public:

    Edge(std::string startNode, std::string endNode, double valueEdge);
    Edge();
    double value();
    std::string startName();
    std::string endName();
};