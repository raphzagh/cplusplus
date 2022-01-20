#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <limits>
#pragma once
#include "edge.h"
#include "adjacentMatrixRepresentation.h"

//Création de la représentation des graphes en mémoire avec des listes d'adjacence
//Les attributs de cette représentation sont un dictionnaire et la dimension du graph

class AdjacentListRepresentation{
    
    std::unordered_map<std::string, std::vector<Edge> > dictOfEdges; // dictOfEdges est un dictionnaire de clé un string et de value un vecteur d'arrêtes
    //Dans la suite, le dictOfEdges est fait de telle façon qu'à chaque clé ie. le nomd'un sommet, on lui associe le vecteur contenant la liste de ses arêtes sortantes
    int dimension;

    public:

    AdjacentListRepresentation(std::unordered_map<std::string, std::vector<Edge> > dictOfEdges, int dimension);
    std::vector<std::string> getKeys(); // Cette fonction retourne la liste des clés du dictionnaire dictOfEdges ce qui permet d'associer
    // à chaque clé (ie. un sommet) un entier qui est son indice dans cette liste
    // Cette fonction sera utile pour passer de la représentation en liste d'adjacence à une représentation en matrice d'adjacence
    double edgeAccess(std::string startName, std::string endName); // retourne la valeur de l'arête de sommet de départ startName et de sommet d'arrivé endName
    AdjacentMatrixRepresentation changeRepresentation(); // retourne le graph dans une représentation matricielle
    int getDimension();
    void print(); // permet d'afficher les arêtes
    std::unordered_map<std::string, std::vector<Edge> > getDict();
    std::unordered_map<std::string, int> dictKeys(); // retourne un dictionnaire qui associe à chaque clé (ie.sommet) son entier issu de la méthode getKeys()
    std::unordered_map<int, std::string> dictReverseKeys();// retourne un dictionnaire qui associe à chaque entier (ie.sommet) sa clé (étiquette string) issue de la méthode getKeys()
    std::vector<std::string> getNeighbors(std::string origin); //retourne le vecteur des noms des sommets voisins à un sommet d'origine en argument
};
