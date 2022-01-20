#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <unordered_map>

// Création de la classe de la représentation des graphes en mémoire sous la forme de matrice d'adjacence
// J'ai fait le choix d'accéder aux valeurs des arrêtes à partir des coordonnées entières de la matrice

class AdjacentMatrixRepresentation {
    std::vector< std::vector<double> > matrix;
    std::unordered_map<std::string, int> dictKeys;

    public:

    AdjacentMatrixRepresentation(std::vector< std::vector<double> > matrix, std::unordered_map<std::string, int> dictKeys); // on génère le graphe à partir d'une matrice contenant les
    double edgeAccess(std::string startNode, std::string endNode); //retourne la valeur de l'arête de coordonnées (i,j)
    int dimension();
    void print(); 
    std::vector< std::vector<double> > getMatrix(); //retourne la matrice
    std::unordered_map<std::string, int> getDictKeys(); // retourne un dictionnaire qui associe à chaque clé (ie.sommet) un entier issu de l'attribut dictKeys
    std::unordered_map<int, std::string> getReverseDictKeys();// retourne un dictionnaire qui associe à chaque entier (ie.sommet) sa clé (étiquette string) issu de l'attribut dictKeys
};