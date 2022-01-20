#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <unordered_map>
#include <limits>
#include <climits>
#include "adjacentListRepresentation.h"
#include "intStack.h"
#include "queue.h"

//Q1: Voir les classes

//Q2: La fonction read

//J'ai choisi de faire la fonction read uniquement pour la représentation en liste d'adjacence
// Pour passer dans la représentation matricielle, on peut utiliser la méthode changeRepresentation()

// La fonction read va prendre en argument un fichier .graph
//et retourner le dictionnaire dont les clés sont les sommets 
//et la valeur associée à chaque clé est une liste d'arêtes sortantes du sommet associé à la clé
std::unordered_map<std::string, std::vector<Edge> > read(std::string filename) {
  std::unordered_map<std::string, std::vector<Edge> > empty;
  // chaque ligne du fichier contient la liste des coordonnées d'un point
  // on ouvre le fichier en lecture
  std::ifstream input_file(filename);
  // si problème d'ouverture de fichier on s'en va
  if (!input_file.is_open()) {
    std::cerr << "cannot open file '" << filename << "'" << std::endl;
    return empty;
  }
  // On initialise le dictionnaire qui va contenir les arêtes
  std::unordered_map<std::string, std::vector<Edge> > dictOfEdges;
  // On initialise le vecteur qui va contenir toutes les arêtes;
  std::vector<Edge> listEdges;

  // on va lire le fichier ligne par ligne

  // par souci de simplicité, on considère qu'on ne connait pas la
  // dimension des points, mais on pourrait la connaître en lisant la
  // première ligne et en comptant le nombre de coordonnées

  // la chaîne de caractères qui contient toute la ligne
  std::string line;
  // on va lire toutes les lignes du fichier: quand il n'y en aura
  // plus getline retournera 0

  // on lit la ligne et on met son contenu dans la variable line
  while(getline(input_file, line)) {

    //std::cout << "analyse de la ligne " << line << std::endl;
    //On construit l'arête associée à la ligne
    std::istringstream iss(line);
    std::string s1,s2;
    double d;
    iss>>s1>>s2>>d;
    Edge edge(s1,s2,d);
    listEdges.push_back(edge);
    
  }
    //on va désormais construire le dictionnaire à partir de la liste de de toutes les arêtes lues
    for (Edge edge : listEdges){ 
        std::vector<Edge >  edgeNeighbors;

        for(Edge edgePrime : listEdges){
            if(edge.startName()==edgePrime.startName()){
                
                edgeNeighbors.push_back(edgePrime);
            }
        }
        std::string name = edge.startName();
        dictOfEdges.insert({name,edgeNeighbors}); // on insert le couple {nom du sommet, liste des arêtes sortantes de ce sommet}
    }
    return dictOfEdges;
  }


// La fonction neighbors renvoie la liste des index des sommets voisins d'un sommet d'origine d'index i
std::vector<int> neighbors(AdjacentMatrixRepresentation& graph, int i){
   std::vector<int> neighbors;
  int dim = graph.dimension();
  for (int j = 0;j<dim;j++){
    if (graph.edgeAccess(graph.getReverseDictKeys()[i],graph.getReverseDictKeys()[j]) != std::numeric_limits<double>::infinity()){ // deux sommets sont voisins si l'arête qui les relie 
    //n'est pas de valeur infinie
      neighbors.push_back(j);
    }
  }
  return neighbors;
}

// Q3: PARCOURS RECURSIF

// PARCOURS EN PROFONDEUR avec la représentation en matrice d'adjacence

// La fonction explore est une fonction récursive qui recherche les sommets voisins d'un sommet d'origine
//qui n'ont pas encore été parcourus (pas flagged)
void explore(AdjacentMatrixRepresentation& graph ,int origin,std::vector<bool>& flaggedNodes){
  int dim = graph.dimension();
  flaggedNodes[origin] = true;
  std::string originName = graph.getReverseDictKeys()[origin];
  std::cout << originName << ", ";
  for (int neighbor : neighbors(graph,origin)){
    if (!flaggedNodes[neighbor]){
      explore(graph, neighbor, flaggedNodes); // La récursivité fonction car flaggedNodes diminue à chaque appel de explore
    }
  }
}

// La fonction depthFirstSearchR parcourt en profondeur un graphe à l'aide de explore
void depthFirstSearchR(AdjacentMatrixRepresentation& graph){
  int dim = graph.dimension();
  std::vector<bool> flaggedNodes(dim, false);
  for (int i = 0;i < dim; i++ ){
    if (!flaggedNodes[i]){
      explore(graph,i,flaggedNodes);
    }
  }
  std::cout << std::endl;
}

// PARCOURS EN PROFONDEUR avec la représentation en liste d'adjacence
// J'ai construit de la même façon que les trois fonctions précédentes
// les fonctions équivalentes pour la représentation avec les listes d'adjacence
std::vector<int> neighborsForList(AdjacentListRepresentation& graph,int& origin){
  std::vector<int> neighbors;
  for(std::string neighbor:graph.getNeighbors(graph.dictReverseKeys()[origin])){
    neighbors.push_back(graph.dictKeys()[neighbor]);
  }
  return neighbors;
}

void exploreList(AdjacentListRepresentation& graph , int& origin, std::vector<int>& flaggedNodes){
  int dim = graph.getDimension();
  std::cout << graph.getKeys()[origin] << ", ";
  flaggedNodes[origin] = 1;
  for (int neighbor : neighborsForList(graph,origin)){
    if (flaggedNodes[neighbor]!=1){
      exploreList(graph, neighbor, flaggedNodes);
    }
  }
}


void depthFirstSearchList(AdjacentListRepresentation& graph){
  int dim = graph.getDimension();
  std::vector<int> flaggedNodes(dim,0);

  for (const auto& kv : graph.dictKeys()){
    if (flaggedNodes[graph.dictKeys()[kv.first]]!=1){
      exploreList(graph,graph.dictKeys()[kv.first],flaggedNodes);
    }
  }
  std::cout << std::endl;
}


//Q4: PARCOURS ITERATIF

// PARCOURS EN PROFONDEUR avec la représentation en matrice d'adjacence

// Le parcours en profondeur itératif fonctionne à l'aide d'une pile d'entier qui contient les index des sommets voisins
void depthFirstSearchI(const std::string& originName, AdjacentMatrixRepresentation& graph){
    int origin = graph.getDictKeys()[originName];
    IntStack stack(graph.dimension());
    std::vector<bool> flaggedNodes(graph.dimension(), false);
    stack.push(origin);
    flaggedNodes[origin] = true;
    while (!stack.isEmpty()) {
      int node = stack.pop(); //On enlève le sommet en haut de la pile
      std::cout << graph.getReverseDictKeys()[node] <<", "; // On l'affiche
      for (int neighbor: neighbors(graph,node)){ //On regarde ses voisins
        if (!flaggedNodes[neighbor]) {
          stack.push(neighbor); //Si le voisin n'a pas encore été parcouru, on l'ajoute à la pile et on le flag
          flaggedNodes[neighbor] = true;
        }
      }
    }
    std::cout << std::endl;
  }


// PARCOURS EN PROFONDEUR itératif avec la représentation en liste d'adjacence
// Le programme fonctionne exactement de la même façon que le précédent,
// on change juste le type de représentation de graphe
void depthFirstSearchI_List(const std::string& origin, AdjacentListRepresentation& graph){
    IntStack stack(graph.getDimension());
    std::vector<int> flaggedNodes(graph.getDimension(), 0);
    stack.push(graph.dictKeys()[origin]);
    flaggedNodes[graph.dictKeys()[origin]] = 1;
    while (!stack.isEmpty()) {
      int node = stack.pop();
      std::cout << graph.dictReverseKeys()[node] <<", ";
      for (int neighbor: neighborsForList(graph,node)){
        if (flaggedNodes[neighbor]!=1) {
          stack.push(neighbor);
          flaggedNodes[neighbor] = 1;
        }
      }
    }
    std::cout << std::endl;
  }


//Q5: PARCOURS EN LARGEUR 

// Parcours en largeur avec la représentation en matrice d'adjacence

int BreadthFirstSearch(const std::string& originName, AdjacentMatrixRepresentation& graph, int depthMax){
    int origin = graph.getDictKeys()[originName];
    int dim = graph.dimension();
    std::vector<bool> flaggedNodes(dim, false);
    Queue nodes(dim+1); //On initialise une file associée aux sommets
    Queue depths(dim+1);  //On initialise une file associée aux profondeurs
    nodes.push(origin);
    depths.push(0);
    flaggedNodes[origin] = true;
    while (!nodes.isEmpty()){ //tant qu'il reste des sommets dans la file on entre dans la boucle
      int node = nodes.pull();
      int depthNode = depths.pull();
      std::cout << "sommet : " << graph.getReverseDictKeys()[node] << ", profondeur : " << depthNode << std::endl;
      if (depthNode == depthMax){ //on teste si la profondeur atteint la profondeur maximale désirée
        return EXIT_SUCCESS;
      }
      //on parcourt les voisins du sommet node qui ne sont pas encore parcourus et on les ajoute à la file
      for (int neighbor : neighbors(graph,node)){
        if (!flaggedNodes[neighbor]) {
          nodes.push(neighbor);
          depths.push(depthNode+1);
          flaggedNodes[neighbor] = true;
        }
      }
    }
    return EXIT_SUCCESS;
  }

// Parcours en largeur avec la représentation en liste d'adjacence

int BreadthFirstSearchList(const std::string& origin, AdjacentListRepresentation& graph, int depthMax){
    int dim = graph.getDimension();
    std::vector<bool> flaggedNodes(dim, false);
    Queue nodes(dim+1); //On initialise une file associée aux sommets
    Queue depths(dim+1);  //On initialise une file associée aux profondeurs
    nodes.push(graph.dictKeys()[origin]);
    depths.push(0);
    flaggedNodes[graph.dictKeys()[origin]] = true;
    while (!nodes.isEmpty()){ //tant qu'il reste des sommets dans la file on entre dans la boucle
      int node = nodes.pull();
      int depthNode = depths.pull();
      std::cout << "sommet: "  << graph.getKeys()[node] << ", profondeur: " << depthNode << std::endl;
      if (depthNode == depthMax){ //on teste si la profondeur atteint la profondeur maximale désirée
        return EXIT_SUCCESS;
      }
      //on parcourt les voisins du sommet node qui ne sont pas encore parcourus et on les ajoute à la file
      for (int neighbor : neighborsForList(graph,node)){
        if (!flaggedNodes[neighbor]) {
          nodes.push(neighbor);
          depths.push(depthNode+1);
          flaggedNodes[neighbor] = true;
        }
      }
    }
    return EXIT_SUCCESS;
  }

//Q6: Algorithme de Dijkstra

//falseRemaining retourne true s'il reste un sommet non flagged, false sinon
bool falseRemaining(std::vector<bool>& flaggedNodes){
  int size = flaggedNodes.size();
  for (int i =0;i<size;i++){
    if (!flaggedNodes[i]){
      return true;
    }
  }
  return false;
}

//findNode_Of_MinimalDistance retourne le sommet voisin d'un sommet d'origine
// dont l'arête (origine,voisin) est l'arête de pondération minimale relativement aux autres arêtes sortantes du sommet d'origine
int findNode_Of_MinimalDistance( std::vector<bool>& flaggedNodes, AdjacentMatrixRepresentation& graph, std::vector<double>& distances){
  int dim = graph.dimension();
  double distanceMin = std::numeric_limits<double>::infinity(); //on initialise la distance minimale à l'infini
  int nodeMin = -1;
  for (int j = 0;j<dim;j++){ //On recherche ici la distance minimale
    if ((!flaggedNodes[j]) and (distances[j] <= distanceMin)){
        distanceMin = distances[j];
        nodeMin = j;
    }
  } 
  return nodeMin;
}

//L'algorithme de Dijkstra fait ici permet d'afficher à partir d'un sommet d'origine
// les distances minimales à tous les sommets du graphe.
void Dijkstra(AdjacentMatrixRepresentation& graph,const std::string& originName){
  int origin = graph.getDictKeys()[originName];
  int dim = graph.dimension();
  std::vector<bool> flaggedNodes(dim,false);
  std::vector<double> distances(dim,std::numeric_limits<double>::infinity());
  std::vector<std::vector<int> > previous(dim, std::vector<int>(dim,-1));
  std::vector<int> preOrigin;
  previous[0].push_back(graph.getDictKeys()[originName]);
  distances[origin] = 0;
  while (falseRemaining(flaggedNodes)){ // tant qu'ils existent des sommets qui ne sont pas flagged
    int a = findNode_Of_MinimalDistance(flaggedNodes, graph, distances);
    flaggedNodes[a] = true;
    //On parcourt les sommets voisins afin d'ajouter leur distance à origin au vecteur distances
    for (int neighbor:neighbors(graph,a)){
      if (!flaggedNodes[neighbor]){
        if ( (distances[neighbor] > distances[a] + graph.edgeAccess(graph.getReverseDictKeys()[a],graph.getReverseDictKeys()[neighbor]) ) ){
          distances[neighbor] = distances[a] + graph.edgeAccess(graph.getReverseDictKeys()[a],graph.getReverseDictKeys()[neighbor]);
          previous[neighbor].push_back(a);
        }
      }
    }
  }
  //On affiche le résultat
  for (int i = 0;i<distances.size();i++){
    std::cout << originName << " -> " <<  graph.getReverseDictKeys()[i] << " : distance minimale : " << distances[i]<< " , " << std::endl;
    std::cout << std::endl;
    if(originName != graph.getReverseDictKeys()[i]){
      std::cout << "Chemin suivi à partir de "<< originName << ": " << std::endl;
      int count =0;
      std::cout << originName<< " -> ";
      for( int j =0; j<previous[i].size(); j++){
        if((previous[i][j]!=-1) and (previous[i][j]!=graph.getDictKeys()[originName])){
          std::cout << graph.getReverseDictKeys()[previous[i][j]] << " -> " ;
        }
      }
      std::cout << graph.getReverseDictKeys()[i] << std::endl;
      }
      std::cout << std::endl;
  }
}

//Q7: Algorithme de Floyd-Warshall

//La fonction min calcule le minimum de deux double et le renvoie
double min(double x, double y){
  if(x<y){
    return x;
  }
  else{
    return y;
  }
}

void FloydWarshall(AdjacentMatrixRepresentation& graph){
  int dim = graph.dimension();
  std::vector< std::vector< double > > matrix(dim, std::vector<double>(dim));
  matrix = graph.getMatrix();
  for (int k=0;k<dim ;k++){
    for (int i=0;i<dim ;i++){
      for (int j=0;j<dim;j++){
        matrix[i][j] = min(matrix[i][j],matrix[i][k] + matrix[k][j]); // on prendre le minimum de la distance
        // (ie. la valeur des arêtes) pour aller du sommet à i au sommet j
      }
    }
  }
  //Cette partie permet d'afficher la matrice et les sommets pour identifier les distances
  std::unordered_map<int, std::string> dictReverse;
    std::cout << "   ";
    for (const auto& kv : graph.getDictKeys()){
        dictReverse[kv.second]=kv.first;
        std::cout << kv.first << ", ";
    }

    /*for (const auto& kv : dictReverse){
        std::cout << kv.second << ", ";
    }*/
    std::cout << std::endl;

  int i =0;
    for (int i = 0; i<matrix.size(); i++){
        std::cout << dictReverse[i] <<": " ;
        int j = 0;
        for (int j = 0; j<matrix.size(); j++){
            std::cout << matrix[i][j] << ", ";
        }
        std::cout << std::endl;
    }
}




int main(){
//Q1 et Q2
// On lit le fichier graph.graph et on créé un dictionnaire
std::unordered_map<std::string, std::vector<Edge> > dict2= read("graph.graph");

//REPRESENTATION EN LISTE D'ADJACENCE
std::cout << "Q1 : REPRESENTATION EN LISTE D'ADJACENCE " << std::endl;
std::cout << std::endl;
//On initialise la représentation en liste à l'aide du dictionnaire crée ci-dessus 
AdjacentListRepresentation graph(dict2, dict2.size()); //pour la dimension je regarde la taille de la liste des clés
//On peut, comme demandé, accéder à une arête à partir du nom de ses sommets
std::cout << "Accès à une arête:  " << std::endl;
std::cout << std::endl;
std::cout << "sommet de départ: " << "d" <<  " -> sommet d'arrivée: " << "a" << " , valeur: " << graph.edgeAccess("d","a") <<std::endl;
std::cout << std::endl;
//On affiche la représentation en liste
std::cout << "print:  " << std::endl;
std::cout << std::endl;
graph.print();
std::cout << std::endl;
//REPRESENTATION EN MATRICE D'ADJACENCE
std::cout << "Q1 : REPRESENTATION EN MATRICE D'ADJACENCE " << std::endl;
std::cout << std::endl;
//On initialise la représentation en matrice à l'aide du changement de représentation
AdjacentMatrixRepresentation mgraph= graph.changeRepresentation();
//On peut, comme demandé, accéder à une arête à partir du nom de ses sommets
std::cout << "Accès à une arête:  " << std::endl;
std::cout << std::endl;
std::cout << "sommet de départ: " << "d" <<  " -> sommet d'arrivée: " << "a" << " , valeur: " << mgraph.edgeAccess("d","a") <<std::endl;
std::cout << std::endl;
//On affiche la représentation matricielle
std::cout << "print:  " << std::endl;
std::cout << std::endl;
std::cout << "ligne: sommet entrant " << std::endl;
std::cout << "colonne: sommet sortant " << std::endl;
std::cout << std::endl;
mgraph.print();
std::cout << std::endl;
//On affiche les équivalences nom <-> index
std::cout << "les équivalences nom <-> index: " << std::endl;
std::cout << std::endl;
std::unordered_map<int, std::string> dictKeysR = mgraph.getReverseDictKeys();
for(const auto& kv: dictKeysR){
  std::cout << kv.second<< " <-> " << kv.first << " , "; 
}
std::cout << std::endl;

//Q3: PARCOURS EN PROFONDEUR RECURSIF
std::cout << std::endl;
std::cout << "Q3: PARCOURS EN PROFONDEUR RECURSIF " << std::endl;
std::cout << std::endl;

// Représentation matricielle
std::cout << "Représentation matricielle: " << std::endl;
std::cout << std::endl;
std::cout << "Algorithme récursif: " << std::endl;
depthFirstSearchR(mgraph);
std::cout << std::endl;

// Représentation par liste d'adjacence
std::cout << "Représentation par liste d'adjacence: " << std::endl;
std::cout << std::endl;
std::cout << "Algorithme récursif: " << std::endl;
std::cout << std::endl;
depthFirstSearchList(graph);

//Q4: PARCOURS EN PROFONDEUR ITERATIF
std::cout << std::endl;
std::cout << "Q4: PARCOURS EN PROFONDEUR ITERATIF " << std::endl;
std::cout << std::endl;

// Représentation matricielle
std::cout << "Représentation matricielle: " << std::endl;
std::cout << std::endl;
std::cout << "Algorithme itératif en partant du sommet d: " << std::endl;
std::cout << std::endl;
depthFirstSearchI("d",mgraph);

// Représentation par liste d'adjacence
std::cout << std::endl;
std::cout << "Représentation par liste d'adjacence: " << std::endl;
std::cout << std::endl;
std::cout << "Algorithme itératif: " << std::endl;
std::cout << std::endl;
depthFirstSearchI_List("d", graph);

//Q5: PARCOURS EN LARGEUR 
std::cout << std::endl;
std::cout << "Q5: PARCOURS EN LARGEUR " << std::endl;
std::cout << std::endl;

// Représentation matricielle
std::cout << "Représentation matricielle: " << std::endl;
std::cout << std::endl;
BreadthFirstSearch("d", mgraph, 5);

// Représentation par liste d'adjacence
std::cout << std::endl;
std::cout << "Représentation par liste d'adjacence: " << std::endl;
std::cout << std::endl;
BreadthFirstSearchList("d", graph, 5);

//Q6: ALGORITHME DE DIJKSTRA
std::cout << std::endl;
std::cout << "Q6: ALGORITHME DE DIJKSTRA en partant du sommet a" << std::endl;
std::cout << std::endl;
Dijkstra(mgraph,"a");

//Q7: ALGORITHME DE FLOYD-WARSHALL
std::cout << std::endl;
std::cout << "Q7: ALGORITHME DE FLOYD-WARSHALL " << std::endl;
std::cout << std::endl;
std::cout << "ligne: sommet entrant " << std::endl;
std::cout << "colonne: sommet sortant " << std::endl;
std::cout << std::endl;
FloydWarshall(mgraph);
}

