#include "adjacentListRepresentation.h"


AdjacentListRepresentation::AdjacentListRepresentation(std::unordered_map<std::string, std::vector<Edge> > dictOfEdges,int dimension):
    dictOfEdges(dictOfEdges),
    dimension(dimension)
{}

std::vector<std::string> AdjacentListRepresentation::getKeys(){
    std::vector<std::string> keys;

    keys.reserve(dictOfEdges.size());
    for (const auto& kv : dictOfEdges){ // on parcourt les clés du dictionnaire
        keys.push_back(kv.first);
    }

    return keys;
}

double AdjacentListRepresentation::edgeAccess(std::string startName, std::string endName){
    for(Edge edge: dictOfEdges[startName]){ //on parcourt les arêtes dont le sommet de départ est startName
        if( edge.endName()!= endName){ // si le sommet d'arrivée n'est pas le même on passe cette itération
            continue;
        }
        else{                          // sinon on retourne la valeur
            return edge.value(); 
        }
    }
    return  std::numeric_limits<double>::infinity(); // si la valeur n'existe pas, 
    //on lui donne la valeur infini, ce qui va m'être utile pour les algorithmes de parcourt et de plus court chemin
}



AdjacentMatrixRepresentation AdjacentListRepresentation::changeRepresentation(){
    int i=0;
    std::vector< std::vector<double> > matrix(dimension, std::vector<double>(dimension));
    std::vector<std::string> keys;
    // Je construis ici la liste des clés de la unordered_map.
    keys.reserve(dictOfEdges.size());
    for (const auto& kv : dictOfEdges ){
        keys.push_back(kv.first);
    }
    
    for(int i =0; i!=dimension; i++ ){
        int j = 0; 
        for(int j =0; j!=dimension; j++ ){
             matrix[i][j]=edgeAccess(keys[i],keys[j]); // on donne au coefficient (i,j) de la matrice de dimension dimxdim
            //la valeur double de l'arrête associée
        }
    }

    std::unordered_map<std::string, int> dictOfKeys;
    for(int i = 0; i<keys.size(); i++){
        dictOfKeys[keys[i]]=int(i);
    }    

    return AdjacentMatrixRepresentation(matrix, dictOfKeys); //on retourne le graphe sous forme de matrice d'ajadjacence associé à matrix
}

int AdjacentListRepresentation::getDimension(){
    return dimension;
}

void AdjacentListRepresentation::print(){
    std::vector<std::string> keys;
    // Je construis ici la liste des clés de la unordered_map.
    keys.reserve(dictOfEdges.size());
    for (const auto& kv : dictOfEdges ){
        keys.push_back(kv.first);
    }
    int i = 0;
    for(int i =0; i!=dimension; i++ ){
        int j = 0; 
        std::cout << keys[i];
        for(int j =0; j!=dimension; j++ ){
            if(edgeAccess(keys[i],keys[j])==std::numeric_limits<double>::infinity()){
                continue;
            }
            else{
                std::cout << " -> "<< keys[j] << ": " << edgeAccess(keys[i],keys[j]) << std::endl;
            }
        }
    }
}

std::unordered_map<std::string, std::vector<Edge> > AdjacentListRepresentation::getDict(){
    return dictOfEdges;
}

std::unordered_map<std::string, int> AdjacentListRepresentation::dictKeys(){
    std::unordered_map<std::string, int> dict;
    std::vector<std::string> keys;
    keys.reserve(dictOfEdges.size());
    for (const auto& kv : dictOfEdges){
        keys.push_back(kv.first);
    }
    for(int i = 0; i<keys.size(); i++){
        dict[keys[i]]=int(i);
    }
    return dict;
}

std::unordered_map<int, std::string> AdjacentListRepresentation::dictReverseKeys(){
    std::unordered_map<int, std::string> dict;
    std::vector<std::string> keys;
    keys.reserve(dictOfEdges.size());
    for (const auto& kv : dictOfEdges){
        keys.push_back(kv.first);
    }
    for(int i = 0; i<keys.size(); i++){
        dict[i]=keys[i];
    }
    return dict;
}
// Cette méthode renvoie le vecteur des noms des sommets voisins à un sommet d'origine en argument
std::vector<std::string> AdjacentListRepresentation::getNeighbors(std::string origin){ 
    std::vector<std::string> neighbors;
    for(Edge edge : dictOfEdges[origin]){
        neighbors.push_back(edge.endName());
    }
    return neighbors;
}