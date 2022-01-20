#include "adjacentMatrixRepresentation.h"

AdjacentMatrixRepresentation::AdjacentMatrixRepresentation(std::vector< std::vector<double> > matrix, std::unordered_map<std::string, int> dictKeys)
: matrix(matrix),
  dictKeys(dictKeys)
{}


double AdjacentMatrixRepresentation::edgeAccess(std::string startNode, std::string endNode){
    int i = dictKeys[startNode];
    int j = dictKeys[endNode];
    return matrix[i][j];
}

int AdjacentMatrixRepresentation::dimension(){
    return matrix.size();
}

void AdjacentMatrixRepresentation::print(){
    std::unordered_map<int, std::string> dictReverse;
    for (const auto& kv : dictKeys){
        dictReverse[kv.second]=kv.first;
    }
    std::cout << "   ";
    for (const auto& kv : dictReverse){
        std::cout << kv.second << ", ";
    }
    std::cout << std::endl;
    int i =0;
    for (int i = 0; i<matrix.size(); i++){
        int j = 0;
        std::cout << dictReverse[i] <<": " ;
        for (int j = 0; j<matrix.size(); j++){
            std::cout << matrix[i][j] << ", ";
        }
        std::cout << std::endl;
    }
}

std::vector< std::vector<double> > AdjacentMatrixRepresentation::getMatrix(){
    return matrix;
}

std::unordered_map<std::string, int> AdjacentMatrixRepresentation::getDictKeys(){
    return dictKeys;
}

std::unordered_map<int, std::string> AdjacentMatrixRepresentation::getReverseDictKeys(){
    std::unordered_map<int, std::string> dictReverse;
    for (const auto& kv : dictKeys){
        dictReverse[kv.second]=kv.first;
    }
    return dictReverse;
}