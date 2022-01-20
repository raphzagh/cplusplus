#include <iostream>

//Création d'une classe pile afin de mettre en place le parcours en profondeur itératif
//J'ai repris la classe que nous avions fait en cours
class IntStack {

    int size; //taille max de la pile 
    int nb; // nombre de termes de la pile occupés
    int* tab; 

    public:

    IntStack(int s);
    ~IntStack(){
        delete[] tab;
    }
    IntStack(const IntStack &r):
        size(r.size),
        nb(r.nb),
        tab(new int[size]){}
    void push (int e); // placer un nouvel entier en haut de la pile
    int pop(); // retirer l'élément en haut de la pile et le retourner
    int sizeStack();
    bool isEmpty();
    bool isFull();
    void print();
    IntStack operator=(const IntStack& r){
        {size= r.size;
        nb= r.nb;
        delete[] tab;
        tab= new int [size];}
        return *this;
    }
};