#include <iostream>
// Création de la classe file
// Je me suis inspiré de la classe pile vue en cours en adaptant ce qu'il fallait pour obtenir une file
//Contrairement à la pile qui a un fonctionnement "dernier arrivé, premier sorti", la file a un fonctionnement "premier arrivé, premier sorti" 
class Queue {

    int size; //taille maximale de la file 
    int start; //Curseur donnant le début de la file
    int end; //Curseur donnant la fin de la file
    int* tab;

    public:

    Queue(int s);
    ~Queue(){
        delete[] tab;
    }
    Queue(const Queue &r):
        size(r.size),
        start(r.start),
        end(r.end),
        tab(new int[size]){}
    void push (int e); // on ajoute à la fin de la file un nouvel entier
    int pull(); //en enlève du début de la file le premier entier arrivé
    int sizeQueue();
    bool isEmpty();
    bool isFull();
    void print();
    Queue operator=(const Queue& r){
        {size= r.size;
        start = r.start;
        end = r.end;
        delete[] tab;
        tab= new int[size];}
        return *this;
    }
};