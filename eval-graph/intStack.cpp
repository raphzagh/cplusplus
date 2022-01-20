#include "intStack.h"

IntStack::IntStack(int s):
    size(s),
    nb(0),
    tab(new int [s])
{}

void IntStack::push(int e){
    if (nb!=size){
        tab[nb]=e;
        nb=nb+1;
    }
}

int IntStack::pop(){
    if(nb!=0){
        nb=nb-1;
        return tab[nb];
    }
    throw "NON";
}

int IntStack::sizeStack(){
    return nb;
}

bool IntStack::isEmpty(){
    return nb==0;
}

bool IntStack::isFull(){
    return nb==size;
}

void IntStack::print(){
    for(int i=0; i<nb; i++){
        std::cout << tab[i] << ',';
    }
    std::cout << std::endl;
}