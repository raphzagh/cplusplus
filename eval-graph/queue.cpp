#include "queue.h"

Queue::Queue(int s):
    size(s),
    start(0),
    end(0),
    tab(new int [s])
{}


void Queue::push(int e){
    if ((start - end)%size != 1){
        tab[end]=e;
        end = end+1;
    }
}

int Queue::pull(){
    if(start-end != 0){
        start=(start+1)%size;
    }
    return tab[start-1];
    throw "NO";
}

int Queue::sizeQueue(){
    return start-end;
}

bool Queue::isEmpty(){
    return start-end==0;
}

bool Queue::isFull(){
    return (start-end)%size==1;
}

void Queue::print(){
    if (start < end){
        for(int i=start; i<end; i++){
        std::cout << tab[i] << ',';
        }
    }
    else{
        for (int i=start; i<end+size; i++){
        std::cout << tab[i%size] << ',';
            }
        }
    std::cout << std::endl;
}