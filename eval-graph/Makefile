CPP=g++ -std=c++20
RM=rm -f

graph : adjacentMatrixRepresentation.o edge.o adjacentListRepresentation.o intStack.o queue.o main.o
	$(CPP) $^ -o $@

# vérifiez les dépendances
edge.o : edge.cpp edge.h
adjacentMatrixRepresentation.o : adjacentMatrixRepresentation.cpp adjacentMatrixRepresentation.h
adjacentListRepresentation.o : adjacentListRepresentation.cpp adjacentListRepresentation.h edge.h adjacentMatrixRepresentation.h
intStack.o : intStack.cpp intStack.h
queue.o : queue.cpp queue.h
main.o : main.cpp adjacentListRepresentation.h intStack.h queue.h

%.o:%.cpp
	$(CPP) -o $@ -c $<

clean:
	$(RM) *.o graph

.PHONY: clean
