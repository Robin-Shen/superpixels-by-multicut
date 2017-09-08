GUROBI   = ./gurobi702/linux64
BOOST    = /usr/include/boost
GNU = /usr/lib/x86_64-linux-gnu/
CPP      = g++
CARGS    = -std=c++11
LIBS   = -lgurobi_g++5.2 -lgurobi70 -lboost_program_options -lboost_system -lopencv_core -lopencv_imgproc -lopencv_highgui# -lpng -ljpeg 
################################################################################
ARGS =  --grid-size 64
################################################################################

sbm: main.o callback.o
	$(CPP) $(CARGS) -L$(GNU) -L$(GUROBI)/lib main.o callback.o $(LIBS) -o ./bin/sbm

main.o: main.cpp callback.h graph.h
	$(CPP) $(CARGS) -I$(BOOST) -I$(GUROBI)/include -c main.cpp

callback.o: callback.cpp callback.h graph.h
	$(CPP) $(CARGS) -I$(BOOST) -I$(GUROBI)/include -c callback.cpp

test:
	./bin/sbm $(ARGS)


