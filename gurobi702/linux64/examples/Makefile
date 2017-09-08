# Copyright 2017, Gurobi Optimization, Inc.

PLATFORM = linux64
INC      = ../include/
CPP      = g++-4.9
CARGS    = -m64 -g
CPPLIB   = -L../lib/ -lgurobi_c++ -lgurobi70

mip1_c++:
	$(CPP) $(CARGS) ./c++/mip1_c++.cpp -o ./build/mip1_c++ -I$(INC) $(CPPLIB) -lm
run_mip1_c++: mip1_c++
	./build/mip1_c++
clean:
	rm -rf *.o *_c *_c++ *.class *.log *.rlp *.lp *.bas *.ilp *.mps
