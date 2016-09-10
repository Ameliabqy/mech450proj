# Compilation flags
CXX_FLAGS = -std=c++11 -Wall -O2 # change -O2 to -g when debugging code
# Include directories
INCLUDE_FLAGS = -I.
# Linker options
LD_FLAGS =
# The c++ compiler to invoke
CXX=c++

all: Project2

Project2: Project2.o CollisionChecking.o
	$(CXX) $(CXX_FLAGS) -o Project2 Project2.o CollisionChecking.o $(LD_FLAGS)

clean:
	rm *.o

%.o: %.cpp
	$(CXX) -c $(CXX_FLAGS) $(INCLUDE_FLAGS) $< -o $@