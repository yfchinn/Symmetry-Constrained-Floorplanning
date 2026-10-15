CXX = g++
CXXFLAGS = -O3 -Wall -std=c++14

all: Lab2

Lab2: main.cpp
	$(CXX) $(CXXFLAGS) main.cpp -o Lab2

clean:
	rm -f Lab2