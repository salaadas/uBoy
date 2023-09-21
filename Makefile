CXX=g++
CFLAGS=-Wall -Wextra -std=c++17 -ggdb
LIBS=-lm
BIN=uBoy

SRC=$(wildcard src/*.cpp)

$(BIN) : $(SRC)
	$(CXX) $(CFLAGS) $^ -o $@ $(LIBS)
