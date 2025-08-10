CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -g
TARGET = m6502
SRC = m6502.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $^ -o $@

clean:
	rm -f $(TARGET)
