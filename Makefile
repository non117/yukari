CXXC = clang++
CXXFLAGS = -O3 --std=c++11
OBJS = yukari.o research.o
TARGET = yukari

all: $(TARGET)

$(TARGET):$(OBJS)
		$(CXXC) $(OBJS) $(CXXFLAGS) -o $(TARGET)

clean:
	-rm -f yukari $(OBJS)

.cpp.o:
	$(CXXC) -c $< $(CXXFLAGS)

yukari.o: research.hpp
research.o: research.hpp
