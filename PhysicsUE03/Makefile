LIBS=-lsfml-graphics -lsfml-window -lsfml-system -lreactphysics3d -lreactphysics3d
SLIBS=-L../reactphysics3d/lib
#isystem suppresses warnings coming from the include dir
INCLUDE=-isystem ../reactphysics3d/src
CXX = clang++
CXXFLAGS = -c -ggdb -O0 -Wall -Wextra -std=c++11
LDFLAGS = -ggdb -O0 -std=c++11

EXECUTABLE=hysteresis
BUILDDIR = ./build
SOURCEDIR = ./src

.PHONY: clean

all: dir $(BUILDDIR)/$(EXECUTABLE)

dir: 
	mkdir -p $(BUILDDIR)

SOURCES = $(wildcard $(SOURCEDIR)/*.cpp)
OBJECTS = $(patsubst $(SOURCEDIR)/%.cpp,$(BUILDDIR)/%.o,$(SOURCES))

#Would love to understand this a bit better, to make this easier
#this is called static pattern and enables "make foo", which 
#I don't need
$(OBJECTS): $(BUILDDIR)/%.o : $(SOURCEDIR)/%.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(INCLUDE)

$(BUILDDIR)/$(EXECUTABLE): $(OBJECTS)
	@echo "** Building the game"
	$(CXX) $^ -o $(BUILDDIR)/$(EXECUTABLE) $(LIBS) $(INCLUDE) $(LDFLAGS) $(SLIBS)

clean:
	@echo "** Removing object files and executable..."
	rm -f $(BUILDDIR)/*.o $(BUILDDIR)/$(EXECUTABLE)
