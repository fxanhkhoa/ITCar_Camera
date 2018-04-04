mainmake: main.cpp itcar_contours.cpp
	g++ main.cpp -o output `pkg-config --cflags --libs opencv`
