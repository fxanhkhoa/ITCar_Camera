mainmake: main.cpp itcar_contours
	g++ itcar_contours.cpp -o output `pkg-config --cflags --libs opencv`
