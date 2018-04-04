mainmake: main.cpp itcar_contours.cpp itcar_vanishingpoint_video.cpp
	g++ itcar_vanishingpoint_video.cpp -o output `pkg-config --cflags --libs opencv`
