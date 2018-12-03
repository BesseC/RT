#!/bin/sh

which sdl2-config
if [ $? == 1 ]; then
	./vendor/SDL/install.sh SDL2-2.0.8
fi
echo `sdl2-config --cflags` > .sdl_includes
echo `sdl2-config --libs` > .sdl_links
