lib1:
	gcc -std=gnu11 -Wall -Wextra -Wpedantic -O3 -ffast-math -march=native -Isrc -I"C:/mjpro150/include" -MMD -c src/dubPendMujoco.c -o obj/dubPendMujoco.o

lib2:
	gcc obj/dubPendMujoco.o -o dubPendMujoco.dll -shared -L"C:/mjpro150/bin" -lmujoco150 -lglfw3

rbdl: 	
	g++ src/dubPendRBDL.cc -I"C:/RBDL/include" "C:/RBDL/lib/librbdl.a" -Isrc -shared -I"C:/mingw64/include" -Wno-attributes -o dubPendRBDL.dll