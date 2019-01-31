default:
	gcc -o quaternion.o -c quaternion.c
	gcc -o calib -l rt calib.c quaternion.o -lm -Ofast
	gcc -o compass_calib compass_calib.c
clean:
	rm calib compass_calib *~ *# *.o
