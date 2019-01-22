default:
	gcc -o calib -l rt calib.c -lm -Ofast
	gcc -o compass_calib compass_calib.c
clean:
	rm calib compass_calib *~ *# *.o
