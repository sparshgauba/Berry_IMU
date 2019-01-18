default:
	gcc -o calib -l rt calib.c MahonyAHRS.c -lm
	gcc -o compass_calib compass_calib.c
clean:
	rm calib compass_calib *~ *# *.o
