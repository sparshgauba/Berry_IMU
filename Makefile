default:
	gcc -o calib -l rt calib.c -lm
clean:
	rm calib *~ *#
