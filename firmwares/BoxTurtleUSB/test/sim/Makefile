# Host-side simulator for the BoxTurtle firmware: make && ./sim
SRC = ../../src/turtle
FW = $(SRC)/json.c $(SRC)/link.c $(SRC)/events.c $(SRC)/sensors.c \
     $(SRC)/proto.c $(SRC)/motion.c $(SRC)/cmds.c $(SRC)/spool.c $(SRC)/jobs.c \
     $(SRC)/guard.c
sim: sim.c $(FW) $(wildcard $(SRC)/*.h) $(wildcard include/*.h include/*/*.h)
	$(CC) -O1 -g -Wall -Wno-unused-function -Iinclude -I$(SRC) -o $@ sim.c $(FW) -lm
clean:
	rm -f sim
