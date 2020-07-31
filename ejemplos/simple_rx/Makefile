# simple_pkt_fwd
# Simple Packet Gateway

CC=g++
CFLAGS=-c -Wall
#LIBS=-lwiringPi -lm -lgps
LIBS=-lwiringPi -lm


all: simple_pkt_fwd

simple_pkt_fwd: base64.o main.o
	$(CC) main.o base64.o $(LIBS) -o simple_pkt_fwd

main.o: main.cpp
	$(CC) $(CFLAGS) main.cpp

base64.o: base64.c
	$(CC) $(CFLAGS) base64.c

clean:
	rm *.o simple_pkt_fwd	
