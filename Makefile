# Makefile

OBJS = main.o frizzController.o common.o serial.o spi.o sensor_buff.o frizzDriver.o 
CC = gcc
TARGET = Sample_demo
CFLAGS = -Wall -Werror -O2 -Iinc
INC_LIBS = -pthread -lwiringPi -lwiringPiDev
.SUFFIXES: .c .o

.PHONY: all
all: $(TARGET) 

$(TARGET): $(OBJS)
	$(CC) $(INC_LIBS) -o $@ $^ 

.c.o:
	$(CC) $(CFLAGS) -c $<

.PHONY: clean
clean:
	$(RM) $(TARGET) $(OBJS) 
