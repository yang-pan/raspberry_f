# Makefile

OBJS = main.o frizzController.o common.o serial.o spi.o sensor_buff.o frizzDriver.o SD_Writer.o normal_sensor_writer.o bmd101_sensor_writer.o
OBJS2 = FuncTest.o common.o serial.o spi.o sensor_buff.o frizzDriver.o SD_Writer.o normal_sensor_writer.o bmd101_sensor_writer.o
CC = gcc
TARGET = Sensor_log
TARGET2 = FuncTest
CFLAGS = -Wall -Werror -O2 -Iinc
INC_LIBS = -pthread -lwiringPi -lwiringPiDev
.SUFFIXES: .c .o

.PHONY: all
all: $(TARGET) $(TARGET2)

$(TARGET): $(OBJS)
	$(CC) $(INC_LIBS) -o $@ $^ 

$(TARGET2): $(OBJS2)
	$(CC) $(INC_LIBS) -o $@ $^ 

.c.o:
	$(CC) $(CFLAGS) -c $<

.PHONY: clean
clean:
	$(RM) $(TARGET) $(OBJS) $(TARGET2) $(OBJS2)
