# Makefile for Writing Make Files Example
 
# *****************************************************
# Variables to control Makefile operation
 
CC = g++
CFLAGS = -Wall -g
 
# ****************************************************
# Targets needed to bring the executable up to date
 
timesync: timesync.o
	$(CC) $(CFLAGS) -o timesync timesync.o
 
# The main.o target can be written more simply
 
timesync.o: timesync.cpp time_util.h timestamp_t.hpp mbot_channels.h lcm_config.h
	$(CC) $(CFLAGS) -c timesync.cpp
