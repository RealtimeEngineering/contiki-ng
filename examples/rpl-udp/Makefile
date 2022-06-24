CONTIKI_PROJECT = udp-client udp-server
all: $(CONTIKI_PROJECT)

TARGET = simplelink
BOARD = launchpad/cc1352r1
OPTIMAIZATION = -O0
CFLAGS += -g

CONTIKI=../..
include $(CONTIKI)/Makefile.dir-variables
MODULES += os/services/shell

include $(CONTIKI)/Makefile.include
