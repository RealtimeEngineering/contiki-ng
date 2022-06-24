CONTIKI_PROJECT = udp-client udp-server
all: $(CONTIKI_PROJECT)

TARGET = simplelink
BOARD = launchpad/cc1352r1
OPTIMAIZATION = -O0
CFLAGS += -g

CONTIKI=../..
include $(CONTIKI)/Makefile.dir-variables
MODULES += $(CONTIKI_NG_SERVICES_DIR)/rpl-border-router
# Include optional target-specific module
include $(CONTIKI)/Makefile.identify-target
MODULES_REL += $(TARGET)

include $(CONTIKI)/Makefile.include
