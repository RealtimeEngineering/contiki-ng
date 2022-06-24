CONTIKI_PROJECT = coap-example-client
# coap-example-observe-client is outdated but will be ported at a later point
all: $(CONTIKI_PROJECT)

TARGET = simplelink
BOARD = launchpad/cc1352r1
OPTIMAIZATION = -O0
CFLAGS += -g

# Do not try to build on Sky because of code size limitation
PLATFORMS_EXCLUDE = sky z1

CONTIKI=../../..
# Include the CoAP implementation
include $(CONTIKI)/Makefile.dir-variables
MODULES += $(CONTIKI_NG_APP_LAYER_DIR)/coap
MODULES += $(CONTIKI_NG_SERVICES_DIR)/rpl-border-router
# Include optional target-specific module
include $(CONTIKI)/Makefile.identify-target
MODULES_REL += $(TARGET)

include $(CONTIKI)/Makefile.include
