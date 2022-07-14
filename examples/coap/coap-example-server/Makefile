CONTIKI_PROJECT = coap-example-server
all: $(CONTIKI_PROJECT)

TARGET = simplelink
BOARD = launchpad/cc1352r1
OPTIMAIZATION = -O0
CFLAGS += -g
MAKE_WITH_DTLS = 1
MAKE_COAP_DTLS_KEYSTORE = MAKE_COAP_DTLS_KEYSTORE_SIMPLE

# Do not try to build on Sky because of code size limitation
PLATFORMS_EXCLUDE = sky z1

# Include CoAP resources
MODULES_REL += ./resources

CONTIKI=../../..

# Include the CoAP implementation
include $(CONTIKI)/Makefile.dir-variables
MODULES += $(CONTIKI_NG_APP_LAYER_DIR)/coap
MODULES += $(CONTIKI_NG_SERVICES_DIR)/rpl-border-router
# Include optional target-specific module
include $(CONTIKI)/Makefile.identify-target
MODULES_REL += $(TARGET)

include $(CONTIKI)/Makefile.include
