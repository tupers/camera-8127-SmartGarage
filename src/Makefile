INCLUDES += -I./ -I$(MODBUS_INCLUDE_DIR) -I$(INTERFACE_INCLUDE_DIR)
C_FLAGS += -Wall -O3 -lpthread
AR_FLAGS += -r

CC = $(CROSS_COMPILE)gcc $(INCLUDES) $(C_FLAGS) -c
AR = $(CROSS_COMPILE)ar
RM = rm -rf

TARGET = Modbus_client

C_FLAGS += -Wall -O3

CPP_FLAGS += -I$(INCLUDES) \
		-I../inc \
		-D_GNU_SOURCE \
		-DAVI_BIG_FILE \
		-D_LARGEFILE64_SOURCE \
		-D_FILE_OFFSET_BITS=64 \

LD_FLAGS += -lpthread -s

COMPILE.c = $(CROSS_COMPILE)gcc $(C_FLAGS) $(CPP_FLAGS) -c
LINK.c = $(CROSS_COMPILE)gcc $(LD_FLAGS)

RELTARGET = $(TARGET)

RELCFLAGS = -fno-strict-aliasing

SOURCES = $(wildcard *.c)
HEADERS = $(wildcard *.h) $(INTERFACE_INCLUDE_DIR)/*.h  $(MODBUS_INCLUDE_DIR)/*.h

RELOBJFILES = $(SOURCES:%.c=release/%.o)

.PHONY: clean release install

all:    release

install: release
	install -d $(EXEC_DIR)
	install $(RELTARGET) $(EXEC_DIR)

release:    $(RELTARGET)
$(RELTARGET):   $(RELOBJFILES) $(MODBUS_LIB_DIR)/libmodbus.a $(INTERFACE_LIB_DIR)/osa_utils.a 
		$(LINK.c) -o $@ $^ $(RELLDFLAGS)
		-$(RM) RELOBJFILES -f


$(RELOBJFILES): release/%.o: %.c $(HEADERS)
	@mkdir -p release
	$(COMPILE.c) $(RELCFLAGS) -o $@ $<

clean:
	-$(RM) -rf release  *.d
