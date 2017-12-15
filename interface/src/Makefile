INCLUDES += -I./ -I$(INTERFACE_INCLUDE_DIR)
C_FLAGS += -Wall -O3 -lpthread
AR_FLAGS += -r

CC = $(CROSS_COMPILE)gcc $(INCLUDES) $(C_FLAGS) -c
AR = $(CROSS_COMPILE)ar
RM = rm -rf

REL_LIB = osa_utils.a

REL_SRC	:= $(wildcard *.c)
REL_OBJ	:= $(REL_SRC:%.c=release/%.o)

HEADERS = $(wildcard *.h) $(INTERFACE_INCLUDE_DIR)/*.h

all: mk $(REL_LIB)
	make install
	-$(RM) *.o
mk:
	-mkdir release
	-mkdir -p $(INTERFACE_LIB_DIR)	
$(REL_LIB): $(REL_OBJ)	
	$(AR) $(AR_FLAGS) $(REL_LIB) $(REL_OBJ)
$(REL_OBJ):	release/%.o: %.c $(HEADERS)
	$(CC) -o $@ $<	
clean:
	-$(RM) -f *.o
	-$(RM) -Rf $(PUBLIC_LIB_DIR)
	
install: 
	cp 	$(REL_LIB) $(INTERFACE_LIB_DIR)/
