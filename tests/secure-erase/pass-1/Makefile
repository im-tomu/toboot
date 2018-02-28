PACKAGE    = secure-erase-pass1
ADD_CFLAGS =
ADD_LFLAGS = 

TRGT      ?= arm-none-eabi-
CC         = $(TRGT)gcc
CXX        = $(TRGT)g++
OBJCOPY    = $(TRGT)objcopy

LDSCRIPT   = tomu.ld
DBG_CFLAGS = -ggdb -g -DDEBUG -Wall
DBG_LFLAGS = -ggdb -g -Wall
CFLAGS     = $(ADD_CFLAGS) \
			 -DTOBOOT_APP_FLAGS=0x00006fb0 \
             -Wall -Wextra \
             -mcpu=cortex-m0plus -mfloat-abi=soft -mthumb \
             -ffunction-sections -fdata-sections -fno-common \
             -fomit-frame-pointer -Os
CXXFLAGS   = $(CFLAGS) -std=c++11 -fno-rtti -fno-exceptions
LFLAGS     = $(ADD_LFLAGS) $(CFLAGS) \
             -nostartfiles \
             -Wl,--gc-sections \
             -Wl,--no-warn-mismatch,--script=$(LDSCRIPT),--build-id=none

OBJ_DIR    = .obj

CSOURCES   = $(wildcard *.c)
CPPSOURCES = $(wildcard *.cpp)
ASOURCES   = $(wildcard *.S)
COBJS      = $(addprefix $(OBJ_DIR)/, $(notdir $(CSOURCES:.c=.o)))
CXXOBJS    = $(addprefix $(OBJ_DIR)/, $(notdir $(CPPSOURCES:.cpp=.o)))
AOBJS      = $(addprefix $(OBJ_DIR)/, $(notdir $(ASOURCES:.S=.o)))
OBJECTS    = $(COBJS) $(CXXOBJS) $(AOBJS)
VPATH      = .

QUIET      = @

ALL        = all
TARGET     = $(PACKAGE).elf
CLEAN      = clean

$(ALL): $(TARGET)

$(OBJECTS): | $(OBJ_DIR)

$(TARGET): $(OBJECTS) $(LDSCRIPT)
	$(QUIET) echo "  LD       $@"
	$(QUIET) $(CXX) $(OBJECTS) $(LFLAGS) -o $@
	$(QUIET) echo "  OBJCOPY  $(PACKAGE).bin"
	$(QUIET) $(OBJCOPY) -O binary $@ $(PACKAGE).bin
	$(QUIET) echo "  IHEX     $(PACKAGE).ihex"
	$(QUIET) $(OBJCOPY) -O ihex $@ $(PACKAGE).ihex

$(DEBUG): CFLAGS += $(DBG_CFLAGS)
$(DEBUG): LFLAGS += $(DBG_LFLAGS)
CFLAGS += $(DBG_CFLAGS)
LFLAGS += $(DBG_LFLAGS)
$(DEBUG): $(TARGET)

$(OBJ_DIR):
	$(QUIET) mkdir $(OBJ_DIR)

$(COBJS) : $(OBJ_DIR)/%.o : %.c Makefile
	$(QUIET) echo "  CC       $<	$(notdir $@)"
	$(QUIET) $(CC) -c $< $(CFLAGS) -o $@ -MMD

$(OBJ_DIR)/%.o: %.cpp
	$(QUIET) echo "  CXX      $<	$(notdir $@)"
	$(QUIET) $(CXX) -c $< $(CXXFLAGS) -o $@ -MMD

$(OBJ_DIR)/%.o: %.S
	$(QUIET) echo "  AS       $<	$(notdir $@)"
	$(QUIET) $(CC) -x assembler-with-cpp -c $< $(CFLAGS) -o $@ -MMD

.PHONY: $(CLEAN)

$(CLEAN):
	$(QUIET) rm -f $(wildcard $(OBJ_DIR)/*.d)
	$(QUIET) rm -f $(wildcard $(OBJ_DIR)/*.o)
	$(QUIET) rm -f $(TARGET) $(PACKAGE).bin $(PACKAGE).symbol

include $(wildcard $(OBJ_DIR)/*.d)
