PACKAGE      = toboot
ADD_CFLAGS   =
ADD_CPPFLAGS =
ADD_LFLAGS   =

GIT_VERSION= $(shell git describe --tags)
TRGT      ?= arm-none-eabi-
CC         = $(TRGT)gcc
OBJCOPY    = $(TRGT)objcopy
# Target board. Possible values are:
#   TOMU         The original Tomu board
#   BAM_03       Buddy-Fox A-10C UFC
BOARD      ?= TOMU

RM         = rm -rf
COPY       = cp -a
PATH_SEP   = /

ifeq ($(OS),Windows_NT)
COPY       = copy
RM         = del /q
PATH_SEP   = \\
endif

LDSCRIPT   = toboot-bl.ld
DBG_CFLAGS = -ggdb -g -DDEBUG -Wall
DBG_LFLAGS = -ggdb -g -Wall
CPPFLAGS   = -D_FORTIFY_SOURCE=2 $(ADD_CPPFLAGS)
CFLAGS     = $(ADD_CFLAGS) $(CPPFLAGS) \
             -DTOBOOT_APP_FLAGS=0x00006fb0 \
             -Wall -Wextra \
             -mcpu=cortex-m0plus -mfloat-abi=soft -mthumb \
             -ffunction-sections -fdata-sections -fno-common \
             -fomit-frame-pointer -O2 \
             -Wformat -Werror=format-security \
             -DTOBOOT_BOARD=TOBOOT_BOARD_$(BOARD) \
             -DGIT_VERSION=u\"$(GIT_VERSION)\" -std=gnu11
LFLAGS     = $(ADD_LFLAGS) $(CFLAGS) \
             -nostartfiles \
             -Wl,--gc-sections \
             -Wl,-z,relro -Wl,-z,now \
             -Wl,--no-warn-mismatch,--script=$(LDSCRIPT),--build-id=none

OBJ_DIR    = .obj

CSOURCES   = $(wildcard *.c)
ASOURCES   = $(wildcard *.S)
COBJS      = $(addprefix $(OBJ_DIR)/, $(notdir $(CSOURCES:.c=.o)))
AOBJS      = $(addprefix $(OBJ_DIR)/, $(notdir $(ASOURCES:.S=.o)))
OBJECTS    = $(COBJS) $(AOBJS)
VPATH      = .

QUIET      = @

ALL        = all
TARGET     = $(PACKAGE).elf
CLEAN      = clean

$(ALL): $(TARGET) $(PACKAGE).bin $(PACKAGE).ihex $(PACKAGE).dfu

$(OBJECTS): | $(OBJ_DIR)

$(TARGET): $(OBJECTS) $(LDSCRIPT)
	$(QUIET) echo "  LD       $@"
	$(QUIET) $(CC) $(OBJECTS) $(LFLAGS) -o $@

$(PACKAGE).bin: $(TARGET)
	$(QUIET) echo "  OBJCOPY  $(PACKAGE).bin"
	$(QUIET) $(OBJCOPY) -O binary $(TARGET) $@

$(PACKAGE).dfu: $(TARGET)
	$(QUIET) echo "  COPY  $(PACKAGE).bin $(PACKAGE).dfu"
	$(QUIET) $(COPY) $(PACKAGE).bin $@
	$(QUIET) dfu-suffix -v 1209 -p 70b1 -a $@

$(PACKAGE).ihex: $(TARGET)
	$(QUIET) echo "  IHEX     $(PACKAGE).ihex"
	$(QUIET) $(OBJCOPY) -O ihex $(TARGET) $@

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

$(OBJ_DIR)/%.o: %.S
	$(QUIET) echo "  AS       $<	$(notdir $@)"
	$(QUIET) $(CC) -x assembler-with-cpp -c $< $(CFLAGS) -o $@ -MMD

.PHONY: $(CLEAN)

clean:
	$(QUIET) echo "  RM      $(subst /,$(PATH_SEP),$(wildcard $(OBJ_DIR)/*.d))"
	-$(QUIET) $(RM) $(subst /,$(PATH_SEP),$(wildcard $(OBJ_DIR)/*.d))
	$(QUIET) echo "  RM      $(subst /,$(PATH_SEP),$(wildcard $(OBJ_DIR)/*.o)) $(OBJ_DIR)"
	-$(QUIET) $(RM) $(subst /,$(PATH_SEP),$(wildcard $(OBJ_DIR)/*.o)) $(OBJ_DIR)
	$(QUIET) echo "  RM      $(TARGET) $(PACKAGE).bin $(PACKAGE).dfu $(PACKAGE).ihex"
	-$(QUIET) $(RM) $(TARGET) $(PACKAGE).bin $(PACKAGE).dfu $(PACKAGE).ihex

include $(wildcard $(OBJ_DIR)/*.d)
