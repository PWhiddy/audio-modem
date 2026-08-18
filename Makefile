CC ?= cc
CFLAGS ?= -O2 -g
CFLAGS += -std=c11 -Wall -Wextra -Wpedantic -Werror -pthread -MMD -MP -D_POSIX_C_SOURCE=200809L -D_DEFAULT_SOURCE
CPPFLAGS += -Isrc

COMMON_SRC = src/main.c src/modem.c src/packet.c
UNAME_S := $(shell uname -s)

ifeq ($(UNAME_S),Linux)
  PLATFORM_SRC = src/audio_linux.c src/tunnel_linux.c
  LDLIBS += -lm -ldl -pthread
else ifeq ($(UNAME_S),Darwin)
  PLATFORM_SRC = src/audio_macos.c src/tunnel_macos.c
  LDLIBS += -lm -pthread -framework AudioToolbox -framework CoreAudio -framework CoreFoundation
else
  $(error Unsupported operating system: $(UNAME_S))
endif

OBJ = $(COMMON_SRC:.c=.o) $(PLATFORM_SRC:.c=.o)
DEP = $(OBJ:.o=.d)
ALL_PLATFORM_OBJ = src/audio_linux.o src/tunnel_linux.o src/audio_macos.o src/tunnel_macos.o
ALL_DEP = $(COMMON_SRC:.c=.d) $(ALL_PLATFORM_OBJ:.o=.d)

.PHONY: all clean test

all: audio-modem

audio-modem: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(OBJ) $(LDLIBS)

test: audio-modem
	./audio-modem --self-test

clean:
	$(RM) $(COMMON_SRC:.c=.o) $(ALL_PLATFORM_OBJ) $(ALL_DEP) audio-modem

-include $(DEP)
