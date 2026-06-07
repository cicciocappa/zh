# Horde fluid POC
CC      ?= cc
CFLAGS  ?= -O2 -Wall -Wextra
LDLIBS  := -lm

# pkg-config name is "sdl3"
SDL_CFLAGS := $(shell pkg-config --cflags sdl3 2>/dev/null)
SDL_LIBS   := $(shell pkg-config --libs   sdl3 2>/dev/null)

.PHONY: all test sandbox clean
all: test sandbox

# headless verification (no SDL needed) — dumps PPM frames into ./frames
test: test_dump.c sim.c sim.h
	$(CC) $(CFLAGS) -o test_dump test_dump.c sim.c $(LDLIBS)

# interactive sandbox (needs SDL3)
sandbox: sandbox_sdl3.c sim.c sim.h
	$(CC) $(CFLAGS) $(SDL_CFLAGS) -o sandbox sandbox_sdl3.c sim.c $(SDL_LIBS) $(LDLIBS)

clean:
	rm -f test_dump sandbox frames/*.ppm
