# Compiler settings
CC = mips-linux-gnu-gcc

# 1. Hardware-specific flags for Ingenic X2000 (Ender 3 V3 KE)
# These tell the compiler EXACTLY what your CPU is capable of.
HW_FLAGS := -mips32r2 -mhard-float -mfp64 -mmsa -mnan=2008 -mtune=interaptiv

# 2. Optimization flags
# -O3 is more aggressive than -O2.
# -ffast-math and -fno-math-errno help the compiler use the FPU more efficiently.
OPT_FLAGS := -O3 -ffast-math -fno-math-errno -funroll-loops -flto -fwhole-program

# 3. Standard CFLAGS
CFLAGS := -Wall -g -fPIC $(HW_FLAGS) $(OPT_FLAGS) -fno-use-linker-plugin

# 4. Linker flags (Must repeat HW/OPT flags for LTO to work correctly)
LDFLAGS := -shared -fPIC $(HW_FLAGS) $(OPT_FLAGS)

OBJECTS = pyhelper.o serialqueue.o stepcompress.o itersolve.o trapq.o \
     pollreactor.o msgblock.o trdispatch.o \
     kin_cartesian.o kin_corexy.o kin_corexz.o kin_delta.o \
     kin_deltesian.o kin_polar.o kin_rotary_delta.o kin_winch.o \
     kin_extruder.o kin_shaper.o kin_idex.o

DIST_LIB = c_helper.so

all: $(DIST_LIB)

clean:
	@echo "Cleaning up directory."
	rm -f *.a *.o $(DIST_LIB) *~

# Linking the final .so library
$(DIST_LIB): $(OBJECTS)
	$(CC) $(LDFLAGS) -o $@ $(OBJECTS)

# Compiling individual .c files into .o objects
%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<