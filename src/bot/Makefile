include ../common.mk

# flags for building the gtk library
CFLAGS = $(CFLAGS_STD) \
	 $(CFLAGS_VX_GTK) \
         $(CFLAGS_IMAGESOURCE) \
	 $(CFLAGS_MATH) \
	 $(CFLAGS_COMMON) \
	 $(CFLAGS_LCMTYPES) \
	 -O0

LDFLAGS = $(LDFLAGS_STD) \
	  $(LDFLAGS_VX_GTK) \
	  $(LDFLAGS_IMAGESOURCE) \
	  $(LDFLAGS_MATH) \
	  $(LDFLAGS_COMMON) \
	  $(LDFLAGS_LCMTYPES)

LIBDEPS = $(call libdeps, $(LDFLAGS))

BIN_BOTLAB_ODOMETRY = $(BIN_PATH)/botlab_odometry
BIN_BOTLAB_APP   = $(BIN_PATH)/botlab_app
BIN_SCANMATCHER   = $(BIN_PATH)/botlab_scanmatcher
BIN_BOTLAB_CAMERA_LIDAR   = $(BIN_PATH)/botlab_camera_lidar
BIN_BOTLAB_XYT_TEST = $(BIN_PATH)/botlab_xyt_test 

ALL = $(BIN_BOTLAB_ODOMETRY) $(BIN_BOTLAB_APP) $(BIN_BOTLAB_CAMERA_LIDAR) $(BIN_BOTLAB_XYT_TEST) $(BIN_SCANMATCHER)

all: $(ALL)

$(BIN_BOTLAB_ODOMETRY): odometry.o xyt.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_BOTLAB_APP): botlab.o xyt.o slam.o pidcontrol.o dStarLite.o smooth.o priorityQueue.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -g -o $@ $^ $(LDFLAGS)

$(BIN_SCANMATCHER): scan_matching.o xyt.o filter.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -g -o $@ $^ $(LDFLAGS)

$(BIN_BOTLAB_CAMERA_LIDAR): camera_lidar.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_BOTLAB_XYT_TEST): xyt_test.o xyt.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
