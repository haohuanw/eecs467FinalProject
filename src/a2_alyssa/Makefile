include ../common.mk

# flags for building the gtk library
CXXFLAGS = $(CXXFLAGS_STD) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_IMAGESOURCE) \
	$(CFLAGS_MATH) \
	$(CFLAGS_COMMON) \
	$(CFLAGS_LCMTYPES) \
	$(CFLAGS_GSL) \
    -O3

CFLAGS = $(CFLAGS_STD) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_IMAGESOURCE) \
	$(CFLAGS_MATH) \
	$(CFLAGS_COMMON) \
	$(CFLAGS_LCMTYPES) \
	$(CFLAGS_GSL) \
    -O3

LDFLAGS = $(LDFLAGS_STD) \
	  $(LDFLAGS_VX_GL) \
	  $(LDFLAGS_VX_GTK) \
	  $(LDFLAGS_IMAGESOURCE) \
	  $(LDFLAGS_MATH) \
	  $(LDFLAGS_COMMON) \
	  $(LDFLAGS_LCMTYPES) \
	  $(LDFLAGS_VX) \
	  $(LDFLAGS_GTK) \
	  $(LDFLAGS_USB) \
	  $(LDFLAGS_PNG) \
	  $(LDFLAGS_DC1394) \
	  $(LDFLAGS_MAPPING) \
      $(LDFLAGS_GSL)

LIBDEPS = $(call libdeps, $(LDFLAGS))
LIB_VXGTK = $(LIB_PATH)/libvxgtk.a
LIB_EECS467 = $(LIB_PATH)/libeecs467.a
LIBEECS467_OBJS = eecs467_util.o

BIN_MAPPING_GUI = $(BIN_PATH)/mapping_gui
BIN_ERROR_GUI = $(BIN_PATH)/error_gui
BIN_EECS467_MAEBOT_MAPPING = $(BIN_PATH)/eecs467_maebot_mapping
BIN_EECS467_MAEBOT_VIDEO_TELOP = $(BIN_PATH)/eecs467_maebot_video_telop
BIN_LCM_SENDER = $(BIN_PATH)/lcm_sender
BIN_LCM_RECEIVER = $(BIN_PATH)/lcm_receiver

ALL = $(LIB_EECS467) $(BIN_MAPPING_GUI) $(BIN_EECS467_APPROX_LASER) \
      $(BIN_EECS467_MAEBOT_MAPPING) $(BIN_EECS467_MAEBOT_VIDEO_TELOP) \
      $(BIN_EECS467_MOVING_LASER) $(BIN_ERROR_GUI)


all: $(ALL)

helpers: $(BIN_LCM_SENDER) $(BIN_LCM_RECEIVER)

eg: $(BIN_ERROR_GUI)

$(LIB_EECS467): $(LIBEECS467_OBJS) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $(LIBEECS467_OBJS)

$(BIN_EECS467_MAEBOT_MAPPING): maebot_mapping.o MovingLaser.o ApproxLaser.o OccupancyGridMapper.o Particles.o Slam.o $(LIB_EECE467) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_MAPPING_GUI): mapping_gui.o $(LIB_EECS467) $(LIBDEPS) $(LIB_VXGTK) $(LIB_VXGL) $(LIB_VX)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS) $(LIB_EECS467)

$(BIN_ERROR_GUI): error_gui.o $(LIB_EECS467) $(LIBDEPS) $(LIB_VXGTK) $(LIB_VXGL) $(LIB_VX)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS) $(LIB_EECS467)

$(BIN_EECS467_PARTICLES): Particles.o $(LIB_EECE467) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_MOVING_LASER): MovingLaser.o $(LIB_EECE467) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_APPROX_LASER): ApproxLaser.o MovingLaser.o $(LIB_EECE467) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_OCCUPANCY_GRID_MAPPER): OccupancyGridMapper.o ApproxLaser.o MovingLaser.o $(LIB_EECE467) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_SLAM): Slam.o Particles.o $(LIB_EECE467) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_MAEBOT_VIDEO_TELOP): maebot_video_teleop.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_LCM_SENDER): lcm_sender.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_LCM_RECEIVER): lcm_receiver.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
