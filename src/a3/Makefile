include ../common.mk

CXXFLAGS = $(CXXFLAGS_STD) \
		   $(CFLAGS_VX_GTK) \
		   $(CFLAGS_IMAGESOURCE) \
		   $(CFLAGS_MATH) \
		   $(CFLAGS_COMMON) \
		   $(CFLAGS_LCMTYPES) \
		   $(CFLAGS_GSL) \
		   -g
LDFLAGS  = $(LDFLAGS_STD) \
		   $(LDFLAGS_VX_GTK) \
		   $(LDFLAGS_IMAGESOURCE) \
		   $(LDFLAGS_MATH) \
		   $(LDFLAGS_COMMON) \
		   $(LDFLAGS_LCMTYPES) \
		   $(LDFLAGS_GSL)

LIB_A3 = $(LIB_PATH)/liba3.a
LIB_A3_OBJ = hsv.o image_processor.o calibration.o vx_state_t.o Navigator.o occupancy_map.o action_model.o particle_data.o maebot_data.o sensor_model.o odometry_matcher.o 

LIB_MAPPING = $(LIB_PATH)/libmapping.a

LIBDEPS  = $(call libdeps, $(LDFLAGS))

BIN_MASK = $(BIN_PATH)/mask
BIN_COLOR_PICKER = $(BIN_PATH)/color_picker
BIN_BLOB_DETECTOR = $(BIN_PATH)/blob_detector
BIN_UI = $(BIN_PATH)/ui
BIN_NAVIGATOR_TEST = $(BIN_PATH)/navigator_test
BIN_BOT_DRIVER = $(BIN_PATH)/bot_driver
BIN_TURN90 = $(BIN_PATH)/turn90
BIN_IMAGE_TO_WORLD_COORD = $(BIN_PATH)/image_to_world_coord
BIN_MANUAL_GEN_TRANS = $(BIN_PATH)/manual_gen_trans
BIN_MAEBOT_SLAM = $(BIN_PATH)/maebot_slam
BIN_WORLD_MANAGER = $(BIN_PATH)/world_manager
BIN_MINI_WORLD_MANAGER = $(BIN_PATH)/mini_world_manager
BIN_MAEBOT_LOCALIZATION = $(BIN_PATH)/maebot_localization

ALL = $(LIB_A3) $(BIN_MASK) $(BIN_COLOR_PICKER) $(BIN_BLOB_DETECTOR) $(BIN_IMAGE_TO_WORLD_COORD) $(BIN_UI) $(BIN_BOT_DRIVER) $(BIN_NAVIGATOR_TEST) $(BIN_MANUAL_GEN_TRANS) $(BIN_WORLD_MANAGER) $(BIN_MINI_WORLD_MANAGER) $(BIN_MAEBOT_LOCALIZATION)

all: $(ALL)

$(LIB_A3): $(LIB_A3_OBJ) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $^

$(BIN_MASK): mask.o $(LIB_A3) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_COLOR_PICKER): color_picker.o $(LIB_A3) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_BLOB_DETECTOR): blob_detector.o $(LIB_A3) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_IMAGE_TO_WORLD_COORD): image_to_world_coord.o $(LIB_A3) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_MANUAL_GEN_TRANS): manual_gen_trans.o $(LIB_A3) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_UI): ui.o $(LIB_A3) $(LIB_MAPPING) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -g -o $@ $^ $(LDFLAGS)

$(BIN_BOT_DRIVER): bot_driver_refactor.o $(LIB_A3) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_TURN90): turn90.o $(LIB_A3) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_NAVIGATOR_TEST): navigatorTest.o $(LIB_A3) $(LIB_MAPPING) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_MAEBOT_SLAM): maebot_slam.o $(LIBDEPS) $(LIB_A3) $(LIB_MAPPING)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_WORLD_MANAGER): world_manager.o $(LIBDEPS) $(LIB_A3) $(LIB_MAPPING)
	@echo "\t$@"
	@$(CXX) -g -o $@ $^ $(LDFLAGS)

$(BIN_MINI_WORLD_MANAGER): mini_world_manager.o $(LIBDEPS) $(LIB_A3) $(LIB_MAPPING)
	@echo "\t$@"
	@$(CXX) -g -o $@ $^ $(LDFLAGS)

$(BIN_MAEBOT_LOCALIZATION): maebot_localization.o $(LIBDEPS) $(LIB_A3) $(LIB_MAPPING)
	@echo "\t$@"
	@$(CXX) -g -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
