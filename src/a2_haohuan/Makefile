include ../common.mk

CXXFLAGS = $(CXXFLAGS_STD) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_IMAGESOURCE) \
     $(CFLAGS_MATH) \
     $(CFLAGS_COMMON) \
     $(CFLAGS_LCMTYPES) \
     $(CFLAGS_GSL) \
     -O3 -w
LDFLAGS  = $(LDFLAGS_STD) \
	$(LDFLAGS_VX_GTK) \
	  $(LDFLAGS_IMAGESOURCE) \
	  $(LDFLAGS_MATH) \
	  $(LDFLAGS_COMMON) \
		$(LDFLAGS_LCMTYPES) \
		$(LDFLAGS_GSL)

#LIB_MAEBOT_DATA = $(LIB_PATH)/libmaebotdata.a
#LIB_MAEBOT_DATA_OBJ = maebot_data.o

#LIB_LASER_MATCHER = $(LIB_PATH)/lasermatcher.a
#LIB_LASER_MATCHER_OBJ = laser_matcher.o

#LIB_OCCUPANCY_MAP = $(LIB_PATH)/occupancymap.a
#LIB_OCCUPANCY_MAP_OBJ = occupancy_map.o


#LIB_MAPPING = $(LIB_PATH)/libmapping.a

LIB_A2 = $(LIB_PATH)/liba2.a
LIB_A2_OBJ = hsv.o image_processor.o a2_inv_kin.o arm_ai.o gameboard.o calibration.o board_state.o

LIBDEPS  = $(call libdeps, $(LDFLAGS))

BIN_A2_MASK = $(BIN_PATH)/a2_mask
BIN_A2_COLOR_PICKER = $(BIN_PATH)/a2_color_picker
BIN_A2_BLOB_DETECTOR = $(BIN_PATH)/a2_blob_detector
BIN_A2_AI = $(BIN_PATH)/a2_ai
BIN_A2_INVERSE_KINEMATICS = $(BIN_PATH)/a2_inverse_kinematics
BIN_A2_IMAGE_TO_ARM_COORD = $(BIN_PATH)/a2_image_to_arm_coord
BIN_A2_AI_TEST = $(BIN_PATH)/a2_ai_test
BIN_CLICK_AND_MOVE = $(BIN_PATH)/click_and_move
BIN_A2_DETERMINE_BOARD_STATE = $(BIN_PATH)/a2_determine_board_state
BIN_A2_TTT = $(BIN_PATH)/a2_ttt


ALL = $(LIB_A2) $(BIN_A2_MASK) $(BIN_A2_COLOR_PICKER) $(BIN_A2_BLOB_DETECTOR) \
	$(BIN_A2_INVERSE_KINEMATICS) $(BIN_A2_IMAGE_TO_ARM_COORD) \
	$(BIN_A2_AI_TEST) $(BIN_CLICK_AND_MOVE) $(BIN_A2_DETERMINE_BOARD_STATE) \
	$(BIN_A2_TTT)

all: $(ALL)

$(LIB_A2): $(LIB_A2_OBJ) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $^

$(BIN_A2_MASK): a2_mask.o $(LIB_A2) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_A2_COLOR_PICKER): a2_color_picker.o $(LIB_A2) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_A2_BLOB_DETECTOR): a2_blob_detector.o $(LIB_A2) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_A2_INVERSE_KINEMATICS): a2_inv_kin.o a2_inverse_kinematics.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_A2_IMAGE_TO_ARM_COORD): a2_image_to_arm_coord.o $(LIBDEPS) $(LIB_A2)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_A2_AI): board_state.o a2_intel.o a2_ai.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_A2_AI_TEST): a2_ai_test.o $(LIB_A2) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_CLICK_AND_MOVE): click_and_move.o $(LIB_A2) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_A2_DETERMINE_BOARD_STATE): a2_determine_board_state.o $(LIB_A2) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_A2_TTT): a2_ttt.o $(LIB_A2) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
