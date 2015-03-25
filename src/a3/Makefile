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

LIB_A3 = $(LIB_PATH)/liba3.a
LIB_A3_OBJ = hsv.o image_processor.o calibration.o vx_state_t.o

LIB_MAPPING = $(LIB_PATH)/libmapping.a

LIBDEPS  = $(call libdeps, $(LDFLAGS))

BIN_MASK = $(BIN_PATH)/mask
BIN_COLOR_PICKER = $(BIN_PATH)/color_picker
BIN_BLOB_DETECTOR = $(BIN_PATH)/blob_detector
BIN_UI = $(BIN_PATH)/ui

ALL = $(LIB_A3) $(BIN_MASK) $(BIN_COLOR_PICKER) $(BIN_BLOB_DETECTOR) $(BIN_UI)

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

$(BIN_UI): ui.o $(LIB_A3) $(LIB_MAPPING) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
