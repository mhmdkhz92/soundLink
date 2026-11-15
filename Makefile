PYTHON ?= python3
SWIG ?= swig
BUILD_DIR := build
SWIG_TOOLS := $(abspath $(BUILD_DIR)/.swig-tools)
SWIG_VERSION := 4.5.1
PYTHON_INCLUDE = $(shell $(PYTHON) -c 'import sysconfig; print(sysconfig.get_config_var("INCLUDEPY") or sysconfig.get_path("include"))')
HEADERS := $(wildcard src/*.h)
HOST_OS := $(shell uname -s)
HOST_ARCH := $(shell uname -m)

ifneq ($(filter arm64 aarch64,$(HOST_ARCH)),)
NATIVE_FLAGS := -mcpu=native
else
NATIVE_FLAGS := -march=native
endif

ifeq ($(HOST_OS),Darwin)
ifeq ($(origin CXX),default)
CXX := xcrun clang++
endif
MODULE_FLAGS := -bundle -undefined dynamic_lookup
AUDIO_LIBS := -framework CoreFoundation -framework CoreAudio -framework AudioToolbox
else ifeq ($(HOST_OS),Linux)
MODULE_FLAGS := -shared -fPIC -pthread
AUDIO_LIBS := -ldl -lm
else
$(error Unsupported build platform: $(HOST_OS))
endif

.PHONY: all
all: $(BUILD_DIR)/_soundlink.so

# One SWIG invocation generates both the C++ wrapper and the Python module.
$(BUILD_DIR)/%_wrap.cxx $(BUILD_DIR)/%.py: swig/%.i Makefile
	mkdir -p "$(BUILD_DIR)"
	@set -e; \
	if command -v "$(SWIG)" >/dev/null 2>&1; then \
		set -- "$(SWIG)"; \
	else \
		if [ ! -f "$(SWIG_TOOLS)/bin/swig" ]; then \
			$(PYTHON) -m pip install --disable-pip-version-check --no-cache-dir --target "$(SWIG_TOOLS)" "swig==$(SWIG_VERSION)"; \
		fi; \
		export PYTHONPATH="$(SWIG_TOOLS)$${PYTHONPATH:+:$$PYTHONPATH}"; \
		set -- $(PYTHON) "$(SWIG_TOOLS)/bin/swig"; \
	fi; \
	"$$@" -c++ -python -outdir "$(BUILD_DIR)" -o "$(BUILD_DIR)/$*_wrap.cxx" "$<"

$(BUILD_DIR)/_soundlink.so: $(BUILD_DIR)/soundlink_wrap.cxx $(BUILD_DIR)/soundlink.py $(HEADERS) Makefile
	$(CXX) -std=c++17 -O2 $(NATIVE_FLAGS) $(MODULE_FLAGS) \
		-I. -I"$(PYTHON_INCLUDE)" "$<" \
		$(AUDIO_LIBS) -o "$@"
