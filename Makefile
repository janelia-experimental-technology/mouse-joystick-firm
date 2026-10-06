# Thin wrapper over the Pixi tasks, so a bare shell can drive the build without
# knowing the Pixi task names. Everything real lives in pixi.toml.
#
# Override any of these on the command line:
#   make upload PORT=/dev/ttyACM1
#   make build EXAMPLE=examples/RP2040SelfTest ENV=mouse_joystick_safeflash

PIXI ?= pixi
EXAMPLE ?= examples/RP2040SelfTest
ENV ?= mouse_joystick
PORT ?= /dev/ttyACM0
BAUD ?= 115200

# Path to a pyControl checkout, for the pycontrol-* targets. Exported explicitly
# rather than relying on make's command-line export rules, because the tool reads
# it from the environment and an empty value silently falls back to autodetection.
PYCONTROL_DIR ?=
export PYCONTROL_DIR

.PHONY: help
help:
	@printf "targets:\\n"
	@printf "  install      resolve the pixi environment\\n"
	@printf "  build        compile EXAMPLE for ENV\\n"
	@printf "  rebuild      clean, then compile\\n"
	@printf "  upload       compile and flash to PORT\\n"
	@printf "  flash        clean, compile and flash to PORT\\n"
	@printf "  monitor      open the USB CDC serial monitor on PORT\\n"
	@printf "  ports        list connected serial ports\\n"
	@printf "  test         run host-native tests\\n"
	@printf "  format       clang-format staged changes\\n"
	@printf "  clean        clean build artifacts for EXAMPLE/ENV\\n"
	@printf "  deepclean    delete the build directory for EXAMPLE\\n"
	@printf "  udev-rules   install the PlatformIO udev rules (needs sudo)\\n"
	@printf "\\npyControl host, a separate environment with no PlatformIO in it:\\n"
	@printf "  pycontrol-install  resolve the pycontrol environment\\n"
	@printf "  pycontrol-check    verify pyControl's dependency gate\\n"
	@printf "  pycontrol-test     host-side pytest (Gate C)\\n"
	@printf "  pycontrol-link     point a pyControl checkout at this repo\\n"
	@printf "  pycontrol-run      run a task on the pyboard headlessly (Gate E); PYB_PORT, HWD, TASK, SECONDS\\n"
	@printf "  gui                launch the pyControl GUI\\n"
	@printf "\\nEXAMPLE=%s ENV=%s PORT=%s\\n" "$(EXAMPLE)" "$(ENV)" "$(PORT)"
	@printf "PYCONTROL_DIR=%s\\n" "$(PYCONTROL_DIR)"

.PHONY: install
install:
	$(PIXI) install

.PHONY: build
build:
	$(PIXI) run build $(EXAMPLE) $(ENV)

.PHONY: rebuild
rebuild:
	$(PIXI) run rebuild $(EXAMPLE) $(ENV)

.PHONY: upload
upload:
	$(PIXI) run upload $(EXAMPLE) $(ENV) $(PORT)

.PHONY: flash
flash:
	$(PIXI) run flash $(EXAMPLE) $(ENV) $(PORT)

.PHONY: monitor
monitor:
	$(PIXI) run monitor $(PORT) $(BAUD)

.PHONY: ports
ports:
	$(PIXI) run ports

.PHONY: test
test:
	$(PIXI) run test

.PHONY: format
format:
	$(PIXI) run format

.PHONY: clean
clean:
	$(PIXI) run clean $(EXAMPLE) $(ENV)

.PHONY: deepclean
deepclean:
	$(PIXI) run deepclean $(EXAMPLE)

# pyControl host targets. These use the `pycontrol` pixi environment, which shares
# nothing with the firmware environment - no PlatformIO, no toolchain, no Qt on
# either side of the fence. Set PYCONTROL_DIR to point at a pyControl checkout,
# or let the tool autodetect one.
#
#   make gui PYCONTROL_DIR=~/Repositories/github/pycontrol-code

PYC_RUN = $(PIXI) run -e pycontrol

.PHONY: pycontrol-install
pycontrol-install:
	$(PIXI) install -e pycontrol

.PHONY: pycontrol-check
pycontrol-check:
	$(PYC_RUN) pycontrol-check

.PHONY: pycontrol-test
pycontrol-test:
	$(PYC_RUN) pycontrol-test

.PHONY: pycontrol-link
pycontrol-link:
	$(PYC_RUN) pycontrol-link

# Gate E: the pyControl framework on the pyboard loads this repo's device class,
# hardware definition and task through its normal path and runs the task; the
# events stream back to the terminal.
PYB_PORT ?= /dev/ttyACM1
HWD ?= mouse_joystick_bringup
TASK ?= mouse_joystick_check
SECONDS ?= 60
.PHONY: pycontrol-run
pycontrol-run:
	$(PYC_RUN) pycontrol-run $(PYB_PORT) $(HWD) $(TASK) $(SECONDS)

.PHONY: pycontrol-unlink
pycontrol-unlink:
	$(PYC_RUN) pycontrol-link --unlink

.PHONY: gui
gui:
	$(PYC_RUN) gui

# The RP2040 in BOOTSEL enumerates as mass storage and needs no rule, but
# picotool talking to running firmware and the CDC port both do.
.PHONY: udev-rules
udev-rules:
	curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
	sudo udevadm control --reload-rules
	sudo udevadm trigger
