# Thin wrapper over the Pixi tasks, so a bare shell can drive the build without
# knowing the Pixi task names. Everything real lives in pixi.toml.
#
# Override any of these on the command line:
#   make upload PORT=/dev/ttyACM1
#   make build EXAMPLE=examples/RP2040SelfTest ENV=mouse_joystick_fastflash

PIXI ?= pixi
EXAMPLE ?= examples/RP2040SelfTest
ENV ?= mouse_joystick
PORT ?= /dev/ttyACM0
BAUD ?= 115200

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
	@printf "\\nEXAMPLE=%s ENV=%s PORT=%s\\n" "$(EXAMPLE)" "$(ENV)" "$(PORT)"

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

# The RP2040 in BOOTSEL enumerates as mass storage and needs no rule, but
# picotool talking to running firmware and the CDC port both do.
.PHONY: udev-rules
udev-rules:
	curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
	sudo udevadm control --reload-rules
	sudo udevadm trigger
