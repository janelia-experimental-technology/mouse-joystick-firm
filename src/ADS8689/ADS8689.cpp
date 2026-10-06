#include <SPI.h>
#include <hardware/gpio.h>
#include <pico/time.h> // busy_wait_us_32

#include "ADS8689.hpp"

namespace ads8689 {

SpiTransport::SpiTransport(SPIClass &spi, uint8_t convst_cs_pin,
                           uint32_t clock_hz, uint32_t conversion_wait_us)
    : spi_(spi), convst_cs_pin_(convst_cs_pin), clock_hz_(clock_hz),
      conversion_wait_us_(conversion_wait_us) {}

void SpiTransport::begin() {
  // Level first, then direction. Arduino's pinMode(OUTPUT) runs gpio_init(),
  // which clears the output register before the pad becomes an output, so a
  // pin meant to idle high drives low until the digitalWrite() that follows
  // -- 34 us measured on this board family (pcb v1.0, 2026-10-05, on the bus
  // switch enable). On CONVST/CS that low would open a frame and its return
  // high would start a conversion. The SDK calls avoid the clear.
  gpio_set_function(convst_cs_pin_, GPIO_FUNC_SIO);
  gpio_put(convst_cs_pin_, 1);
  gpio_set_dir(convst_cs_pin_, GPIO_OUT);
}

uint32_t SpiTransport::transferFrame(uint32_t out) {
  // SPI-00-S, the part's power-up protocol: CPOL 0, CPHA 0. SDI is captured on
  // the rising SCLK edge; SDO's MSB is already valid from the CS falling edge
  // and each next bit is launched on the falling SCLK edge. Mode 0.
  spi_.beginTransaction(SPISettings(clock_hz_, MSBFIRST, SPI_MODE0));
  gpio_put(convst_cs_pin_, 0);
  uint32_t in = 0;
  in = (in << 8) | spi_.transfer(static_cast<uint8_t>(out >> 24));
  in = (in << 8) | spi_.transfer(static_cast<uint8_t>(out >> 16));
  in = (in << 8) | spi_.transfer(static_cast<uint8_t>(out >> 8));
  in = (in << 8) | spi_.transfer(static_cast<uint8_t>(out));
  // This edge executes the command AND starts a conversion.
  gpio_put(convst_cs_pin_, 1);
  spi_.endTransaction();
  // t_conv_max before anyone touches the pin again. A busy wait, not
  // delayMicroseconds(): on this core that goes through the SDK's alarm-pool
  // sleep, which under a 1 kHz tick interrupt stretched a 6 us wait to about
  // a millisecond and pushed the QP sampler into decimation (v1.0 unit A,
  // 2026-10-06, SAMPLE_STAT last 1.2-1.6 ms with the rail up).
  busy_wait_us_32(conversion_wait_us_);
  return in;
}

void SpiTransport::pulseConvst() {
  gpio_put(convst_cs_pin_, 0);
  // No minimum low time is specified beyond the setup to a first clock that
  // never comes; a microsecond is generous and visible on an analyser.
  busy_wait_us_32(1);
  gpio_put(convst_cs_pin_, 1);
  busy_wait_us_32(conversion_wait_us_);
}

} // namespace ads8689
