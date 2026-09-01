// RP2040 GPIO allocation for mouse-joystick-pcb.
//
// Single source of truth for this firmware. Every number below was read back
// from the board's own pad-to-net map (`kicad/mouse-joystick-pcb.kicad_pcb`,
// footprint `U501`) and cross-checked against
// `mouse-joystick-pcb/docs/interface-map.md`. Rev 0 and rev 1 share this
// allocation by design -- no rev 0 pin moved -- so one table serves both.
//
// Names here are the board's net names, not Pico conventions. Trailing `_N`
// marks an active-low signal, which the schematic spells `~{NAME}`.
//
// Do not hand-edit a number here without changing `docs/interface-map.md` in
// the PCB repository in the same breath. The two must agree or bring-up
// results mean nothing.

#pragma once

#include <stdint.h>

namespace mouse_joystick {
namespace pins {

// -----------------------------------------------------------------------------
// pyControl behaviour port, RJ45. UART0.
// -----------------------------------------------------------------------------
// The alphabetical order reverses at the sheet boundary: connector DIO-A is
// this board's RX, DIO-B is this board's TX. Both lines carry a 100 ohm series
// resistor (R301/R302); PYC_UART_RX also has a 10k pull-up to +3.3V_MCU (R403),
// so RX idles high with nothing plugged in.
constexpr uint8_t kPycUartTx = 0; // PYC_UART_TX,  UART0 TX
constexpr uint8_t kPycUartRx = 1; // PYC_UART_RX,  UART0 RX

// -----------------------------------------------------------------------------
// TMC5130A driver. SPI0, behind the GPIO isolation bus switch.
// -----------------------------------------------------------------------------
// Chip select is plain GPIO, never hardware CSn: the RP2040's PL022 does not
// hold CS low across the TMC5130A's 40-bit datagrams.
//
// All six of these sit on the B (MCU) side of the SN74CB3Q3245 bus switch and
// reach the driver only while kBusSwEnN is low. The MCU-side nets carry a
// trailing underscore in the schematic; the driver-side nets do not.
//
// kTmcDrvEnN reaches the driver's ~{DRV_EN}: low enables the output stage.
// SPI0 assignment is forced by RP2040 muxing -- SCK can only be 2, 6, 18 or 22,
// TX only 3, 7, 19 or 23, and RX only 0, 4, 16 or 20.
constexpr uint8_t kTmcCsN = 2;     // ~{TMC_CS_}
constexpr uint8_t kTmcDrvEnN = 3;  // ~{TMC_DRV_EN_}, low enables
constexpr uint8_t kTmcSpiCipo = 4; // TMC_SPI_CIPO_, SPI0 RX
constexpr uint8_t kTmcDiag0 = 5;   // TMC_DIAG0_
constexpr uint8_t kTmcSpiSck = 6;  // TMC_SPI_SCK_, SPI0 SCK
constexpr uint8_t kTmcSpiCopi = 7; // TMC_SPI_COPI_, SPI0 TX
constexpr uint8_t kTmcDiag1 = 8;   // TMC_DIAG1_

// -----------------------------------------------------------------------------
// GPIO isolation.
// -----------------------------------------------------------------------------
// Four SN74CB3Q3245 packages, one shared active-low OE, all four VCC pins on
// the always-on +3.3V_MCU rail. R601 7.87k pulls this high, so the bus is
// isolated from reset and firmware must lower it deliberately. That pull-up is
// the *only* isolation mechanism -- the packages are never unpowered, so Ioff
// never engages.
//
// "MCU running, switched rails at 0 V" is this board's normal idle state, and
// an MCU pin driving into an unpowered TMC5130A forward-biases its ESD clamps
// and back-powers a dead rail. That is what this switch exists to prevent.
constexpr uint8_t kBusSwEnN = 9; // ~{BUS_SW_EN}, bypasses the switch itself

// -----------------------------------------------------------------------------
// Analog angle output DAC80501. Rev 1. SPI1, behind the bus switch.
// -----------------------------------------------------------------------------
// Reserved at fixed numbers so rev 1 populates them without moving anything.
// Not fitted on rev 0.
constexpr uint8_t kDacSpiSck = 10;  // DAC_SPI_SCK_   SPI1 SCK
constexpr uint8_t kDacSpiCopi = 11; // DAC_SPI_COPI_  SPI1 TX
constexpr uint8_t kDacCsN = 13;     // ~{DAC_CS_}     DAC ~SYNC

// -----------------------------------------------------------------------------
// 36 V input switch, TPS1HTC100-Q1. All four bypass the bus switch.
// -----------------------------------------------------------------------------
// HV_EN is the board's reset button: it power-cycles every chip on the switched
// domain while the MCU stays alive on USB or pyControl. R1901 1k in series,
// with a 10k pull-down at the switch's EN pin, so the rail is off at reset.
//
// HV_DIAG_EN must never idle high. Held high, the switch's OFF-state open-load
// detection keeps an internal ~150k pull-up from VS to VOUT enabled, trickling
// ~240 uA into +36V_SW's ~208 uF. The rail creeps past the buck's 3.6 V minimum
// in about three seconds and the switched domain hiccups instead of staying
// off, which silently destroys the power-cycle-as-reset property and corrupts
// the rail sense. Protection is unaffected by DIAG_EN -- only reporting is
// gated -- so raise it *only* while reading kHvSns or kHvFaultN.
constexpr uint8_t kHvDiagEn = 14; // HV_DIAG_EN, via R1902 1k
constexpr uint8_t kHvEn = 19;     // HV_EN,      via R1901 1k
constexpr uint8_t kHvSns = 28;    // HV_SNS,     ADC2, R1907 249 ohm to GND
constexpr uint8_t kHvFaultN = 29; // ~{HV_FAULT}, open drain, 10k pull-up

// -----------------------------------------------------------------------------
// Instrumentation.
// -----------------------------------------------------------------------------
// The three (rev 0) SN74LVC125A buffer packages all run from +3.3V_MCU, so they
// work with the switched domain off. Their shared active-low OE has a 10k
// pull-up per package (R1601/R1701/R1801, plus R2101 on rev 1), so the port is
// silent from reset.
//
// FW_TRIG0/FW_TRIG1 land on InstrumentPort-40's TRIG0/TRIG1 through U1801.
// There are no LEDs and no test points anywhere on this board, so these two
// pins plus USB CDC are the entire observable surface.
constexpr uint8_t kInstrBufOeN = 24; // ~{INSTR_BUF_OE}
constexpr uint8_t kFwTrig1 = 21;     // FW_TRIG1
constexpr uint8_t kFwTrig0 = 25;     // FW_TRIG0

// -----------------------------------------------------------------------------
// Switched-rail sense.
// -----------------------------------------------------------------------------
// R401/R402, 1k/1k inside the mcu sheet, divide raw +3.3V_SW by two. The
// divider is also what holds +3.3V_SW at a true 0 V when the domain is off,
// which is what makes a cross-domain pull-up to that rail a valid LOW.
constexpr uint8_t kSwRailSenseDiv2 = 26; // SW_RAIL_SENSE_DIV_2, ADC0

// -----------------------------------------------------------------------------
// Spare pads. Routed nowhere on any revision so far.
// -----------------------------------------------------------------------------
// Safe to exercise with internal pulls, which is exactly what the self test
// does with them: a pad that cannot be pulled both ways has a solder defect.
constexpr uint8_t kSpares[] = {12, 15, 16, 17, 18, 20, 22, 23, 27};
constexpr uint8_t kSpareCount = sizeof(kSpares) / sizeof(kSpares[0]);

// -----------------------------------------------------------------------------
// Pins that are NOT on this board.
// -----------------------------------------------------------------------------
// Rev 0 has no SWD header and no RUN connection: SWCLK, SWD and RUN are all
// no-connect. There is no debugger during bring-up, and BOOTSEL recovery needs
// USB *and* pyControl unplugged, because the MCU domain is ORed from both.
//
// Two traps come from the PlatformIO `generic` variant's default pin mapping,
// which was written for a Pico and does not describe this board:
//
//   * LED_BUILTIN / PIN_LED resolves to GPIO 25, which here is FW_TRIG0. There
//     is no LED on this board. Toggling it is harmless -- it drives an
//     InstrumentPort trigger -- but it is not an LED.
//   * Serial2 defaults to GPIO 8/9, which here are TMC_DIAG1_ and
//     ~{BUS_SW_EN}. Never call Serial2 on this board.
//   * Wire1 defaults to GPIO 26/27, which here are the rail sense ADC and a
//     spare pad. Never call Wire1 on this board.

} // namespace pins
} // namespace mouse_joystick
