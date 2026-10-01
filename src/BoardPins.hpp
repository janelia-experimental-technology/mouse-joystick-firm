// RP2040 GPIO allocation for mouse-joystick-pcb.
//
// Single source of truth for this firmware. Every number below was read back
// from the board's own pad-to-net map -- footprint `U501` in
// `kicad/mouse-joystick-pcb.kicad_pcb` -- at git tag v0.1, the revision that
// was manufactured as rev 0. Cross-checked against
// `mouse-joystick-pcb/docs/interface-map.md`.
//
// Tags v0.1 and v0.2 have byte-identical RP2040 pad-to-net maps. Tag v0.3 and
// current HEAD differ from them on exactly three pins, GP10/GP11/GP13, which
// v0.3 and HEAD route to the angle-output DAC and v0.1/v0.2 leave unrouted. See
// MJ_BOARD_REV below.
//
// A v0.3 BOARD IS MJ_BOARD_REV 1. Build env:mouse_joystick_rev1 for it. Read
// off the v0.3 kicad_pcb, footprint `U501`: pads 13, 14 and 16 carry
// /mcu/DAC_SPI_SCK_, /mcu/DAC_SPI_COPI_ and /mcu/~{DAC_CS_}. Confirmed on board
// E4654C159F171E34, stage 1, 2026-09-09.
//
// Names here are the board's net names, not Pico conventions. Trailing `_N`
// marks an active-low signal, which the schematic spells `~{NAME}`.
//
// ---------------------------------------------------------------------------
// REFERENCE DESIGNATORS IN THESE COMMENTS ARE v0.1 DESIGNATORS.
// ---------------------------------------------------------------------------
// The project uses KiCad automatic annotation, which orders by sheet position,
// so inserting a part rotates every designator after it. Between v0.1 and HEAD
// the 36 V input block moved from the `U201`/`R20x` range to `U1901`/`R190x`,
// and `R1901`/`R1902` came to mean something completely different: 100 ohm
// pyControl series resistors at v0.1, 1 kohm HV_EN/HV_DIAG_EN series resistors
// at HEAD. **A designator alone is not enough to identify a part across
// revisions.** Where it matters, this file gives the value and the net too;
// those are stable and the designator is not.
//
// Do not hand-edit a number here without changing `docs/interface-map.md` in
// the PCB repository in the same breath. The two must agree or bring-up results
// mean nothing.

#pragma once

#include <stdint.h>

// Which manufactured revision this build targets. 0 = tags v0.1 and v0.2 only,
// the boards sent for assembly as rev 0. 1 = tag v0.3 and later, including
// HEAD, with the angle-output DAC fitted. Only the spare-pad set differs; no
// connected pin moves.
//
// The default below is 0, so an env that forgets to define this silently
// targets rev 0 -- which is wrong for every board fabricated since v0.2.
#ifndef MJ_BOARD_REV
#define MJ_BOARD_REV 0
#endif

namespace mouse_joystick {
namespace pins {

// -----------------------------------------------------------------------------
// pyControl behaviour port, RJ45. UART0.
// -----------------------------------------------------------------------------
// The alphabetical order reverses at the sheet boundary: connector DIO-A is
// this board's RX, DIO-B is this board's TX. Both lines carry a 100 ohm series
// resistor (`R1901`/`R1902` at v0.1); PYC_UART_RX also has a 10 kohm pull-up to
// +3.3V_MCU (`R403`), so RX idles high with nothing plugged in.
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
// Four SN74CB3Q3245 packages (`U601`-`U604`), one shared active-low OE, all
// four VCC pins on the always-on +3.3V_MCU rail. `R601` 7.87 kohm pulls this
// high, so the bus is isolated from reset and firmware must lower it
// deliberately. That pull-up is the *only* isolation mechanism -- the packages
// are never unpowered, so Ioff never engages.
//
// "MCU running, switched rails at 0 V" is this board's normal idle state, and
// an MCU pin driving into an unpowered TMC5130A forward-biases its ESD clamps
// and back-powers a dead rail. That is what this switch exists to prevent.
constexpr uint8_t kBusSwEnN = 9; // ~{BUS_SW_EN}, bypasses the switch itself

// -----------------------------------------------------------------------------
// Analog angle output DAC80501. Rev 1 only. SPI1, behind the bus switch.
// -----------------------------------------------------------------------------
// Fitted from v0.3 onward, and reserved at fixed numbers on v0.1/v0.2 so that
// populating them moved nothing. On v0.1 and v0.2 these three pads are routed
// nowhere at all, which is why MJ_BOARD_REV 0 folds them into the spare-pad
// test below; on v0.3 they are a live SPI bus and a live chip select.
//
// The DAC80501 and its OPA197 output stage run from +5V_SW, not +5V_MCU, so
// nothing in this chain can be reached on USB power alone. It needs 36 V.
constexpr uint8_t kDacSpiSck = 10;  // DAC_SPI_SCK_, SPI1 SCK  (rev 1)
constexpr uint8_t kDacSpiCopi = 11; // DAC_SPI_COPI_, SPI1 TX  (rev 1)
constexpr uint8_t kDacCsN = 13;     // ~{DAC_CS_}, DAC ~SYNC   (rev 1)
// SPI1 needs an RX pin assigned even though the DAC has no SDO; GP12 is routed
// nowhere on v0.3 (MotionAndHoming used the same choice).
constexpr uint8_t kSpi1RxUnused = 12; // (rev 1)

// -----------------------------------------------------------------------------
// 36 V input switch, TPS1HTC100-Q1 (`U201` at v0.1). All four bypass the bus
// switch -- the switch itself sits upstream of the switched domain, not on it.
// -----------------------------------------------------------------------------
// HV_EN is the board's reset button: it power-cycles every chip on the switched
// domain while the MCU stays alive on USB or pyControl. `R201` 1 kohm in
// series, with a 10 kohm pull-down at the switch's EN pin, so the rail is off
// at reset.
//
// HV_DIAG_EN must never idle high. Held high, the switch's OFF-state open-load
// detection keeps an internal ~150 kohm pull-up from VS to VOUT enabled,
// trickling ~240 uA into +36V_SW's ~208 uF. The rail creeps past the buck's
// 3.6 V minimum in about three seconds and the switched domain hiccups instead
// of staying off, which silently destroys the power-cycle-as-reset property and
// corrupts the rail sense. Protection is unaffected by DIAG_EN -- only
// reporting is gated -- so raise it *only* while reading kHvSns or kHvFaultN.
// The TPS1HTC100-Q1's LATCH pin is NOT on a GPIO on pcb v0.1, v0.2 or v0.3.
// A 1 kohm 0402 straps it to `+3.3V_MCU` -- `R203` on v0.1/v0.2, and `R1903` on
// v0.3, where the whole block moved into the R19xx range. That strap selects
// latch-off over
// the device's own 2 ms auto-retry -- and auto-retry is the only thing that can
// charge this board's 208 uF of `+36V_SW` bulk, because the switch trips its
// relative thermal shutdown 1-2 ms in, at only ~10-12 V. Measured 2026-09-02 on
// board E4654C159F2C2534; removing `R203` fixes it outright.
//
// So firmware has to do the retrying. See `PowerSequence::staircaseStart()`.
//
// WHEN A FUTURE REVISION ROUTES LATCH TO A GPIO, build with
//   -DMJ_HAS_LATCH_GPIO=1 -DMJ_LATCH_GPIO=<n>
// and `PowerSequence::powerUp()` switches to the direct path: drive LATCH low,
// assert HV_EN, then raise LATCH once the rail is up so real faults latch off.
// `PowerSequence::probeLatchBehaviour()` reports whether a board needs the
// workaround, so a new revision can be checked in one command.
#ifndef MJ_HAS_LATCH_GPIO
#define MJ_HAS_LATCH_GPIO 0
#endif
#if MJ_HAS_LATCH_GPIO
#ifndef MJ_LATCH_GPIO
#error "MJ_HAS_LATCH_GPIO=1 requires MJ_LATCH_GPIO=<pin>. Do not guess it -- \
read it off the revision's netlist."
#endif
constexpr uint8_t kHvLatch = MJ_LATCH_GPIO;
#endif

constexpr uint8_t kHvDiagEn = 14; // HV_DIAG_EN, via `R202` 1k
constexpr uint8_t kHvEn = 19;     // HV_EN,      via `R201` 1k
constexpr uint8_t kHvSns = 28;    // HV_SNS,     ADC2, `R207` 249 ohm to GND
constexpr uint8_t kHvFaultN = 29; // ~{HV_FAULT}, open drain, `R205` 10k pull-up

// -----------------------------------------------------------------------------
// Instrumentation. InstrumentPort-40, `J1501`.
// -----------------------------------------------------------------------------
// The SN74LVC125A buffer packages all run from +3.3V_MCU, so they work with the
// switched domain off. There are three on rev 0 -- `U1601`, `U1701`, `U1801` --
// and four on v0.3, which adds `U2101`. Their shared active-low OE has a 10
// kohm pull-up per package (`R1601`/`R1701`/`R1801`, plus `R2101` on v0.3), so
// the port is silent from reset. Every buffer input is either driven or held by
// a resistor, so enabling the port never floats an LVC input.
//
// FW_TRIG0 and FW_TRIG1 land on InstrumentPort-40 through `U1801`:
//
//   FW_TRIG0 -> J1501 pin 26 -> Strand TRIG0 -> LA5032 channel 16
//   FW_TRIG1 -> J1501 pin 28 -> Strand TRIG1 -> LA5032 channel 17
//
// There are no LEDs and no test points anywhere on this board, and rev 0 has no
// SWD header, so these two pins plus USB CDC are the entire observable surface.
// A logic analyser on TRIG0/TRIG1 is the only proof of life that does not
// depend on USB working.
constexpr uint8_t kInstrBufOeN = 24; // ~{INSTR_BUF_OE}
constexpr uint8_t kFwTrig1 = 21;     // FW_TRIG1 -> pin 28, TRIG1, LA ch 17
constexpr uint8_t kFwTrig0 = 25;     // FW_TRIG0 -> pin 26, TRIG0, LA ch 16

// Other stage 2 taps, for reference when setting up a capture. All buffered,
// all on the driver side of the bus switch.
//
//   TMC_SPI_SCK  -> J1501 pin  5 -> DIG2  -> LA5032 channel  2
//   TMC_SPI_COPI -> J1501 pin  7 -> DIG3  -> LA5032 channel  3
//   TMC_SPI_CIPO -> J1501 pin  8 -> DIG4  -> LA5032 channel  4
//   ~{TMC_CS}    -> J1501 pin 10 -> DIG5  -> LA5032 channel  5
//   PYC_UART_RX  -> J1501 pin 13 -> DIG7  -> LA5032 channel  7
//   PYC_UART_TX  -> J1501 pin 14 -> DIG8  -> LA5032 channel  8
//   ENC_A        -> J1501 pin  2 -> DIG0  -> LA5032 channel  0
//   ENC_B        -> J1501 pin  3 -> DIG1  -> LA5032 channel  1
//   LIMIT_SW     -> J1501 pin 19 -> DIG11 -> LA5032 channel 11
//
// Odd-numbered pins from 1 to 31 that are not listed are GND. Connect at least
// one, and more for a fast or many-channel capture.

// -----------------------------------------------------------------------------
// Switched-rail sense.
// -----------------------------------------------------------------------------
// `R401`/`R402`, 1 kohm/1 kohm inside the mcu sheet, divide raw +3.3V_SW by
// two. The divider is also what holds +3.3V_SW at a true 0 V when the domain is
// off, which is what makes a cross-domain pull-up to that rail a valid LOW.
constexpr uint8_t kSwRailSenseDiv2 = 26; // SW_RAIL_SENSE_DIV_2, ADC0

// -----------------------------------------------------------------------------
// Spare pads. Routed nowhere on this revision.
// -----------------------------------------------------------------------------
// Safe to exercise with internal pulls, which is exactly what the self test
// does with them: a pad that cannot be pulled both ways has a solder defect.
//
// On rev 0 the three DAC pads are unrouted too, so they get tested as spares:
// 12 pads, and stage 1 reports 27 checks. On rev 1 -- v0.3 and later -- they
// carry the DAC bus and are left alone: 9 pads, 24 checks. A v0.3 board that
// reports 27 checks was built with the wrong env.
#if MJ_BOARD_REV == 0
constexpr uint8_t kSpares[] = {10, 11, 12, 13, 15, 16, 17, 18, 20, 22, 23, 27};
#else
constexpr uint8_t kSpares[] = {12, 15, 16, 17, 18, 20, 22, 23, 27};
#endif
constexpr uint8_t kSpareCount = sizeof(kSpares) / sizeof(kSpares[0]);

// -----------------------------------------------------------------------------
// Pins that are NOT on this board.
// -----------------------------------------------------------------------------
// Neither rev 0 nor v0.3 has an SWD header or a RUN connection: on both,
// `U501` pads 24, 25 and 26 -- SWCLK, SWD and RUN -- are no-connect. There is
// no debugger during bring-up, and no reset button, only a power cycle. BOOTSEL
// recovery by button needs USB *and* pyControl unplugged, because the MCU domain
// is ORed from both through `D301` and `D1901`.
//
// The 1200 baud touch and the self test's `B` command are therefore the only
// routes into BOOTSEL that do not involve the button. The touch was verified on
// v0.3 board E4654C159F171E34 on 2026-09-09: a reflash over the running
// application, no `SW501`, no unplugging.
//
// BOOTSEL itself is the standard circuit: `SW501` shorts ~{USB_BOOT} to GND,
// `R505` 1 kohm ties that to QSPI_SS, and `R504` 10 kohm pulls QSPI_SS up to
// +3.3V_MCU. Holding the button at power-on brings up the RPI-RP2 drive.
//
// Three traps come from the PlatformIO `generic` variant's default pin mapping,
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

// The revision name `ID` reports as `hw`, from the same selector that chooses
// the pad map above. The two cannot disagree.
#if MJ_BOARD_REV == 0
constexpr char kHwRevName[] = "rev0";
#elif MJ_BOARD_REV == 1
constexpr char kHwRevName[] = "rev1";
#else
#error "MJ_BOARD_REV must be 0 (pcb v0.1/v0.2) or 1 (pcb v0.3 and later)"
#endif

} // namespace mouse_joystick
