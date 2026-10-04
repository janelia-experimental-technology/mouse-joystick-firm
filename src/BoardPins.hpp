// RP2040 GPIO allocation for mouse-joystick-pcb.
//
// Single source of truth for this firmware. Every number below was read back
// from the board's own pad-to-net map in `kicad/mouse-joystick-pcb.kicad_pcb`
// at the revision that was manufactured -- footprint `U501` through tag v0.3,
// footprint `U601` at v1.0, where the RP2040 moved into the combined MCU block
// and `U501` became the 36 V switch. Cross-checked against
// `mouse-joystick-pcb/docs/interface-map.md`.
//
// Three pad maps exist. See MJ_BOARD_REV below.
//
//   rev 0  tags v0.1 and v0.2: byte-identical RP2040 pad-to-net maps.
//   rev 1  tag v0.3: differs from rev 0 on exactly three pins, GP10/GP11/GP13,
//          which carry the angle-output DAC's SPI bus. Confirmed on board
//          E4654C159F171E34, stage 1, 2026-09-09.
//   rev 2  pcb v1.0: read off `U601` in the board file, which is unchanged
//          from commit d5ae4b0 (2026-09-21) through the fabrication package in
//          646b8f4 and HEAD 6d5ffd3 (2026-10-02). No v1.0 tag existed when this
//          map was written (2026-10-04); the three commits carry one pad map.
//          Eight pins change role and one leaves the sheet; everything the
//          pyControl link, the TMC bus and the power-monitoring group use is
//          unchanged. Not yet confirmed on hardware.
//
// Names here are the board's net names, not Pico conventions. Trailing `_N`
// marks an active-low signal, which the schematic spells `~{NAME}`.
//
// ---------------------------------------------------------------------------
// REFERENCE DESIGNATORS IN THESE COMMENTS ARE v0.1 DESIGNATORS UNLESS A
// REVISION IS NAMED.
// ---------------------------------------------------------------------------
// The project uses KiCad automatic annotation, which orders by sheet position,
// so inserting a part rotates every designator after it. Between v0.1 and v0.3
// the 36 V input block moved from the `U201`/`R20x` range to `U1901`/`R190x`;
// at v1.0 it is `U501`/`R50x`, and `U501` had been the RP2040. `R1901`/`R1902`
// mean 100 ohm pyControl series resistors at v0.1 and 1 kohm HV_EN/HV_DIAG_EN
// series resistors at v0.3. **A designator alone is not enough to identify a
// part across revisions.** Where it matters, this file gives the value and the
// net too; those are stable and the designator is not.
//
// Do not hand-edit a number here without changing `docs/interface-map.md` in
// the PCB repository in the same breath. The two must agree or bring-up results
// mean nothing.

#pragma once

#include <stdint.h>

// Which manufactured revision this build targets.
//
//   0  tags v0.1 and v0.2 only, the boards sent for assembly as rev 0.
//   1  tag v0.3, with the angle-output DAC on SPI1. Only the spare-pad set
//      differs from rev 0; no connected pin moves.
//   2  pcb v1.0: I2C DACs (angle 0x48, brake 0x49), the torque ADC on SPI1,
//      the brake fault input, LATCH on a GPIO, ~{HV_FAULT} moved, the
//      instrument buffers' enable gone (InstrumentPort-Digital-50 is always
//      on), two more ADC channels in use. Pins move; read the v1.0 section.
//
// The default below is 0, so an env that forgets to define this silently
// targets rev 0 -- which is wrong for every board fabricated since v0.2.
#ifndef MJ_BOARD_REV
#define MJ_BOARD_REV 0
#endif

#if (MJ_BOARD_REV < 0) || (MJ_BOARD_REV > 2)
#error "MJ_BOARD_REV must be 0 (pcb v0.1/v0.2), 1 (pcb v0.3) or 2 (pcb v1.0)"
#endif

namespace mouse_joystick {
namespace pins {

// -----------------------------------------------------------------------------
// pyControl behaviour port, RJ45. UART0. Unchanged on every revision.
// -----------------------------------------------------------------------------
// The alphabetical order reverses at the sheet boundary: connector DIO-A is
// this board's RX, DIO-B is this board's TX. Both lines carry a 100 ohm series
// resistor (`R1901`/`R1902` at v0.1); PYC_UART_RX also has a 10 kohm pull-up to
// +3.3V_MCU (`R403` at v0.1, `R401` at v1.0), so RX idles high with nothing
// plugged in.
constexpr uint8_t kPycUartTx = 0; // PYC_UART_TX,  UART0 TX
constexpr uint8_t kPycUartRx = 1; // PYC_UART_RX,  UART0 RX

// -----------------------------------------------------------------------------
// TMC5130A driver. SPI0, behind the GPIO isolation bus switch. Unchanged on
// every revision.
// -----------------------------------------------------------------------------
// Chip select is plain GPIO, never hardware CSn: the RP2040's PL022 does not
// hold CS low across the TMC5130A's 40-bit datagrams.
//
// All six of these sit on the B (MCU) side of the SN74CB3Q3245 bus switch and
// reach the driver only while kBusSwEnN is low. The MCU-side nets carry a
// trailing underscore in the schematic through v0.3; at v1.0 the combined MCU
// block names them `GPIOn` on the MCU side and `ISOn` on the peripheral side.
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
// GPIO isolation. Unchanged on every revision.
// -----------------------------------------------------------------------------
// Four SN74CB3Q3245 packages (`U601`-`U604` through v0.3, `U603`-`U606` at
// v1.0), one shared active-low OE, all four VCC pins on the always-on
// +3.3V_MCU rail. A 7.87 kohm pull-up (`R601` through v0.3, `R607` at v1.0)
// holds this high, so the bus is isolated from reset and firmware must lower
// it deliberately. That pull-up is the *only* isolation mechanism -- the
// packages are never unpowered, so Ioff never engages.
//
// "MCU running, switched rails at 0 V" is this board's normal idle state, and
// an MCU pin driving into an unpowered TMC5130A forward-biases its ESD clamps
// and back-powers a dead rail. That is what this switch exists to prevent.
//
// At v1.0 the brake and torque blocks and both DACs are behind the same switch
// (see below), so the same rule covers them: nothing on the switched domain is
// reachable, or may be driven, until the rail is up and this pin is low.
constexpr uint8_t kBusSwEnN = 9; // ~{BUS_SW_EN}, bypasses the switch itself

#if MJ_BOARD_REV < 2
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
//
// AT v1.0 THESE THREE PADS CARRY SOMETHING ELSE: GP10/GP11 are the torque
// ADC's SPI1 clock and data in, and GP13 is ~{BRAKE_FAULT}, an INPUT driven by
// the brake block. Driving GP13 as a chip select on a v1.0 board with the bus
// closed shorts an output against an output. That is why these constants do
// not exist for MJ_BOARD_REV 2 and the SPI DAC code paths do not compile for
// it.
constexpr uint8_t kDacSpiSck = 10;  // DAC_SPI_SCK_, SPI1 SCK  (rev 1)
constexpr uint8_t kDacSpiCopi = 11; // DAC_SPI_COPI_, SPI1 TX  (rev 1)
constexpr uint8_t kDacCsN = 13;     // ~{DAC_CS_}, DAC ~SYNC   (rev 1)
// SPI1 needs an RX pin assigned even though the DAC has no SDO; GP12 is routed
// nowhere on v0.3 (MotionAndHoming used the same choice).
constexpr uint8_t kSpi1RxUnused = 12; // (rev 1)
#else
// -----------------------------------------------------------------------------
// v1.0: the two DACs on I2C0, the torque ADC on SPI1, the brake fault input,
// and the torque calibration switch. All behind the bus switch.
// -----------------------------------------------------------------------------
// Both DAC80501Z parts strap SPI2C high (`R808`, `R1809`, 1 kohm to
// +3.3V_FILT) and share one I2C bus. A0 selects the address: the angle DAC's
// A0 goes to GND through `R806` (0x48), the brake DAC's to +3.3V_FILT through
// `R1807` (0x49). The addresses are in namespace mouse_joystick below.
//
// Pull-ups exist on BOTH sides of the bus switch, which is deliberate: 4.7 kohm
// to +3.3V_SW on the peripheral side (`R406`/`R407`) and 4.7 kohm to +3.3V_MCU
// on the MCU side (`R402`/`R403`, on the `DAC_*_BYP` taps). So with the bus
// isolated the MCU pads read high, which stage 1 checks, and with it closed
// and the rail up the lines see about 2.35 kohm.
//
// Both DACs run from +3.3V_FILT (filtered +3.3V_SW), the ADS8689's DVDD from
// +3.3V_SW and its AVDD from +5V_FILT, the brake and torque amplifiers from
// +5V_FILT and +12V_FILT. Nothing in these chains can be reached on USB power
// alone; it needs 36 V, the rail up, and the bus closed. The DAC80501Z resets
// its data register to zero code, so the brake sink commands 0 mA from the
// instant the switched domain comes up until firmware says otherwise, and a
// wrong GAIN write drives the output to 0 V rather than full scale.
constexpr uint8_t kDacSda = 16; // DAC_SDA, I2C0 SDA (v1.0)
constexpr uint8_t kDacScl = 17; // DAC_SCL, I2C0 SCL (v1.0)

// ADS8689 torque ADC (`U2002` at v1.0). SPI1 assignment is forced by RP2040
// muxing -- SPI1 SCK exists only on GP10, GP14 and GP26, and 14 and 26 are
// taken. The board's net names are COPI/CIPO and they land on the ADC's SDI
// and SDO-0 pins respectively; CONVST/~{CS} is one pin on this part, plain
// GPIO because the conversion starts on its rising edge and timing matters.
// `R2012` 10 kohm holds CIPO low; the ADC's ~{RST} has `R2011` 10 kohm to
// +3.3V_SW; NO pull exists on TQ_CONVST_CS on this board (the design note
// claims one; the netlist does not have it), so firmware must drive it.
constexpr uint8_t kTqSpiSck = 10;    // TQ_SPI_SCK,  SPI1 SCK      (v1.0)
constexpr uint8_t kTqSpiCopi = 11;   // TQ_SPI_COPI, SPI1 TX -> SDI (v1.0)
constexpr uint8_t kTqSpiCipo = 12;   // TQ_SPI_CIPO, SPI1 RX <- SDO (v1.0)
constexpr uint8_t kTqConvstCsN = 15; // TQ_CONVST_CS, plain GPIO    (v1.0)

// Torque bridge shunt-calibration switch: high puts `R2003` 75 kohm across one
// bridge arm. Must idle low; SafeState drives it low behind the open switch.
constexpr uint8_t kTqCalEn = 18; // TQ_CAL_EN (v1.0)

// Brake current-limiter indication, active low, from `U1803` (SN74LVC2G17 on
// +3.3V_FILT). An INPUT. Not a latch: it is low only while the ceiling
// amplifier is limiting. High-Z whenever the switched domain is down.
constexpr uint8_t kBrakeFaultN = 13; // ~{BRAKE_FAULT} (v1.0)
#endif

// -----------------------------------------------------------------------------
// 36 V input switch, TPS1HTC100-Q1 (`U201` at v0.1, `U1901` at v0.3, `U501` at
// v1.0). All of these bypass the bus switch -- the switch itself sits upstream
// of the switched domain, not on it.
// -----------------------------------------------------------------------------
// HV_EN is the board's reset button: it power-cycles every chip on the switched
// domain while the MCU stays alive on USB or pyControl. 1 kohm in series
// (`R201` v0.1, `R501` v1.0), with a 10 kohm pull-down at the switch's EN pin
// (`R506` at v1.0), so the rail is off at reset.
//
// HV_DIAG_EN must never idle high. Held high, the switch's OFF-state open-load
// detection keeps an internal ~150 kohm pull-up from VS to VOUT enabled,
// trickling ~240 uA into +36V_SW's ~208 uF. The rail creeps past the buck's
// 3.6 V minimum in about three seconds and the switched domain hiccups instead
// of staying off, which silently destroys the power-cycle-as-reset property and
// corrupts the rail sense. Protection is unaffected by DIAG_EN -- only
// reporting is gated -- so raise it *only* while reading kHvSns or kHvFaultN.
//
// LATCH. On pcb v0.1, v0.2 and v0.3 the TPS1HTC100-Q1's LATCH pin is NOT on a
// GPIO: a 1 kohm 0402 straps it to +3.3V_MCU -- `R203` on v0.1/v0.2, `R1903` on
// v0.3. That strap selects latch-off over the device's own 2 ms auto-retry, and
// auto-retry is the only thing that can charge this board's 208 uF of +36V_SW
// bulk, because the switch trips its relative thermal shutdown 1-2 ms in, at
// only ~10-12 V. Measured 2026-09-02 on board E4654C159F2C2534; removing `R203`
// fixes it outright. So on those revisions firmware has to do the retrying:
// see `PowerSequence::staircaseStart()`.
//
// v1.0 ROUTES LATCH TO GP24 through `R503` 10 kohm (the pin ~{INSTR_BUF_OE}
// freed), which is the real fix: `PowerSequence::powerUp()` holds it low for
// the power-up ramp so the switch retries its own way through the capacitive
// charge, then raises it once the rail is up so that a genuine fault latches
// off. Low is also what the pad's reset pull-down gives and what SafeState
// drives, so a board with no firmware, or in its safe state, is in the
// auto-retry configuration -- the one that can bring the rail up.
// `PowerSequence::probeLatchBehaviour()` reports whether a board needs the
// workaround, so a new revision can be checked in one command.
//
// Another revision that routes LATCH to a GPIO builds with
//   -DMJ_HAS_LATCH_GPIO=1 -DMJ_LATCH_GPIO=<n>
// and gets the same direct path. v1.0 sets both itself and refuses overrides.
#if MJ_BOARD_REV >= 2
#if defined(MJ_HAS_LATCH_GPIO) && !MJ_HAS_LATCH_GPIO
#error "pcb v1.0 routes the 36 V switch's LATCH to GP24. MJ_HAS_LATCH_GPIO=0 would run the staircase workaround against a board that does not need it."
#endif
#if defined(MJ_LATCH_GPIO) && (MJ_LATCH_GPIO != 24)
#error "pcb v1.0 LATCH is GP24 (U601 pad 36). Do not override MJ_LATCH_GPIO."
#endif
#undef MJ_HAS_LATCH_GPIO
#define MJ_HAS_LATCH_GPIO 1
#undef MJ_LATCH_GPIO
#define MJ_LATCH_GPIO 24
#endif
#ifndef MJ_HAS_LATCH_GPIO
#define MJ_HAS_LATCH_GPIO 0
#endif
#if MJ_HAS_LATCH_GPIO
#ifndef MJ_LATCH_GPIO
#error "MJ_HAS_LATCH_GPIO=1 requires MJ_LATCH_GPIO=<pin>. Do not guess it -- \
read it off the revision's netlist."
#endif
constexpr uint8_t kHvLatch = MJ_LATCH_GPIO; // HV_LATCH, via `R503` 10k (v1.0)
#endif

constexpr uint8_t kHvDiagEn = 14; // HV_DIAG_EN, via `R202` 1k (v0.1), `R502` 10k (v1.0)
constexpr uint8_t kHvEn = 19;     // HV_EN,      via `R201` 1k (v0.1), `R501` 1k (v1.0)
constexpr uint8_t kHvSns = 28;    // HV_SNS,     ADC2, 249 ohm to GND (`R207` v0.1,
                                  //             `R509` v1.0, behind `R505` 1k at v1.0)
// ~{HV_FAULT} is open drain with a 10 kohm pull-up to +3.3V_MCU (`R205` v0.1,
// `R1905` v0.3, `R507` v1.0, where it sits on the far side of `R504` 10 kohm).
// It moved at v1.0 because it was spending the last ADC-capable pin: GP29 is
// now ANGLE_OUT_MON. Reading GP29 as the fault line on a v1.0 board reads an
// analog node as a digital level and finds no fault, ever.
#if MJ_BOARD_REV >= 2
constexpr uint8_t kHvFaultN = 23; // ~{HV_FAULT}, U601 pad 35 (v1.0)
#else
constexpr uint8_t kHvFaultN = 29; // ~{HV_FAULT}, U501 pad 41 (v0.1-v0.3)
#endif

// -----------------------------------------------------------------------------
// Instrumentation.
// -----------------------------------------------------------------------------
#if MJ_BOARD_REV < 2
// InstrumentPort-40, `J1501`. The SN74LVC125A buffer packages all run from
// +3.3V_MCU, so they work with the switched domain off. There are three on
// rev 0 -- `U1601`, `U1701`, `U1801` -- and four on v0.3, which adds `U2101`.
// Their shared active-low OE has a 10 kohm pull-up per package
// (`R1601`/`R1701`/`R1801`, plus `R2101` on v0.3), so the port is silent from
// reset. Every buffer input is either driven or held by a resistor, so
// enabling the port never floats an LVC input.
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
#else
// InstrumentPort-Digital-50, `J1301` (Hirose FH12-50S-0.5SVA), four SN74LV541A
// buffers `U1301`-`U1304` on +3.3V_MCU with their ~OE pins tied low inside the
// block. The port is read-only and ALWAYS ENABLED: there is no enable pin, so
// `SafeState::setInstrumentationEnabled()` has nothing to drive on this
// revision and the protocol's `INSTR` verb always reports on. GP24 went to
// HV_LATCH instead. FW_TRIG0/FW_TRIG1 carry 10 kohm pull-downs (`R404`/`R405`)
// because every buffer input must be tied.
//
// The 32 strands map 1:1 onto `la5032-instrument-port-adapter` v2.0 channels
// (CHn observes DIGn) -- a mapping that is designed, not yet bench-verified:
//
//   DIG0  FW_TRIG0        DIG8  TMC_SPI_SCK    DIG16 TQ_SPI_SCK      DIG24 HV_EN
//   DIG1  FW_TRIG1        DIG9  ~{TMC_CS}      DIG17 TQ_CONVST_CS    DIG25 HV_DIAG_EN
//   DIG2  PYC_UART_TX     DIG10 TMC_SPI_COPI   DIG18 TQ_SPI_COPI     DIG26 ~{HV_FAULT}
//   DIG3  PYC_UART_RX     DIG11 TMC_SPI_CIPO   DIG19 TQ_SPI_CIPO     DIG27 ~{BUS_SW_EN}
//   DIG4  (GND)           DIG12 TMC_DIAG0      DIG20 TQ_CAL_EN       DIG28 ENCA
//   DIG5  (GND)           DIG13 TMC_DIAG1      DIG21 ~{BRAKE_FAULT}  DIG29 ENCB
//   DIG6  (GND)           DIG14 ~{TMC_DRV_EN}  DIG22 DAC_SDA         DIG30 PG_5V
//   DIG7  (GND)           DIG15 LIMIT_SW       DIG23 DAC_SCL         DIG31 PG_12V
//
// The TMC, torque, brake and DAC taps are on the peripheral (ISO) side of the
// bus switch, so they show what the chips receive and are live only with the
// rail up. PG_5V and PG_12V are the only view firmware never gets of the two
// bucks. Still no LEDs and no test points, and SWD is still not fitted; the
// v1.0 addition is a RUN (reset) switch.
#endif
constexpr uint8_t kFwTrig1 = 21; // FW_TRIG1 -> DIG1 / TRIG1
constexpr uint8_t kFwTrig0 = 25; // FW_TRIG0 -> DIG0 / TRIG0

#if MJ_BOARD_REV < 2
// Other stage 2 taps on InstrumentPort-40, for reference when setting up a
// capture. All buffered, all on the driver side of the bus switch.
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
#endif

// -----------------------------------------------------------------------------
// ADC. Only GP26-GP29 can reach it, so there are exactly four channels.
// -----------------------------------------------------------------------------
// `R401`/`R402` (v0.1; `R408`/`R409` at v1.0), 1 kohm/1 kohm inside the mcu
// sheet, divide raw +3.3V_SW by two. The divider is also what holds +3.3V_SW
// at a true 0 V when the domain is off, which is what makes a cross-domain
// pull-up to that rail a valid LOW.
constexpr uint8_t kSwRailSenseDiv2 = 26; // SW_RAIL_SENSE_DIV_2, ADC0

#if MJ_BOARD_REV >= 2
// v1.0 spends the other two channels on analog readback of the two output
// chains. Both bypass the bus switch, like HV_SNS, and both sit at 0 V with
// the switched domain down (held by `R1818` 110 kohm and `R805` 100 kohm), so
// stage 1 can check them for a dead short and nothing more.
//
// BRAKE_ISENSE: the brake sink's current monitor after its 3.2k/3.2k divider,
// 0 to 1.006 V for 0 to 48.78 mA of coil current, through `R1819` 249 ohm.
// ANGLE_OUT_MON: the same node the angle BNC is driven from, 0 to 1.650 V,
// through `R810` 1 kohm. The ADC is rail-referenced and 16x coarser than the
// DAC's LSB: this distinguishes a live chain from a dead one and does not
// calibrate anything. A DMM on the BNC still does that.
constexpr uint8_t kBrakeIsense = 27; // BRAKE_ISENSE,  ADC1 (v1.0)
constexpr uint8_t kAngleOutMon = 29; // ANGLE_OUT_MON, ADC3 (v1.0)
#endif

// -----------------------------------------------------------------------------
// Spare pads. Routed nowhere on this revision.
// -----------------------------------------------------------------------------
// Safe to exercise with internal pulls, which is exactly what the self test
// does with them: a pad that cannot be pulled both ways has a solder defect.
//
// On rev 0 the three DAC pads are unrouted too, so they get tested as spares:
// 12 pads, and stage 1 reports 27 checks. On rev 1 they carry the DAC bus and
// are left alone: 9 pads, 24 checks. At v1.0 only GP20 and GP22 are free
// (reserved for PYC_PULL_EVENT and PYC_FAULT, both retired); their MCU pads
// reach only the B side of an idle bus-switch channel: 2 pads, and stage 1
// reports 20 checks. A board that reports a different count was built with
// the wrong env.
#if MJ_BOARD_REV == 0
constexpr uint8_t kSpares[] = {10, 11, 12, 13, 15, 16, 17, 18, 20, 22, 23, 27};
#elif MJ_BOARD_REV == 1
constexpr uint8_t kSpares[] = {12, 15, 16, 17, 18, 20, 22, 23, 27};
#else
constexpr uint8_t kSpares[] = {20, 22};
#endif
constexpr uint8_t kSpareCount = sizeof(kSpares) / sizeof(kSpares[0]);

// -----------------------------------------------------------------------------
// Pins that are NOT on this board.
// -----------------------------------------------------------------------------
// No revision has an SWD header: `U501` pads 24 and 25 (v0.1-v0.3) and `U601`
// pads 24 and 25 (v1.0) -- SWCLK and SWD -- are no-connect. Through v0.3 RUN
// (pad 26) is no-connect too, so there is no reset button, only a power
// cycle; v1.0 fits a RUN switch (`C615` 100 nF on the RUN net). BOOTSEL
// recovery by button needs USB *and* pyControl unplugged, because the MCU
// domain is ORed from both through `D301` and `D1901` (v0.1 designators).
//
// The 1200 baud touch and the self test's `B` command are therefore the only
// routes into BOOTSEL that do not involve the button. The touch was verified on
// v0.3 board E4654C159F171E34 on 2026-09-09: a reflash over the running
// application, no `SW501`, no unplugging.
//
// BOOTSEL itself is the standard circuit: `SW501` shorts ~{USB_BOOT} to GND,
// `R505` 1 kohm ties that to QSPI_SS, and `R504` 10 kohm pulls QSPI_SS up to
// +3.3V_MCU (v0.1 designators; `R606` 1 kohm and `R605` 10 kohm at v1.0).
// Holding the button at power-on brings up the RPI-RP2 drive.
//
// Three traps come from the PlatformIO `generic` variant's default pin mapping,
// which was written for a Pico and does not describe this board:
//
//   * LED_BUILTIN / PIN_LED resolves to GPIO 25, which here is FW_TRIG0. There
//     is no LED on this board. Toggling it is harmless -- it drives an
//     instrument-port marker -- but it is not an LED.
//   * Serial2 defaults to GPIO 8/9, which here are TMC_DIAG1_ and
//     ~{BUS_SW_EN}. Never call Serial2 on this board.
//   * Wire1 defaults to GPIO 26/27, which here are the rail sense ADC and
//     (v1.0) BRAKE_ISENSE. Never call Wire1 on this board. Wire (I2C0) is
//     the DAC bus at v1.0, on GP16/GP17, and must be given those pins
//     explicitly: its default GP4/GP5 are the TMC's CIPO and DIAG0.

} // namespace pins

// The revision name `ID` reports as `hw`, from the same selector that chooses
// the pad map above. The two cannot disagree.
#if MJ_BOARD_REV == 0
constexpr char kHwRevName[] = "rev0";
#elif MJ_BOARD_REV == 1
constexpr char kHwRevName[] = "rev1";
#else
constexpr char kHwRevName[] = "rev2";
#endif

#if MJ_BOARD_REV >= 2
// I2C addresses of the two DAC80501Z parts, set by their A0 straps on the
// board (`R806` to GND, `R1807` to +3.3V_FILT). Board facts, like the pins.
constexpr uint8_t kAngleDacI2cAddress = 0x48;
constexpr uint8_t kBrakeDacI2cAddress = 0x49;
#endif

} // namespace mouse_joystick
