// ADS8689 (ADS868x family) register map, command framing and transfer
// function. Arduino-free, header-only, host tested.
//
// Source: TI SBAS633E, "ADS868x 16-Bit, High-Speed, Single-Supply, SAR ADC
// Data Acquisition System With Programmable, Bipolar Input Ranges", revision E
// (August 2022). Section numbers below are that document's. The cached copy is
// `kicad-libs/tmp/datasheets/ads8689.pdf`; the facts layer that cites it is
// `kicad-libs/db/facts/OW-SENS-ADS8689IPWR.yaml`.
//
// ---------------------------------------------------------------------------
// HOW THIS PART TALKS (section 7.5)
// ---------------------------------------------------------------------------
// The host sees one 32-bit shift register. A data transfer frame starts at the
// falling edge of CONVST/CS and ends at its rising edge, and THAT RISING EDGE
// STARTS A CONVERSION. Within the frame the host shifts a 32-bit command in on
// SDI, MSB first, while the device shifts its output word out on SDO.
//
//   command word   [31:25] opcode   [24:16] 9-bit register address   [15:0] data
//   output word    [31:16] the latest conversion result, OR the 16-bit register
//                  requested by a READ_HWORD in the PREVIOUS frame, OR a fixed
//                  pattern selected by DATAOUT_CTL.DATA_VAL; [15:0] zeros
//                  unless data flags are enabled
//
// Two consequences shape the device class:
//
//   * A register read takes TWO frames: the command in frame F, the value in
//     frame F+1. A single-frame "read" returns a conversion result and looks
//     plausible, which is the trap.
//   * Every frame starts a conversion, so after every frame the host must wait
//     t_conv_max (5 us on the ADS8689) before the next frame or CONVST edge.
//     The RVS pin that signals completion is not routed to the MCU on
//     mouse-joystick-pcb, so the wait is by time.
//
// A frame of fewer than 32 SCLKs is treated as a NOP: a bare CONVST/CS pulse
// therefore starts a conversion without a transfer, and the following NOP frame
// returns its result. Frames longer than 32 clocks execute the LAST 32 bits.
//
// After power-up or RST the device speaks SPI-00-S: CPOL 0, CPHA 0, SDI
// captured on the rising SCLK edge, SDO's MSB launched on the CS falling edge
// and each following bit on the falling SCLK edge (tables 7-8 and 7-9). That is
// SPI mode 0 as the Arduino SPI library numbers it, and this driver never
// selects another protocol.
//
// ---------------------------------------------------------------------------
// WHY RANGE_SEL IS WRITTEN BEFORE ANY RESULT IS TRUSTED
// ---------------------------------------------------------------------------
// RANGE_SEL resets to +/-3 x VREF, i.e. +/-12.288 V across 65536 codes. A
// 0-5 V signal read in that range uses a fifth of the codes, and -- worse --
// reads as plausible numbers. The board's signal is 0-5 V, so the range is set
// to 0 to 1.25 x VREF (0-5.12 V, 78.125 uV per code) and read back before the
// device is marked configured. A wrong range is a silent 4.8x scale error, the
// same class of mistake as the DAC80501's GAIN and treated the same way.

#pragma once

#include <stdint.h>

namespace ads8689 {

// ---------------------------------------------------------------------------
// Command opcodes, bits [31:25] of the command word (table 7-5). The two low
// bits of the 7 select the byte for the WRITE family and are don't-care for
// the HWORD commands; they are shown as the datasheet writes them.
// ---------------------------------------------------------------------------
constexpr uint32_t kOpNop = 0x00u;        // 00000_00
constexpr uint32_t kOpClearHword = 0x60u; // 11000_00  clear the 1 bits of data
constexpr uint32_t kOpReadHword = 0x64u;  // 11001_00  16-bit read, next frame
constexpr uint32_t kOpReadByte = 0x24u;   // 01001_00  8-bit read, next frame
constexpr uint32_t kOpWriteHword = 0x68u; // 11010_00  write both bytes
constexpr uint32_t kOpWriteMsb = 0x69u;   // 11010_01  write the MS byte only
constexpr uint32_t kOpWriteLsb = 0x6Au;   // 11010_10  write the LS byte only
constexpr uint32_t kOpSetHword = 0x6Cu;   // 11011_00  set the 1 bits of data

constexpr uint32_t kOpcodeMask = 0x7Fu;
constexpr uint32_t kAddressMask = 0x1FFu;

// Assemble a command word. The 9-bit address is the 8-bit register address
// with a leading zero; HWORD commands ignore its LSB and act on the
// half-word, so 0x04 and 0x05 both reach RST_PWRCTL[15:0].
constexpr uint32_t frame(uint32_t opcode, uint16_t address, uint16_t data) {
  return ((opcode & kOpcodeMask) << 25) |
         ((static_cast<uint32_t>(address) & kAddressMask) << 16) |
         static_cast<uint32_t>(data);
}

constexpr uint32_t kFrameNop = 0x00000000u;

constexpr uint32_t writeFrame(uint8_t reg, uint16_t value) {
  return frame(kOpWriteHword, reg, value);
}
constexpr uint32_t readFrame(uint8_t reg) {
  return frame(kOpReadHword, reg, 0x0000u);
}
constexpr uint32_t setBitsFrame(uint8_t reg, uint16_t mask) {
  return frame(kOpSetHword, reg, mask);
}
constexpr uint32_t clearBitsFrame(uint8_t reg, uint16_t mask) {
  return frame(kOpClearHword, reg, mask);
}

// The 16 bits that carry a conversion result, a register read-back or a fixed
// pattern: the upper half of the output word.
constexpr uint16_t upperHalf(uint32_t output_word) {
  return static_cast<uint16_t>(output_word >> 16);
}

// ---------------------------------------------------------------------------
// Register addresses (table 7-10). Each is a 32-bit register addressed by its
// low byte; the HWORD commands reach its lower 16 bits at this address and its
// upper 16 bits at address + 2.
// ---------------------------------------------------------------------------
constexpr uint8_t kRegDeviceId = 0x00;
constexpr uint8_t kRegRstPwrctl = 0x04;
constexpr uint8_t kRegSdiCtl = 0x08;
constexpr uint8_t kRegSdoCtl = 0x0C;
constexpr uint8_t kRegDataoutCtl = 0x10;
constexpr uint8_t kRegRangeSel = 0x14;
constexpr uint8_t kRegAlarm = 0x20;
constexpr uint8_t kRegAlarmHTh = 0x24;
constexpr uint8_t kRegAlarmLTh = 0x28;

// DEVICE_ID (7.6.1.1). The only writable field, DEVICE_ADDR[3:0], sits in
// register bits [19:16], i.e. bits [3:0] of the UPPER half-word at address
// 0x02. It exists for daisy chains; here it is a scratch field that proves a
// write reached the part and came back.
constexpr uint8_t kRegDeviceIdHigh = 0x02;
constexpr uint16_t kDeviceAddrMask = 0x000Fu;

// RST_PWRCTL (7.6.1.2). Every write must carry WKEY = 0x69 in [15:8] or the
// low bits are not written.
constexpr uint16_t kRstPwrctlKey = 0x6900u;
constexpr uint16_t kRstPwrctlPwrdn = 1u << 0;    // 1 = power down
constexpr uint16_t kRstPwrctlNapEn = 1u << 1;    // 1 = NAP after each conversion
constexpr uint16_t kRstPwrctlRstnApp = 1u << 2;  // 1 = RST pin is an application reset
constexpr uint16_t kRstPwrctlInAlDis = 1u << 4;  // 1 = input alarm disabled
constexpr uint16_t kRstPwrctlVddAlDis = 1u << 5; // 1 = VDD alarm disabled
constexpr uint16_t kRstPwrctlReset = 0x0000u;

// SDI_CTL (7.6.1.3). SDI_MODE[1:0]: 00 = CPOL 0 CPHA 0, the power-up protocol.
constexpr uint16_t kSdiModeMask = 0x0003u;
constexpr uint16_t kSdiModeSpi00 = 0x0000u;
constexpr uint16_t kSdiModeSpi01 = 0x0001u;
constexpr uint16_t kSdiModeSpi10 = 0x0002u;
constexpr uint16_t kSdiModeSpi11 = 0x0003u;

// SDO_CTL (7.6.1.4). Reset 0: SDO follows the SDI protocol, single SDO-0, the
// ALARM/SDO-1/GPO pin tri-stated.
constexpr uint16_t kSdoCtlDefault = 0x0000u;
constexpr uint16_t kSdoCtlSsyncClk = 1u << 6;
constexpr uint16_t kSdoCtlSdo1Alarm = 1u << 8;
constexpr uint16_t kSdoCtlSdo1Gpo = 2u << 8;
constexpr uint16_t kSdoCtlSdo1Dual = 3u << 8;
constexpr uint16_t kSdoCtlGpoVal = 1u << 12;

// DATAOUT_CTL (7.6.1.5). DATA_VAL[2:0] selects what the upper half of the
// output word carries: 0xx = the conversion, 100 = all zeros, 101 = all ones,
// 110 = alternating bits, 111 = alternating bit pairs. The fixed patterns are
// a loopback test of SDO, the bus switch and the host's capture edge, with no
// analog input involved.
constexpr uint16_t kDataValMask = 0x0007u;
constexpr uint16_t kDataValConversion = 0x0000u;
constexpr uint16_t kDataValAllZeros = 0x0004u;
constexpr uint16_t kDataValAllOnes = 0x0005u;
constexpr uint16_t kDataValAlternatingBits = 0x0006u;
constexpr uint16_t kDataValAlternatingPairs = 0x0007u;
constexpr uint16_t kDataoutParEn = 1u << 3;
constexpr uint16_t kDataoutRangeIncl = 1u << 8;
constexpr uint16_t kDataoutInAlarmIncl = 3u << 10;
constexpr uint16_t kDataoutVddAlarmIncl = 3u << 12;
constexpr uint16_t kDataoutDeviceAddrIncl = 1u << 14;
constexpr uint16_t kDataoutCtlReset = 0x0000u;

// RANGE_SEL (7.6.1.6). RANGE_SEL[3:0]; bit 3 set = unipolar. INTREF_DIS bit 6.
constexpr uint16_t kRangeSelMask = 0x000Fu;
constexpr uint16_t kRangeBipolar3 = 0x0u;      // +/-3     x VREF  +/-12.288 V
constexpr uint16_t kRangeBipolar2p5 = 0x1u;    // +/-2.5   x VREF  +/-10.24 V
constexpr uint16_t kRangeBipolar1p5 = 0x2u;    // +/-1.5   x VREF  +/-6.144 V
constexpr uint16_t kRangeBipolar1p25 = 0x3u;   // +/-1.25  x VREF  +/-5.12 V
constexpr uint16_t kRangeBipolar0p625 = 0x4u;  // +/-0.625 x VREF  +/-2.56 V
constexpr uint16_t kRangeUnipolar3 = 0x8u;     // 0 to 3    x VREF  12.288 V
constexpr uint16_t kRangeUnipolar2p5 = 0x9u;   // 0 to 2.5  x VREF  10.24 V
constexpr uint16_t kRangeUnipolar1p5 = 0xAu;   // 0 to 1.5  x VREF  6.144 V
constexpr uint16_t kRangeUnipolar1p25 = 0xBu;  // 0 to 1.25 x VREF  5.12 V
constexpr uint16_t kRangeUnipolarFlag = 0x8u;
constexpr uint16_t kRangeIntrefDis = 1u << 6;
constexpr uint16_t kRangeSelReset = 0x0000u; // +/-3 x VREF, internal reference on

// ALARM (7.6.1.7), read only.
constexpr uint16_t kAlarmOvw = 1u << 0;
constexpr uint16_t kAlarmTrpInH = 1u << 4;
constexpr uint16_t kAlarmTrpInL = 1u << 5;
constexpr uint16_t kAlarmTrpVddH = 1u << 6;
constexpr uint16_t kAlarmTrpVddL = 1u << 7;
constexpr uint16_t kAlarmActiveInH = 1u << 10;
constexpr uint16_t kAlarmActiveInL = 1u << 11;
constexpr uint16_t kAlarmActiveVddH = 1u << 14;
constexpr uint16_t kAlarmActiveVddL = 1u << 15;

// ---------------------------------------------------------------------------
// Timing, ADS8689 column (sections 6.6 to 6.8). Maximums where the host must
// wait, minimums where it must not hurry.
// ---------------------------------------------------------------------------
constexpr uint32_t kConversionTimeMaxNs = 5000;   // t_conv, after the CS rising edge
constexpr uint32_t kAcquisitionTimeMinNs = 5000;  // t_acq, CS low plus any idle before the next edge
constexpr uint32_t kSampleRateMaxHz = 100000;     // 100 kSPS
constexpr uint32_t kSclkMaxHz = 66670000;         // 66.67 MHz
constexpr uint32_t kPorResetDelayMaxMs = 20;      // t_D_RST_POR, RST rising to ready
constexpr uint32_t kReferenceTurnOnTypMs = 20;    // internal reference, 10 uF on REFCAP

// ---------------------------------------------------------------------------
// Transfer function.
// ---------------------------------------------------------------------------
constexpr float kInternalReferenceVolts = 4.096f;
constexpr uint32_t kCodeCount = 65536u;
constexpr uint16_t kCodeMax = 0xFFFFu;

constexpr bool rangeIsUnipolar(uint16_t range) {
  return (range & kRangeUnipolarFlag) != 0u;
}

// The positive full scale as a multiple of VREF, for each RANGE_SEL code.
constexpr float rangeMultiplier(uint16_t range) {
  return ((range & 0x7u) == 0x0u)   ? 3.0f
         : ((range & 0x7u) == 0x1u) ? 2.5f
         : ((range & 0x7u) == 0x2u) ? 1.5f
         : ((range & 0x7u) == 0x3u) ? 1.25f
         : ((range & 0x7u) == 0x4u) ? 0.625f
                                    : 0.0f; // 0x5-0x7: not a range
}

// Positive full-scale input, in volts.
constexpr float fullScaleVolts(uint16_t range,
                               float reference_volts = kInternalReferenceVolts) {
  return rangeMultiplier(range) * reference_volts;
}

// The whole span the 65536 codes cover: FS for a unipolar range, 2 FS for a
// bipolar one.
constexpr float spanVolts(uint16_t range,
                          float reference_volts = kInternalReferenceVolts) {
  return rangeIsUnipolar(range) ? fullScaleVolts(range, reference_volts)
                                : 2.0f * fullScaleVolts(range, reference_volts);
}

// One code. 78.125 uV on the 0 to 1.25 x VREF range this board uses.
constexpr float voltsPerCode(uint16_t range,
                             float reference_volts = kInternalReferenceVolts) {
  return spanVolts(range, reference_volts) / static_cast<float>(kCodeCount);
}

// Output code to input volts. Straight binary on every range: a unipolar range
// puts 0 V at code 0; a bipolar range puts -FS at code 0 and 0 V at 0x8000
// (section 7.3.5's transfer characteristic). The bipolar half of that is from
// the datasheet, not from this bench -- mouse-joystick-pcb only ever uses the
// unipolar range, where the two readings agree.
constexpr float codeToVolts(uint16_t code, uint16_t range,
                            float reference_volts = kInternalReferenceVolts) {
  return rangeIsUnipolar(range)
             ? static_cast<float>(code) * voltsPerCode(range, reference_volts)
             : (static_cast<float>(code) - 32768.0f) *
                   voltsPerCode(range, reference_volts);
}

// Input volts to the nearest code, clamped to the representable range.
constexpr uint16_t voltsToCode(float volts, uint16_t range,
                               float reference_volts = kInternalReferenceVolts) {
  return [](float exact) -> uint16_t {
    return (exact <= 0.0f) ? 0u
           : (exact >= static_cast<float>(kCodeMax))
               ? kCodeMax
               : static_cast<uint16_t>(exact + 0.5f);
  }(rangeIsUnipolar(range)
        ? volts / voltsPerCode(range, reference_volts)
        : volts / voltsPerCode(range, reference_volts) + 32768.0f);
}

} // namespace ads8689
