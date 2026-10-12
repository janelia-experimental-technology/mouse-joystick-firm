# Evidence summary

<!-- lai:begin la1 -->
## la1

Decoded 2026-10-09T09:20:14-04:00 with `lai monitor decode`, profile `mouse-joystick-pcb-v1.0-digital50-pycontrol`.

| Capture | Value |
| --- | --- |
| file | `la1.sr` |
| sample rate | 1 MSa/s |
| samples | 3,000,716 (3.001 s) |
| channels | CH0 CH1 CH2 CH3 |
| roles present | marker0 (CH0), marker1 (CH1), trace (CH1), link_tx (CH2), link_rx (CH3), beacon_clock (CH0), beacon_data (CH1) |
| roles missing | enc_a, enc_b, limit, hv_en, tmc_sck, tmc_cs, tmc_copi, tmc_cipo, tmc_drv_en, bus_sw_en |
| **health** | **silent**: no link traffic in this window; the trace is alive (no host, or between sessions) |

### Link

- no link lines on CH3, CH2

### Trace

- CH1 at 115200 8N1: 861 bytes, 0 frame errors, 0.218–2.235 s, longest gap 0.983 s
- qspy: 13 records, dictionaries Obj-Dict 3, Usr-Dict 16, Sig-Dict 2, Fun-Dict 1
- records: TRACE_STAT 3, SAMPLE_STAT 3, LINK_STAT 3, POLL_STAT 3, BOOT_INFO 1

### Markers

- `marker0` FW_TRIG0 (CH0): 0 rising, 0 falling
- `marker1` FW_TRIG1 (CH1): 1996 rising, 1996 falling; shares its channel with a UART role; edges are serial bits

### Beacon

- none: the clock line never rose in this capture: no beacon. Check that the firmware beacons on request (some builds beacon only at boot: wait and power-cycle the board), the profile's channels, the cable and buffer power.
<!-- lai:end la1 -->

<!-- lai:begin la3 -->
## la3

Decoded 2026-10-09T09:20:14-04:00 with `lai monitor decode`, profile `mouse-joystick-pcb-v1.0-digital50-pycontrol`.

| Capture | Value |
| --- | --- |
| file | `la3.sr` |
| sample rate | 1 MSa/s |
| samples | 3,001,496 (3.001 s) |
| channels | CH0 CH1 CH2 CH3 |
| roles present | marker0 (CH0), marker1 (CH1), trace (CH1), link_tx (CH2), link_rx (CH3), beacon_clock (CH0), beacon_data (CH1) |
| roles missing | enc_a, enc_b, limit, hv_en, tmc_sck, tmc_cs, tmc_copi, tmc_cipo, tmc_drv_en, bus_sw_en |
| **health** | **dead**: no link lines and no trace bytes: board off, cable, or wrong channels |

### Link

- no link lines on CH3, CH2

### Trace

- CH1 at 115200 8N1: 0 bytes, 0 frame errors
- qspy: not run (see warnings)

### Markers

- `marker0` FW_TRIG0 (CH0): 0 rising, 0 falling
- `marker1` FW_TRIG1 (CH1): 0 rising, 0 falling; shares its channel with a UART role; edges are serial bits

### Beacon

- none: no frame start: no interval with data high while clock is low. Check the profile's beacon channels, the cable and buffer power, the capture timing, and whether firmware emitted a beacon.
<!-- lai:end la3 -->

<!-- lai:begin la4 -->
## la4

Decoded 2026-10-09T09:20:15-04:00 with `lai monitor decode`, profile `mouse-joystick-pcb-v1.0-digital50-pycontrol`.

| Capture | Value |
| --- | --- |
| file | `la4.sr` |
| sample rate | 1 MSa/s |
| samples | 3,001,041 (3.001 s) |
| channels | CH0 CH1 CH2 CH3 |
| roles present | marker0 (CH0), marker1 (CH1), trace (CH1), link_tx (CH2), link_rx (CH3), beacon_clock (CH0), beacon_data (CH1) |
| roles missing | enc_a, enc_b, limit, hv_en, tmc_sck, tmc_cs, tmc_copi, tmc_cipo, tmc_drv_en, bus_sw_en |
| **health** | **silent**: no link traffic in this window; the trace is alive (no host, or between sessions) |

### Link

- no link lines on CH3, CH2

### Trace

- CH1 at 115200 8N1: 861 bytes, 0 frame errors, 0.210–2.227 s, longest gap 0.983 s
- qspy: 13 records, dictionaries Obj-Dict 3, Usr-Dict 16, Sig-Dict 2, Fun-Dict 1
- records: TRACE_STAT 3, SAMPLE_STAT 3, LINK_STAT 3, POLL_STAT 3, BOOT_INFO 1

### Markers

- `marker0` FW_TRIG0 (CH0): 0 rising, 0 falling
- `marker1` FW_TRIG1 (CH1): 1984 rising, 1984 falling; shares its channel with a UART role; edges are serial bits

### Beacon

- none: the clock line never rose in this capture: no beacon. Check that the firmware beacons on request (some builds beacon only at boot: wait and power-cycle the board), the profile's channels, the cable and buffer power.
<!-- lai:end la4 -->
