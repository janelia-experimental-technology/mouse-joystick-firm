# Hardware definition for the dual-joystick Neuropixels rig (Rig 1).
# Breakout Board v1.2.
#
# Last verified: 2026-08-03 17:45 EDT
#
# This is the CANONICAL hardware definition for this rig. Load it via the
# pyControl GUI: Config -> "Load hardware definition".
#
# NOTE: "Load framework" DELETES hardware_definition.py from the pyboard
# (see pycboard.load_framework). Any time you reload the framework you must
# re-load this file, or task upload fails with:
#     ImportError: no module named 'hardware_definition'
#
# EDIT CYCLE: editing this file on disk does NOT change the pyboard. To make a
# change take effect:  edit -> Config -> Load hardware definition -> re-Upload
# the task -> restart the session. 
#
# ---------------------------------------------------------------------------
# pyControl pin map (kept consistent with the IBL rig where the feature exists)
# ---------------------------------------------------------------------------
#   BNC_1 (X12)     - camera frame trigger, 150 Hz  -> both cameras Line 0
#   DAC_2 (X6)      - rsync (irregular sync pulses) -> both cameras Line 3
#                                                   -> OneBox IO0 (ADC input)
#   Port 4 POW_A    - solenoid drive (low-side MOSFET, 12 V)
#   BNC_2 (X11)     - opto: TTL gate -> Plexon LD-1 -> PlexBright LED module
#                     (single channel, wired by Frank 2026-08-27)
#
# SAFETY: DAC_1/DAC_2 are pyboard pins X5/X6 and are NOT 5 V tolerant.
#   OneBox IO0 must be configured as an ADC INPUT on the SpikeGLX Obx Setup
#   tab. DAC_2 is the only driver on that 4-way tee; if IO0 were ever set as a
#   DAC output, two devices would drive the same node.
#
# Reserved / not wired yet:
#   BNC_2 (X11)     - now wired: the confirmed opto TTL gate (see pin map
#                     above). Also used, incidentally, by the AD3
#                     latency-validation task (tasks/rig1/led_flash_test.py),
#                     since that task exercises this exact signal chain.
#                     NOTE: X11 is electrically the SAME pin as port_6.DIO_B
#                     in Breakout_1_2 (see devices/breakout_1_2.py). Port 6
#                     is unused in this file today, so there is no conflict,
#                     but do not wire port_6 without accounting for this.
#                     STALE DIAGRAM: Hardware/rig1_signal_topology_interactive_
#                     20260828_1.html still lists BNC-2 as a free pin -- that
#                     diagram needs to be regenerated/corrected separately.
#   Port 3          - LaserLickPort (DIO_A=X9, DIO_B=X10). NOTE: the
#                     laser_lick_port.py driver is NOT in this repo's devices/
#                     folder yet — copy it from the IBL rig before using.
#   DAC_1 (X5)      - shared with Port 3 DIO_C. Was an opto gate in the old
#                     joystick_rig.py definition; opto is now resolved onto
#                     BNC_2 instead (Plexon LD-1, single channel -- see pin
#                     map above), so DAC_1 stays free for a future Port 3
#                     lick sensor.
#
# ---------------------------------------------------------------------------
# Camera wiring — 2x FLIR Blackfly S BFS-U3-20S4M-C
# ---------------------------------------------------------------------------
# Sony IMX422, 1616 x 1240, global shutter, USB 3.1 Gen 1, 175 fps max.
# (Corrected 2026-08-03: this rig previously ran BFS-U3-31S4M. The GPIO pinout
#  is unchanged, but resolution and frame-rate limits are different.)
#
# The 6-pin GPIO pinout below is COMMON TO ALL BFS-U3 CASED MODELS — verified
# against FLIR's Digital IO Control pages for 04S2, 13Y3, 31S4 and 51S5, which
# are identical. It is therefore also valid for the IBL rig's BFS-U3-13Y3M-C.
# (Corrected 2026-08-03: an earlier version of this file warned that the line
#  numbering differed between models. It does not.)
#
#   Pin 1  Green  Line 3  non-isolated input   <- pyControl DAC_2 (rsync)
#                         GPI high level 2.6-3.6 V; propagation delay 1 us
#                         !! DUAL FUNCTION: also V_AUX, 8-24 V DC power input.
#                            Input-high ceiling is 3.6 V. NEVER apply 5 V here.
#   Pin 2  Black  Line 0  opto-isolated input  <- pyControl BNC_1 (trigger),
#                         DIRECT 3.3V drive, NO series resistor in this path.
#                         (Corrected 2026-08-04: an earlier version of this
#                          comment claimed a series current-limit resistor
#                          existed here and left its value as a TODO. There is
#                          no resistor on CAM IN. Confirmed working via
#                          sanity_check_rig1.py: 4499 frames/camera, 0 dropped,
#                          0 incomplete, at both 150 Hz and 75 Hz. Input is
#                          rated 2.6-30 V, so 3.3V direct drive is within spec
#                          at the low end, not a risk of over-drive.)
#   Pin 3  Red    Line 2  +3.3 V out (V_OUT, 120 mA) -> pull-up source for Line 1
#   Pin 4  White  Line 1  opto-isolated output -> Exposure-Active ("CamOut"),
#                         open-drain: 10 kOhm pull-up to Pin 3, then -> OneBox
#                         IO1 (cam1) / IO2 (cam2)
#                         propagation delay 36 us low->high, 18 us high->low
#   Pin 5  Blue   opto GND     -\ Pins 5 + 6 bonded, then to CAM IN BNC shell
#   Pin 6  Brown  camera GND   -/ (see "grounding" note below)
#
# GROUNDING — why Pin 5 and Pin 6 are bonded, and what it costs:
#   Line 1 is open-drain: it can only pull toward opto GND (Pin 5). The high
#   level comes entirely from the external pull-up, which is sourced from Pin 3
#   (V_OUT) — a NON-isolated rail referenced to Pin 6. The current loop is
#       Pin3 -> 10k -> Pin4 -> [transistor] -> Pin5
#   and it only closes if Pin 5 connects to Pin 6. The bond is therefore
#   REQUIRED by this topology, not a wiring error. FLIR's own multi-camera sync
#   application note does the same.
#   CONSEQUENCE: galvanic isolation is forfeited. Opto GND, camera GND, USB/PC
#   ground, pyControl ground and OneBox aux ground are one node. Record this in
#   the Neuropixels grounding plan; it is a known ground-loop path.
#   To restore isolation later: source the Line 1 pull-up from an external 5 V
#   referenced to the OneBox side, leave Pin 6 unconnected. This also frees
#   Line 2 as a third non-isolated GPIO and gives a cleaner 5 V swing into the
#   OneBox ADC.
#
# OPEN ITEM — CAM OUT ground return:
#   Pins 5+6 are documented above as going to the CAM IN BNC shell. That leaves
#   the CAM OUT BNC shell with no local ground connection at the camera, so the
#   OneBox references that signal to camera ground only via PC/USB ground — a
#   large-area loop on the very signal used for frame timing. TRACE THIS with a
#   continuity meter and update this comment with what is actually built.
#
# Both cameras share one trigger and one rsync line via BNC tees, so this file
# does not change when going from 1 to 2 (or 4) cameras.
#
# SpikeGLX path for the exposure strobes:
#   Cam1 Line 1 -> breakout IO1 -> ADC1 -> XA1
#   Cam2 Line 1 -> breakout IO2 -> ADC2 -> XA2
#   rsync       -> breakout IO0 -> ADC0 -> XA0
#
# ---------------------------------------------------------------------------
# 150 Hz constraints (BFS-U3-20S4M)
# ---------------------------------------------------------------------------
#  - Full resolution is 1616 x 1240 with a 175 fps ceiling, so 150 Hz does NOT
#    require a windowed ROI on this sensor. (Corrected 2026-08-03: the previous
#    text described the 31S4M's 2048x1536 / ~55 fps limit and was wrong for
#    these cameras.)
#  - BANDWIDTH, not readout, is the real limit with two cameras:
#        1616 x 1240 Mono8 = 2.00 MB/frame
#        at 150 Hz         = 300 MB/s per camera -> 600 MB/s for the pair
#        one USB 3.1 Gen 1 host controller ~ 380-400 MB/s in practice
#    Both cameras currently enumerate under the SAME host controller. Either
#    crop the ROI (1024x768 -> 118 MB/s each) or split across controllers.
#    UNVERIFIED as of 2026-08-03: whether both cameras were streaming
#    simultaneously when 150 Hz was observed. Confirm with FrameID gap counts.
#  - Exposure constraint: ExposureTime must be shorter than the frame period
#    (6.67 ms at 150 Hz) minus readout, NOT shorter than the trigger high time.
#    Exposure duration is set by ExposureTime and is independent of trigger
#    pulse width in ExposureMode=Timed. (Corrected 2026-08-03: the previous
#    "keep exposure below the 3.33 ms high time" note applies only to
#    ExposureMode=TriggerWidth. 3998 us against a 3333 us high time works.)
#  - CAMERA SETTINGS DO NOT PERSIST unless saved: UserSetSelector=UserSet0 ->
#    UserSetSave -> UserSetDefault=UserSet0. A USB reconnect otherwise reloads
#    the factory Default set, in which TriggerMode is Off. This is what caused
#    the 2026-08-03 free-running fault.
#
# ---------------------------------------------------------------------------
# Planned change, NOT yet implemented (do not action until TriggerMode is
# confirmed and the cameras are known to be triggering):
#   Swap trigger onto Line 3 (Green) and rsync onto Line 0 (Black).
#   Rationale: Line 3 has a 1 us propagation delay vs 18 us for the opto input,
#   and takes 3.3 V natively with no current-drive requirement. Rsync pulses are
#   ~25 ms wide and only need to be identified, not precisely timed, so the
#   18 us belongs there. Requires: 2-wire swap at the BNC pigtails,
#   TriggerSource Line0 -> Line3 in the camera config, and updating the rsync
#   bit index in validate_camera_sync.py.
# ---------------------------------------------------------------------------

from pyControl.hardware import *
from devices import *

board = Breakout_1_2()

# --- Rsync: sync pulses at RANDOM intervals -------------------------------
# The randomness is the point: the sequence of gaps is a unique fingerprint, so
# post hoc alignment (Rsync_aligner / TPrime) can identify individual pulses
# even if some are missed. A regular square wave cannot — its edges are
# interchangeable. mean_IPI is the mean gap in ms; actual gaps are drawn
# uniformly between 0.1x and 1.9x that value (100-1900 ms here).
#
# pulse_dur is now set EXPLICITLY rather than left at the default. The cameras
# sample this line ONCE PER FRAME, at exposure end — a 6.67 ms grid at 150 Hz.
# A pulse shorter than the frame period can fall entirely between two
# exposure-ends and be invisible to the cameras, and you would not discover it
# until alignment failed. 25 ms spans 3-4 frames, with wide margin against the
# 100 ms minimum inter-pulse interval.
sync_output = Rsync(pin=board.DAC_2, mean_IPI=1000, pulse_dur=25, event_name="rsync")

# --- Camera frame trigger: REGULAR square wave ----------------------------
# This paces camera capture, so unlike rsync it must be perfectly regular.
# 150 Hz = 150 frames per second. Rising-edge times and pulse numbers are
# written to a separate pyControl analog data file (.pca) automatically —
# that file is a per-frame time record and feeds the alignment pipeline.
#
# Per the pyControl docs this train runs continuously WHILE THE FRAMEWORK IS
# RUNNING and should stop on session end. If OneBox shows it continuing after
# Stop, that is a genuine anomaly worth reporting upstream — and in the interim
# add an explicit stop to the task's run_end().
camera_trigger = Frame_trigger(pin=board.BNC_1, pulse_rate=150, name="cam_trigger")

# --- Water reward solenoid ------------------------------------------------
solenoid = Digital_output(pin=board.port_4.POW_A)

# --- Optogenetics (BNC_2) ---------------------------------------------------
# TTL gate into the Plexon LD-1 -> PlexBright LED module (single channel,
# wired 2026-08-27). Plain logic-level output; current/intensity control
# lives entirely in the Plexon driver, external to the pyboard.
#
# hw.opto.on()/.off()/.toggle() -- inherited from Digital_output, e.g. for
#     constant-light calibration (tasks/rig1/opto_constant_light_test.py).
# hw.opto.burst(n_pulses, ...) -- fixed-count pulse train, e.g. for a
#     hardcoded-duration stim (tasks/rig1/opto_fixed_stim.py).
# hw.opto.configure(...) once, then .start()/.stop() -- for event-triggered
#     stimulation of unknown duration (fixed frequency/duty, but arbitrary
#     start/stop timing) once real trial trigger events exist.
# Both burst() and configure() accept the pulse timing as exactly one of:
#     freq+duty_cycle, freq+pulse_width_ms, or pulse_width_ms+wait_width_ms
#     -- see devices/opto_led.py.
#
# Also exercised directly by tasks/rig1/led_flash_test.py, an AD3 bench
# validation task that predates this being confirmed as the real opto line.
opto = Opto_LED(pin=board.BNC_2)
