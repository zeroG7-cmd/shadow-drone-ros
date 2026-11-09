#!/usr/bin/env python3
"""
Shadow autotune runner - ArduCopter-native rewrite.

Nov 6, 2026 - full replacement of the Sep 20 PX4-era version. That one
climbed in OFFBOARD, switched to POSCTL, and triggered autotune with
MAV_CMD_DO_AUTOTUNE_ENABLE (command 212) - all PX4-specific mechanisms
that don't exist, or don't mean the same thing, on the ArduCopter stack
Shadow has run since the migration. This version uses ArduCopter's real
AUTOTUNE flight mode instead: a mode switch, not a command layered on
top of another mode.

Sourcing note, logged honestly because this is a safety-relevant
rewrite and I could not reach the primary doc page: ardupilot.org
rejected this session's fetch tool outright (a provenance/allowlist
error, not a 404 - same for the GitHub-hosted .rst source of that same
page). Everything below is sourced instead from QGroundControl's own
ArduCopter AutoTune tuning guide (docs.qgroundcontrol.com), ArduPilot's
GitHub issue #14584 ("AutoTune should always save tuning gains after
disarming"), an ardupilot.org discourse thread on triggering AUTOTUNE
from MAVProxy/a companion computer with no RC transmitter, and a
third-party walkthrough (ardudeck.com) - fetched 2026-11-06. Treat the
two points below as confirmed by independent secondary sources, and
everything in the "NOT independently confirmed" list further down as
this script's own best-effort design, to be watched live on the first
real run exactly like every other fix this session - not assumed safe
unattended.

Confirmed by multiple independent sources:
  - AUTOTUNE is entered by a plain flight-mode change - the same
    mechanism as switching to GUIDED or LAND, triggerable by a
    companion computer with no RC transmitter involved (QGC's own
    guide describes picking it from a GCS mode menu as an alternative
    to a transmitter switch; the discourse thread shows it driven
    entirely from MAVProxy).
  - Gains are only saved if the vehicle lands AND disarms WHILE STILL
    IN AUTOTUNE MODE. Switching to another mode (LAND, RTL, anything
    else) before disarming discards the tune and reverts to the
    pre-tune gains - this was bad enough to get its own GitHub issue
    (#14584) after Copter 4.0.3 made it worse than earlier versions.
    This is the one rule this script cannot violate: _land_and_save()
    below never calls set_mode() again after entering AUTOTUNE.

NOT independently confirmed - this script's own design, flagged for
live verification on the first real run:
  - Whether AUTOTUNE mode needs continuous RC input (real or
    override) to avoid an RC-failsafe / mode-init failure the way a
    hands-off hover would in GUIDED. The discourse thread's own
    working SITL sequence used explicit MAVProxy `rc 3 1800` /
    `rc 3 1500` throttle overrides around the mode switch, which is
    the only direct evidence this script has - no primary doc
    confirms or denies whether that's load-bearing or incidental to
    that one thread's setup. This script follows that evidence:
    it streams a centered RC override (roll/pitch/yaw at 1500,
    throttle at RC_THROTTLE_HOLD) continuously from a few seconds
    before the mode switch through the whole hold, then eases the
    throttle override down to descend while still in AUTOTUNE,
    matching that same thread's pattern.
  - The actual PWM value that holds altitude in AltHold-style control
    (RC_THROTTLE_HOLD below) is a guess (centered stick, 1500) - it
    is NOT derived from Shadow's real hover throttle. If Shadow
    climbs or sinks once AUTOTUNE engages, that's the first thing to
    adjust. Watch the altitude log lines during AUTOTUNE_HOLD_SECONDS
    on the first run and be ready to hit Stop Simulation.
  - AUTOTUNE_AXES / AUTOTUNE_AGGR are left exactly as already
    configured on the vehicle (ArduCopter's own defaults are
    documented elsewhere as roll+pitch, middle aggressiveness) -
    this script does not change them. /mavros/param/get is read
    once, best-effort, purely to log what's active; the Sep 20
    session already found that service unreliable in this MAVROS
    build (see the old script's _check_mc_at_en history below this
    docstring in git blame), so nothing here depends on it working.
  - Exact ArduCopter STATUSTEXT strings for autotune progress
    ("AutoTune: Started" etc.) are not confirmed from a primary
    source either - this script logs every statustext line verbatim
    (so they're visible in autotune.log either way) but does NOT gate
    any decision on matching specific text. The real gate is the
    fixed hold timer plus watching current_state.mode for an
    unexpected departure from AUTOTUNE, same as the rest of this
    flight stack already does for mode-switch confirmation.

Run the stack with its default world (shadow_runway - the only one that
exists for the current ArduPilot/Gazebo Harmonic stack; `empty` and
`living_room` were retired in the Oct 17 migration and no longer have
`.sdf` files at all): `~/shadow_ws/scripts/run_full_sim.sh`, then this
(or the dashboard's Autotune button, which calls run_autotune.sh, which
calls this unchanged). shadow_runway.sdf was read directly on Nov 6 and
confirmed to have open sky above Shadow's spawn point, so the 6m climb
below has real headroom there - this is not the "empty was safer"
claim the Sep 20 version of this comment made, which no longer applies.

Nov 6, 2026, first real run - GUIDED->AUTOTUNE directly was rejected by
the FC ("Mode change to Autotune failed: init failed", visible in
autotune.log). Read ArduCopter's own source
(ArduCopter/mode_autotune.cpp, ModeAutoTune::start()) to find out why:
AUTOTUNE only accepts a mode switch from STABILIZE, ALT_HOLD, LOITER,
or POSHOLD - GUIDED was never going to work, no matter how clean the
climb was. Fixed below by switching to ALT_HOLD first (confirmed real,
not just mode_sent) before requesting AUTOTUNE. This also resolves the
"does AUTOTUNE need continuous RC input" question in the NOT
independently confirmed list below: ALT_HOLD is itself throttle-stick-
driven, so the RC override this script already streams is load-bearing
from the ALT_HOLD switch onward, not just a precaution taken before
AUTOTUNE specifically.

Nov 6, 2026, second real run - the ALT_HOLD switch above was accepted,
but the static RC_THROTTLE_HOLD guess (1500, centered stick) was wrong:
Shadow sank 3.06m in the 3s this script used to just watch-and-abort.
Rather than swap in a different static guess (same problem, just a
different wrong number), this script was changed to MEASURE the real
hover throttle instead - see the Nov 6 third-run note below for why
that measurement was taken in the wrong mode entirely, and the actual
fix.

Nov 6, 2026, third real run - the hover-throttle search (previous note)
made things WORSE, not better: Shadow fell from ~5.7m to 0.04m (the
ground) in under 5 seconds while the search was supposedly converging,
with each successive PWM step showing a steeper sink, not a smaller
one. Read ArduPilot's own "Altitude Hold Mode" doc
(ardupilot.org/copter/docs/altholdmode.html, mirrored on gitea.com
since ardupilot.org itself is still blocked for this session's fetch
tool - same issue noted above) to find out why, and it invalidates the
entire premise of that search: in ALT_HOLD, the throttle channel is
NOT a direct thrust value at all. It is a CLIMB-RATE command, gated by
a deadzone around mid-stick (THR_DZ; "if the throttle stick is in the
middle (40%~60%) the vehicle will maintain the current altitude" -
ArduCopter's own internal altitude controller does the actual
holding). Outside that deadzone, deflection sets a climb/descend RATE
(up to PILOT_SPD_UP/PILOT_SPD_DN at full deflection), not a thrust
level. So a proportional search that treats PWM as "more PWM = more
lift, converge where vz=0" is using the wrong control model for this
mode entirely: every PWM this script tried (1500, 1561, 1791, 1800)
was likely still being read as "near mid-stick" by ALT_HOLD's own
dead-zone logic, meaning the override was asking for little or no rate
change the whole time - the accelerating fall was ALT_HOLD's own
internal altitude-hold loop failing to hold, not something any
override PWM value could have fixed, since the override was never
really driving thrust in that mode to begin with.

Fix: stopped trying to hold in ALT_HOLD at all. AUTOTUNE's own
whitelist (see the first Nov 6 note above) also accepts STABILIZE,
which has no internal altitude-hold loop whatsoever - throttle there
really is the FC's only lift input (modulated by attitude stabilization,
not by a baro-based rate controller), which is the model the hover-
throttle search actually assumes. So the flight sequence now switches
GUIDED -> STABILIZE directly (ALT_HOLD is no longer used at all) and
runs the same measurement-based search there instead, where "more PWM
-> more lift" is actually true. This also makes _land_and_save()'s
existing descend-by-easing-throttle-down logic consistent with the mode
AUTOTUNE was actually entered from, which it wasn't before (that
function always assumed throttle=thrust; entering AUTOTUNE from
ALT_HOLD meant it wasn't, entering from STABILIZE means it now is).
Also added a hard altitude floor inside the search loop itself
(HOVER_SEARCH_ABORT_ALTITUDE_M), checked every override tick rather
than only at the end of each sample window - the search no longer has
to wait out a whole 1.5s window, let alone the full 12s budget, before
giving up if the vehicle is actually falling fast. Not yet verified:
whether STABILIZE's throttle-to-thrust response is linear enough for
the existing proportional gain to converge cleanly, or whether that
gain (now HOVER_SEARCH_GAIN_PWM_PER_MS) needs retuning for this
different mode's response curve - watch the next run for oscillation
vs. clean convergence, same as before.

Nov 6, 2026, fourth real run - switching to STABILIZE itself was the
next problem. The search started its first sample at the same seed PWM
the ALT_HOLD version used, 1500 (50% stick) - and the altitude-floor
abort fired 1.235s later, having already lost about 4.3m (settled
altitude down to 1.29m), before even one full sample window completed
or a single vz measurement was logged. The abort worked exactly as
designed (it caught this mid-window instead of waiting the full 1.5s,
and far short of the 3rd run's trip to the ground) - but by the time it
fired, the vehicle was almost certainly already falling faster than
LAND mode could arrest in the 1.29m of altitude that was left (rough
stopping-distance math - v^2/(2a) - says a vehicle falling anywhere
near as fast as the data implies needs more like 2m to stop under
plausible deceleration, not 1.29m), which is almost certainly what
Zer0 saw as a hard, fast touchdown rather than a clean abort-to-LAND.

Root cause: 1500 PWM (50% stick) in STABILIZE is drastically below
Shadow's real hover throttle - not just "a bit off" the way the
ALT_HOLD guess was, but low enough to produce something close to a
free fall from the very first instant STABILIZE took over from
GUIDED's position hold. The "Adequate Power" guidance that a
well-matched copter hovers around 50% throttle (quoted in Session 5's
notes, from the same ArduPilot doc source used for the 3rd-run fix) is
a design target for a well-matched vehicle, not a fact about Shadow's
actual configuration - and this run is direct evidence Shadow's real
hover point is well above that.

Three changes, aimed at both not guessing low again AND reacting much
faster if the next guess is still wrong:
  - New seed, STABILIZE_SEARCH_SEED_PWM = 1650, replacing RC_THROTTLE_HOLD
    as the search's starting point (RC_THROTTLE_HOLD is now only used
    while still in GUIDED, before any mode switch, where it has no
    effect on altitude at all - GUIDED holds position via its own
    controller regardless of the RC override value). 1650 is still a
    guess, but a deliberately safer one: erring high in STABILIZE means
    climbing further into open sky (shadow_runway.sdf has headroom, and
    the new ceiling check below catches an excessive climb anyway);
    erring low is what just happened. The STABILIZE switch itself now
    also streams this seed during its confirm window, not
    RC_THROTTLE_HOLD, so there's no gap where the proven-bad 1500 is
    live in STABILIZE even briefly.
  - HOVER_SEARCH_SAMPLE_SECONDS cut from 1.5s to 0.5s, so a wrong guess
    gets corrected three times faster instead of being held constant
    for a second and a half before the loop reacts at all. Trade-off,
    not yet verified: a shorter window means a noisier vz measurement
    from position data alone - watch whether this makes the search
    jumpier rather than just faster to correct.
  - HOVER_SEARCH_ABORT_ALTITUDE_M raised from 2.0m to 3.0m - the 4th
    run's own numbers suggest 2.0m didn't leave enough stopping
    distance once a real fall was already underway. New
    HOVER_SEARCH_ABORT_CEILING_M (8.0m, TAKEOFF_ALTITUDE_M + 2.0) added
    as the symmetric check against overshooting upward, now that the
    search deliberately starts on the high side - checked every tick
    the same way the floor is.
None of this is a substitute for watching the next run closely. If
1650 is itself still badly wrong in either direction, the faster
reaction and wider margins should mean a smaller, slower correction
instead of another hard abort - but that is exactly the part this
session cannot verify without a real flight.

Nov 6, 2026, fifth real run - an unrelated transient first (armed
rejected once with "Arm: Accels inconsistent", ArduCopter's own IMU-
consistency pre-arm check; nothing in this script's flight sequence
touches arming logic, and a plain retry armed cleanly, so treated as
a one-off SITL glitch, not a bug here). The retry reached the search
and logged:
    throttle trim: pwm=1650 -> vz=-1.10m/s (alt=5.49m)
    throttle trim: pwm=1782 -> vz=-4.09m/s (alt=3.44m)
    [ERROR] altitude 2.69m crossed the 3.0m floor mid-search - aborting
Raising the override PWM made the sink nearly 4x WORSE, not better -
the same "more PWM -> less sink" assumption fix #3/#4 both depend on
just broke down a second time (the first time being ALT_HOLD's
deadzone, already found and fixed above).

Read Shadow's actual Gazebo config directly on Zer0's machine
(ardu_ws/src/ardupilot_gazebo/models/shadow/model.sdf and
shadow_with_ardupilot/model.sdf) rather than guess at another
constant. The real control chain for channel 3 - commanded rotor
velocity (linear in PWM, multiplier=838 over servo_min=1100/
servo_max=1900), the velocity controller (P-only, tiny torque clamp
but also tiny rotor inertia, so it tracks a new command in well under
10ms), and LiftDrag thrust (scales with blade speed squared) - is
monotonic by construction. It cannot produce a worse sink from more
PWM on its own, so the chain itself isn't the explanation.

Leading hypothesis, NOT yet confirmed: the attitude mixer ran out of
headroom. The RC override here only sets the common throttle; STABILIZE's
attitude controller still adds a per-motor correction on top to hold
the vehicle level, and that correction needs room under the 1900 PWM
ceiling. At 1650 PWM there was 250 counts of headroom; at 1782 there
was only 118. If a motor needed more correction than that, it clips at
1900 while its diagonal partner keeps moving normally - an asymmetric
thrust split that tips the vehicle, and tipping the thrust vector away
from vertical reduces the climb component of total thrust even as
total thrust rose, which reads exactly as "more throttle, faster
sink." A full writeup of this chain and the hypothesis, with the real
numbers, is at https://claude.ai/artifact/WQXDJGNMBN83tvd5SbZy13

This script had no way to check that hypothesis - no attitude or
per-rotor telemetry anywhere in the search. Fixed below: every sample
in _find_hover_throttle_in_stabilize() now also logs roll/pitch
(decoded from MAVROS's own pose quaternion, same source
current_yaw()/current_orientation_quat() already use in
ArduPilotBody - no change needed there) and the real per-channel PWM
ArduPilot is actually sending to the four rotors (/mavros/rc/out,
MAVROS's rc_io plugin - the mixed, post-attitude-correction output,
not the common override this script sends in). If a motor is pinned
at or near 1900 while its diagonal partner isn't, or roll/pitch spikes
right as the sink worsens, that confirms the hypothesis directly on
the next run. If attitude stays flat and nothing saturates, this
hypothesis is wrong and the next place to look is the LiftDrag
angle-of-attack term itself - deliberately not opened yet, since this
telemetry is the cheaper check to run first.

Nov 7, 2026, ninth real run - first run with the RC-override sysid bug
actually fixed (MAV_GCS_SYSID, see shadow_sitl.parm) AND a converged
hover-throttle search (1796 PWM). The hold then switched to AUTOTUNE
cleanly, logged "AutoTune: pilot overrides active" repeatedly for the
whole hold, and altitude climbed almost perfectly linearly the entire
time - 14.84m at t=10s to 156.33m at t=110s, roughly +1.4 m/s sustained,
never leveling off. _land_and_save()'s 30s touchdown-confirmation
timeout then fired with the vehicle still 100+ meters up ("Did not
confirm touchdown within 30s - disarming anyway"), and disarming at
that altitude is what Zer0 watched fall/crash via the FPV camera.

Root cause: this hold streamed hover_pwm (1796, the STABILIZE-measured
absolute thrust value) as the AUTOTUNE throttle override. AUTOTUNE is
NOT direct-thrust like STABILIZE - ArduCopter/mode_autotune.cpp's
AutoTune::get_desired_climb_rate_ms() calls the exact same
Attitude.cpp Copter::get_pilot_desired_climb_rate_ms() that ALT_HOLD
uses (the same climb-rate-not-thrust mechanism already found and fixed
once before, in the 3rd-run note above - just hit again in a different
mode this script didn't know shared it). With THR_DZ at its default
(100 control-in units, ArduCopter/config.h's THR_DZ_DEFAULT) and
get_throttle_mid() at the default 500 control-in (radio.cpp's
get_control_mid(), from RC3_TRIM 1500 centered between RC3_MIN/MAX
1100/1900), the no-climb deadband is control-in 400-600, roughly PWM
1420-1580. 1796 PWM sits far above that top edge, so AUTOTUNE read it
as close to a full-stick-up climb-rate command (scaled by
PILOT_SPD_UP/get_pilot_speed_up_adjusted_ms()) for the entire 120s
AUTOTUNE_HOLD_SECONDS - which lines up with the observed near-constant
+1.4 m/s for the whole hold.

Fix: the hold now streams AUTOTUNE_HOLD_PWM (= RC_CENTER, 1500 PWM -
dead center of the deadband above) instead of hover_pwm, commanding
~0 climb rate and letting AUTOTUNE's own internal altitude controller
hold position. _land_and_save() had the identical bug one level down -
its old descend_pwm = hover_pwm - DESCEND_THROTTLE_OFFSET (1796-200=
1596) is STILL above the deadband's 1580 top edge, so the "descent"
would have kept climbing, just slightly slower, never actually
bringing the vehicle down. That function now streams a new fixed
AUTOTUNE_DESCEND_PWM (1350) instead, picked to sit clearly below the
deadband's 1420 bottom edge without being anywhere near full-stick-
down. Neither new constant has been flight-verified yet - watch the
next run's actual climb rate during the hold (should read ~flat, not
+1.4 m/s) and the actual descent rate during landing closely.

Still NOT addressed by this fix, flagged for later: AC_AutoTune.cpp's
own pilot-override detection ("pilot overrides active" -> load_gains
(GainType::ORIGINAL), found by searching that exact statustext string)
means AUTOTUNE pauses its real tuning test maneuvers and flies on
unmodified gains for as long as it sees a continuous RC override - which
this script streams for the entire hold, by design, in every run so
far including this one. Fixing the climb-rate bug makes the hold safe
to run, but on its own it does NOT yet explain how a real tune is
supposed to ever get captured while the override that keeps the vehicle
safe is also the exact signal that tells AUTOTUNE "a pilot is flying,
don't test." Not yet investigated or discussed with Zer0 - the next
honest question once a hold completes safely is whether the FC log
shows any actual tuning maneuvers happening underneath that message, or
whether it's quietly doing nothing the entire 120s.

Nov 7, 2026, tenth real run - the "pilot overrides active" question
above resolved itself favorably: with AUTOTUNE_HOLD_PWM centered, the
hold produced real tuning progress (GCS "AutoTune: Roll Rate D Up/Down"
messages reaching 75%), so a perfectly-centered override does NOT block
AUTOTUNE's own test maneuvers - only a non-centered one would, per
get_pilot_desired_climb_rate_ms()'s deadband. Landed safely, but
`ros2 param get /mavros ATC_RAT_RLL_D` immediately after confirmed the
tune did NOT actually save (read back exactly at the ArduCopter stock
default, 0.0036) - and that first attempt also failed outright with
"Node not found", which led to the real mavros-architecture finding
below.

Root cause of the "Node not found" / long-standing /mavros/param/get
flakiness (checked directly against mavros's own source,
mavlink/mavros's src/plugins/param.cpp on GitHub): the custom
mavros_msgs::srv::ParamGet service this script and check_tune_gains.py
both assumed existed is commented out in this mavros build entirely -
`// NOTE(vooon): override standard ROS2 parameter services`. This
mavros version runs one ROS2 node PER PLUGIN inside the /mavros
namespace (confirmed via `ros2 node list`: /mavros/param, /mavros/cmd,
/mavros/mavros, etc - never a bare /mavros node), each exposing the
standard ROS2 parameter services instead. Pulled FCU params live as
ROS2 parameters on the "param" plugin's own node specifically, reachable
via `ros2 param get /mavros/param <name>` or, programmatically, the
rcl_interfaces::srv::GetParameters service at
/mavros/param/get_parameters - confirmed working:
`ros2 param get /mavros/param ATC_RAT_RLL_D` -> 0.003599999938160181
(stock default, confirming the non-save above). check_tune_gains.py
(scripts/check_tune_gains.py) now uses the corrected service name.

Root cause of why the tune never saved, checked directly against
ArduPilot's libraries/AC_AutoTune/AC_AutoTune.cpp and
AC_AutoTune_Multi.cpp on GitHub: disarmed() does attempt
save_tuning_gains() on every disarm-while-in-AUTOTUNE (true for this
script's flow), but AC_AutoTune_Multi::save_tuning_gains() opens with
`if (axes_completed == 0) { return; }` - axes_completed is a per-axis
bit only set when that axis's tune_type state machine reaches
TUNE_COMPLETE. Tonight's run got Roll through RATE_D_UP and
RATE_D_DOWN (the 75% progress seen) but then stalled at a flat
"RATE_P_UP 0%" for the rest of the old fixed 120s AUTOTUNE_HOLD_SECONDS
- GCS_SEND_TEXT's own `%u%%` is `success_counter*(100/AUTOTUNE_SUCCESS_
COUNT)`, and success_counter only ever decrements on a failed test
(never resets), so a flat repeated 0% means every attempt failed, not
slow progress. Roll never reached TUNE_COMPLETE, axes_completed stayed
0, nothing was eligible to save - exactly matching the stock-default
readback above. Fix: replaced the fixed 120s hold with event-driven
_hold_until_autotune_done(), which watches the FC's own "AutoTune:
Success"/"AutoTune: Failed" statustext (via _statustext_cb(), see
AC_AutoTune.cpp's update_gcs() for the exact strings) and only gives up
on a generous AUTOTUNE_MAX_HOLD_SECONDS (600s) safety ceiling instead
of guessing how long tuning needs. Not yet flight-verified.

Nov 7, 2026, eleventh real run - first run with the event-driven hold
above in place, but it never reached AUTOTUNE at all: the STABILIZE
hover-throttle search (_find_hover_throttle_in_stabilize()) itself
oscillated - pwm=1796->vz=+0.67m/s, 1715->+0.95 (still climbing,
FASTER, right after a cut), ...1309->-0.84 (now overshot into sinking),
1410->-1.30 (still sinking, faster, right after an increase)... a
sustained, non-damping swing between ~1300-1800 PWM that exhausted the
whole 12s HOVER_SEARCH_MAX_SECONDS budget without ever converging, on
the exact same seed/gain that had converged cleanly just one run
earlier (the ninth run, 1796 PWM). See HOVER_SEARCH_GAIN_PWM_PER_MS's
comment for the full diagnosis (motor-spool/vehicle-inertia lag inside
the 0.5s sample window making the OLD gain-based update's correction
size unreliable) and the fix (a direction-reversal step-halving/
bisection search that only trusts vz's sign, never its magnitude).
Not yet flight-verified.

Nov 7, 2026, twelfth real run - first run with the bisection search
above, and the first run this entire project that actually CRASHED
(not a safe, intentional abort - a real ground strike, confirmed by
Zer0 watching it happen and reproduced on an immediate retry). From
1796 PWM the search stepped down cleanly (1796->+0.64, 1646->+0.96,
1496->+0.98, 1346->+0.18 - nearly converged, well inside sight of
HOVER_SEARCH_CONVERGED_VZ_MS) but then applied one more FULL,
un-halved 150 PWM step (no reversal had happened yet, so the step
hadn't shrunk) straight to the 1200 MIN_PWM clamp. That one step was
enough to push the vehicle from near-equilibrium into a real,
accelerating sink - 1200->-0.96, 1275->-3.09, 1350->-3.99, still
getting WORSE despite PWM being raised back up each step - and the
existing floor check (HOVER_SEARCH_ABORT_ALTITUDE_M, position-only,
checked once it's already crossed) didn't fire until 2.98m, by which
point vz was already around -4m/s and still worsening. The search
logged a clean, intentional-looking abort at that point - but the
actual ground strike happened AFTER that log line, in the abort
hand-off itself: the old pattern released the RC override entirely
(_zero_override() hands throttle back to whatever RC_IN the FC sees
with no transmitter attached - it does not mean "zero thrust", it
means "no override at all") and called set_mode('LAND') in the same
breath, with no guarantee LAND's throttle control takes over before
gravity does. With under 3m and an accelerating -4m/s fall, that gap
was the crash.

Two root causes here, not one: (1) the vertical-motion model this
whole search has always assumed - "hold a PWM, measure the vz it
settles to" - doesn't actually hold. This is a double integrator
(thrust error -> acceleration -> velocity), so holding ANY off-hover
PWM keeps accelerating vz for as long as it's held; it never settles
to a steady value for that PWM. A step applied from a near-converged
point can still overshoot badly if the step size doesn't also shrink
as the reading approaches convergence, not just on reversal. (2) every
abort path in this script that released the override and switched to
LAND in one breath was relying on an unverified assumption about how
fast that handoff actually is - fine for a dozen earlier aborts that
all happened from a roughly-level state, not survivable from a fast,
already-accelerating dive with little altitude left.

Fixes: HOVER_SEARCH_STEP_PWM cut 150->80 (shrinks the worst-case single
overshoot, doesn't fix the underlying model mismatch). A new fast
rolling-window vz check (HOVER_SEARCH_FAST_VZ_WINDOW_SEC,
HOVER_SEARCH_EMERGENCY_VZ_MS) runs every override tick inside the
search loop now, not just once per HOVER_SEARCH_SAMPLE_SECONDS window,
so a dangerous trend triggers an abort well before the slower
position-only floor/ceiling check would. And a new
_arrest_fall_and_land() helper replaces the bare "zero override + LAND"
pattern everywhere in this file that could plausibly fire mid-dive: it
holds a known-safe, above-hover PWM (STABILIZE_SEARCH_SEED_PWM) for up
to HOVER_SEARCH_ARREST_SECONDS, watching for the vertical rate to turn
non-negative, before ever releasing the override. None of this has
been flight-verified yet - the honest status is "less likely to crash
the same way again," not "proven safe." Watch the next run closely,
including the arrest-phase log lines if it ever triggers.
"""

import math
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.msg import ExtendedState, OverrideRCIn, RCIn, RCOut, StatusText
from mavros_msgs.srv import CommandLong, ParamGet

from shadow.body import ArduPilotBody


TAKEOFF_ALTITUDE_M = 6.0        # open-world safe altitude, well clear of ground effects
CLIMB_TIMEOUT_SEC = 20.0        # how long to wait for the takeoff to actually reach altitude
GUIDED_SETTLE_SECONDS = 5.0     # hold in GUIDED before handing off to RC override, so the
                                 # transition isn't also fighting a still-accelerating climb

RC_CENTER = 1500                # roll/pitch/yaw override - centered stick, no deflection
RC_THROTTLE_HOLD = 1500         # Nov 6, 4th run: ONLY used now while still in GUIDED, before
                                 # any mode switch - GUIDED holds position via its own
                                 # controller regardless of this value, so it's a safe no-op
                                 # placeholder there. Proven dangerously low if actually used as
                                 # a thrust value in STABILIZE (see docstring) - the search below
                                 # no longer starts from this, see STABILIZE_SEARCH_SEED_PWM.
STABILIZE_SEARCH_SEED_PWM = 1796  # Nov 7, 2026 (10th real run) - REVERTING the 1300 value from
                                   # earlier tonight, which was wrong and caused a real abort (see
                                   # below). That 1300 came from linearly extrapolating two data
                                   # points (1650 -> +0.89m/s, 1543 -> +0.62m/s) to vz=0 - but those
                                   # two points were measured right after the GUIDED->STABILIZE
                                   # switch, almost certainly still carrying upward momentum from
                                   # GUIDED's climb, not steady-state STABILIZE response. Trusting a
                                   # 2-point extrapolation from likely-transient data over an actual
                                   # CONVERGED measurement from the same session's 9th run (the
                                   # adaptive search below ran to completion and settled on 1796,
                                   # confirmed in autotune.log) was the mistake - 1796 is real,
                                   # measured, steady-state data; 1300 was derived, noisy, and
                                   # wrong. The 10th run proved it: seeded at 1300 (496 PWM below
                                   # the real hover point), the vehicle sank from 6.04m to 2.95m in
                                   # about 1.5s - each search step raised PWM (1300->1477->1769) but
                                   # ALL of those were still below the real 1796 hover point, so
                                   # thrust stayed below weight the entire time and downward
                                   # velocity just kept accumulating; the search correctly aborted
                                   # at the 3.0m floor and the script safely landed (no crash this
                                   # time - confirmed via /shadow/telemetry: armed=false, alt=0.04m,
                                   # zero velocity). This also DISPROVES the 5th-run "attitude mixer
                                   # headroom" hypothesis still sitting unresolved further up this
                                   # file: this run's rotor telemetry showed all four motors
                                   # perfectly equal at every step (no asymmetry, no tilt) - the
                                   # same "more PWM -> worse sink" pattern that hypothesis was meant
                                   # to explain has a much simpler explanation (seeded way below
                                   # hover, velocity keeps accumulating) that needs no mixer-
                                   # clipping theory at all. Starting from the confirmed real value
                                   # this time, not another guess.
AUTOTUNE_HOLD_PWM = RC_CENTER   # Nov 7, 2026 (9th real run) - see docstring. AUTOTUNE's
                                 # throttle channel is a CLIMB-RATE command through a
                                 # deadband centered on mid-stick, exactly like ALT_HOLD
                                 # (ArduCopter/mode_autotune.cpp's get_desired_climb_rate_ms()
                                 # calls the same Attitude.cpp get_pilot_desired_climb_rate_ms()
                                 # ALT_HOLD does) - NOT direct thrust like STABILIZE. Streaming
                                 # the STABILIZE-measured hover_pwm (1796 on the 9th run) during
                                 # the AUTOTUNE hold sat far above that deadband and was read as
                                 # "near-max commanded climb rate" for the whole 120s hold -
                                 # confirmed as the cause of the runaway climb to 156m and the
                                 # subsequent crash (see docstring). RC_CENTER (1500 PWM) is dead
                                 # center of the deadband here: with THR_DZ at its default of 100
                                 # (ArduCopter/config.h, THR_DZ_DEFAULT) and get_throttle_mid()
                                 # at 500 control-in (radio.cpp's get_control_mid(), from
                                 # RC3_TRIM 1500 centered between RC3_MIN 1100/RC3_MAX 1900), the
                                 # deadband spans control-in 400-600, i.e. roughly PWM 1420-1580 -
                                 # streaming 1500 commands ~0 climb rate and lets AUTOTUNE's own
                                 # internal altitude controller hold position instead.
AUTOTUNE_DESCEND_PWM = 1350      # Nov 7, 2026 - replaces the old DESCEND_THROTTLE_OFFSET-from-
                                 # hover_pwm formula, which had the exact same direct-thrust-vs-
                                 # climb-rate bug: hover_pwm(1796) - 200 = 1596 is STILL above the
                                 # ~1580 deadband top edge above, so the old code would have kept
                                 # commanding a (slightly slower) CLIMB during "descent," not a
                                 # descent at all. 1350 PWM is control-in ~313 - meaningfully below
                                 # the ~1420 deadband bottom edge (so it does command a real
                                 # negative climb rate, scaled by PILOT_SPD_DN/get_pilot_speed_dn_
                                 # adjusted_ms() per Attitude.cpp), but nowhere near full-stick-
                                 # down (1100) - a deliberately mild first guess, same spirit as
                                 # STABILIZE_SEARCH_SEED_PWM's "err safe, not aggressive" choice
                                 # above. NOT yet verified by a real run - watch the next attempt's
                                 # actual descent rate closely.
PRE_AUTOTUNE_OVERRIDE_SECONDS = 3.0   # stream the centered override before switching modes,
                                       # so there's no gap where AUTOTUNE sees no RC input at all
OVERRIDE_RATE_HZ = 20

HOVER_SEARCH_SAMPLE_SECONDS = 0.5     # Nov 6, 2026 (4th real run) - was 1.5s; cut to react to a
                                      # bad guess three times faster rather than holding one
                                      # constant value for a second and a half first (see
                                      # docstring). Trade-off not yet verified: a shorter window
                                      # means noisier vz from position data alone - watch for
                                      # jumpiness, not just speed, on the next run.
HOVER_SEARCH_MAX_SECONDS = 12.0      # total search budget before giving up and landing rather
                                      # than committing to AUTOTUNE on an unconverged guess
DIAGNOSTIC_HOLD_SECONDS = 0.5         # Nov 7, 2026 - cut from 2.0s now that this hold's one job
                                      # (prove the override doesn't silently decay over time,
                                      # independent of command) is DONE: the Nov 7 MAV_GCS_SYSID
                                      # fix produced a clean, non-decaying climb for the full
                                      # original 2.0s, with rc_in_thr tracking the command the
                                      # whole time. Running the full 2.0s again on every future
                                      # run would just re-spend altitude margin for information we
                                      # already have. Kept non-zero (not removed outright) so a
                                      # future regression in the override path would still show up
                                      # here before the search even starts, just cheaply now.
DIAGNOSTIC_HOLD_LOG_HZ = 20.0         # Nov 6, 2026 (8th real run) - raised from 4.0 to the full
                                      # OVERRIDE_RATE_HZ: the QoS fix above made the override
                                      # actually arrive (confirmed - the "incompatible QoS"
                                      # warning is gone and t=0 showed a real, plausible
                                      # response to the command for the first time all night),
                                      # but a SECOND decay still pulls PWM down starting around
                                      # t=0.3s even though the command is unchanged. At 4Hz
                                      # logging that decay is only visible as a handful of
                                      # points - not enough to tell a sudden cliff (a timeout/
                                      # expiry firing) from a gradual slope (a filter or ramp).
                                      # Every-tick logging for this one 2s diagnostic (40 lines)
                                      # is an acceptable one-off cost in autotune.log to find out
                                      # which it is.
HOVER_SEARCH_GAIN_PWM_PER_MS = 120.0  # Nov 7, 2026 (11th real run) - NO LONGER USED, kept only
                                       # for this comment's paper trail. This fixed gain is what
                                       # caused the 11th run's failure: pwm=1796->vz=+0.67,
                                       # 1715->+0.95 (still climbing, FASTER, after a PWM cut),
                                       # 1601->+0.86, 1498->+1.11, 1364->+0.46, 1309->-0.84 (now
                                       # overshot into sinking), 1410->-1.30 (still sinking,
                                       # faster, after a PWM *increase*)... a sustained
                                       # non-damping oscillation between ~1300-1800 PWM that ate
                                       # the whole 12s budget without ever converging, right after
                                       # this exact seed/gain had converged cleanly on the
                                       # immediately preceding (9th) run. The repeated pattern of
                                       # "vz keeps moving the WRONG way for a full sample window
                                       # right after the command changed" is the signature of
                                       # motor-spool/vehicle-inertia lag inside the 0.5s sample
                                       # window: each window's measured vz is still responding
                                       # mostly to the PREVIOUS window's PWM, not the current one,
                                       # so a magnitude-proportional update (this gain times the
                                       # stale vz) overcorrects by an amount that has no reliable
                                       # relationship to how far off the real hover point actually
                                       # is - fine when the vehicle happens to settle fast enough
                                       # relative to the window, fatal (self-reinforcing swings)
                                       # when it doesn't, and nothing here can guarantee which
                                       # case a given run lands in. Replaced below by a
                                       # direction-reversal step-halving (bisection) search, which
                                       # only trusts the SIGN of vz each window (which way to move
                                       # next), never its magnitude - so stale/lagged vz can still
                                       # point the right way even while its size is unreliable,
                                       # and a wrong guess just costs one extra reversal-and-halve
                                       # instead of a runaway swing.
HOVER_SEARCH_STEP_PWM = 80.0          # Nov 7, 2026 (12th real run) - CUT from 150 after the
                                       # bisection search's first real flight produced the first
                                       # actual CRASH this whole project (not just a safe abort -
                                       # see HOVER_SEARCH_EMERGENCY_VZ_MS and
                                       # _arrest_fall_and_land() below for the full story). The
                                       # step-halves-only-on-reversal design still applies a FULL
                                       # step even from a point that's already near-converged: at
                                       # pwm=1346, vz was +0.18m/s - barely above the 0.15
                                       # HOVER_SEARCH_CONVERGED_VZ_MS threshold - yet because no
                                       # reversal had happened yet, the step was still the full
                                       # 150, cutting all the way to 1200 (the MIN_PWM clamp) in
                                       # one move. That one step is what put the vehicle into a
                                       # real, accelerating sink with only ~3m of floor margin
                                       # left. 80 halves the size of that worst-case overshoot
                                       # without meaningfully hurting convergence speed (1796 to
                                       # ~1340 is still only ~6 steps before any reversal is even
                                       # needed, comfortably inside the 12s budget) - this does
                                       # NOT fix the underlying cause (a double-integrator system:
                                       # holding ANY off-hover PWM keeps accelerating vz for as
                                       # long as it's held, it never settles to a steady value the
                                       # way the search's model implicitly assumes), it only
                                       # shrinks how much damage one wrong-sized step can do. The
                                       # real backstop for that is the emergency brake below.
HOVER_SEARCH_MIN_STEP_PWM = 5.0       # floor on the step size - once halving would go below this,
                                       # hold it here instead of continuing to shrink toward zero,
                                       # so the search can't stall out taking smaller and smaller
                                       # steps forever against vz noise instead of either
                                       # converging or genuinely running out of budget.
HOVER_SEARCH_FAST_VZ_WINDOW_SEC = 0.15  # Nov 7, 2026 (12th real run) - rolling window used by the
                                       # new in-loop "fast vz" emergency estimate below, checked
                                       # every override tick (20Hz) instead of only once per full
                                       # HOVER_SEARCH_SAMPLE_SECONDS (0.5s) window. The 12th run's
                                       # crash happened because the ONLY altitude safety check that
                                       # runs faster than a full window (the HOVER_SEARCH_ABORT_
                                       # ALTITUDE_M/CEILING_M floor/ceiling checks) is reactive -
                                       # position-only, no sense of HOW FAST things are getting
                                       # worse - so it didn't fire until 2.98m, by which point vz
                                       # was already -4m/s and still accelerating, leaving no real
                                       # margin for whatever happens next. This shorter rolling
                                       # window gives a much faster (though noisier) read on
                                       # vertical speed, so a dangerous trend can be caught and
                                       # acted on well before the slow per-window average or the
                                       # absolute-altitude floor would ever catch it.
HOVER_SEARCH_EMERGENCY_VZ_MS = 2.0   # Nov 7, 2026 (12th real run) - if the fast rolling vz above
                                       # exceeds this in EITHER direction mid-window, abort the
                                       # search immediately (same as a floor/ceiling crossing)
                                       # rather than finishing out the window on a PWM that's
                                       # already producing a dangerous rate. Chosen well above the
                                       # +-1.1m/s peaks seen during normal, safe search excursions
                                       # on earlier runs (so this won't fire on ordinary hunting),
                                       # but well below the -3 to -4m/s the 12th run's crash
                                       # actually reached - the goal is to catch the trend early,
                                       # not to wait for confirmation it's already severe.
HOVER_SEARCH_ARREST_SECONDS = 2.0    # Nov 7, 2026 (12th real run) - see _arrest_fall_and_land():
                                       # how long to hold a known-safe recovery PWM, continuously
                                       # checking for the vertical rate to turn non-negative,
                                       # before giving up on waiting and handing off to LAND
                                       # anyway. Picked to be comfortably longer than the ~1-1.5s
                                       # the 9th/11th runs' diagnostic holds took to show a clear,
                                       # real response to a constant PWM, not derived from a
                                       # precise settling-time measurement.
HOVER_SEARCH_CONVERGED_VZ_MS = 0.15  # "stable enough" vertical speed to call the search done
HOVER_SEARCH_MIN_PWM = 1200          # safety clamp on the search either direction - never let
HOVER_SEARCH_MAX_PWM = 1800          # it wander to something that would be a real red flag
HOVER_SEARCH_ABORT_ALTITUDE_M = 3.0  # Nov 6, 2026 (3rd real run) - the ALT_HOLD-era search
                                      # fell from ~5.7m to the ground in under 5s without this
                                      # check ever firing, because it only looked at altitude
                                      # between sample windows, not during one. Checked on every
                                      # override tick now - abort the search the instant this is
                                      # crossed, not at the end of the current sample window.
                                      # Raised from 2.0 to 3.0 after the 4th real run (see
                                      # docstring): 2.0m didn't leave enough stopping distance
                                      # once the vehicle was already falling fast by the time
                                      # the check fired - this needs real margin, not just a
                                      # non-zero one.
HOVER_SEARCH_ABORT_CEILING_M = TAKEOFF_ALTITUDE_M + 3.0  # Nov 6, 2026 (4th real run) - added
                                      # alongside the seed PWM being raised (STABILIZE_SEARCH_
                                      # SEED_PWM) to bias the search toward overshooting up
                                      # rather than falling: this is the symmetric check against
                                      # that overshoot actually running away, checked every tick
                                      # exactly like the floor above.
                                      # Nov 7, 2026 - widened from +2.0 to +3.0 (8m -> 9m) for
                                      # extra margin on this first real search attempt now that
                                      # the override actually works: the Nov 7 run climbed from
                                      # 6.04m to 7.15m during the diagnostic hold ALONE (now cut
                                      # to 0.5s above) before the search even started, leaving
                                      # too little room for the search to converge inside the old
                                      # 8.0m ceiling. A lower seed PWM and shorter hold both help
                                      # too, but this is cheap, safe extra headroom for a case
                                      # we've never actually exercised before tonight.

AUTOTUNE_HOLD_SECONDS = 120.0   # Nov 7, 2026 - DEPRECATED, see AUTOTUNE_MAX_HOLD_SECONDS
                                 # below. No longer used as the hold's actual duration - kept
                                 # only because older log entries above reference it by name.
                                 # Confirmed by the 11th real run this was never going to be
                                 # enough: read ArduPilot's own AC_AutoTune.cpp/AC_AutoTune_
                                 # Multi.cpp source (GitHub) and found save_tuning_gains()
                                 # starts with `if (axes_completed == 0) { return (nothing to
                                 # save) }` - axes_completed is only set per-axis when that
                                 # axis's tune_type reaches TUNE_COMPLETE (ArduCopter's own
                                 # multi-stage sequence per axis: RATE_D_UP -> RATE_D_DOWN ->
                                 # RATE_P_UP -> ANGLE_P_DOWN -> ANGLE_P_UP -> RATE_FF_UP ->
                                 # MAX_GAINS -> TUNE_CHECK -> TUNE_COMPLETE). The 9th run's
                                 # fixed 120s hold got Roll through RATE_D_UP and RATE_D_DOWN
                                 # (confirmed via a real post-flight ATC_RAT_RLL_D param read -
                                 # see check_tune_gains.py - it came back at the stock 0.0036
                                 # default, proving nothing saved) then stalled in RATE_P_UP at
                                 # a flat 0% (GCS text's own "%u%%" is literally
                                 # success_counter*100/AUTOTUNE_SUCCESS_COUNT, so "0%" repeated
                                 # ~18 times in a row means every single test iteration failed,
                                 # not just slow progress) for the rest of the 120s - Roll never
                                 # reached TUNE_COMPLETE, so axes_completed stayed 0 and nothing
                                 # was ever eligible to save, exactly matching the read-back
                                 # default values. A fixed duration can only ever be a guess at
                                 # how long this takes - replaced with an event-driven hold
                                 # (AUTOTUNE_MAX_HOLD_SECONDS) that actually watches for
                                 # ArduCopter's own "AutoTune: Success"/"AutoTune: Failed"
                                 # statustext instead of guessing.
AUTOTUNE_MAX_HOLD_SECONDS = 600.0   # Nov 7, 2026 - safety ceiling, not a target. The hold now
                                 # ends as soon as the FC itself reports AutoTune: Success or
                                 # AutoTune: Failed (see _hold_until_autotune_done() and
                                 # _statustext_cb() below) - this is only the hard stop if
                                 # neither ever arrives, so a genuinely stuck test (like the
                                 # RATE_P_UP stall above) still lands safely instead of hanging
                                 # forever. 600s (10 min) chosen as "clearly more than even a
                                 # badly-behaved tune should need," not derived from any forum
                                 # number - if this run actually hits this ceiling, that itself
                                 # is a real finding (a stuck test that time alone won't fix),
                                 # not just "need more time again."
DESCEND_MAX_SECONDS = 30.0      # safety bound on the final descent-while-in-AUTOTUNE step
GROUND_ALTITUDE_TOLERANCE_M = 0.3

STATUS_LOG_INTERVAL = 10.0      # same real-elapsed-time status-logging pattern used
                                 # everywhere else in this codebase


def _drain_spin(node, budget_sec, drains=10):
    """Same fix run_skill.py already needed and documented (Oct 25,
    2026): a single rclpy.spin_once() call services at most ONE ready
    callback, not every pending one. This node now has three
    subscriptions (state, pose, statustext) - state and pose are
    continuously busy, so a single spin_once per loop tick can starve
    statustext out entirely for the whole run, the same way a Skill's
    own later subscription was starved in run_skill.py before that fix.
    Spreads one tick's time budget over several short spins instead of
    one, so every subscription gets an actual chance each tick."""
    for _ in range(drains):
        rclpy.spin_once(node, timeout_sec=budget_sec / drains)


def _zero_override():
    msg = OverrideRCIn()
    msg.channels = [0] * len(msg.channels)
    return msg


def _rpy_throttle_override(throttle_pwm):
    msg = _zero_override()
    msg.channels[0] = RC_CENTER       # roll
    msg.channels[1] = RC_CENTER       # pitch
    msg.channels[2] = int(throttle_pwm)
    msg.channels[3] = RC_CENTER       # yaw
    return msg


class AutotuneRunner:
    """Wraps one ArduPilotBody node with the extra pieces autotune needs
    that nothing else in this codebase does yet (RC override, statustext,
    a best-effort param read) - added as subscriptions/publishers/clients
    on the body node itself, same pattern deliver_payload.py's
    PayloadSensor already uses for Senses: nothing above the Body layer
    should own a second rclpy.Node, so this doesn't either."""

    def __init__(self):
        self.body = ArduPilotBody('autotune_runner')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.body.create_subscription(
            StatusText, '/mavros/statustext/recv', self._statustext_cb, qos)

        # Nov 6, 2026 (6th/7th real run) - THE likely real root cause of
        # every hover-throttle search failure tonight, found only after
        # the diagnostic hold above proved the problem was time-dependent,
        # not command-dependent. This publisher was reusing the
        # BEST_EFFORT `qos` object above (meant for the incoming sensor-
        # style subscriptions), and autotune.log has logged this on every
        # single run, including before any of tonight's other fixes:
        #   "New subscription discovered on topic '/mavros/rc/override',
        #    requesting incompatible QoS. No messages will be sent to it.
        #    Last incompatible policy: RELIABILITY"
        # That is ROS2's own middleware stating plainly that MAVROS's
        # subscriber on this topic requests RELIABLE, this publisher only
        # offered BEST_EFFORT, the two are incompatible, and therefore NO
        # override message this script has ever sent has actually reached
        # MAVROS. Once a mode with no internal altitude loop (STABILIZE)
        # takes over with no real override arriving, ArduPilot falls back
        # to its own default/failsafe RC behavior - which is exactly the
        # ~1s decay toward ~1100 PWM the diagnostic hold just measured,
        # happening completely independent of what this script commands,
        # because the command was never arriving in the first place.
        # Fixed with its own RELIABLE profile, separate from the
        # BEST_EFFORT `qos` object the incoming sensor-style subscriptions
        # correctly keep using - this is a command channel, not a sensor
        # stream, and should never have shared that profile.
        override_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.override_pub = self.body.create_publisher(
            OverrideRCIn, '/mavros/rc/override', override_qos)

        self.param_get_client = self.body.create_client(
            ParamGet, '/mavros/param/get')

        # Nov 7, 2026 - trying a different mechanism to get
        # EXTENDED_SYS_STATE flowing, rather than another MAV1_* stream-
        # rate param guess (MAV1_EXT_STAT already didn't produce it).
        # MAV_CMD_SET_MESSAGE_INTERVAL (MAVLink command 511) asks the FC
        # to send one specific message ID at a specific rate directly,
        # independent of the coarser per-GCS-link stream groups - this is
        # the real MAVROS/ROS2 equivalent of the raw-pymavlink
        # command_long_send() suggestion someone else gave Zer0 tonight
        # (that exact code can't run in this codebase - we're on MAVROS,
        # not pymavlink - so this calls the actual /mavros/cmd/command
        # service with the same MAVLink command instead).
        self.command_client = self.body.create_client(
            CommandLong, '/mavros/cmd/command')

        # Nov 6, 2026 (5th real run) - added to check the mixer-saturation
        # hypothesis in the module docstring: this is ArduPilot's REAL,
        # post-mix per-motor output (MAVROS's rc_io plugin), not the
        # common-mode override this script sends in. If one channel is
        # pinned near servo_max (1900) while its diagonal partner isn't,
        # that's direct evidence of the clipping this docstring suspects.
        self._last_rc_out = None
        self.body.create_subscription(
            RCOut, '/mavros/rc/out', self._rc_out_cb, qos)

        # Nov 6, 2026 (9th real run) - MOT_HOVER_LEARN 0 changed NOTHING:
        # this run's diagnostic-hold numbers (t=0.0s rotors=[1150,1151,
        # 1150,1152] -> t=1.0s rotors=[1100,1100,1100,1100], abort at
        # 2.81m) are the same ramp, to the floor, in the same ~1s, as the
        # run before this param was ever set. That hypothesis is now
        # DISPROVEN, same as mixer-saturation and RC3 calibration before
        # it. New angle: the decay converges smoothly to EXACTLY 1100 -
        # servo_min, the absolute floor the mixer can ever output, not to
        # some lower-but-still-positive value that would just track a
        # mis-scaled stick position. Converging to the hard floor and
        # sitting there, uniformly on all 4 motors, looks less like "the
        # throttle mapping is wrong" and more like "something is forcing
        # a full cut independent of the RC3 value entirely." ArduCopter's
        # land detector is the standing candidate for that: it runs
        # regardless of flight mode, and the vehicle inherited a near-
        # zero climb rate and near-zero acceleration from 5+ seconds of
        # rock-steady GUIDED position hold right before this - exactly
        # the "looks landed" signature the detector watches for - even
        # though it's actually 6m up. Rather than guess another param
        # blind, subscribe to ArduPilot's own answer: EXTENDED_SYS_STATE
        # (MAV1_EXT_STAT is already streaming per the Oct 24 fix above)
        # carries landed_state directly, so we can just read whether the
        # FC itself believes it's on the ground, in the air, or landing
        # during this exact window instead of inferring it secondhand.
        self._last_extended_state = None
        self.body.create_subscription(
            ExtendedState, '/mavros/extended_state',
            self._extended_state_cb, qos)

        # Nov 6, 2026 (10th real run) - EXTENDED_SYS_STATE came back dead:
        # `ros2 topic hz /mavros/extended_state` said "does not appear to
        # be published yet" and `ros2 topic info --verbose` confirmed a
        # publisher is advertised but nothing has ever actually arrived -
        # the same "advertised but ArduCopter never sends the underlying
        # MAVLink message" shape as the /mavros/rc/out gap earlier
        # tonight, except this time there's no primary source to confirm
        # which stream parameter (if any) gates it, so guessing a third
        # SITL param blind isn't a good next move. Testing a different,
        # more answerable question instead: is the RC override this
        # script streams actually ARRIVING at ArduPilot at all during the
        # decay? /mavros/rc/in (RC_CHANNELS, not SERVO_OUTPUT_RAW) is
        # ArduPilot's own report of what it currently sees as RC input,
        # from whatever source - override included - and RC_CHANNELS
        # shares the same stream group as the SERVO_OUTPUT_RAW data
        # MAV1_RC_CHAN already proved works, so this doesn't need a new
        # unconfirmed param guess. If channel 3 here holds steady at 1650
        # the whole time, the override is reaching the FC fine and the
        # decay is happening entirely inside ArduCopter's own throttle
        # handling. If it DOESN'T - drops out, times out, reads something
        # else - that's direct evidence the override itself isn't being
        # treated as live RC input, which would point at an RC-failsafe-
        # style fallback instead of land-detection.
        self._last_rc_in = None
        self.body.create_subscription(
            RCIn, '/mavros/rc/in', self._rc_in_cb, qos)

        # Nov 7, 2026 - set by _statustext_cb() on the FC's own real
        # completion signal, watched by _hold_until_autotune_done() instead
        # of a blind fixed-duration timer. See AUTOTUNE_HOLD_SECONDS's
        # deprecation note above for why the old timer was never enough.
        self._autotune_success = False
        self._autotune_failed = False

    def _request_message_interval(self, message_id, interval_us):
        """Ask the FC to stream one specific MAVLink message at a fixed
        rate via MAV_CMD_SET_MESSAGE_INTERVAL (511), param1=message id,
        param2=interval in microseconds. Soft-fails the same way
        _log_autotune_params_best_effort() does - this is a best-effort
        telemetry request, not a precondition for flight.

        Nov 7, 2026 (same night, later) - this method used to be mis-
        placed INSIDE __init__ (a stray `def` dropped in the middle of
        __init__'s own body), which silently moved every line that used
        to follow it - the RCOut/ExtendedState/RCIn subscriptions below -
        into THIS method's scope instead of __init__'s. ast.parse() still
        passed (it's valid Python either way), so the first sign of
        trouble was the very next run crashing with `NameError: name
        'qos' is not defined` the moment this method actually got called,
        because `qos` only ever existed in __init__'s local scope. My
        mistake, not ArduPilot's or MAVROS's - fixed by moving this
        method to its correct place, after __init__ actually ends."""
        if not self.command_client.wait_for_service(timeout_sec=5.0):
            self.body.get_logger().warn(
                '/mavros/cmd/command not available after 5s - could not '
                f'request message {message_id} explicitly.')
            return
        req = CommandLong.Request()
        req.command = 511  # MAV_CMD_SET_MESSAGE_INTERVAL
        req.param1 = float(message_id)
        req.param2 = float(interval_us)
        future = self.command_client.call_async(req)
        rclpy.spin_until_future_complete(self.body, future, timeout_sec=3.0)
        result = future.result()
        if result and result.success:
            self.body.get_logger().info(
                f'Requested message {message_id} at {interval_us}us interval '
                f'(result={result.result}).')
        else:
            self.body.get_logger().warn(
                f'MAV_CMD_SET_MESSAGE_INTERVAL for message {message_id} '
                f'failed or returned no result: {result}')

    def _rc_in_cb(self, msg):
        self._last_rc_in = msg

    def _rc_in_str(self):
        """What ArduPilot currently reports seeing as raw RC input on
        channel 3 (throttle), regardless of source - tests whether this
        script's override is actually arriving, independent of whatever
        the FC then does with it internally."""
        msg = self._last_rc_in
        if msg is None or len(msg.channels) < 3:
            return 'rc_in_thr=no data yet'
        return f'rc_in_thr={int(msg.channels[2])}'

    def _rc_out_cb(self, msg):
        self._last_rc_out = msg

    def _extended_state_cb(self, msg):
        self._last_extended_state = msg

    def _landed_state_str(self):
        """ArduPilot's own landed_state from EXTENDED_SYS_STATE, decoded
        from the MAV_LANDED_STATE enum (0 UNDEFINED, 1 ON_GROUND, 2
        IN_AIR, 3 TAKEOFF, 4 LANDING). Added this run specifically to
        check the land-detector hypothesis above with real data instead
        of another blind parameter guess."""
        msg = self._last_extended_state
        if msg is None:
            return 'landed_state=no data yet'
        names = {0: 'UNDEFINED', 1: 'ON_GROUND', 2: 'IN_AIR', 3: 'TAKEOFF',
                 4: 'LANDING'}
        return f'landed_state={names.get(msg.landed_state, msg.landed_state)}'

    def _attitude_deg_str(self):
        """Roll/pitch in degrees, decoded from the same pose quaternion
        current_yaw()/current_orientation_quat() already read in
        ArduPilotBody - no new subscription needed for this part.
        Standard ZYX (yaw-pitch-roll) Euler extraction. Returns a
        placeholder string if no pose estimate is available yet."""
        quat = self.body.current_orientation_quat()
        if quat is None:
            return 'roll=?,pitch=?'
        x, y, z, w = quat
        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        # pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        return f'roll={math.degrees(roll):+.1f}deg,pitch={math.degrees(pitch):+.1f}deg'

    def _rotor_pwm_str(self):
        """The real, post-mix PWM ArduPilot just sent to rotor_0..3, from
        the ArduPilotPlugin control-channel mapping confirmed in
        shadow_with_ardupilot/model.sdf (channel 0->rotor_0, 1->rotor_1,
        2->rotor_2, 3->rotor_3). 'no data yet' if /mavros/rc/out hasn't
        published anything, which rc_io can be slow to start - don't
        block the search waiting on it, just say so in the log."""
        msg = self._last_rc_out
        if msg is None or len(msg.channels) < 4:
            return 'rotors=no data yet'
        ch = list(msg.channels[:4])
        return 'rotors=[' + ','.join(str(int(c)) for c in ch) + ']'

    def _statustext_cb(self, msg):
        # Logged verbatim for autotune.log's benefit either way.
        self.body.get_logger().info(f'[FC] {msg.text}')
        # Nov 7, 2026 - these two ARE now matched against expected strings,
        # specifically, confirmed from AC_AutoTune.cpp's own update_gcs():
        #   case AUTOTUNE_MESSAGE_SUCCESS: GCS_SEND_TEXT(..., "AutoTune: Success");
        #   case AUTOTUNE_MESSAGE_FAILED:  GCS_SEND_TEXT(..., "AutoTune: Failed");
        # These are the real completion signals _hold_until_autotune_done()
        # watches for below, replacing the old blind AUTOTUNE_HOLD_SECONDS
        # timer - see that constant's Nov 7 docstring note for why.
        if msg.text == 'AutoTune: Success':
            self._autotune_success = True
        elif msg.text == 'AutoTune: Failed':
            self._autotune_failed = True

    def _log_autotune_params_best_effort(self):
        # Sep 20 session already found /mavros/param/get unreliable in
        # this MAVROS build (never appeared within 15s on a real run).
        # Soft-fail here too - this is a log line, not a precondition.
        if not self.param_get_client.wait_for_service(timeout_sec=5.0):
            self.body.get_logger().warn(
                '/mavros/param/get not available after 5s - skipping the '
                'AUTOTUNE_AXES/AUTOTUNE_AGGR log line (not fatal, this '
                'script never changes them anyway).')
            return
        for param_id in ('AUTOTUNE_AXES', 'AUTOTUNE_AGGR'):
            req = ParamGet.Request()
            req.param_id = param_id
            future = self.param_get_client.call_async(req)
            rclpy.spin_until_future_complete(self.body, future, timeout_sec=3.0)
            result = future.result()
            if result and result.success:
                self.body.get_logger().info(
                    f'{param_id} on the vehicle = integer={result.value.integer} '
                    f'real={result.value.real} (left unchanged by this script)')
            else:
                self.body.get_logger().warn(f'Could not read {param_id}: {result}')

    def _stream_override_for(self, throttle_pwm, seconds, note=None):
        if note:
            self.body.get_logger().info(note)
        msg = _rpy_throttle_override(throttle_pwm)
        period = 1.0 / OVERRIDE_RATE_HZ
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            loop_start = time.monotonic()
            self.override_pub.publish(msg)
            _drain_spin(self.body, period * 0.5)
            remaining = period - (time.monotonic() - loop_start)
            if remaining > 0:
                time.sleep(remaining)

    def _hold_override_and_log(self, throttle_pwm, seconds, note=None):
        """Same job as _stream_override_for, but also logs mode/armed
        status every STATUS_LOG_INTERVAL and bails out immediately if
        the FC leaves AUTOTUNE on its own (RTL/failsafe/etc.) - the same
        class of silent-mode-departure bug the PX4 version of this
        script got bitten by once already. Returns True if AUTOTUNE was
        still the active mode for the whole hold, False if it left early."""
        if note:
            self.body.get_logger().info(note)
        msg = _rpy_throttle_override(throttle_pwm)
        period = 1.0 / OVERRIDE_RATE_HZ
        start = time.monotonic()
        next_log = STATUS_LOG_INTERVAL
        while (time.monotonic() - start) < seconds:
            loop_start = time.monotonic()
            self.override_pub.publish(msg)
            _drain_spin(self.body, period * 0.5)
            elapsed = time.monotonic() - start
            if elapsed >= next_log:
                self.body.get_logger().info(
                    f'  ...{elapsed:.0f}s in - mode={self.body.current_mode()}, '
                    f'armed={self.body.is_armed()}, alt={self._altitude_str()}')
                next_log += STATUS_LOG_INTERVAL
            if self.body.current_mode() != 'AUTOTUNE':
                self.body.get_logger().error(
                    f'Left AUTOTUNE mode on its own after {elapsed:.0f}s '
                    f'(now reporting mode={self.body.current_mode()}) - the FC '
                    'did this, not this script. Stopping the hold early; the '
                    'tune almost certainly was not saved.')
                return False
            if not self.body.is_armed():
                self.body.get_logger().error(
                    f'Disarmed unexpectedly after {elapsed:.0f}s during the '
                    'hold - stopping early.')
                return False
            remaining = period - (time.monotonic() - loop_start)
            if remaining > 0:
                time.sleep(remaining)
        return True

    def _hold_until_autotune_done(self, throttle_pwm, max_seconds, note=None):
        """Nov 7, 2026 - replaces the old blind-timer _hold_override_and_log()
        call for the AUTOTUNE hold specifically. See AUTOTUNE_HOLD_SECONDS's
        deprecation note in the constants above for the full story: a fixed
        duration can only ever be a guess, and the 9th run's guess (120s)
        was confirmed wrong by a real post-flight param read - Roll never
        reached TUNE_COMPLETE in that time, so nothing was eligible to save.

        This watches self._autotune_success / self._autotune_failed (set by
        _statustext_cb() on ArduCopter's own real "AutoTune: Success" /
        "AutoTune: Failed" statustext - the exact strings confirmed from
        AC_AutoTune.cpp's update_gcs()) and ends the hold the moment either
        one fires, logging which. max_seconds is a safety CEILING, not a
        target - if neither ever arrives by then, this gives up and returns
        'timeout' so the caller can still land safely, but that outcome
        itself means something is genuinely stuck (like the Roll Rate P Up
        stall that sat at a flat 0% for ~47s straight on the 9th run), not
        just "needed a bit more time."

        Returns one of 'success', 'failed', 'timeout', or 'left_mode' (same
        early-exit safety behavior as _hold_override_and_log - if the FC
        leaves AUTOTUNE or disarms on its own, stop immediately rather than
        keep streaming into a mode that no longer cares)."""
        if note:
            self.body.get_logger().info(note)
        msg = _rpy_throttle_override(throttle_pwm)
        period = 1.0 / OVERRIDE_RATE_HZ
        start = time.monotonic()
        next_log = STATUS_LOG_INTERVAL
        while (time.monotonic() - start) < max_seconds:
            loop_start = time.monotonic()
            self.override_pub.publish(msg)
            _drain_spin(self.body, period * 0.5)
            elapsed = time.monotonic() - start

            if self._autotune_success:
                self.body.get_logger().info(
                    f'AutoTune: Success seen at {elapsed:.0f}s in - at least '
                    'one axis reached TUNE_COMPLETE. Ending the hold now '
                    'rather than waiting out the rest of the ceiling.')
                return 'success'
            if self._autotune_failed:
                self.body.get_logger().warn(
                    f'AutoTune: Failed seen at {elapsed:.0f}s in - the FC '
                    'gave up on this tune itself. Ending the hold; whether '
                    'anything partial saved depends on axes_completed (see '
                    'AC_AutoTune_Multi::save_tuning_gains() - unverified '
                    'from here, check with check_tune_gains.py after.')
                return 'failed'

            if elapsed >= next_log:
                self.body.get_logger().info(
                    f'  ...{elapsed:.0f}s in - mode={self.body.current_mode()}, '
                    f'armed={self.body.is_armed()}, alt={self._altitude_str()} '
                    '(waiting for AutoTune: Success/Failed, not a fixed timer)')
                next_log += STATUS_LOG_INTERVAL
            if self.body.current_mode() != 'AUTOTUNE':
                self.body.get_logger().error(
                    f'Left AUTOTUNE mode on its own after {elapsed:.0f}s '
                    f'(now reporting mode={self.body.current_mode()}) - the FC '
                    'did this, not this script. Stopping the hold early; the '
                    'tune almost certainly was not saved.')
                return 'left_mode'
            if not self.body.is_armed():
                self.body.get_logger().error(
                    f'Disarmed unexpectedly after {elapsed:.0f}s during the '
                    'hold - stopping early.')
                return 'left_mode'
            remaining = period - (time.monotonic() - loop_start)
            if remaining > 0:
                time.sleep(remaining)

        self.body.get_logger().error(
            f'Neither AutoTune: Success nor AutoTune: Failed arrived within '
            f'the {max_seconds:.0f}s safety ceiling - something is genuinely '
            'stuck (not just slow). Landing now rather than holding forever; '
            'check autotune.log for which test/axis the progress messages '
            'were stalled on when this happened.')
        return 'timeout'

    def _altitude_str(self):
        pos = self.body.current_position()
        return f'{pos[2]:.2f}m' if pos is not None else 'unknown'

    def _switch_mode_confirmed(self, mode_name, throttle_pwm, timeout_sec=3.0):
        """set_mode() plus the same real-mode confirmation every switch in
        this script needs: ArduPilotBody.set_mode() only confirms the
        request was SENT (mode_sent), not that ArduPilot actually
        switched - the old PX4 version of this script got burned by
        trusting mode_sent alone once already. Keeps streaming the RC
        override throughout the confirmation window too, so ap.throttle_zero
        never goes true and the target mode never sees a gap in RC input -
        this matters for the GUIDED->STABILIZE switch specifically, since
        AUTOTUNE's own start() (see module docstring) checks throttle is
        non-zero at the moment it's requested, right after this."""
        if not self.body.set_mode(mode_name):
            return False
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.override_pub.publish(_rpy_throttle_override(throttle_pwm))
            _drain_spin(self.body, 0.05)
            if self.body.current_mode() == mode_name:
                return True
        return False

    def _find_hover_throttle_in_stabilize(self):
        """Nov 6, 2026 (3rd real run) - this search used to run in
        ALT_HOLD, on the wrong assumption that throttle PWM there is a
        direct thrust value. It isn't (see module docstring) - ALT_HOLD
        throttle is a climb-RATE command gated by a mid-stick deadzone,
        so "sweep PWM, converge where vz=0" was measuring an internal
        altitude-hold loop this script was never actually driving, and
        that loop was failing (fell to the ground in under 5s). Runs in
        STABILIZE now instead, where there is no internal altitude loop
        at all and throttle really is the FC's only lift input - the
        model this search has assumed all along, now actually true for
        the mode it runs in.

        Holds one throttle value for HOVER_SEARCH_SAMPLE_SECONDS,
        computes the vertical speed that actually produced from real
        position readings, and nudges toward zero by a fixed step that
        halves on every direction reversal (bisection) - see the Nov 7,
        2026 (11th real run) notes on HOVER_SEARCH_GAIN_PWM_PER_MS and
        HOVER_SEARCH_STEP_PWM for why this replaced an earlier
        magnitude-proportional update - repeating until it converges or
        the search budget runs out.
        Checks HOVER_SEARCH_ABORT_ALTITUDE_M and HOVER_SEARCH_ABORT_CEILING_M
        on every override tick (not just between windows) - the
        ALT_HOLD-era version of this search only checked at window
        boundaries, which wasn't fast enough to catch the 3rd-run fall
        before it reached the ground. Starts from STABILIZE_SEARCH_SEED_PWM,
        not RC_THROTTLE_HOLD - see docstring's 4th-run note for why that
        matters. Returns the converged PWM value, or None if it never
        converges within HOVER_SEARCH_MAX_SECONDS or crosses the floor or
        ceiling (caller should abort to LAND either way, not fall back to
        guessing).

        Nov 6, 2026 (6th/7th real run) - before the adaptive loop below
        even starts, this now holds ONE fixed PWM value
        (STABILIZE_SEARCH_SEED_PWM) for DIAGNOSTIC_HOLD_SECONDS and logs
        rotor output every DIAGNOSTIC_HOLD_LOG_HZ. Added because the
        RC3_REVERSED/MIN/MAX fix in shadow_sitl.parm (same night) had
        ZERO effect on the pattern this telemetry first revealed - real
        rotor PWM falling as the commanded override rises (1650->~1150,
        1800->1100, on every run, identically across all 4 motors, no
        tilt) - which means that fix's theory was wrong. The real problem
        is this: every run so far has the commanded PWM and the elapsed
        time since the STABILIZE switch rising together (the adaptive
        search always raises PWM in response to a falling vz), so there
        has been no way yet to tell whether rotor output falls because of
        what's COMMANDED or just because TIME is passing since the mode
        switch, for some unrelated reason (a failsafe response, a
        throttle ramp, anything else). This hold isolates that: the
        command never changes during it, so if rotor PWM still drifts
        down, the cause is time/state-based, not command-based - a
        completely different, more findable problem than another thrust-
        curve or mixer theory would be."""
        period = 1.0 / OVERRIDE_RATE_HZ

        self.body.get_logger().info(
            f'Diagnostic hold: streaming a FIXED {STABILIZE_SEARCH_SEED_PWM:.0f} PWM for '
            f'{DIAGNOSTIC_HOLD_SECONDS:.1f}s before the adaptive search starts (see this '
            'method\'s Nov 6 6th/7th-run docstring note) - isolates whether real rotor '
            'PWM falls because of a RISING command, or on its own over time even while '
            'the command never changes.')
        hold_deadline = time.monotonic() + DIAGNOSTIC_HOLD_SECONDS
        diag_log_period = 1.0 / DIAGNOSTIC_HOLD_LOG_HZ
        next_diag_log = time.monotonic()
        hold_start = time.monotonic()
        while time.monotonic() < hold_deadline:
            loop_start = time.monotonic()
            self.override_pub.publish(_rpy_throttle_override(STABILIZE_SEARCH_SEED_PWM))
            _drain_spin(self.body, period * 0.5)
            pos_now = self.body.current_position()
            if pos_now is not None and pos_now[2] < HOVER_SEARCH_ABORT_ALTITUDE_M:
                self.body.get_logger().error(
                    f'Diagnostic hold: altitude {pos_now[2]:.2f}m crossed the '
                    f'{HOVER_SEARCH_ABORT_ALTITUDE_M:.1f}m floor while holding a FIXED '
                    f'{STABILIZE_SEARCH_SEED_PWM:.0f} PWM ({self._landed_state_str()}, '
                    f'{self._rc_in_str()}) - aborting before the adaptive search even '
                    'starts.')
                return None
            if time.monotonic() >= next_diag_log:
                self.body.get_logger().info(
                    f'  diagnostic hold: t={time.monotonic() - hold_start:.1f}s, '
                    f'pwm={STABILIZE_SEARCH_SEED_PWM:.0f} (constant), '
                    f'alt={self._altitude_str()}, {self._attitude_deg_str()}, '
                    f'{self._rotor_pwm_str()}, {self._landed_state_str()}, '
                    f'{self._rc_in_str()}')
                next_diag_log += diag_log_period
            remaining = period - (time.monotonic() - loop_start)
            if remaining > 0:
                time.sleep(remaining)

        throttle_pwm = float(STABILIZE_SEARCH_SEED_PWM)
        step = HOVER_SEARCH_STEP_PWM
        last_direction = None  # Nov 7, 2026 (11th real run): +1 means the previous step RAISED
                                # pwm (we were sinking), -1 means it LOWERED pwm (we were
                                # climbing). None before the first step - see docstring note
                                # below and the HOVER_SEARCH_GAIN_PWM_PER_MS comment above for
                                # why this replaced a magnitude-proportional update.
        search_deadline = time.monotonic() + HOVER_SEARCH_MAX_SECONDS
        # Nov 7, 2026 (12th real run): rolling (timestamp, altitude) buffer for a FAST
        # vz estimate, checked every tick below alongside the floor/ceiling checks -
        # see HOVER_SEARCH_FAST_VZ_WINDOW_SEC/HOVER_SEARCH_EMERGENCY_VZ_MS. Lives
        # outside the per-window loop so it keeps a continuous view across window
        # boundaries, not just within one.
        fast_samples = []
        while time.monotonic() < search_deadline:
            pos_start = self.body.current_position()
            window_start = time.monotonic()
            window_deadline = window_start + HOVER_SEARCH_SAMPLE_SECONDS
            while time.monotonic() < window_deadline:
                loop_start = time.monotonic()
                self.override_pub.publish(_rpy_throttle_override(throttle_pwm))
                _drain_spin(self.body, period * 0.5)
                pos_now = self.body.current_position()
                if pos_now is not None and pos_now[2] < HOVER_SEARCH_ABORT_ALTITUDE_M:
                    self.body.get_logger().error(
                        f'Hover-throttle search: altitude {pos_now[2]:.2f}m crossed the '
                        f'{HOVER_SEARCH_ABORT_ALTITUDE_M:.1f}m floor mid-search (last try: '
                        f'{throttle_pwm:.0f} PWM, {self._attitude_deg_str()}, '
                        f'{self._rotor_pwm_str()}) - aborting the search immediately '
                        'rather than finishing this sample window.')
                    return None
                if pos_now is not None and pos_now[2] > HOVER_SEARCH_ABORT_CEILING_M:
                    self.body.get_logger().error(
                        f'Hover-throttle search: altitude {pos_now[2]:.2f}m crossed the '
                        f'{HOVER_SEARCH_ABORT_CEILING_M:.1f}m ceiling mid-search (last try: '
                        f'{throttle_pwm:.0f} PWM, {self._attitude_deg_str()}, '
                        f'{self._rotor_pwm_str()}) - aborting the search immediately '
                        'rather than finishing this sample window.')
                    return None
                # Nov 7, 2026 (12th real run): predictive brake - this is what the 12th
                # run's crash was missing. The floor/ceiling checks above only fire on
                # absolute position, which for a vehicle already accelerating downward
                # (see HOVER_SEARCH_STEP_PWM's comment - holding any off-hover PWM keeps
                # accelerating vz for as long as it's held, it never settles) can mean
                # "too late" - 2.98m with vz already at -4m/s and still worsening, on the
                # 12th run, left no real margin. This fast estimate reacts within
                # HOVER_SEARCH_FAST_VZ_WINDOW_SEC instead of a full HOVER_SEARCH_SAMPLE_
                # SECONDS window, so a dangerous trend gets caught while there's still
                # altitude to do something about it.
                now = time.monotonic()
                if pos_now is not None:
                    fast_samples.append((now, pos_now[2]))
                while fast_samples and now - fast_samples[0][0] > HOVER_SEARCH_FAST_VZ_WINDOW_SEC:
                    fast_samples.pop(0)
                if len(fast_samples) >= 2:
                    fast_dt = fast_samples[-1][0] - fast_samples[0][0]
                    if fast_dt > 0:
                        vz_fast = (fast_samples[-1][1] - fast_samples[0][1]) / fast_dt
                        if abs(vz_fast) >= HOVER_SEARCH_EMERGENCY_VZ_MS:
                            self.body.get_logger().error(
                                f'Hover-throttle search: fast vz={vz_fast:+.2f}m/s '
                                f'(alt={pos_now[2]:.2f}m) crossed the '
                                f'{HOVER_SEARCH_EMERGENCY_VZ_MS:.1f}m/s emergency threshold '
                                f'mid-window (last try: {throttle_pwm:.0f} PWM) - aborting '
                                'immediately rather than waiting for the slower floor/ceiling '
                                'check or the rest of this window.')
                            return None
                remaining = period - (time.monotonic() - loop_start)
                if remaining > 0:
                    time.sleep(remaining)
            pos_end = self.body.current_position()
            elapsed = time.monotonic() - window_start
            if pos_start is None or pos_end is None or elapsed <= 0:
                self.body.get_logger().warn(
                    'Lost position estimate during the hover-throttle search.')
                return None
            vz = (pos_end[2] - pos_start[2]) / elapsed
            self.body.get_logger().info(
                f'  throttle trim: pwm={throttle_pwm:.0f} -> vz={vz:+.2f}m/s '
                f'(alt={pos_end[2]:.2f}m, {self._attitude_deg_str()}, '
                f'{self._rotor_pwm_str()})')
            if abs(vz) <= HOVER_SEARCH_CONVERGED_VZ_MS:
                self.body.get_logger().info(
                    f'Converged: {throttle_pwm:.0f} PWM holds altitude within '
                    f'{HOVER_SEARCH_CONVERGED_VZ_MS:.2f}m/s.')
                return throttle_pwm
            # Nov 7, 2026 (11th real run): direction-reversal step-halving (bisection),
            # replacing the magnitude-proportional update - see the HOVER_SEARCH_GAIN_PWM_PER_MS
            # comment above for why. Only the SIGN of vz decides which way to move next
            # (climbing -> less throttle, sinking -> more); the step only shrinks when that
            # direction reverses from the previous step, i.e. when the search has just
            # overshot the true hover point, which is exactly the signal that the step was too
            # big rather than too small. A direction that holds steady across steps means we're
            # still closing in from the same side, so the step stays put rather than being
            # grown or shrunk on a guess.
            direction = -1 if vz > 0 else 1
            if last_direction is not None and direction != last_direction:
                step = max(step / 2.0, HOVER_SEARCH_MIN_STEP_PWM)
            last_direction = direction
            throttle_pwm += direction * step
            throttle_pwm = max(HOVER_SEARCH_MIN_PWM, min(HOVER_SEARCH_MAX_PWM, throttle_pwm))
        self.body.get_logger().error(
            f'Hover-throttle search did not converge within '
            f'{HOVER_SEARCH_MAX_SECONDS:.0f}s (last try: {throttle_pwm:.0f} PWM).')
        return None

    def _arrest_fall_and_land(self, reason):
        """Nov 7, 2026 (12th real run - the first actual CRASH, not just a
        safe abort). Every other abort path in this script used to do
        exactly this:

            self.override_pub.publish(_zero_override())
            self.body.set_mode('LAND')

        in one breath, on the unchecked assumption that LAND engages and
        takes over throttle before gravity finishes whatever the vehicle
        was already doing. That assumption had held across half a dozen
        earlier aborts in this project, but only because none of them
        happened mid-dive. This one did: the hover-throttle search's own
        floor check didn't fire until 2.98m, by which point vz was
        already around -4m/s and still getting worse (see
        HOVER_SEARCH_STEP_PWM and HOVER_SEARCH_EMERGENCY_VZ_MS's
        comments for why) - and _zero_override() doesn't command zero
        thrust, it RELEASES the override entirely, handing throttle back
        to whatever RC_IN the FC sees with no transmitter attached. There
        is no guarantee LAND's own throttle control takes over before
        that gap lets the fall continue, and with under 3m and a fast,
        still-accelerating sink, that gap is exactly what turned a
        logged, intentional abort into a real ground strike.

        This function replaces that pattern everywhere it mattered: hold
        a known-safe, above-hover PWM (STABILIZE_SEARCH_SEED_PWM -
        confirmed across multiple runs to produce real climb) for up to
        HOVER_SEARCH_ARREST_SECONDS, watching for the vertical rate to
        turn non-negative, BEFORE ever releasing the override and
        switching to LAND. If the vehicle wasn't actually falling when
        this was called, it just climbs briefly instead - a small, safe
        cost compared to the alternative. Still a best-effort fix, not a
        guarantee: it depends on STABILIZE_SEARCH_SEED_PWM genuinely
        being enough thrust to arrest whatever rate this is called at,
        which has not been tested from as fast a fall as the 12th run's.
        Watch the next crash-path trigger closely rather than assuming
        this is settled."""
        self.body.get_logger().warn(
            f'{reason} - holding {STABILIZE_SEARCH_SEED_PWM:.0f} PWM for up to '
            f'{HOVER_SEARCH_ARREST_SECONDS:.1f}s to arrest any fall before releasing '
            'the override and switching to LAND (see Nov 7 12th-run docstring note on '
            '_arrest_fall_and_land - never hand off cold again).')
        period = 1.0 / OVERRIDE_RATE_HZ
        deadline = time.monotonic() + HOVER_SEARCH_ARREST_SECONDS
        pos_prev = self.body.current_position()
        t_prev = time.monotonic()
        while time.monotonic() < deadline:
            loop_start = time.monotonic()
            self.override_pub.publish(_rpy_throttle_override(STABILIZE_SEARCH_SEED_PWM))
            _drain_spin(self.body, period * 0.5)
            pos_now = self.body.current_position()
            now = time.monotonic()
            if pos_now is not None and pos_prev is not None:
                dt = now - t_prev
                if dt >= 0.1:
                    vz = (pos_now[2] - pos_prev[2]) / dt
                    self.body.get_logger().info(
                        f'  arresting: vz={vz:+.2f}m/s, alt={pos_now[2]:.2f}m, '
                        f'{self._rotor_pwm_str()}')
                    if vz >= 0.0:
                        self.body.get_logger().info(
                            'Fall arrested - releasing override and switching to LAND.')
                        break
                    pos_prev, t_prev = pos_now, now
            remaining = period - (time.monotonic() - loop_start)
            if remaining > 0:
                time.sleep(remaining)
        else:
            self.body.get_logger().warn(
                f'{HOVER_SEARCH_ARREST_SECONDS:.1f}s arrest window ran out without '
                'confirming a non-negative vertical rate - releasing the override and '
                'switching to LAND anyway rather than holding indefinitely.')
        self.override_pub.publish(_zero_override())
        if self.body.is_armed():
            self.body.set_mode('LAND')

    def _land_and_save(self, hover_pwm):
        """The one function in this file that must never call set_mode()
        again once it starts - per GitHub issue #14584 and QGC's own
        guide, switching out of AUTOTUNE before disarming discards the
        tune. Descends purely by easing the RC override throttle down,
        staying in AUTOTUNE the whole way, then disarms.

        Nov 7, 2026: the old descend_pwm = hover_pwm - DESCEND_THROTTLE_OFFSET
        formula assumed AUTOTUNE throttle is direct thrust, same mistaken
        assumption that caused the runaway climb this hold phase crashed
        from (see module docstring and AUTOTUNE_HOLD_PWM's comment above).
        hover_pwm isn't usable here at all for computing a descent command -
        it's a STABILIZE thrust value, meaningless as an AUTOTUNE climb-rate
        command - so this now streams the fixed AUTOTUNE_DESCEND_PWM instead
        and only keeps hover_pwm around for the log line."""
        descend_pwm = AUTOTUNE_DESCEND_PWM
        self.body.get_logger().info(
            f'Descending at a fixed {descend_pwm:.0f} PWM (a climb-rate command, well '
            f'below AUTOTUNE\'s throttle deadband - NOT derived from the STABILIZE hover '
            f'PWM of {hover_pwm:.0f}, which would mean something different here) while '
            'staying in AUTOTUNE (required to save the tune - see module docstring)...')
        deadline = time.monotonic() + DESCEND_MAX_SECONDS
        msg = _rpy_throttle_override(descend_pwm)
        period = 1.0 / OVERRIDE_RATE_HZ
        landed = False
        while time.monotonic() < deadline:
            loop_start = time.monotonic()
            self.override_pub.publish(msg)
            _drain_spin(self.body, period * 0.5)
            pos = self.body.current_position()
            if pos is not None and pos[2] <= GROUND_ALTITUDE_TOLERANCE_M:
                landed = True
                break
            remaining = period - (time.monotonic() - loop_start)
            if remaining > 0:
                time.sleep(remaining)

        if not landed:
            self.body.get_logger().error(
                f'Did not confirm touchdown within {DESCEND_MAX_SECONDS:.0f}s - '
                'disarming anyway rather than hanging, but check the sim '
                'manually. Still in AUTOTUNE, so this is still the tune\'s '
                'only chance to save.')

        if self.body.current_mode() != 'AUTOTUNE':
            self.body.get_logger().error(
                f'Mode is {self.body.current_mode()}, not AUTOTUNE, at the '
                'moment of disarm - the tune was already discarded before '
                'this point, disarming only to make the vehicle safe.')

        disarmed = self.body.disarm()
        self.override_pub.publish(_zero_override())  # release all channels
        if disarmed and landed and self.body.current_mode() == 'AUTOTUNE':
            self.body.get_logger().info(
                'Landed and disarmed while still in AUTOTUNE - gains should '
                'be saved. Verify with a param read or a second flight.')
        else:
            self.body.get_logger().warn(
                f'Landed={landed}, disarmed={disarmed}, '
                f'mode_at_disarm_check=AUTOTUNE - tune save is NOT confirmed. '
                'Treat the gains as unchanged until verified.')

    def run(self):
        self.body.get_logger().info('Waiting for MAVROS connection to ArduPilot...')
        if not self.body.wait_until_connected(timeout_sec=30.0):
            self.body.get_logger().error('Never connected - aborting.')
            return

        self.body.get_logger().info('Connected. Waiting for a position estimate...')
        if not self.body.wait_for_position_estimate(timeout_sec=25.0):
            self.body.get_logger().error('Never got a position estimate - aborting.')
            return

        self._log_autotune_params_best_effort()
        # 245 = EXTENDED_SYS_STATE, 100000us = 10Hz - see
        # _request_message_interval()'s docstring for why this is being
        # tried instead of another MAV1_EXT_STAT-style param guess.
        self._request_message_interval(245, 100000)

        self.body.get_logger().info(f'Taking off to {TAKEOFF_ALTITUDE_M}m in GUIDED...')
        if not self.body.is_armed():
            if not self.body.set_mode('GUIDED'):
                return
            if not self.body.arm():
                return
        if not self.body.takeoff(TAKEOFF_ALTITUDE_M):
            self.body.get_logger().error('Takeoff rejected - landing and aborting.')
            self.body.set_mode('LAND')
            return

        deadline = time.monotonic() + CLIMB_TIMEOUT_SEC
        reached_altitude = False
        while time.monotonic() < deadline:
            _drain_spin(self.body, 0.2)
            pos = self.body.current_position()
            if pos is not None and pos[2] >= TAKEOFF_ALTITUDE_M - 0.5:
                reached_altitude = True
                break
        if not reached_altitude:
            self.body.get_logger().error(
                f'Did not reach {TAKEOFF_ALTITUDE_M}m within {CLIMB_TIMEOUT_SEC:.0f}s '
                '- landing and aborting rather than switching modes at an '
                'uncertain altitude.')
            self.body.set_mode('LAND')
            return

        self.body.get_logger().info(f'At altitude ({self._altitude_str()}). Settling in GUIDED...')
        settle_deadline = time.monotonic() + GUIDED_SETTLE_SECONDS
        while time.monotonic() < settle_deadline:
            _drain_spin(self.body, 0.2)

        # Start streaming a centered RC override BEFORE the first mode
        # switch away from GUIDED - see docstring: the one working
        # companion-computer example found for this had RC override
        # active going into AUTOTUNE, not started after. Kept running
        # continuously through both mode switches below
        # (GUIDED -> STABILIZE -> AUTOTUNE), not just right before
        # AUTOTUNE - see the next comment for why that now matters.
        self._stream_override_for(
            RC_THROTTLE_HOLD, PRE_AUTOTUNE_OVERRIDE_SECONDS,
            note='Streaming centered RC override before switching modes...')

        # AUTOTUNE's own start() only accepts a mode switch from
        # STABILIZE, ALT_HOLD, LOITER, or POSHOLD (confirmed from
        # ArduCopter source - see module docstring's Nov 6 note) - GUIDED
        # is not on that list, so going there directly always fails with
        # "init failed". Route through STABILIZE, not ALT_HOLD (see the
        # docstring's 3rd-run note for why ALT_HOLD was the wrong choice
        # here - its throttle channel is a climb-rate command, not thrust,
        # which broke the hover-throttle search below).
        self.body.get_logger().info(
            'Switching to STABILIZE (required before AUTOTUNE - see Nov 6 3rd-run '
            'note in docstring)...')
        if not self._switch_mode_confirmed('STABILIZE', STABILIZE_SEARCH_SEED_PWM):
            self.body.get_logger().error(
                f'STABILIZE was not confirmed (still reporting '
                f'mode={self.body.current_mode()}) - aborting before even '
                'trying AUTOTUNE, since AUTOTUNE requires STABILIZE, ALT_HOLD, '
                'LOITER, or POSHOLD first.')
            self._arrest_fall_and_land('STABILIZE not confirmed')
            return

        # Measure the real hover throttle in STABILIZE, where throttle is
        # actually direct thrust (see docstring's Nov 6 3rd-run note for
        # why this used to run in ALT_HOLD and why that was wrong).
        self.body.get_logger().info(
            'In STABILIZE. Searching for the real hover throttle before AUTOTUNE '
            '(see module docstring)...')
        hover_pwm = self._find_hover_throttle_in_stabilize()
        if hover_pwm is None:
            self.body.get_logger().error(
                'Hover-throttle search failed - aborting before AUTOTUNE rather '
                'than tuning while climbing or sinking.')
            self._arrest_fall_and_land('Hover-throttle search failed')
            return

        self.body.get_logger().info('Switching to AUTOTUNE...')
        if not self._switch_mode_confirmed('AUTOTUNE', hover_pwm):
            self.body.get_logger().error(
                f'AUTOTUNE was sent but ArduPilot never actually switched '
                f'(still reporting mode={self.body.current_mode()}) - '
                'aborting instead of holding in an uncertain mode.')
            self._arrest_fall_and_land('AUTOTUNE switch not confirmed')
            return
        self.body.get_logger().info('AUTOTUNE mode confirmed active.')

        hold_result = self._hold_until_autotune_done(
            AUTOTUNE_HOLD_PWM, AUTOTUNE_MAX_HOLD_SECONDS,
            note=(f'Holding in AUTOTUNE at {AUTOTUNE_HOLD_PWM:.0f} PWM, centered in '
                  f'AUTOTUNE\'s throttle deadband (NOT the measured STABILIZE hover PWM '
                  f'of {hover_pwm:.0f} - AUTOTUNE reads throttle as a climb-rate command, '
                  'not direct thrust; see the Nov 7 docstring/AUTOTUNE_HOLD_PWM note on '
                  'the runaway climb this caused before). Nov 7 (later): this now waits '
                  'for ArduCopter\'s own AutoTune: Success/Failed statustext instead of a '
                  f'fixed timer - up to {AUTOTUNE_MAX_HOLD_SECONDS:.0f}s ceiling. Watch '
                  'the dashboard / this log for FC statustext in the meantime.'))
        if hold_result in ('left_mode',):
            # _hold_until_autotune_done already logged why. Don't try to
            # "save" anything - that ship almost certainly already sailed
            # per the log above - but still arrest any fall before handing
            # off to LAND (see _arrest_fall_and_land's Nov 7 12th-run note);
            # it checks is_armed() itself before the final set_mode(), so
            # this is a safe no-op if we're not even armed any more.
            self._arrest_fall_and_land('Left mode during AUTOTUNE hold')
            return
        # 'success', 'failed', and 'timeout' all still land via
        # _land_and_save() while staying in AUTOTUNE - per AC_AutoTune.cpp's
        # disarmed() handler, save_tuning_gains() is attempted on ANY
        # disarm-while-in-AUTOTUNE-mode regardless of success/failed/
        # timeout, and itself no-ops safely if axes_completed is still 0
        # (nothing to save) - see AUTOTUNE_HOLD_SECONDS's deprecation note.
        # So there's no reason to skip landing-while-in-AUTOTUNE on
        # 'failed' or 'timeout' - whatever partial progress exists (if any)
        # still gets its one chance to save, exactly as it would on
        # 'success'.
        self.body.get_logger().info(f'Hold ended with result={hold_result!r}.')
        self._land_and_save(hover_pwm)
        self.body.get_logger().info('Autotune run finished.')


def main(args=None):
    rclpy.init(args=args)
    runner = AutotuneRunner()
    try:
        runner.run()
    except KeyboardInterrupt:
        pass
    finally:
        runner.body.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
