# Live feed wire format

Live feed override is merged into edge-next for 450.05, including ordered
program permission and job resets. Physical pulse, interrupt-time and stack
qualification are complete; the normal and hooks gate suites passed.

`{"mfo":0.5}` or `$mfo=0.5` requests 50% feed. The accepted range is 0.05 through
2.0; values outside it return a nonzero status. The reply acknowledges the
request. `mfc` changes only when exec accepts the private speed plan at an
eligible future segment. If no remaining eligible segment can fit the transition,
the request stays pending. A held machine can accept a request; speed planning
waits until motion resumes. Neither request handling nor publication resumes it.

Each newline-terminated rt envelope consumes one line slot and produces one
ordinary response with a footer, regardless of how many fields it contains.
A host must keep its control reserve and account for that response. Optional
standalone feed bytes consume no line slots, and remain disabled after reset.
`ovb` must be the only field inside its `rt` envelope (a top-level `tid` is
allowed). For example, send `{"rt":{"mfo":0.5,"mfe":1}}` and then
`{"rt":{"ovb":1}}` as two lines. Combining `ovb` with another rt field is
refused with status 106 before applying the envelope. Byte opt-in changes the
receive interpretation and requires a quiet machine and an input-line boundary;
keeping it exclusive gives that transition its own acknowledged request.

The primary settings work in flat JSON and text mode:

| Token | Access | Meaning | Default |
|---|---|---|---|
| `mfo` | read/write | Requested feed factor, 0.05 through 2.0 | 1.0 |
| `mfoe` | read/write | Operator feed enable, 0 or 1 | 1 |
| `m48e` | read/write | Operator master enable, 0 or 1 | 1 |
| `mfc` | read only | Committed feed factor | 1.0 |
| `mcp` | read only | Accepted plan capped by speed limits | false |
| `mfe` | read only | Effective feed permission | 1 |

Read with `{"mfo":null}` or `$mfo`; use the same forms for the other tokens.
`mfo`, `mfc`, `mcp` and `mfe` remain selectable status-report fields. The
`rt.mfo` request/read and `rt.mfe` operator-enable write remain aliases;
`rt.mfe` reads effective permission, while `mfoe` reads the operator switch.
`rt.mfc` and `rt.mcp` remain read aliases. `ov` and exclusive `ovb` remain
rt-only controls. Each newline-terminated flat JSON or text command consumes
one line slot, just like its rt alias. Read-only token writes are refused.

These cfgArray entries have neither `F_PERSIST` nor `F_INITIALIZE`: ordinary
SET dispatch never writes their values to EEPROM. `$defa` does not reset them;
they are not persisted settings and are not inputs to settings migration.

These reads also work in alarm, shutdown and panic, including after job kill.
A group containing any write is refused in those states before any child is
executed; a mixed read/write request does not partially apply. Configuration
objects such as `sr` remain writes even when their values are null.

`{"rt":{"ov":1}}` enables change events; `0` disables them. Events are
coalesced, at most one per status interval. `{"rt":{"ov":null}}` requests
one event even when automatic events are off; its reply gives the revision
observed at request time. A newer revision may be coalesced into the event
before transmission. The event is its own line, without a response footer:

```
{"ov":{"v":1,"q":4,"mfo":1.500,"mfc":1.500,"mfe":1,"capped":true}}
```

`v` versions the frame; `q` is a state revision. `capped` says that axis or
motor speed limits reduced the accepted plan's cruise speed. A committed factor
of 1.5 or 2.0 does not promise that speed: acceleration, jerk and axis/motor
limits still apply. The frame describes the factor accepted for the plan,
not the instantaneous speed during a transition. Read `vel` for speed.

All of these values are RAM only. Board reset clears events and the byte opt-in,
and restores requested/committed factors to unity, `m48e=mfoe=1`, and program
permission enabled.
M2/M30 reset factors, both operator switches and program permission in queue order when program end executes;
job kill clears them immediately and invalidates private plans. Both disable the
byte opt-in. The event opt-in survives job end/kill, so the host can see the reset.
Feedhold and flush invalidate pending plans but retain the operator request;
a change during hold is accepted and considered after resume. Neither resumes motion.

M48 enables feed and spindle program permission, M49 disables both, and M50 defaults to P1 (enable feed);
M50 P0 disables it. Other P values fail before executing any part of the line.
These commands execute in queue order. Program permission is independent of
`m48e` and `mfoe`: effective `mfe` is true only when both operator
switches AND the ordered program permission permit feed override. Writing either
operator switch to 1 cannot bypass M49 or M50 P0. Permission changes retain
the requested factor. M2/M30 restore both operator switches and program permission,
following the
[LinuxCNC program-end rule](https://www.linuxcnc.org/docs/html/gcode/m-code.html).

Live spindle override (450.06)
------------------------------

`{"sso":0.5}` or `$sso=0.5` requests 50% of the programmed spindle S.
`sso` accepts 0.05 through 2.0 and defaults to 1.0; `ssoe` is the operator
spindle-enable switch and defaults to 1. Both are RAM-only, accepted during
motion and hold, never saved in EEPROM or changed by restoring defaults.
Read-only `sse` is effective permission: `ssoe` AND `m48e` AND spindle program
permission. `scp` reports whether the active speed was clamped; it is false
while off. `sps` is the applied, clamped RPM request, not a tachometer reading.
`ssc` reports the multiplier applied to the running spindle (unity while off
or disabled); a clamp can limit the actual RPM even when `ssc` is 2.0.
These fields are available as ordinary GETs and selectable status fields.

M51 or M51 P1 enables spindle program permission; M51 P0 disables it. Other P
values return 161. M50 changes feed permission only; M51 changes spindle
permission only. All M-code permissions execute in queue order. Operator writes
cannot bypass a disabled program permission. Changing permission retains `sso`.

While M3/M4 is active, live changes update PWM in the main-loop callback after
acceptance, without waiting for a motion boundary. Computation uses the S that
has executed, never a future queued S. CW and CCW use their respective configured
speed and PWM ranges. The original S is retained when the result is clamped.
The reply accepts the request; `ssc`/`sps` describe the applied output.

A factor or permission write cannot start a stopped spindle. M5, alarm,
shutdown and kill keep their off behavior. Feedhold changes motion only, so
spindle speed can be changed while held; neither a factor change nor a flush
resumes motion. Hold then flush retains the spindle output. Board reset and
job kill restore `sso=1`, `ssoe=1` and spindle program permission. M2/M30 perform
those resets and stop the spindle when their queued command executes.
Alarm-state reads work; writes are refused with the existing alarm status.

`rt.sso` remains an alias for `sso`; the legacy writable `rt.sse` aliases the
operator switch `ssoe` (its GET reports effective permission). Flat `sse` is
read-only. Each newline takes one normal line slot and returns one footer.
The version-1 `ov` event remains feed-only; use GET or status reports for spindle
fields. No new asynchronous frame or spindle-byte controls are added.

The flat names and non-persistent settings follow g2core; TinyG defaults operator
switches on and retains separate ordered program permission. Rapid override
remains unsupported. Opted-in spindle/rapid bytes remain consumed as unsupported,
never interpreted as feed changes; live spindle control uses the settings above.
