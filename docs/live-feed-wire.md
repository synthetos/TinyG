# Live feed wire format (development branch)

This draft includes ordered program permission and job resets. Physical pulse,
interrupt-time and stack qualification remain required before the 450.05 gate.

A pure `{"rt":{"mfo":0.5}}` requests 50% feed. The accepted range is 0.05 through
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

Read requested feed with `{"rt":{"mfo":null}}`, permission with
`{"rt":{"mfe":null}}`, and committed feed with `{"rt":{"mfc":null}}`.
Read the cap flag with `{"rt":{"mcp":null}}`. Flat `mfo`, `mfc`, `mfe` and
`mcp` are read-only and can be selected in `sr`. `mcp` is a boolean; the short
name fits the existing five-character token limit including the rt prefix.

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
and restores requested/committed factors to unity with feed permission enabled.
M2/M30 reset factors and both permissions in queue order when program end executes;
job kill clears them immediately and invalidates private plans. Both disable the
byte opt-in. The event opt-in survives job end/kill, so the host can see the reset.
Feedhold and flush invalidate pending plans but retain the operator request;
a change during hold is accepted and considered after resume. Neither resumes motion.

M48 enables program permission, M49 disables it, and M50 defaults to P1 (enable);
M50 P0 disables it. Other P values fail before executing any part of the line.
These commands execute in queue order. Program permission is independent of
`rt.mfe`, the operator enable: effective `mfe` is true only when both permit feed
override. A live request or byte knob cannot bypass M49. Permission changes retain
the requested factor. M2/M30 restore both permissions, following the
[LinuxCNC program-end rule](https://www.linuxcnc.org/docs/html/gcode/m-code.html).

Spindle and rapid override are outside this feature. Spindle factor/enable writes
are refused; their factors stay unity. Opted-in spindle/rapid bytes are consumed
as unsupported, never interpreted as feed changes. No output is changed by a
feed override command.
