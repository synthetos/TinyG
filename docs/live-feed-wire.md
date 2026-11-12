# Live feed wire format (publication increment)

This is the publication increment on the development branch. Ordered program
permissions and the final reset/program-end lifecycle are not complete yet.
Do not treat this draft as the final 450.05 interface.

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
and restores requested/committed factors to unity. The final lifecycle increment
will implement and test program-end and job-kill resets and separate ordered
M48/M49/M50 program permission from the live operator request. Spindle override
is outside this feature; its outputs must remain unaffected by feed changes.
