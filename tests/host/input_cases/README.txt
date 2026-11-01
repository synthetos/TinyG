Generated G-code, JSON and text input checks
==========================================
Base: edge-next 222a45c0. Desk only; no firmware changes or device access.
Run: python3 -m unittest discover -s tests/host
Focused: python3 -m unittest discover -s tests/host -p 'test_*input*.py'

The harness extracts the real normalizer, word reader, block parser, JSON
scanner/tokenizer/marshaller/dispatcher, text parser, numeric converters, NV list
initialization/traversal/string copying and JSON NV dispatch. Firmware headers
provide structures, enums and limits. Motion execution, device/storage operations
and a small configuration table are capture stubs. This checks parser behavior,
not the final physical outcome of every accepted motion command. JSON table row
zero deliberately retains the real fb read-only setter. AVR NULL-parent NV reset
is adapted to an empty desktop string; function bodies otherwise stay intact.

Compiler: cc -std=gnu99 -Wall -Werror=format -O1 -g -funsigned-char,
-fsanitize=address,undefined,float-cast-overflow -fno-sanitize-recover=all.
Inputs are separately allocated with exactly one terminating NUL. This exposes
cross-line accesses hidden by spare capacity in a pooled serial buffer. Each
call has a 200 ms timer, each subprocess a 30 s timeout, and each campaign a
55 s budget. Fixed seeds: G-code 168001, JSON 168002, text 168003. The corpus has
4517 / 4395 / 4332 distinct inputs (13244 total), including generated CAM lines,
all printable characters, whitespace, comments, number extremes, exponents,
random edits, missing quotes, deep nesting and long direct-call probes.

Every completed line must return a status. The G-code oracle independently
reads source words and compares the reader trace and reported model values.
JSON/text checks compare captured values for supported scalar inputs, preserving
TinyG's bare keys and legacy n/t/f syntax. Unknown sanitizer failures or unmatched
word/value results fail the campaign. Known findings are narrowly classified;
the campaign resumes after each such case so an early failure does not hide the
rest. Ten minimal desired-behavior tests strictly expect failure: an unexpected
success fails the suite until the corresponding marker is removed.

Limits: host strtof accepts hexadecimal values that installed AVR libc's decimal
converter does not. These observations are explicitly labeled HOST_LIBC_HEX,
not assigned firmware defect numbers. Lines beyond the current 254-character
transport limit are direct-call bounds/termination probes. This is a finite,
repeatable campaign, not a proof for all inputs or all hardware executions.
The complete host suite is also necessary for the E150-E161 checks below.

Findings (locations refer to base 222a45c0)
-----------------------------------------
E168 HIGH: nonfinite G-code values reach execution.
  gcode_parser.c:234 and :531 (also F/S/P/I/J/K/R assignments).
  Minimal: Xnan. Parser status 0, executor called, X target NaN. Very large
  decimal coordinates also become infinity. Invalid numeric values pass the
  parser into motion calculations. Physical effects are not asserted here.
  Test: test_e168_nonfinite_axis_is_rejected.
E169 MEDIUM: N converted to uint32 without range/integrality checking.
  gcode_parser.c:546. Minimal: N-1. The sanitizer detects the invalid conversion;
  N1.5 is accepted as N1 and high integers round before conversion. A line number
  can be changed or converted outside the C type's range instead of rejected.
  Test: test_e169_negative_line_number_is_rejected_without_bad_cast.
E170 MEDIUM: repeated nonmodal words silently use the last value.
  gcode_parser.c:48 SET_NON_MODAL and :531. Minimal: X1X2. Status 0, X=2;
  duplicate axes should be rejected rather than selecting a different target.
  Test: test_e170_duplicate_axis_is_rejected.
E171 MEDIUM: trailing comma advances beyond the text input terminator.
  text_parser.c:123-125. Minimal: $,. The comma consumes NUL, then the loop
  increments again and reads outside the input array. Continuing over adjacent
  bytes can also copy them; the minimal test proves the first invalid read.
  Test: test_e171_trailing_comma_stays_in_input.
E172 MEDIUM: junk after the root JSON object applies the prefix SET.
  json_parser.c:367-378 and :477-479. Minimal: {"xvm":1}x. Tokenization accepts
  the suffix at depth zero; the first SET runs, then an empty NV is dispatched
  to row zero (fb) and returns status 104 (read-only). This is not a hang.
  Test: test_e172_trailing_json_junk_does_not_execute.
E173 MEDIUM: invalid JSON literal tails are accepted.
  json_parser.c:312-313 and _marshall_nv_pair literal cases (:553 onward).
  Minimal: {"ee":trux}. Status 0, boolean true is dispatched. Null-like garbage
  similarly becomes a GET. Legitimate short n/t/f spellings remain controls.
  Test: test_e173_invalid_literal_is_rejected.
E174 MEDIUM: whitespace inside a JSON number joins separate digits.
  json_parser.c:353-354. Minimal: {"xvm":1 2}. Status 0, SET value 12.
  Test: test_e174_separated_digits_are_not_joined.
E175 MEDIUM: a partial smart-quote byte sequence reads beyond NUL.
  util.c:194-201. Minimal bytes: 7b e2 80. str_asciify advances to NUL and then
  reads one byte beyond it. Exact-size input allocation detects that access.
  Test: test_e175_partial_smart_quote_stays_in_input.
E176 MEDIUM: JSON control escapes lose their backslash without decoding.
  json_parser.c:340-344. Minimal: {"gc":"a\tb"}. Captured string is "atb",
  instead of a, tab, b. A JSON-carried command can change meaning. Quote and
  backslash handling remain supported; this finding concerns the other escapes.
  Test: test_e176_json_tab_escape_is_decoded.
E177 MEDIUM: characters after a closing string quote become string content.
  json_parser.c:346-349 and VALUE state. Minimal: {"gc":"a"b}. Status 0;
  captured string is a"b. Invalid quoting should be rejected before dispatch.
  Test: test_e177_characters_after_closing_quote_are_rejected.

E150-E161 regression coverage
-----------------------------
E150 dotted codes and E159 exponent separation: test_unsupported_overrides.py.
E151 system group table membership/token lengths: test_spindle_polarity.py;
     NV bounds also exercised in generated JSON and existing JSON storage tests.
E152 modal conflicts: test_modal_groups.py.
E153 text and JSON writes rejected in alarm/shutdown/panic, reads allowed:
     test_input_regressions.py (real cm_is_alarmed).
E154 F/S/T ranges and E160 L ranges: test_input_regressions.py.
E155 XYZ inches versus ABC degrees for G28.3/G92/G43.1:
     added real-function checks in test_coordinate_offsets.py.
E156 homing callback reports its specific failure:
     test_input_regressions.py (real _homing_error_exit).
E157 jog admission while busy: test_jog_allowed.py.
E158 motor/jerk zero, negative and nonfinite rejection, E160 qv range:
     added checks against real table setters in test_config_defaults.py.
E160 status messages/format precision: test_status_messages.py and configuration
     table checks; physical HIL wire formatting is outside this desk campaign.
E161 punctuation cannot splice words: test_normalize_block.py and source oracle.
