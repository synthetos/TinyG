# Configuration layout 3 and motion-only feedhold

Feedhold stops motion along the programmed path. It does not change spindle or
coolant outputs. Resume releases motion without starting a spindle delay. A hold
followed by queue flush discards motion and leaves outputs unchanged. M5, M9,
program end, job kill and alarm keep their own output behaviour.

The three former output-pause settings and their hidden activation stamp are
removed. They are not accepted settings, status fields or runtime-session fields.
Spindle and coolant readouts report the actual outputs throughout hold and resume.

## Saved settings

Layout 3 uses EEPROM marker 10003. Layout 2 (450.03 and 450.04) is migrated before
any settings are loaded. The unchanged prefix is retained. Before moving the
suffix down, the firmware copies its old records and the 30 saved status-report
selections to unused EEPROM. A stamp commits that copy. Only then are destination
records written. The layout marker is written last and the journal stamp cleared.
An interrupted copy starts again from the untouched source; an interrupted move
starts again from the committed copy. Removed report selections are discarded;
all other selections retain their token names despite the changed table indexes.
Saved output-pause values, including an enabled pause, are discarded.

The mapping uses the compiled table boundaries, including conditional motor and
diagnostic rows. The journal is checked against EEPROM capacity before writing.
The active AVR table needs records through 452 for this journal, below the
checkpoint area even in a 512-record EEPROM model.

Master 440.20 and 440.21 upgrades retain the existing settings migration and its
power-cut journal. Saved status-report selections now have their own journal and
are translated by the original master token table, including read-only fields.
The existing deliberate conversions remain: JSON verbosity 1/2 becomes 3, Z-min
mode 0 becomes probe mode 4, and character receive mode remains selected. Invalid
saved values retain the existing reported fallback-to-default behaviour.
Layout 1 and legacy 449.01 retain the append-only prefix migration. Unknown or
erased layouts load defaults. Older firmware does not understand layout 3;
downgrades still require a saved settings snapshot and explicit restoration.

## Retained motion fixes

- The terminal DDA and dwell timer overflow flags are still cleared before the
  next segment; startup and cancelled-dwell flag clearing are also unchanged.
- Resume keeps HOLD and program-stop state latched during the interruptible
  planner walk. A new hold, flush, kill, limit, shutdown or alarm vetoes release,
  including a new hold followed by resume during that walk.
- Contact holds still enter through their dedicated cycle API and cancel pending
  resume. Both contact and operator holds preserve outputs, so output ownership
  state is no longer needed.
- Startup/prepared-segment checks, empty-queue resume, held-block replan geometry,
  flush position resynchronization and source cancellation remain in place.
- N99 combined-state reporting remains: HOLD during a held dwell; RUN for the
  remaining dwell after resume; explicit M0/M1 remains PROGRAM_STOP until resume.
- Feed-limit accounting, advisor frames and status scheduling are unchanged.

## Verification

Host tests exercise real sequencing functions with stop requests injected during
resume planning, output preservation, explicit emergency output stop, dwell
reporting, and masked callers. Layout tests cut every byte of layout-2 migration
and validate the actual compiled AVR table mapping. Master tests cover 440.20 and
440.21, every byte cut, repeated cuts, and torn checkpoints with saved report
selections. Physical outputs, upgrade persistence and motion are verified by the
bench operator before merge; host tests do not substitute for those checks.
