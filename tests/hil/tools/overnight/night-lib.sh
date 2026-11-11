# Shared by night-b1.sh / night-b2.sh. job <label> <test path> [ENV=value ...]: run once; on a failure reset the
# board (^X) and run the identical job again (same env, same seeds). Logs one line per job, and one JSON line per
# failure to replications.jsonl: replicated or not, plus the job that ran before (to spot one test affecting the next).
# A setup error (the run errored within seconds, nothing passed or failed) is the bench, not a finding: clear the
# alarm and reset, then reflash the night build, retrying after each; three in a row stop this bench (night/stop-<bench>).
# Every flash is checked against the flash record, so the loop can never carry on with edge-0.97 left on the board.
# _pt runs pytest and prints its summary line; the whole output stays in logs/<bench>-last.out, so instrument faults
# (a stuck Saleae capture, a busy board) count as bench errors wherever they appear in it.
_pt() {
  env "$@" uv run pytest $t $O > $N/logs/$BENCH-last.out 2>&1
  local r; r=$(tail -1 $N/logs/$BENCH-last.out)
  [[ $r == *failed* || $r == *error* ]] && cp $N/logs/$BENCH-last.out "$N/logs/jobs/$BENCH-$(date +%H%M%S).out"   # keep every failure's output
  echo "$r"
}
_setup_err() {
  [[ $1 == *error* && $1 != *passed* && $1 != *failed* && $1 =~ ' in [0-9]+\.[0-9]+s' ]] && return 0   # errors only, however long the setup took
  [[ $1 != *passed* ]] && grep -q "saleae start_capture\|INTERNAL_ERROR\|BoardBusy" $N/logs/$BENCH-last.out
}
_flash_record() {
  local current="${TINYG_HIL_STATE:-/Volumes/External/projects/TinyG-work/state}/tinyg-hil-${P:t}.flashed"
  local legacy="/tmp/tinyg-hil-${P:t}.flashed"
  if [[ ! -f "$current" || "$legacy" -nt "$current" ]]; then
    print -r -- "$legacy"
  else
    print -r -- "$current"
  fi
}
_flashed() { grep -q "$1" "$(_flash_record)" 2>/dev/null; }
# A board whose main loop is gone ignores ^X; the debugger resets it (nothing in flash or EEPROM changes).
# Before the reset, where the CPU is (three samples) and the key state go to the log as a BOARD LOCKED finding,
# so a dead main loop is a scoreboard entry and not a silent bench stop.
_ice_reset() {
  local pc st
  pc=$(for i in 1 2 3; do uv run python -m tinyg_hil.ice where --port $P 2>/dev/null; done | sort -u | tr '\n' ' ')
  st=$(uv run python -m tinyg_hil.ice peek --port $P cm.machine_state cs.controller_state us[0].rx_buf_count cs.saved_buf 2>/dev/null | cut -c1-300)
  echo "$(date '+%m-%d %T') $BENCH BOARD LOCKED (no answer, ^X ignored) pc: ${pc:-unknown} | state: ${st:-unreadable} | after job: ${PREV:-?} | firmware: $(grep -o 'wt-[a-z0-9]*' "$(_flash_record)" 2>/dev/null | head -1)" >> $LOG
  uv run python -m tinyg_hil.ice reset --port $P >/dev/null 2>&1; sleep 3
  # a session that died with the board leaves its settings behind: back to the day's first snapshot
  local snap; snap=$(ls ${TINYG_SCRATCH:-/Volumes/External/projects/TinyG-worktrees}/wt-night/tests/hil/.snapshots/$(uv run python -m tinyg_hil info --port $P 2>/dev/null | grep -o '"id": "[^"]*"' | cut -d'"' -f4)* 2>/dev/null | head -1)
  [[ -n $snap ]] && uv run python -m tinyg_hil restore --port $P $snap >> $N/logs/$BENCH-restore.log 2>&1
}
_unstick() {
  uv run python -m tinyg_hil json --port $P '{"stat":null}' >/dev/null 2>&1 || _ice_reset
  uv run python -m tinyg_hil json --port $P '{"clear":null}' >/dev/null 2>&1
  env uv run pytest tests/diagnostics/test_reset_board.py $O --diagnostics >/dev/null 2>&1
  uv run python -m tinyg_hil json --port $P '{"clear":null}' >/dev/null 2>&1
}
_flash_night() {
  local i; for i in 1 2; do
    eval "uv run python -m tinyg_hil flash --port $P --build ../.. $FLASH_EXTRA" >> $N/logs/$BENCH-flash.log 2>&1
    _flashed wt-night && return 0
    _ice_reset                                           # a locked board cannot enter the bootloader
  done
  echo "$(date '+%m-%d %T') $BENCH BENCH STOPPED: the night build would not flash (logs/$BENCH-flash.log)" >> $LOG
  touch $N/stop-$BENCH; return 1
}
SETUP_FAILS=0

job() {
  local label=$1 t=$2; shift 2
  local r rr verdict
  _flashed wt-night || _flash_night || return
  r=$(_pt "$@")
  if _setup_err "$r"; then
    _unstick; r=$(_pt "$@")
    if _setup_err "$r"; then _flash_night || return; r=$(_pt "$@"); fi
    if _setup_err "$r"; then
      SETUP_FAILS=$((SETUP_FAILS + 1))
      echo "$(date '+%m-%d %T') $BENCH $label | BENCH SETUP ERROR, not a finding ($r)" >> $LOG
      if (( SETUP_FAILS >= 3 )); then
        echo "$(date '+%m-%d %T') $BENCH BENCH STOPPED: three setup errors in a row" >> $LOG; touch $N/stop-$BENCH
      fi
      PREV="$label"; return
    fi
  fi
  SETUP_FAILS=0
  if [[ $r == *failed* || $r == *error* ]]; then
    env uv run pytest tests/diagnostics/test_reset_board.py $O --diagnostics >/dev/null 2>&1
    rr=$(_pt "$@")
    local edge="" re=""
    if _setup_err "$rr"; then
      verdict=bench-error; _unstick
    elif [[ $rr == *failed* || $rr == *error* ]]; then
      verdict=replicated
      # regression or not: the identical job on plain edge-0.97 (push off: edge-0.97 has no rt fi), then back.
      # The source-guided corpus uses the edge runtime JSON interface, so it is not compared on edge-0.97.
      if [[ "$*" == *corpus-sg* ]]; then
        echo "$(date '+%m-%d %T') $BENCH $label | $r | rerun after reset: $verdict ($rr) | edge-0.97: n/a (edge-only corpus)" >> $LOG
        python3 -I -c 'import json,sys; print(json.dumps({"when": sys.argv[1], "bench": sys.argv[2], "label": sys.argv[3], "test": sys.argv[4], "env": sys.argv[5], "first": sys.argv[6], "rerun": sys.argv[7], "verdict": sys.argv[8], "previous": sys.argv[9], "edge097": "n/a", "edge097_result": ""}))' \
          "$(date '+%m-%d %T')" "$BENCH" "$label" "$t" "$*" "$r" "$rr" "$verdict" "$PREV" >> $N/replications.jsonl
        PREV="$label"; return
      fi
      uv run python -m tinyg_hil flash --port $P --build $S/wt-edge097 >/dev/null 2>&1
      re=$(_pt "$@" SOAK_FI=0)
      if [[ -z $re ]] || _setup_err "$re" || ! _flashed wt-edge097; then edge=unclear
      elif [[ $re == *failed* || $re == *error* ]]; then edge=old; elif [[ $re == *passed* ]]; then edge=new; else edge=unclear; fi
      _flash_night || true
    else
      verdict=not-replicated
    fi
    echo "$(date '+%m-%d %T') $BENCH $label | $r | rerun after reset: $verdict ($rr)${edge:+ | edge-0.97: $edge ($re)}" >> $LOG
    python3 -I -c 'import json,sys; print(json.dumps({"when": sys.argv[1], "bench": sys.argv[2], "label": sys.argv[3], "test": sys.argv[4], "env": sys.argv[5], "first": sys.argv[6], "rerun": sys.argv[7], "verdict": sys.argv[8], "previous": sys.argv[9], "edge097": sys.argv[10], "edge097_result": sys.argv[11]}))' \
      "$(date '+%m-%d %T')" "$BENCH" "$label" "$t" "$*" "$r" "$rr" "$verdict" "$PREV" "$edge" "$re" >> $N/replications.jsonl
  else
    echo "$(date '+%m-%d %T') $BENCH $label | $r" >> $LOG
  fi
  PREV="$label"
}
