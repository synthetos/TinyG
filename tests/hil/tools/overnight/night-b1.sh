#!/bin/zsh
# Overnight bench 1 (ICE, test-hooks build): the whole suite in shuffled order (--ice --streaming --large-travel),
# with ICE homing, switch trips, the G-code probe, coordinate and soft-limit soaks, generated-file soaks with holds,
# the stack check and up to four settings sweeps; failures are reset and rerun identically. Until 07:30.
S=${TINYG_SCRATCH:?set TINYG_SCRATCH}; N=$S/night
P=${TINYG_BENCH1_PORT:?set TINYG_BENCH1_PORT}; R=${TINYG_RUNS:?set TINYG_RUNS}; LOG=$N/logs/b1.log; BENCH=b1; PREV=""
export TINYG_OWNER=${TINYG_OWNER:-hunt}
FLASH_EXTRA="--make TEST_HOOKS=1"
source $N/night-lib.sh
cd $S/wt-night/tests/hil
O=(--port $P --allow-motion --ice --diagnostics --streaming --large-travel --runs-dir=$R -q -rxXs -p no:cacheprovider --run-purpose regression)
end=$(date -j -f "%Y-%m-%d %H:%M" "$(date +%Y-%m-%d) ${NIGHT_END:-07:30}" +%s); [[ $end -lt $(date +%s) ]] && end=$((end + 86400))
sweeps=0; i=0; modes=(line g2)   # line mode only: RTS/CTS and XON/count streams can lock up the FTDI driver
d=(test_homing_repeat test_mounted_switch_trip test_gcode_probe test_stack_highwater test_coords_soak test_soft_limits_soak)
while [[ $(date +%s) -lt $end && ! -f $N/stop && ! -f $N/stop-$BENCH ]]; do
  for t in ${(f)"$(ls tests/test_*.py tests/bugs/test_*.py tests/streaming/test_*.py | grep -v test_e164_count_accounting | sort -R)"}; do
    [[ $(date +%s) -ge $end || -f $N/stop || -f $N/stop-$BENCH ]] && break
    i=$((i+1))
    if [[ $t == tests/streaming/* ]]; then job "suite $t" $t PYTEST_ADDOPTS="-k 'line or g2'"; else job "suite $t" $t; fi
    if (( i % 3 == 0 )); then
      d=(); while IFS= read -r x; do [[ -n $x && $x != \#* ]] && d+=("$x"); done < $N/diags.txt   # re-read each time
      dt=${d[$((RANDOM % ${#d} + 1))]}; seed=$((RANDOM * 32768 + RANDOM + 1))
      [[ -f $N/stop || -f $N/stop-$BENCH ]] && break
      job "diag $dt seed=$seed" tests/diagnostics/$dt.py COORDS_SEED=$seed SOFTLIM_SEED=$seed INT_SEED=$seed
      g=""; for cand in ${(f)"$(ls $N/gen/gen-*-0[0-7][0-9].nc $N/corpus/*.nc $N/corpus-sg/*.nc $N/corpus-sg/*.nc | sort -R | head -8)"}; do   # skip jobs over ~6 min ideal (2-3x real with acceleration)
        (( $(python3 -I $N/est.py $cand 2>/dev/null || echo 99) < 6 )) 2>/dev/null && { g=$cand; break; }; done; [[ -z $g ]] && continue
      [[ -f $N/stop || -f $N/stop-$BENCH ]] && break
      m=${modes[$((RANDOM % ${#modes} + 1))]}
      holds=$(( RANDOM % 3 == 0 ? 3 : 0 )); [[ $m == rts ]] && holds=0   # N04
      job "file $(basename $g) $m holds=$holds seed=$seed" tests/diagnostics/test_soak_file.py \
          SOAK_FILE=$g SOAK_MODE=$m SOAK_FI=$((RANDOM % 2)) SOAK_HOLDS=$holds SOAK_HOLD_SEED=$seed
    fi
    if (( i % 60 == 0 && sweeps < 4 )); then
      sweeps=$((sweeps+1)); job "diag settings_sweep" tests/diagnostics/test_settings_sweep.py
    fi
  done
done
echo "$(date '+%m-%d %T') b1 NIGHT-DONE" >> $LOG
