#!/bin/zsh
# Overnight bench 2 (Saleae): true-path soaks of the built-in programs, then every corpus and generated file with a
# random mode, motor geometry, push on/off and feedholds; failures are reset and rerun identically. Until 07:30.
S=${TINYG_SCRATCH:?set TINYG_SCRATCH}; N=$S/night
P=${TINYG_BENCH2_PORT:?set TINYG_BENCH2_PORT}; R=${TINYG_RUNS:?set TINYG_RUNS}; LOG=$N/logs/b2.log; BENCH=b2; PREV=""
export TINYG_OWNER=${TINYG_OWNER:-hunt}
FLASH_EXTRA=""
source $N/night-lib.sh
cd $S/wt-night/tests/hil
O=(--port $P --allow-motion --ice --diagnostics --runs-dir=$R -q -rxXs -p no:cacheprovider --run-purpose adhoc)
end=$(date -j -f "%Y-%m-%d %H:%M" "$(date +%Y-%m-%d) ${NIGHT_END:-07:30}" +%s); [[ $end -lt $(date +%s) ]] && end=$((end + 86400))
modes=(line g2)   # line mode only: RTS/CTS and XON/count streams can lock up the FTDI driver
while [[ $(date +%s) -lt $end && ! -f $N/stop && ! -f $N/stop-$BENCH ]]; do
  geos=("" ""); while IFS= read -r g; do [[ $g == \#* ]] && continue; geos+=("$g"); done < $N/geos.txt   # re-read each pass
  jobs=()
  for p in squares circles goose mixed; do for m in line noflow; do jobs+=("true:$p:$m"); done; done
  for f in $N/corpus/*.nc $N/gen/gen-*-0[0-7][0-9].nc $N/corpus-sg/*.nc $N/corpus-sg/*.nc; do [[ -L $f ]] || jobs+=("file:$f"); done
  for j in ${(f)"$(print -l $jobs | sort -R)"}; do
    [[ $(date +%s) -ge $end || -f $N/stop || -f $N/stop-$BENCH ]] && break
    if [[ $j == true:* ]]; then
      p=${${j#true:}%%:*}; m=${j##*:}
      geo=${geos[$((RANDOM % ${#geos} + 1))]}
      [[ $geo == *ma=* ]] && geo=""                     # the true path needs X and Y on M1/M2 (no axis remaps)
      job "true $p $m geo=$geo" tests/diagnostics/test_soak_true_path.py SOAK_PROGRAM=$p SOAK_MODE=$m SOAK_SETTINGS=$geo
    else
      f=${j#file:}; (( $(python3 -I $N/est.py $f 2>/dev/null || echo 99) < 6 )) 2>/dev/null || continue   # skip jobs over ~6 min ideal (2-3x real with acceleration)
      [[ -f $N/stop || -f $N/stop-$BENCH ]] && break
      m=${modes[$((RANDOM % ${#modes} + 1))]}; fi=$((RANDOM % 2)); geo=${geos[$((RANDOM % ${#geos} + 1))]}
      holds=$(( RANDOM % 3 == 0 ? 3 : 0 )); seed=$((RANDOM * 32768 + RANDOM + 1))
      [[ $m == rts ]] && holds=0                        # N04: a hold in RTS/CTS mode with a full queue wedges the board
      job "file $(basename $f) $m fi=$fi holds=$holds seed=$seed geo=$geo" tests/diagnostics/test_soak_file.py \
          SOAK_FILE=$f SOAK_MODE=$m SOAK_FI=$fi SOAK_SETTINGS=$geo SOAK_HOLDS=$holds SOAK_HOLD_SEED=$seed
    fi
  done
done
echo "$(date '+%m-%d %T') b2 NIGHT-DONE" >> $LOG
