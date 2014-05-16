#!/bin/bash

# TinyG simulator tests. They don't require any hardware, 
# but also don't guarantee that the simulation is precise.

set -o nounset
set -o errexit

EXIT_CODE=0

fail() {
  EXIT_CODE=1
  echo "FAILED"
}

echo "============================================="
echo "Running very basic test with full output ..."
echo "============================================="
echo ""

./tinyg.elf < test.gcode

echo -n "test.gcode -- "
( ./tinyg.elf test.gcode && echo "OK" ) || fail

echo "============================================="
echo "OK"
echo "============================================="
echo ""

echo ""
echo "============================================="
echo "Running full gcode test suite ..."
echo "Note: currently, it only tests that the simulator exit code."
echo "============================================="

for i in ../../../gcode_samples/*.gcode
do
  echo -n "$i -- "
  OUT_FILE=`basename $i`.out
  (( ./tinyg.elf $i > $OUT_FILE 2>&1 ) && echo "OK" ) || ( fail && cat $OUT_FILE )
done

echo ""
if [ $EXIT_CODE -eq 0 ];
then
  echo "All tests PASSED"
else
  echo "Some tests FAILED"
fi

exit $EXIT_CODE
