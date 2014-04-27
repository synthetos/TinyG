#!/bin/bash

# TinyG simulator tests. They don't require any hardware, 
# but also don't guarantee that the simulation is precise.

set -o nounset
set -o errexit

./tinyg.elf < test.gcode
