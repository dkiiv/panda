#!/usr/bin/env sh
set -e

cd ..
scons -u -j$(nproc) --eepb
cd eepb

../../tests/eepb/enter_canloader.py obj/eepb.bin.signed
