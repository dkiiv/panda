#!/usr/bin/env sh
set -e

cd ..
scons -u -j$(nproc) --epla
cd epla

../../tests/epla/enter_canloader.py obj/epla.bin.signed
