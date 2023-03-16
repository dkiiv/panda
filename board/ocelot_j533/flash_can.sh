#!/usr/bin/env sh
set -e

cd ..
OJ533=1 scons -u
cd ocelot_j533

../../tests/gateway/enter_canloader.py ../obj/panda.bin.signed
