#!/bin/bash

# in case build is executed from outside current dir be a gem and change the dir
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd -P)"
cd $SCRIPT_DIR

if [ "$1" = "host" ]; then
  export CROSS_PREFIX=mips-linux-gnu-
fi

cp .config.$1 .config
mkdir -p outfw/
make clean
make

if [ "$1" = "host" ]; then
  mv out/klipper.elf outfw/klipper_host_mcu
elif [ "$1" = "btteddy" ]; then
  mv out/klipper.uf2 outfw/btteddy.uf2
else
  mv out/klipper.bin outfw/${1}.bin
fi
