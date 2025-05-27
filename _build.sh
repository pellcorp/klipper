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
  mv out/klipper.elf fw/K1/klipper_host_mcu
elif [ "$1" = "btteddy" ]; then
  mv out/klipper.uf2 fw/K1/btteddy.uf2
else
  echo "$1 not supported"
fi
