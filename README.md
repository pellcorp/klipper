# Simple AF Klipper fork for K1 Series (MIPS)

This includes K1, K1C, K1 Max, K1SE as well as Ender 3 V3 KE, Ender 5 Max and Nebula Pad

## Building firmware

The host, btteddy and Nebula Pad specific firmware for older ender 3s is done here via docker

```
docker run -ti -v $PWD:$PWD pellcorp/k1-klipper-fw-build $PWD/build.sh
```

## Building Chelper

The MIPS Creality OS does not have gcc so we need to prebuild c_helper.so

```
cd klippy/chelper
docker run -ti -v $PWD:$PWD pellcorp/k1-klipper-fw-build /bin/bash -c "cd $PWD && make clean && make"
```
