#! /bin/bash
mkdir ./binaries &>/dev/null
make clean
make -j3
cp usbdfu.bin ./binaries/boot.F303.bin
make clean
