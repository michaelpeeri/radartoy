# radartoy

## Build instructions (using image from https://github.com/MightyDevices/docker-gcc-arm-none-eabi-builder)
docker exec gcc-arm-none-eabi-container sh -c "cd /share && make"

## Program over USB using DFU
dfu-util -a 0 --dfuse-address 0x08000000 -D build/RadarToy.bin
