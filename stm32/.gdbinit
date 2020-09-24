file stm32
target extended-remote localhost:3333

define flash
make
mon flash write_image erase stm32.bin 0x08000000
r
end
