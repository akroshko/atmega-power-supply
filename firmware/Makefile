firmware_name=atmega-power-supply

hex: $(firmware_name).hex

$(firmware_name).bin :
	avr-gcc -Wall -Wl,-u,vfprintf -lprintf_flt -lm -g -Os -mmcu=atmega168 -o $(firmware_name).bin $(firmware_name).c

$(firmware_name).hex : $(firmware_name).bin
	avr-objcopy -j .text -j .data -O ihex $(firmware_name).bin $(firmware_name).hex

flash : $(firmware_name).hex
	avrdude -p atmega168 -P /dev/ttyACM0 -c avrisp -b 19200 -U flash:w:$(firmware_name).hex

clean :
	rm $(firmware_name).hex $(firmware_name).bin
