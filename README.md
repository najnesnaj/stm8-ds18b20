temperature measurement ds18b20 




it uses cheap components and cheap development


a TM1637 4 segment LED display -- very cheap = 60 cent

stm8 board -- very cheap = 60 cent

It would like to thank tiger762 : https://github.com/tiger762/STM8S103
and rogerdahl : https://github.com/rogerdahl/stm32-tm1637  
which code is used.





As far as software packages, you'll need at the very least, the SDCC compiler and a way to upload the resulting IHX file to the device:

On my Orange Pi running Armbian:

apt-get install sdcc

(optional if you want serial comms back to the host) apt-get install minicom

git clone http://github.com/vdudouyt/stm8flash
cd stm8flash
make
make install

The hardware programmer is sold on Ebay for around $4 each. Look for "st-link v2". Note that a hardware programmer is required to program the stm8 family. There is not a serial uploader installed like the stm32 has.


I used serial port of Orange Pi for debugging : #minicom -b 115200 -D /dev/ttyS1 (pin 38 - 40 on the GPIO)

Optional but extremely useful:
Saleae Logic Analyzer (About $7 each) -- to see up to 8 channels of digital waveform. EXTREMELY useful for debugging I2C. Seriously...

