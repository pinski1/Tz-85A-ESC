#Tz-85A ESC
Firmware for the popular Tz-85A brushless speed controller to modify it to control brushed motors.
<!-- little more words -->

##Hardware Modifications
**IN PROGRESS**

<!-- add instructions
1. Remove plastic casing.
2. Snip off wire on *tive side.
3. Remove switch & short pads. (OPTIONAL)
4. Lift pin ? and connect POT wires (OPTIONAL)
-->

##Programming
**CODE NOT YET FINISHED**

~~Please download the speed controller .hex file to turn your brushless speed controller into a brushed speed controller from [here](.\Release\Tz-85A-ESC.hex).~~

~~Please download the servo controller .hex file to turn yout brushless speed controller into a brushed servo controller from [here](.\Release\Tz-85A_SERVO.hex).~~

To reprogram your Tz-85A using AVRDUDE use Ladyada's AVRDUDE tutorial [here](http://www.ladyada.net/learn/avr/avrdude.html).

To reprogram your Tz-85A using Atmel Studio:

1. Open up Atmel Studio
2. Click on the top menu item 'Tools', then 'Device Programming' or press Ctrl+Shift+P
3. Select the correct programmer from the 'Tool' dropdown selection
4. Select the correct device (ATMega8) from the 'Device' dropdown selection
5. Click on the 'Memories' tab on the left hand side.
6. In the 'Flash (8 KB)' section select the correct .hex file with the 'Browse' button
7. Check that both 'Erase device before programming' and 'Verify Flash after programming' are checked
8. Check the programmer is connected to the microcontroller
9. Click the button 'Program'
10. Done!

##Compiling
If you want to modify the firmware then download the source branch, unzip it in an appropriate place. Tz-85A makes use of the free AVRLib libraries.

##Licence
<a rel="license" href="http://creativecommons.org/licenses/by/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by/4.0/88x31.png" /></a><br /><span xmlns:dct="http://purl.org/dc/terms/" property="dct:title">Tz-85A-ESC</span> by <a xmlns:cc="http://creativecommons.org/ns#" href="https://github.com/pinski1/Tz-85A-ESC" property="cc:attributionName" rel="cc:attributionURL">Pinski1</a> is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by/4.0/">Creative Commons Attribution 4.0 International License</a>.