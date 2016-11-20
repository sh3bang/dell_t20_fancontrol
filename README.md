# Dell T20 - Contol more then one case FAN

These program is written for a ARM based Development Board (STM32F103C8T6).
This is to extend the DELL T20 Fan controller to contol more then one fan on the single case fan pin header.

My project is described here (in german but with Pics!):

http://www.hardwareluxx.de/community/f101/dell-poweredge-t20-ft-dell-perc-h730-aufgebohrt-und-zugenietet-1112100.html

You can also use it to control FANs other brands (i.e. with less RPM) then the proprietary one, because these types of server dosent allow to adjust the FAN thresholds!

## How does it work

The programm simulate the original / proprietary fan when the system is running.
This is to pass the "internal fan test programm" because without successfull test the server doesn't start and stop with the error message "Alert - Previous fan failure.".

The fan control signal from the mainboard will be detectet and a fake fan clock signal generated.
The conroller send the same PWM signal like detected to four additional fans while one will get a bit higher PWM duty cycle (+33%) to spin faster then the other because of handling more air (mounted on the back of case).

If any connected fan failed, the fake clock signal will stoped and the mainboad detect a fan failure.

### The programm have three different modes:

### learning mode
Generates a fan control signal (PWM) in 100 steps from 100 to 0 percent duty cyle and capture the fan clock signal to flash (four times per step and calculate the average).
After accomplishing this mode you can switch to the fan mode ("production use").

Note: You need the original fan to lern the speed range! The original fan is not needed to be connected anymore after learning mode :-)

### fan mode
Detecting fan control signal (PWM duty cylce) from mainboard and deliver the captured fan clock signal back ("fake fan").
On fan failure stop sending signal to mainboard

### reset mode
Delete all captured data during learning mode ("factory reset")

--
I am to lazy today to give help of connecting GPIO and so on, just study the main program.
Maybe its not my best program and not well commented, but it´s working fine.
