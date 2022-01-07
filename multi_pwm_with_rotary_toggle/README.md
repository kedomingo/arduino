# Dual frequency generator controlled by rotary encoder

This supports frequencies up to the clockspeed of the chip being used
For atmega328p, this is 16MHz. Calculation of the appropriate pre-scalers
as well as setting the timing registers for both Timer 1 and Timer 2 are included in the code.

**IMPORTANT**: The frequencies are halved on the Arduino pro mini 3.5v (8MHz). I'm not 
really sure why since the clockspeed constant F_CPU should be 8000000 in the pro mini
and should adjust the other values accordingly, but the oscilloscope shows otherwise.
I didn't bother spending time with it, so just DOUBLE THE FREQUENCY ON THE 8MHz ARDUINO PRO MINI

## Pins

### PWM Pins
Timer 1 generates a PWM signal on pin OC1A (Digital 9)
Timer 2 generates a PWM signal on pin OC2B (Digital 3)

### Rotary Encoder pins

Duty cycle adjustment using rotary encoder on digital pins 4, 5, 6 for CLK, DT, SW
Toggles between frequency 1 and frequency 2 duty cycle adjustment using
the push button on the rotary encoder.



**Author** Kyle Domingo https://github.com/kedomingo/arduino

Main references:
 - https://wolles-elektronikkiste.de/en/timer-and-pwm-part-2-16-bit-timer1
 - https://wolles-elektronikkiste.de/en/timer-and-pwm-part-1-8-bit-timer0-2


## License
[Creative Commons Attribution-NonCommercial-ShareAlike (CC-BY-NC-SA)](https://creativecommons.org/licenses/by-nc-sa/4.0/)

Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made. You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.

NonCommercial — You may not use the material for commercial purposes.

ShareAlike — If you remix, transform, or build upon the material, you must distribute your contributions under the same license as the original.

