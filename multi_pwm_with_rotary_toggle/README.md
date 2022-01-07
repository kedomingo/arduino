# Dual frequency generator controlled by rotary encoder

This is a working example of how to run PWM in Arduino with frequencies outside the default 500 MHz.
I use this application in components that require PWM with 13kHz frequency. This makes use of two of 
the three available timers in the chip.

## Math

Theoreticlaly, this supports frequencies up to the clockspeed of the chip being used
For atmega328p, this is 16MHz. Calculation of the appropriate pre-scalers
as well as setting the timing registers for both Timer 1 and Timer 2 are included in the code.

### Timers

Timer 1 is 16-bit counting from 0 to 65,535, while Timers 2 and 3 are 8 bit 
only, counting from 0-255. 1 clock cycle is 1 count.

For 16MHz CPU, Timer completes its counting 16,000,000/65,535 or just over 244 times per second.
Timers 2 and 3 on the other hand complete their counting to 255 in 16,000,000/255 or just over 62,745
times in a second.

#### Timer 1 logic

We achieve PWM using Timer 1 by starting the count at a certain point and letting it overflow to 65,535.
Say, to achieve a frequency of 244 Hz, we can set the starting point at 0 so it counts the full 0-65,535,
completing the 244 counts every second at 16MHz.

Duty cycle is achieved by using `OCR1A` to turn off the signal at a certain point in the count. Using the
same example, we can achieve a 50% duty cycle by setting `OCR1A` to 32,767. In this case, signal at PIN 9
is high at count 0, and becomes low at count 32,767, until we reach 65,535 then it turns to high again.

#### Timer 2 logic

On the other hand, when using Timers 2 or 3, we can achieve PWM by starting at 0 and stopping at a certain
point in their count to 255. Say to achieve a frequency of 62.745 KHz, we can let the counter complete its
count from 0-255 everytime.

Duty cycle is achieved by using `OCR2B` to turn off the signal. The signal is reset to high when the 
counter stopping point is reached.

#### Limitations

Due to the integer nature of the Timing registers, we cannot achieve exact frequencies at all 
times due to rounding errors. For example, to achieve 13 KHz PWM at 8 MHz clock speed, Timer 1 needs to 
restart 13,000 times in 1 second. To do this, Timer 1 starts counting from 64,921 or 615 clock cycles
before overflowing to 65,536.

If we count to 615 thirteen thousand times per second, we achieve 7,995,000 which is just short of the chip clock speed.
If we divide the clock speed of 8,000,000 by 615, we achieve a frequency of 13.008 KHz

Timers 2 and 3 will be worse when it comes to rounding errors because its resolution is only 0-255 compared to Timer 1's 0-65,535

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


## UPDATE (Jan 7 2022)
**The frequencies are halved on the Arduino pro mini 3.5v (8MHz)**. I'm not 
really sure why since the clockspeed constant F_CPU should be 8000000 in the pro mini
and should adjust the other values accordingly, but the oscilloscope shows otherwise.
I didn't bother spending time with it, so just DOUBLE THE FREQUENCY ON THE 8MHz ARDUINO PRO MINI

