# arduino101-pwm-dac
Small sketch showing how to use two timers in the Arduino101 (Intel Curie) to generate PWM audio

## Overview:

This module uses two hardware timer/counters.  The first timer is configured
to generate a pulse-width modulated output (PWM) signal.  The second timer is
configured as a sample clock: it rolls over and generates an interrupt once
every sample period.  The interrupt routine fetches the next audio sample and
adjusts the duty cycle of the PWM timer.
 
We use ARC_V2_TRM1 for the sample clock and one of the Quark PWM timers for
PWM.

This pencil sketch plays 8KHz 8-bit precomputed samples from in-memory array
(sounddata.h).  A more comprehensive version would fetch data from an SD
card or filesystem.

## Future Enhancement

Modify this to use only Quark timers so audio playback could coexist with 
PWM generation for servo motors, etc.
