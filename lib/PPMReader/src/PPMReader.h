/*
 * Copyright 2016 Aapo Nikkilä
 *
 * This file is part of PPM Reader.
 *
 * PPM Reader is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * PPM Reader is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with PPM Reader.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PPM_READER
#define PPM_READER

#include <Arduino.h>

// Self-contained PPM decoder (no InterruptHandler base class).
// A single static instance pointer + a free ISR forwards pulses to the
// instance, avoiding the duplicate AVR interrupt-vector table that the
// original packaged WInterrupts.cpp collided with.

class PPMReader {
    public:
    // The range of a channel's possible values
    unsigned long minChannelValue = 1000;
    unsigned long maxChannelValue = 2000;

    /* The maximum error (in either direction) in channel value
     * with which the channel value is still considered valid */
    unsigned long channelValueMaxError = 10;

    /* The minimum value (time) after which the signal frame is considered to
     * be finished and we can start to expect a new signal frame. */
    unsigned long blankTime = 2100;

    PPMReader(byte interruptPin, byte channelAmount);
    ~PPMReader();

    /* Returns the latest raw (not necessarily valid) value for the
     * channel (starting from 1). */
    unsigned long rawChannelValue(byte channel);

    /* Returns the latest received value that was considered valid for the channel (starting from 1).
     * Returns defaultValue if the given channel hasn't received any valid values yet. */
    unsigned long latestValidChannelValue(byte channel, unsigned long defaultValue);

    private:
    // The pin from which to listen for interrupts
    byte interruptPin = 0;

    // The amount of channels to be expected from the PPM signal.
    byte channelAmount = 0;

    // Arrays for keeping track of channel values
    volatile unsigned long *rawValues = NULL;
    volatile unsigned long *validValues = NULL;

    // A counter variable for determining which channel is being read next
    volatile byte pulseCounter = 0;

    // A time variable to remember when the last pulse was read
    volatile unsigned long microsAtLastPulse = 0;

    // The single active instance (used by the free ISR)
    static PPMReader *activeInstance;

    // Free-function ISR trampoline
    static void isr();

    // Instance-side pulse handler
    void handlePulse();
};

#endif
