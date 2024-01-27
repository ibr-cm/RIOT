DMTS MAC test application
======================
This application is a showcase for testing the synchronization that is achieved by the
DMTS MAC protocol.

Usage
=====

To use this test application you will need two boards with a STM32 MCU and an AT86RF215 connected
to it (or on board).

One of the boards can be normally flashed:
```
make flash test BOARD=your_board term
```

The other one has to be flashed with an additional flag:
```
make flash test BOARD=your_board OPTIONAL_CFLAGS+=-Dtx_node term
```

Then connect a simple push button to GND and the specified pins of the two boards.

Example output
==============

After pressing the push button, a timestamp should be printed in both terminals. The difference
between the two timestamps should be within a few microseconds.
