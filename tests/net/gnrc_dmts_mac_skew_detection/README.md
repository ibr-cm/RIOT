DMTS MAC test application
======================
This application serves as a demonstration for evaluating the synchronization with skew detection
capabilities achieved by the DMTS MAC protocol.

Usage
=====

To use this test application you will need two boards with a STM32 MCU and an AT86RF215 connected
to it (or on board).
The STM32 MCU you are using has to support the "Smooth Calibration" feature! You can check that
in the reference manual of the corresponding STM32 MCU.

One of the boards can be flashed with the following parameters:
```
make flash BOARD=your_board DMTS_INTERVAL=1 term
```
The other one has to be flashed with additional parameters.
```
make flash BOARD=your_board DMTS_INTERVAL=1 DMTS_MASTER=1 term
```

Then connect a simple push button to GND and the specified pins of the two boards.

Example output
==============

After pressing the push button, a timestamp should be printed in both terminals. The time disparity
between the two timestamps should be minimal, typically within a few microseconds. Even with the
application running for several minutes, synchronization between the two nodes should remain within
a few microseconds. Note that the sending stops after a couple of minutes and the synchronization
is maintained through minimizing the skew.

If you want to use a bigger synchronization interval than one second, you can choose between
32 and 64 seconds.
