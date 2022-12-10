#!/bin/bash

/home/cwinter/openocd/src/openocd -s /home/cwinter/opentherm -f interface/picoprobe.cfg -f target/rp2040.cfg -s /home/cwinter/openocd/tcl -c "init; reset run; exit"
