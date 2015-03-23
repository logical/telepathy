#!/bin/bash

pcb2gcode --mill-speed 10000 --mill-feed 100 --zwork -0.005 --offset 1 --zsafe 5 --zchange 5 --front telepathy-Front.gtl --back telepathy-Back.gbl #--drill telepathy.drl


