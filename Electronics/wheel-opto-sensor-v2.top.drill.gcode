;(.../Electronics/Eagle/pcb-gcode/pcb-gcode.ulp)
;(Copyright 2005 - 2012 by John Johnson)
;(See readme.txt for licensing terms.)
;(This file generated from the board:)
;(.../Electronics/wheel-opto-sensor-v2.brd)
;(Current profile is .../pcb-gcode/profiles/generic.pp  )
;(This file generated 01/02/2018 15:14)
;(Settings from pcb-machine.h)
;(spindle on time = 1000.0000)
;(spindle speed = 20000.0000)
;(tool change at 0.0000 0.0000 5.0000 )
;(feed rate xy = F0.00  )
;(feed rate z  = F200.00 )
;(Z Axis Settings)
;(  High     Up        Down     Drill)
;(5.0000 	2.0000 	-0.2000 	-2.0000 )
;(Settings from pcb-defaults.h)
;(isolate min = 0.0250)
;(isolate max = 0.5000)
;(isolate step = 0.0800)
;(Generated top outlines, top drill, )
;(Unit of measure: mm)
;( Tool|       Size       |  Min Sub |  Max Sub |   Count )
;( T01  0.350mm 0.0138in 0.0000in 0.0000in )
;( T02  0.800mm 0.0315in 0.0000in 0.0000in )
;( T03  1.016mm 0.0400in 0.0000in 0.0000in )
;( T04  1.800mm 0.0709in 0.0000in 0.0000in )
;(Metric Mode)
G21
;(Absolute Coordinates)
G90
;S20000
G00 Z5.0000  F3000
G00 X0.0000 Y0.0000  F3000
;M03 S1
G04 P1000.000000
;M03 S0
;M06 T01  ; 0.3500 
G01 Z0.0000 F200.00 
;M06 
G00 Z2.0000  F3000
;M03 S1
G04 P1000.000000
G00 X8.2550 Y14.9225  F3000
G01 Z-2.0000 F200.00 
G00 Z2.0000  F3000
G00 X13.6525 Y14.6050  F3000
G01 Z-2.0000 F200.00 
G00 Z2.0000  F3000
G00 X9.5250 Y6.9850  F3000
G01 Z-2.0000 F200.00 
G00 Z2.0000  F3000
G00 X13.6525 Y2.2225  F3000
G01 Z-2.0000 F200.00 
G00 Z2.0000  F3000
;M03 S0
G00 Z5.0000  F3000
G00 X0.0000 Y0.0000  F3000
;M06 T02  ; 0.8000 
G01 Z0.0000 F200.00 
;M06 
G00 Z2.0000  F3000
;M03 S1
G04 P1000.000000
G00 X18.4950 Y7.2725  F3000
G01 Z-2.0000 F200.00 
G00 Z2.0000  F3000
G00 X18.4950 Y9.8725  F3000
G01 Z-2.0000 F200.00 
G00 Z2.0000  F3000
G00 X28.4950 Y9.8725  F3000
G01 Z-2.0000 F200.00 
G00 Z2.0000  F3000
G00 X28.4950 Y7.2725  F3000
G01 Z-2.0000 F200.00 
G00 Z2.0000  F3000
;M03 S0
G00 Z5.0000  F3000
G00 X0.0000 Y0.0000  F3000
;M06 T03  ; 1.0160 
G01 Z0.0000 F200.00 
;M06 
G00 Z2.0000  F3000
;M03 S1
G04 P1000.000000
G00 X2.5400 Y5.7150  F3000
G01 Z-2.0000 F200.00 
G00 Z2.0000  F3000
G00 X2.5400 Y8.2550  F3000
G01 Z-2.0000 F200.00 
G00 Z2.0000  F3000
G00 X2.5400 Y10.7950  F3000
G01 Z-2.0000 F200.00 
G00 Z2.0000  F3000
;M03 S0
G00 Z5.0000  F3000
G00 X0.0000 Y0.0000  F3000
;M06 T04  ; 1.8000 
G01 Z0.0000 F200.00 
;M06 
G00 Z2.0000  F3000
;M03 S1
G04 P1000.000000
G00 X18.0975 Y2.5400  F3000
G01 Z-2.0000 F200.00 
G00 Z2.0000  F3000
G00 X18.0975 Y13.9700  F3000
G01 Z-2.0000 F200.00 
G00 Z2.0000  F3000
;T01 
G00 Z5.0000  F3000
;M03 S0
M0
