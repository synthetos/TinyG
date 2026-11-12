(host input gaps: the host stops sending mid chain [bench only; host replay sees no gap])
G21 G90 G94 G17 G64
G92 X0 Y0 Z0
G1 X10 F1500
G1 X20
;HOST-PAUSE 1.5
G1 X30
G1 X40
;HOST-PAUSE 0.5
G1 X0
M2
