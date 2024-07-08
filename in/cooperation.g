World 	{  X:<[0, 0, 0, 1, 0, 0, 0]> }                         

table_base (World) {
    Q:[0 0 .6]
    shape:marker, size:[.03],
}
table (table_base){
    shape:ssBox, Q:[0 0.05 -.05], size:[2.3 1.44 .05 .02], color:[.3 .3 .3]
    contact, logical:{ }
}

bin1 (table_base){ type:ssBox, size:[0.3 0.3 0.01 .005], contact:0 Q:<[  0.0, 0.3, 0.0, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.0]}
_wall1 (bin1){ type:ssBox, size:[0.05 0.45 0.1 .01], contact:1 Q:<[  -0.25, .0, 0.025, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}
_wall2 (bin1){ type:ssBox, size:[0.05 0.45 0.1 .01], contact:1 Q:<[  0.25, .0, 0.025, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}
_wall3 (bin1){ type:ssBox, size:[0.55 0.05 0.1 .01], contact:1 Q:<[  0.0, .25, 0.025, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}
_wall4 (bin1){ type:ssBox, size:[0.55 0.05 0.1 .01], contact:1 Q:<[  0.0, -.25, 0.025, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}

bin2 (table_base){ type:ssBox, size:[0.3 0.3 0.01 .005], contact:0 Q:<[  0.0, -0.3, 0.0, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.0]}
_wall5 (bin2){ type:ssBox, size:[0.05 0.45 0.1 .01], contact:1 Q:<[  -0.25, .0, 0.025, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}
_wall6 (bin2){ type:ssBox, size:[0.05 0.45 0.1 .01], contact:1 Q:<[  0.25, .0, 0.025, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}
_wall7 (bin2){ type:ssBox, size:[0.55 0.05 0.1 .01], contact:1 Q:<[  0.0, .25, 0.025, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}
_wall8 (bin2){ type:ssBox, size:[0.55 0.05 0.1 .01], contact:1 Q:<[  0.0, -.25, 0.025, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}
_obstacle (bin1){ type:ssBox, size:[0.3 0.001 0.2 .01], contact:1 Q:<[ -0, -0.25, 0.7, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}


#goal1 (bin1){ joint:rigid type:ssBox, size:[0.25 0.2 0.1 .01], contact:0 Q:<[  0, 0.3, 0.3, 0.714, 0.714, .0, 0]> color:[0.4, 1, 1, 0.2]}
goal1 (bin1){ joint:rigid type:ssBox, size:[0.25 0.2 0.1 .01], contact:0 Q:<[  0, 0., 0.03, 1., 0., .0, 0]> color:[0.4, 1, 1, 0.2]}


obj1 (bin2){ joint:rigid type:ssBox, size:[0.25 0.2 0.1 .01], contact:1 Q:<[ -0.07, -0., 0.03, 1, 0, .0, 0]> color:[0.4, 1, 1, 1]}

#obj1 (bin1){ joint:rigid type:ssBox, size:[0.25 0.2 0.1 .01], contact:1 Q:<[  -0.07, -0., 0.03, 1, 0, .0, 0]> color:[0.4, 1, 1, 1]}


#goal1 (bin2){ joint:rigid type:ssBox, size:[0.25 0.2 0.1 .01], contact:0 Q:<[  -0.1, -0.1, 0.3, 0.714, 0.714, .0, 0]> color:[0.4, 1, 1, 0.2]}