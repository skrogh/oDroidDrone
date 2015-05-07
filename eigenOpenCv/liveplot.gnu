set term x11

set tmargin 0
set bmargin 0
set lmargin 0
set rmargin

set multiplot layout 4,1

plot "< tail -2000 imudata.csv" using 2 title 'Acc x'
plot "< tail -2000 imudata.csv" using 3 title 'Acc y'
pause 0.25
reread
