set tmargin 0
set bmargin 0
set lmargin 0.2
set rmargin 0
unset xtics

set multiplot layout 4,1

plot "< tail -2000 imudata.csv" using 1 title 'Time'  with lines
plot "< tail -2000 imudata.csv" using 2 title 'Acc x'  with lines
plot "< tail -2000 imudata.csv" using 3 title 'Acc y'  with lines
plot "< tail -2000 imudata.csv" using 4 title 'Acc z'  with lines
pause 0.25
reread
