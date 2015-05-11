set tmargin 0
set bmargin 0
set lmargin 0.2
set rmargin 0
unset xtics

set multiplot layout 2,1

plot "< tail -2000 logMain.csv" using 1:6  title 'pos x'  with lines , \
plot "< tail -2000 logMain.csv" using 1:7  title 'pos y'  with lines , \
plot "< tail -2000 logMain.csv" using 1:8  title 'pos z'  with lines
plot "< tail -2000 logMain.csv" using 1:9  title 'vel x'  with lines , \
plot "< tail -2000 logMain.csv" using 1:10 title 'vel y'  with lines , \
plot "< tail -2000 logMain.csv" using 1:11 title 'vel z'  with lines
