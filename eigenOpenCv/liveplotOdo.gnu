set tmargin 2
set bmargin 2
set lmargin 2
set rmargin 2
set size square

plot "< tail -2000 log.csv" using 5:6 title 'position'  with lines

pause 0.25
reread
