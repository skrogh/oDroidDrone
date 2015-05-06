set term x11
plot "< tail -2000 imudata.csv" using 2 title 'time'
pause 0.25
reread
