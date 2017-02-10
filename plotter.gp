set datafile separator ","
set title "Left and Right Velocity"
set xlabel "Time"
set ylabel "Velocity"
plot 'profile.csv' using 0:1 with lines title "Left", 'profile.csv' using 0:2 with lines title "Right"
pause -1
