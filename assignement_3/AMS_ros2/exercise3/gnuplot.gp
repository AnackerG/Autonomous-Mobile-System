set terminal png size 800,400
set output "histogram.png"

set title "laser scan histogram"
set ylabel "Frequ."
set xlabel "Sums"

binwidth = 1
bin(x,width) = width*floor(x/width)

plot "sums.txt" using (bin($1,binwidth)):(1.0) smooth freq with boxes
