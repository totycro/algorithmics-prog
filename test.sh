#!/bin/bash

make -j4

single_model_test() {
	nice ./kmst -f data/g03.dat -m $1 -k 10 -l log.txt -r 10
	nice ./kmst -f data/g03.dat -m $1 -k 25 -l log.txt -r 10
	nice ./kmst -f data/g04.dat -m $1 -k 14 -l log.txt -r 5
	nice ./kmst -f data/g04.dat -m $1 -k 35 -l log.txt -r 5
	nice ./kmst -f data/g05.dat -m $1 -k 20 -l log.txt -r 5
	nice ./kmst -f data/g05.dat -m $1 -k 50 -l log.txt -r 5

	nice ./kmst -f data/g06.dat -m $1 -k 40 -l log.txt
	nice ./kmst -f data/g06.dat -m $1 -k 100 -l log.txt
}

#single_model_test scf ; exit 0

#./kmst -f data/g05.dat -m mtz -k 20 -l log.txt
#exit 1

nice ./kmst -f data/g01.dat -m mtz -k 2 -l log.txt -r 10
nice ./kmst -f data/g01.dat -m scf -k 2 -l log.txt -r 10
nice ./kmst -f data/g01.dat -m mcf -k 2 -l log.txt -r 10
nice ./kmst -f data/g01.dat -m mtz -k 5 -l log.txt -r 10
nice ./kmst -f data/g01.dat -m scf -k 5 -l log.txt -r 10
nice ./kmst -f data/g01.dat -m mcf -k 5 -l log.txt -r 10

nice ./kmst -f data/g02.dat -m mtz -k 4 -l log.txt -r 10
nice ./kmst -f data/g02.dat -m scf -k 4 -l log.txt -r 10
nice ./kmst -f data/g02.dat -m mcf -k 4 -l log.txt -r 10
nice ./kmst -f data/g02.dat -m mtz -k 10 -l log.txt -r 10
nice ./kmst -f data/g02.dat -m scf -k 10 -l log.txt -r 10
nice ./kmst -f data/g02.dat -m mcf -k 10 -l log.txt -r 10

nice ./kmst -f data/g03.dat -m mtz -k 10 -l log.txt -r 10
nice ./kmst -f data/g03.dat -m scf -k 10 -l log.txt -r 10
nice ./kmst -f data/g03.dat -m mcf -k 10 -l log.txt -r 10
nice ./kmst -f data/g03.dat -m mtz -k 25 -l log.txt -r 10
nice ./kmst -f data/g03.dat -m scf -k 25 -l log.txt -r 10
nice ./kmst -f data/g03.dat -m mcf -k 25 -l log.txt -r 10

nice ./kmst -f data/g04.dat -m mtz -k 14 -l log.txt -r 5
nice ./kmst -f data/g04.dat -m scf -k 14 -l log.txt -r 5
nice ./kmst -f data/g04.dat -m mcf -k 14 -l log.txt -r 5
nice ./kmst -f data/g04.dat -m mtz -k 35 -l log.txt -r 5
nice ./kmst -f data/g04.dat -m scf -k 35 -l log.txt -r 5
nice ./kmst -f data/g04.dat -m mcf -k 35 -l log.txt -r 5

nice ./kmst -f data/g05.dat -m mtz -k 20 -l log.txt -r 5
nice ./kmst -f data/g05.dat -m scf -k 20 -l log.txt -r 5
nice ./kmst -f data/g05.dat -m mcf -k 20 -l log.txt -r 5
nice ./kmst -f data/g05.dat -m mtz -k 50 -l log.txt -r 5
nice ./kmst -f data/g05.dat -m scf -k 50 -l log.txt -r 5
nice ./kmst -f data/g05.dat -m mcf -k 50 -l log.txt -r 5

nice ./kmst -f data/g07.dat -m mtz -k 60 -l log.txt
nice ./kmst -f data/g07.dat -m scf -k 60 -l log.txt
nice ./kmst -f data/g07.dat -m mtz -k 150 -l log.txt
nice ./kmst -f data/g07.dat -m scf -k 150 -l log.txt

nice ./kmst -f data/g08.dat -m mtz -k 80 -l log.txt
nice ./kmst -f data/g08.dat -m scf -k 80 -l log.txt
nice ./kmst -f data/g08.dat -m mtz -k 200 -l log.txt
nice ./kmst -f data/g08.dat -m scf -k 200 -l log.txt

nice ./kmst -f data/g06.dat -m mtz -k 40 -l log.txt
nice ./kmst -f data/g06.dat -m scf -k 40 -l log.txt
nice ./kmst -f data/g06.dat -m mcf -k 40 -l log.txt
nice ./kmst -f data/g06.dat -m mtz -k 100 -l log.txt
nice ./kmst -f data/g06.dat -m scf -k 100 -l log.txt
nice ./kmst -f data/g06.dat -m mcf -k 100 -l log.txt

