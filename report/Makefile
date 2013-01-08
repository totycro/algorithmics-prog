TEXFILE="report.tex"
PDFFILE="report.pdf"
BASENAME="report"

all: pdf

pdf:
	pdflatex ${TEXFILE} >/tmp/pdflatexer &

pdf2:
	#latex ${TEXFILE}
	#dvips report.dvi
	#ps2pdf report.ps


spell:
	aspell -l en -t -c ${TEXFILE}

v: view

view:
	okular ${PDFFILE} 2>/dev/null 1>&2 &

e: edit

edit:
	vim ${TEXFILE}

bib:
	bibtex ${BASENAME}

