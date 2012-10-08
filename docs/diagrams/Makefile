
DIAGRAMS = costas_loop.tex \
					 2nd_order_dpll.tex \
					 2nd_order_loop_filter.tex \
					 wgsecef2azel.tex

PDFS = $(DIAGRAMS:.tex=.pdf)

PNGS = $(PDFS:.pdf=.png)

all: $(PNGS)

%.pdf: %.tex
	pdflatex $<

%.png: %.pdf
	 convert -density 120 -trim -bordercolor White -border 5 $< $@

clean:
	rm *.aux *.log *.pdf *.png
