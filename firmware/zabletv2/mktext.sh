#!/bin/bash

if [ $# -lt 1 ]; then
	echo "Usage: $0 \"text...\" [outfile.png]"
	exit 1
fi

if [ $# -ge 2 ]; then
	outfile="$2"
else
	outfile="mktext-generated-image.png"
fi

echo "$1" | convert \
	-size 256x32 \
	xc:white \
	-box white \
	-pointsize 10 \
	-font Bitstream-Vera-Sans-Roman \
	-gravity center \
	-annotate 0 "@-" \
	-trim \
	+repage \
	-monochrome \
	-rotate 90 \
	"$outfile"

[ $? -ne 0 ] && exit 1

# Check the output image to ensure 8 pixel width
FI=( $(file "$outfile" | cut -d',' -f2 | sed -e's/x//') )
if [ ${FI[0]} -ne 8 ]; then
	echo "Error: image width not 8 (but ${FI[0]})"
	exit 1
fi

if [ ${FI[1]} -gt 59 ]; then
	echo "Warning: image length larger than 59 (== ${FI[1]})"
fi

exit 0
