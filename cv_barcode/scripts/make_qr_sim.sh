#!/bin/bash

### Generates a bunch of QR code objects for use in Gazebo
### Requires qrencode, install with `sudo apt-get install qrencode`
###
### To change the size and stuff of the object, just edit the template object,
### e.g. to change the size find the <size> tags in model.sdf
###
### TODO: Since these are procedurally generated, you could rig up some magic
### to have catkin run this script at buildtime, and avoid committing all the
### files that this script generates to git. But I (John) never got around to 
### testing that, and always just committed everything.

# The folder you want the landmarks to go into
DESTPATH="./"
# The folder to find the template object int
SEARCHPATH="$DESTPATH"
# The template object name
SEARCH="RonnyLM"
# The text that will prepend each QR tag. e.g. if QRNAME="XYZ" then the first tag will be "XYZ01"
# Note that if QRNAME is more than 10 characters, this will bump size up to 25x25 changing the image size,
# so if using 2 digit numbers, QRPREFIX should be 8 or less characters to stay within the V1 size with high error correction.
# See https://www.qrcode.com/en/about/version.html for more details on sizes.
# Also, I (John) think this may be case insensitive, but I'm not certain.
QRPREFIX="FitzB"
# How many tags to make
NUMTAGS=9

for i in `seq 1 ${NUMTAGS}`;
do
  # Generate the QR text we want
	QRNAME=$(printf "${QRPREFIX}%02d" $i)
	echo $QRNAME

  # Make sure we don't already have something with this name
	rm -R -f $DESTPATH/$QRNAME

  # Copy the template to the new folder
	cp -R ${SEARCHPATH}/${SEARCH} $DESTPATH/$QRNAME

	# Delete the old QR code
	rm $DESTPATH/$QRNAME/materials/textures/${SEARCH}.png

	# This generates a 168 px qrcode btw, 21*8=168 plus 2*8*2=200 for a 200px square image, with a 168 px qr in the middle.
  # Note that if QRNAME is more than 10 characters, this will bump size up to 25x25 changing the image size.
  # For best results, the smaller the barcode the better, as it will be readable from further away.
	qrencode  -l H -s 8 -m 2 -o $DESTPATH/$QRNAME/materials/textures/${QRNAME}.png "${QRNAME}"
	#qrencode  -l H -s 128 -m 2 -o $DESTPATH/${QRNAME}.png "${QRNAME}"

  # Rename the material file
	mv $DESTPATH/$QRNAME/materials/scripts/${SEARCH}.material $DESTPATH/$QRNAME/materials/scripts/${QRNAME}.material

  # Search and replace for the name in all the relevant files
	sed -i -e "s/${SEARCH}/${QRNAME}/g" "$DESTPATH/$QRNAME/materials/scripts/${QRNAME}.material"
	sed -i -e "s/${SEARCH}/${QRNAME}/g" "$DESTPATH/$QRNAME/model.config"
	sed -i -e "s/${SEARCH}/${QRNAME}/g" "$DESTPATH/$QRNAME/model.sdf"
done

