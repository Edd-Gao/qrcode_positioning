#!/usr/bin/env python
# coding: u8

import zbar
import Image
import cv2

# create a reader
import time

scanner = zbar.ImageScanner()

# configure the reader
scanner.parse_config('disable')
scanner.parse_config('qrcode.enable')


# obtain image data
#pil = Image.open('./640/1.jpg').convert('L')

img = cv2.imread('/home/edward/workspace/qrcode_positioning/resources/10.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
height, width = gray.shape

start = time.clock()
# wrap image data
image = zbar.Image(width, height, 'Y800', gray.tostring())

# scan the image for barcodes
scanner.scan(image)
stop = time.clock()
print str((stop - start) * 1000) + "ms"

# extract results
for symbol in image:
    # do something useful with results
    print symbol.type, 'symbol:\n%s' % symbol.data

# clean up
del(image)