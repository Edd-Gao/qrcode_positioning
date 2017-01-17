from PIL import Image
import qrencode
import os

# parameters
paper = "A0"    # set the paper type to A0,A1,A2,A3,A4
lines = 3   # total lines of qrcodes
rows = 4    # total rows of qrcodes
margin = 50     # margin value in pixels
qrsize = 200    # qrcode size in milimeters

# paper size dictionary, width by height
paper_size = {"A0":[841,1189],
              "A1":[594,841],
              "A2":[420,594],
              "A3":[297,420],
              "A4":[210,297]}

width = lines * qrsize + (lines + 1) * margin   # total width of the generated image
height = rows * qrsize + (rows + 1) * margin    # total height of the generated image

# if the marker board generated is larger than the paper size,abort
if width > paper_size[paper][0] or height > paper_size[paper][1]:
    print "the marker board can not fit in the paper size:" + paper
    os.abort()

result = Image.new("RGB", (width, height), (255, 255, 255))     # the result image
paper_image = Image.new("RGB", (paper_size[paper][0],paper_size[paper][1]), (255, 255, 255))

# generate qrcode and paste into result image
for i in range(rows):
    for j in range(lines):
        coordinate_x = i * (qrsize + margin)
        coordinate_y = j * (qrsize + margin)
        version, rawsize, qrcodeIm = qrencode.encode(str(qrsize) + "mm-" + str(coordinate_x/10) + "," + str(coordinate_y/10))
        qrcodeIm = qrcodeIm.resize((qrsize, qrsize))
        box = (j*qrsize + (j+1)*margin, i*qrsize + (i+1)*margin)
        result.paste(qrcodeIm, box)

# paste the result in paper image for print
box = ((paper_size[paper][0] - width)/2, (paper_size[paper][1] - height)/2)
paper_image.paste(result, box)
paper_image.save("qrcode.png")




