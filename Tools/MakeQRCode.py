from PIL import Image
import qrencode


#parameters
lines = 10  #total lines of qrcodes
rows = 10   #total rows of qrcodes
margin = 50 #margin value in pixels
qrsize = 200 # qrcode size in milimeters

width = lines * qrsize + (lines + 1) * margin   #total width of the generated image
height = rows * qrsize + (rows + 1) * margin    #total height of the generated image

result = Image.new("RGB", (width, height), (255, 255, 255)) #the result image

for i in range(rows):
    for j in range(lines):
        coordinate_x = i * (qrsize + margin)
        coordinate_y = j * (qrsize + margin)
        version, rawsize, qrcodeIm = qrencode.encode(str(qrsize) + "mm-" + str(coordinate_x/10) + "," + str(coordinate_y/10))
        qrcodeIm = qrcodeIm.resize((qrsize, qrsize))
        box = (j*qrsize + (j+1)*margin, i*qrsize + (i+1)*margin)
        result.paste(qrcodeIm, box)

result.save("qrcode.png")




