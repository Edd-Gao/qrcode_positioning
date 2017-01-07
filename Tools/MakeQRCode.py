from PIL import Image
import qrencode

lines = 10
rows = 10
margin = 50
size = (200, 200)
qrsize = "20cm"

width = lines * size[0] + (lines + 1) * margin
height = rows * size[1] + (rows + 1) * margin

result = Image.new("RGB", (width, height), (255, 255, 255))

for i in range(rows):
    for j in range(lines):
        version, rawsize, qrcodeIm = qrencode.encode(qrsize + "-" + str(i) + str(j))
        qrcodeIm = qrcodeIm.resize(size)
        box = (j*size[0] + (j+1)*margin, i*size[1] + (i+1)*margin)
        result.paste(qrcodeIm, box)

result.save("qrcode.png")




