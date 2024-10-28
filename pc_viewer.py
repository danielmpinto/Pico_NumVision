import numpy as np
from PIL import Image
from matplotlib import pyplot as plt
import serial

ser = serial.Serial('/dev/ttyACM1')

fig = plt.figure(figsize=(1, 2))
viewer = fig.add_subplot(111)
plt.ion()  # Ativa o modo interativo
fig.show()  # Mostra a figura inicialmente

while True:
    data = ser.readline().decode("utf-8").strip()  # Remove espaços em branco e novas linhas
    if data[0] != '[':
        continue

    # Extrai o cabeçalho e os dados
    header = data[:15]
    img = data[16:]

    # Verifica se os dados da imagem terminam com '$' e remove
    if img.endswith('$'):
        img = img[:-1]  # Remove o caractere '$'

    bitmap = list(map(int, img.split()))
    print(header)
    xdim = int(header[1:4])
    ydim = int(header[5:8])
    print("{}x{}".format(xdim, ydim))
    
    # Gera a imagem em escala de cinza
    im = Image.new("L", (xdim, ydim))
    im.putdata(bitmap)
    viewer.clear()  # Limpa a imagem anterior
    viewer.imshow(im, cmap='gray', vmin=0, vmax=255)  # Carrega a nova imagem
    plt.pause(0.1)  # Atraso em segundos
    fig.canvas.draw()  # Desenha a imagem na tela
