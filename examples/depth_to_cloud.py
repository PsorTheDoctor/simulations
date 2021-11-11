import numpy as np
from PIL import Image, ImageOps
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def plot_scatter(xs, ys, zs):

    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter(xs, ys, zs)
    plt.show()


depth = Image.open('depth.jpg')
gray = ImageOps.grayscale(depth)
width, height = depth.size

step = 5
xs = []
ys = []
zs = []
for x in range(0, width, step):
    for y in range(0, height, step):

        z = depth.getpixel((x, y))[0]
        xs.append(x)
        ys.append(y)
        zs.append(z)

plot_scatter(xs, ys, zs)
