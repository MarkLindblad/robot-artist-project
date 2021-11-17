import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.collections import PatchCollection
from matplotlib import colors
from matplotlib.collections import cm
import shapely.geometry as sg
import descartes
import random
from scipy import ndimage

def crop(img):
    if img.shape[0] > img.shape[1]:
        img = img[(img.shape[0] - img.shape[1])//2:-(img.shape[0] - img.shape[1])//2, :]
    else:
        img = img[:, (img.shape[1] - img.shape[0])//2:-(img.shape[1] - img.shape[0])//2]
    
    return img

def contrast(img, black, white):
    img = ((255)/(white - black))*(img - black)
    img = np.clip(img, 0, 255)

    return img

def rotate(img, angle):
    angle %= 90
    
    if angle != 90:
        border = np.zeros((int(1.5*img.shape[0]), int(1.5*img.shape[1])))
        border[img.shape[0] // 4: -(img.shape[0] - 1) // 4, img.shape[1] // 4: -(img.shape[1] - 1) // 4] = img
        img = border

        center = tuple(np.array(img.shape[1::-1]) / 2)
        rot_mat = cv2.getRotationMatrix2D(center, angle, 1.0)
        img = cv2.warpAffine(img, rot_mat, img.shape[1::-1], flags = cv2.INTER_LINEAR)

    return img

def dots_union(x, y, thickness):
    if len(x) == 1:
        return sg.Point()
    if len(x) == 1:
        return sg.Point(x[0], y[0]).buffer(thickness)
    elif len(x) == 2:
        return (sg.Point(x[0], y[0]).buffer(thickness)).union(sg.Point(x[1], y[1]).buffer(thickness))
    else:
        return (dots_union(x[:len(x)//2], y[:len(x)//2], thickness)).union(dots_union(x[len(x)//2:], y[len(x)//2:], thickness))

def grey_dither(img, size = 50, black = 0, white = 255, scatter = 0, off_x = 0, off_y = 0, angle = 0):

    img = crop(img)

    img = cv2.resize(img, (size, size))

    img = contrast(img, black, white)

    angle = 90 - angle

    img = rotate(img, angle)

    for y in range(img.shape[0]):
        for x in range(img.shape[1]):
            oldpixel = img[y, x]
            newpixel = 255*(round(oldpixel/255))
            img[y, x] = newpixel
            error = oldpixel - newpixel

            if y < img.shape[0] - 1:
                img[y + 1][x    ] = img[y + 1][x    ] + 7*error/16
            
            if y > 0 and x < img.shape[1] - 1:
                img[y - 1][x + 1] = img[y - 1][x + 1] + 3*error/16
            
            if x < img.shape[1] - 1:
                img[y    ][x + 1] = img[y    ][x + 1] + 5*error/16

            if y < img.shape[0] - 1 and x < img.shape[1] - 1:
                img[y + 1][x + 1] = img[y + 1][x + 1] + 1*error/16

    bits = np.vectorize(round)(img/255)
    
    dots_x = np.array([x + random.uniform(-scatter, scatter) + off_x for y in range(bits.shape[0]) for x in range(bits.shape[1]) if bits[y, x] == 0])
    dots_y = np.array([-(y + random.uniform(-scatter, scatter) + off_y) for y in range(bits.shape[0]) for x in range(bits.shape[1]) if bits[y, x] == 0])

    for b in range(len(dots_x)):
        dots_x[b] = dots_x[b] - np.max(dots_x) / 2
        dots_y[b] = dots_y[b] - np.min(dots_y) / 2

    if angle != 90:
        for b in range(len(dots_x)):
            x, y = dots_x[b], dots_y[b]

            back = -(angle)*np.pi/180

            dots_x[b] = np.cos(back)*x - np.sin(back)*y
            dots_y[b] = np.cos(back)*y + np.sin(back)*x
    
    b = 0
    while b < len(dots_x):
        if dots_x[b] > size/2 - 2 or dots_x[b] < -size/2 + 1 or dots_y[b] > size/2 - 2 or dots_y[b] < -size/2 + 1:
            dots_x = np.delete(dots_x, b)
            dots_y = np.delete(dots_y, b)
        else:
            b += 1

    return dots_x, dots_y, size

def grey_dither_display(dither, thickness = 0.7):
    dots_x, dots_y, size = dither

    fig, ax = plt.subplots(figsize = (5, 5))

    dots_k = dots_union(dots_x, dots_y, thickness)
    ax.add_patch(descartes.PolygonPatch(dots_k, fc = 'k', lw = 0))
    ax.axis([-size/2, size/2, -size/2, size/2])
    ax.axis('off')
    
    print(f'Number of Black dots:   {len(dots_x)}')
    print('\n')


    plt.show()

def color_dither(img, size = 50, black = 0, white = 255, scatter = 0):
    R = img[:, :, 2]
    G = img[:, :, 1]
    B = img[:, :, 0]
    C = np.zeros(R.shape)
    M = np.zeros(R.shape)
    Y = np.zeros(R.shape)
    K = np.zeros(R.shape)

    for y in range(R.shape[0]):
        for x in range(R.shape[1]):
            K[y, x] = 255 - max(R[y, x], G[y, x], B[y, x])
            if K[y, x] == 255:
                C[y, x] = 0
                M[y, x] = 0
                Y[y, x] = 0
            else:
                C[y, x] = 255*(255 - R[y, x] - K[y, x])/(255 - K[y, x])
                M[y, x] = 255*(255 - G[y, x] - K[y, x])/(255 - K[y, x])
                Y[y, x] = 255*(255 - B[y, x] - K[y, x])/(255 - K[y, x])

    c_x, c_y, _ = grey_dither(255 - C, black = black, white = white, angle = 0, size = size)
    m_x, m_y, _ = grey_dither(255 - M, black = black, white = white, angle = 30, size = size)
    y_x, y_y, _ = grey_dither(255 - Y, black = black, white = white, angle = 45, size = size)
    k_x, k_y, _ = grey_dither(255 - K, black = black, white = white, angle = 60, size = size)

    print(f'Number of Cyan dots:    {len(c_x)}')
    print(f'Number of Magenta dots: {len(m_x)}')
    print(f'Number of Yellow dots:  {len(y_x)}')
    print(f'Number of Black dots:   {len(k_x)}')
    print(f'Number of Total dots:   {len(c_x) + len(m_x) + len(y_x) + len(k_x)}')
    print('\n')

    return c_x, c_y, m_x, m_y, y_x, y_y, k_x, k_y, size

def color_dither_display(dither, thickness = 0.7):
    c_x, c_y, m_x, m_y, y_x, y_y, k_x, k_y, size = dither

    fig, ax = plt.subplots(figsize = (5, 5))

    dots_c = dots_union(c_x, c_y, thickness)
    dots_m = dots_union(m_x, m_y, thickness)
    dots_y = dots_union(y_x, y_y, thickness)
    dots_k = dots_union(k_x, k_y, thickness)

    dots_r = dots_m.intersection(dots_y)
    dots_g = dots_y.intersection(dots_c)
    dots_b = dots_c.intersection(dots_m)

    ax.add_patch(descartes.PolygonPatch(dots_c, fc = 'cyan', lw = 0))
    ax.add_patch(descartes.PolygonPatch(dots_m, fc = 'magenta', lw = 0))
    ax.add_patch(descartes.PolygonPatch(dots_y, fc = 'yellow', lw = 0))

    try:
        ax.add_patch(descartes.PolygonPatch(dots_r, fc = 'r', lw = 0))
    except:
        pass
    try:
        ax.add_patch(descartes.PolygonPatch(dots_g, fc = 'g', lw = 0))
    except:
        pass    
    try:
        ax.add_patch(descartes.PolygonPatch(dots_b, fc = 'b', lw = 0))
    except:
        pass

    ax.add_patch(descartes.PolygonPatch(dots_k, fc = 'k', lw = 0))

    ax.axis([-size/2, size/2, -size/2, size/2])
    ax.axis('off')

    plt.show()

if __name__ == "__main__":
    size = 50

    img = np.array(cv2.imread('cal.png', 0))
    grey_dither_display(grey_dither(img, angle = 30, size = size))

    img = np.array(cv2.imread('cal.png'))
    color_dither_display(color_dither(img, size = size))