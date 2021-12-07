import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import cm

def crop(img):
    if img.shape[0] > img.shape[1]:
        img = img[(img.shape[0] - img.shape[1])//2:-(img.shape[0] - img.shape[1])//2, :]
    elif img.shape[0] < img.shape[1]:
        img = img[:, (img.shape[1] - img.shape[0])//2:-(img.shape[1] - img.shape[0])//2]
    
    return img

def hatch(img, threshhold = 0.5, angle = 45, number = 30, cropped = False):
    if not cropped:
        crop(img)

    spacing = img.shape[0]/(number + 2)
    m = np.tan(angle*np.pi/180)
    dy = abs(spacing/np.cos(angle*np.pi/180))
    lines = np.array([0, 0, 0, 0])

    done = False
    i = 1
    while not done:
        if m > 0:
            b = img.shape[0] - i*dy
        else:
            b = i*dy

        done = True
        drawing = False
        
        for x in np.arange(0, img.shape[1] - 0.5, 0.5):
            y = m*x + b
            px = round(x)
            py = -(round(y) + 1)

            if py < 0 and py > -(img.shape[0] + 1):
                done = False

                if not drawing and img[py, px] <= threshhold*255:
                    drawing = True
                    x1, y1 = x/img.shape[1], y/img.shape[0]
                elif drawing and img[py, px] > threshhold*255:
                    drawing = False
                    lines = np.vstack((lines, [x1, x/img.shape[1], y1, y/img.shape[0]]))
        
        if drawing:
            drawing = False
            lines = np.vstack((lines, [x1, x/img.shape[1], y1, y/img.shape[0]]))    

        i += 1

    return lines

def crosshatch(img, layers = 10, blacks = 0, whites = 1, brightness = 0, number = 30, angle = 35):
    img = crop(img)

    lines = np.array([0, 0, 0, 0])
    
    for i in range(layers):
        threshhold = (i/(layers - 1))*(whites - blacks) + blacks + brightness

        new = hatch(img, threshhold = threshhold, angle = angle + 70*i, number = number, cropped = True)
        lines = np.vstack((lines, new))

    lines = lines[~np.all(lines == 0, axis = 1)]
    print('Number of lines: ', lines.shape[0])

    return lines

def preview(lines, lw = 1):
    fig, ax = plt.subplots(1, 2, figsize = (11, 5))

    ax[0].imshow(crop(img), cmap = 'gray')
    ax[0].axis('off')

    for l in lines:
        ax[1].plot(l[:2], l[2:], 'k-', lw = 1)
    ax[1].axis([0, 1, 0, 1])
    ax[1].axis('off')

    plt.show()


def cmyk_crosshatch(img, layers = 10, blacks = 0, whites = 1, number = 30, angle = 35):
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
    
    clines = crosshatch(255 - C, layers = layers, blacks = blacks, whites = whites, number = number, angle = angle)
    mlines = crosshatch(255 - M, layers = layers, blacks = blacks, whites = whites, number = number, angle = angle + 45)
    ylines = crosshatch(255 - Y, layers = layers, blacks = blacks, whites = whites, number = number, angle = angle + 90)
    klines = crosshatch(255 - K, layers = layers, blacks = blacks, whites = whites, number = number, angle = angle + 135)

    print('Number of lines: ', clines.shape[0] + mlines.shape[0] + ylines.shape[0] + klines.shape[0])

    return clines, mlines, ylines, klines

def cmyk_preview(clines, mlines, ylines, klines, lw = 2):
    fig, ax = plt.subplots(1, 2, figsize = (11, 5))

    ax[0].imshow(cv2.cvtColor(crop(img), cv2.COLOR_BGR2RGB))
    ax[0].axis('off')

    for l in clines:
        ax[1].plot(l[:2], l[2:], color = 'cyan', lw = lw)
    for l in ylines:
        ax[1].plot(l[:2], l[2:], color = 'yellow', lw = lw)
    for l in mlines:
        ax[1].plot(l[:2], l[2:], color = 'magenta', lw = lw)
    for l in klines:
        ax[1].plot(l[:2], l[2:], 'k-', lw = lw)
    ax[1].axis([0, 1, 0, 1])
    ax[1].axis('off')

    plt.show()

def levels(img):
    fig, ax = plt.subplots(figsize = (5, 5))
    ax.hist(crop(img)/255, 20, facecolor = 'k')

    plt.show()

if __name__ == "__main__":
    img = np.array(cv2.imread('ball2.jpg', 0))
    lines = crosshatch(img, blacks = 0, whites = 0.9, layers = 7, number = 60)
    preview(lines)

    img = np.array(cv2.imread('steve.jpg', 0))
    lines = crosshatch(img, blacks = 0, whites = 0.9, layers = 7, number = 60)
    preview(lines)

    img = np.array(cv2.imread('lady.jpg', 0))
    lines = crosshatch(img, blacks = 0, whites = 0.85, layers = 7, number = 60)
    preview(lines)

    img = np.array(cv2.imread('man.png'))
    clines, mlines, ylines, klines = cmyk_crosshatch(img, blacks = 0, whites = 0.9, layers = 5, number = 30)
    cmyk_preview(clines, mlines, ylines, klines)

    img = np.array(cv2.imread('starry.png'))
    clines, mlines, ylines, klines = cmyk_crosshatch(img, blacks = 0, whites = 0.9, layers = 5, number = 30)
    cmyk_preview(clines, mlines, ylines, klines)