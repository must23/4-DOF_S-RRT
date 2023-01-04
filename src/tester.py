import random

from PIL import Image
from numpy import asarray
import matplotlib.pyplot as plt


# load the image and convert into
# numpy array
img = Image.open('map.png')
numpydata = asarray(img)

def get_slope(x,y):
    slope=numpydata[int(y),int(x),0]
    return (slope)