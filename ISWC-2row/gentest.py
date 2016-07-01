from PIL import Image
import numpy as np

w, h = 24, 16
data = np.zeros((h, w, 3), dtype=np.uint8)
for i in range(2, 16, 2):
    for j in range(i):
        
        

img = Image.fromarray(data, 'RGB')
img.save('my.gif')
img.show()