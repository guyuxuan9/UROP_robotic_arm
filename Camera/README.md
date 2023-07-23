# Color Identification
**Color_Identification.ipynb** shows the code to identify different colours using OpenCV in python and draw a bounding box around the colour. Please refer to the file for details and the key points are listed below:
- Capture a video frame using __cv2.VideoCapture(0)__. The image has the shape (480,640,3) meaning that it has the size $640 \times 480$ and 3 colour channels (RGB)
- **Gaussian Smoothing** is used to filter out gaussian noise
- **LAB** colour space is used instead of RGB because it separates luminance and chrominance, which makes it closer to human perception of color. Equal distances in color values generally correspond to roughly equal perceptual differences.  
- **Morphological Transformations** is used to further remove noises and detect the boundaries.
In particular, two techniques **Opening** and **Closing** are used.
- Get the contour coordinates and draw the bounding box.