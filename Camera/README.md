# Color Identification
**Color_Identification.ipynb** shows the code to identify different colours using OpenCV in python and draw a bounding box around the colour. Please refer to the file for details and the key points are listed below:
- Capture a video frame using __cv2.VideoCapture(0)__. The image has the shape (480,640,3) meaning that it has the size $640 \times 480$ and 3 colour channels (RGB)
- **Gaussian Smoothing** is used to filter out gaussian noise
- **LAB** colour space is used instead of RGB because it separates luminance and chrominance, which makes it closer to human perception of color. Equal distances in color values generally correspond to roughly equal perceptual differences.  
- **Morphological Transformations** is used to further remove noises and detect the boundaries.
In particular, two techniques **Opening** and **Closing** are used.
- Get the contour coordinates and draw the bounding box.


# Color Tracking

https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/ed90fef1-d914-4d23-9ec1-9ac7635570ad

**Color_Tracking.ipynb** shows the important part of the code to make the robotic arm track the coloured object. Please refer to the file for key points and some key points are listed here: 
- Compared to **Color_Identification.ipynb** which uses trial and error to determine the LAB range for each colour, **Color_Tracking.ipynb** proposes a more systematic way. Since the light condition and position of the camera might affect colour detection threshold, a picture is taken first and the blue part is extracted. **cv2.split()** is used to get L, A, B values and numpy is used to find the max and min value of each component. This gives the range of L, A and B value in the current environment.

E.g. 

![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/efca7e5a-d85a-4d9d-900b-245ca6a744d3)

- Follow similar image processing techniques mentioned in **Color_Identification.ipynb** and draw the bounding box and minimum enclosing circle.
- Three **PID controllers** are used to track the object.They are used to control x, y , and z position of the robotic arm respectively. Please refer to [Color_Tracking.ipynb](https://github.com/guyuxuan9/UROP_robotic_arm/blob/main/Camera/Color_Tracking.ipynb) for details.

![7a6ed6d79620f30066b6d5e61e47cb9](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/585e4179-ebe4-4afc-8aab-3482d3260578)


**Video:**

[![Color Tracking using robotic arm](https://img.youtube.com/vi/W-es5zbS3Cg/2.jpg)](https://www.youtube.com/watch?v=W-es5zbS3Cg)
