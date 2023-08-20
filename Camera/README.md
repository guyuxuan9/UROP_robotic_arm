# Color Identification
**Color_Identification.ipynb** shows the code to identify different colours using OpenCV in python and draw a bounding box around the colour. Please refer to the file for details and the key points are listed below:
- Capture a video frame using __cv2.VideoCapture(0)__. The image has the shape (480,640,3) meaning that it has the size $640 \times 480$ and 3 colour channels (RGB)
- **Gaussian Smoothing** is used to filter out gaussian noise
- **LAB** colour space is used instead of RGB because it separates luminance and chrominance, which makes it closer to human perception of color. Equal distances in color values generally correspond to roughly equal perceptual differences.  
- **Morphological Transformations** is used to further remove noises and detect the boundaries.
In particular, two techniques **Opening** and **Closing** are used.
- Get the contour coordinates and draw the bounding box.
- To see the Color Identification effect from the camera, please run **ColorCoordinate.py** in raspberry pi
    ```
    python3 ColorCoordinate.py
    ```


# Color Tracking

https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/ed90fef1-d914-4d23-9ec1-9ac7635570ad

**Color_Tracking.ipynb** shows the important part of the code to make the robotic arm track the coloured object. Please refer to the file for key points and some key points are listed here: 
- Compared to **Color_Identification.ipynb** which uses trial and error to determine the LAB range for each colour, **Color_Tracking.ipynb** proposes a more systematic way. Since the light condition and position of the camera might affect colour detection threshold, a picture is taken first and the blue part is extracted. **cv2.split()** is used to get L, A, B values and numpy is used to find the max and min value of each component. This gives the range of L, A and B value in the current environment.

E.g. 

![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/efca7e5a-d85a-4d9d-900b-245ca6a744d3)

- Follow similar image processing techniques mentioned in **Color_Identification.ipynb** and draw the bounding box and minimum enclosing circle.
- Three **PID controllers** are used to track the object.They are used to control x, y , and z position of the robotic arm respectively. Please refer to [Color_Tracking.ipynb](https://github.com/guyuxuan9/UROP_robotic_arm/blob/main/Camera/Color_Tracking.ipynb) for details.

![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/0f46046e-3612-4351-a31c-1b7c18a9fc5a)

**x_pid**:

![55c41d88218b285b01990533fc2f970](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/71549ebd-265b-4964-96a7-b01b72d1e8b2)

The aim of this PID controller is to align the object at the centre of the x-axis in the image window. It directly controls the servo angle of motor 6 since motor 6 is the only motor that can change the x position of the object in the image window.

**y_pid**:

![dffb8bbb5cf2e2d0c674889f27cb64d](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/1c131a4e-8722-4218-a77a-f6689bae93ad)

The goal of this PID controller is to maintain a consistent distance between the object and the camera. Since directly measuring the distance with a camera can be challenging, the controller utilizes the object's area in the image window as an indicative measure of the distance.

**z_pid**:

![0367e1b250c1cfc4ef9bdda46b77fb5](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/d8b1e7f7-07db-4de5-bbd3-9affdaf0f4b5)

The objective of this controller is to center the object along the y-axis within the image window. It's important to note that the y-axis within the image window corresponds to the z-axis in the reference frame of the robotic arm.

- Note that the robotic arm will stop moving when the calculated angle from inverse kinematics is out of the given range. Normally, this happens when the object moves is "too high" for the camera. To make the robotic arm behave normally again, simply place the object at a high position and move downwards slowly and it will track the object successfully. 


**Video:**

[![Color Tracking using robotic arm](https://user-images.githubusercontent.com/58468284/257811600-6ce9408b-ddb9-4418-8195-19763f5aa868.png)](https://youtu.be/yXHFDTK_ZeQ)
