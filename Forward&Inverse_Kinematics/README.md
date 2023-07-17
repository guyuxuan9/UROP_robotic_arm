# Model the robotic arm

The following shows the acutal robotic arm. 

![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/78968c38-b390-4825-9674-be5da6f4395b)

It has 6 Degrees-Of-Freedom (DOF) as illustrated in the figure below. All of the 6 joints are revolute (rotary), meaning that it is like a hinge that allows relative rotation between two links.

![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/0ddacc5c-5a10-4a96-a956-ddc97259c69f)

In order to simplify the calculation, we do NOT consider servo 1 (end-effector) and Servo 6 (pan-tilt). Therefore, it becomes a 4-DOF robotic arm. The 3 links are discribed below (also shown in the figure below):
- $l_1$: link between servo 6 and servo 5
- $l_2$: link between servo 5 and servo 4
- $l_3$: link between servo 4 and servo 3
- $l_4$: link between servo 3 and the end effector

The coordinates of the end effector in 3-dimentional space is ($x_p$, $y_p$, $z_p$) and the base frame is $x_0y_0z_0$. The projection of the end effector $P$ onto the $x_0y_0$ plane.

![a1028c346b93e54072199e643151906](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/826cb142-af14-4274-964c-48433ab73657)

According to the figure above, given the position of the end effector ($x_p$, $y_p$, $z_p$), the angle between the paw and the horizontal plane $\alpha$, and lengths of the 4 links $l_1$, $l_2$, $l_3$, $l_4$, the angles $\theta_3$, $\theta_4$ and $\theta_5$ can be solved using geometric methods.

From the figure, the lengths of some segments can be written directly, 
$$\alpha = \theta_3 + \theta_4 + \theta_5$$ $$PD = l_4 sin(\alpha)$$ $$CD = l_4 cos(\alpha)$$ $$OP\_ = \sqrt{x_p^2 + y_p^2}$$ $$AF = AE - FE = OP\_ - CD$$ $$CF = DE = z_p - l_1 - PD$$ $$AC = \sqrt{AF^2 + CF^2}$$

### Solve $\theta_4$
Using cosine rule, $$cos \angle ABC = \frac {l_2^2 + l_3^2 - AC^2}{2l_2l_3}$$ Therefore, $$\theta_4 = 180\degree - \angle ABC $$

### Solve $\theta_5$
Again, using cosine rule, $$cos \angle CAB = \frac{AC^2+l_2^2-l_3^2}{2l_2AC}$$ $$\angle CAF = arctan(\frac{CF}{AF})$$ Therefore, $$\theta_5 = \angle CAF - \angle CAB$$

### Solve $\theta_3$
$$\theta_3 = \alpha - \theta_5 + \theta_4$$
Here, instead of $-\theta_4$, $+ \theta_4$ is used because it is in the negative direction (clockwise).
