# path_smooth_with_kappa


This is used for smoothing path. SQP optimization model is established in Cartesian coordinate system.

State space:

![image](https://user-images.githubusercontent.com/32810296/201311718-9f37594a-ba9a-4882-88a6-124a1294053d.png)

In this repo, θ is the bias between the angle of reference point and decision variable. k is the curvature.

1. Objective function:

![image](https://user-images.githubusercontent.com/32810296/201308568-f1bac6df-a060-4973-b662-5bcc70f0b984.png)

2. Constraints:

![image](https://user-images.githubusercontent.com/32810296/201308814-58fe1638-e5cc-4847-bce7-8f6e4a537a48.png)

In the kinematic model constrians, Taylor expansion is used based on reference points.  Sin(θ) = θ， cos(θ) = 1.

As shown in figure below, SQP is the smoothing result, where IHA* represents the original path.

![SQP](https://user-images.githubusercontent.com/32810296/201310585-d97a453f-668f-466b-b0f6-3d6bdd1dc5c7.jpg)

