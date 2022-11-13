# path_smooth_with_kappa


This is used for smoothing path. SQP optimization model is established in Cartesian coordinate system.

State space:

![image](https://user-images.githubusercontent.com/32810296/201311718-9f37594a-ba9a-4882-88a6-124a1294053d.png)

In this repo, θ is the bias between the angle of reference point and decision variable. k is the curvature. In this model, speed is considered as an constant value, is therefore replaced by a constant distance ds.

1. Objective function:

![image](https://user-images.githubusercontent.com/32810296/201517261-563861c4-a74c-4097-9235-780cb34c47f3.png)

2. Constraints:

![image](https://user-images.githubusercontent.com/32810296/201517312-3381dbeb-fc1d-4a41-803f-4690a801f5c7.png)

In the kinematic model constrians, Taylor expansion is used based on reference points. 

As shown in figure below, SQP is the smoothing result, where IHA* represents the original path.

![SQP](https://user-images.githubusercontent.com/32810296/201310585-d97a453f-668f-466b-b0f6-3d6bdd1dc5c7.jpg)

