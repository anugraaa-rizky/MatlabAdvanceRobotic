# MatlabAdvanceRobotic
Lead/follow robotic simulation w Mathlab App (feel free to use and no need to add any citations)

🚀 3D Leader–Follower Multi-Robot Navigation System
📌 Overview

This project implements a multi-robot leader–follower system in a 3D visualization environment using:

PID-based heading control (leader)

Artificial Potential Field (APF) for obstacle avoidance

Formation control (V-shape ↔ single-file switching)

Inter-robot collision avoidance

The system follows a hybrid reactive control architecture, meaning all decisions are made locally without global path planning.

🧠 1. System Model (Kinematics)

Each robot follows a unicycle kinematic model:

𝑥
𝑘
+
1
=
𝑥
𝑘
+
𝑣
cos
⁡
(
𝜃
𝑘
)
⋅
𝑑
𝑡
x
k+1
	​

=x
k
	​

+vcos(θ
k
	​

)⋅dt
𝑦
𝑘
+
1
=
𝑦
𝑘
+
𝑣
sin
⁡
(
𝜃
𝑘
)
⋅
𝑑
𝑡
y
k+1
	​

=y
k
	​

+vsin(θ
k
	​

)⋅dt
𝜃
𝑘
+
1
=
𝜃
𝑘
+
𝜔
⋅
𝑑
𝑡
θ
k+1
	​

=θ
k
	​

+ω⋅dt

Where:

𝑥
,
𝑦
x,y → position (m)

𝜃
θ → heading angle (rad)

𝑣
v → linear velocity (m/s)

𝜔
ω → angular velocity (rad/s)

🎯 2. Leader Navigation Strategy
2.1 Waypoint Tracking

The leader navigates using predefined waypoints:

𝑑
⃗
=
(
𝑥
𝑤
𝑝
−
𝑥
,
  
𝑦
𝑤
𝑝
−
𝑦
)
∥
⋅
∥
d
=
∥⋅∥
(x
wp
	​

−x,y
wp
	​

−y)
	​


Velocity direction:

𝑣
𝑥
=
𝑑
𝑥
⋅
𝑉
,
𝑣
𝑦
=
𝑑
𝑦
⋅
𝑉
v
x
	​

=d
x
	​

⋅V,v
y
	​

=d
y
	​

⋅V

Waypoint switching condition:

∥
(
𝑥
,
𝑦
)
−
(
𝑥
𝑤
𝑝
,
𝑦
𝑤
𝑝
)
∥
<
𝜖
∥(x,y)−(x
wp
	​

,y
wp
	​

)∥<ϵ
2.2 PID Heading Control

The leader uses a PID controller to align its heading:

Heading target:
𝜃
𝑡
𝑎
𝑟
𝑔
𝑒
𝑡
=
atan2
(
𝑣
𝑦
,
𝑣
𝑥
)
θ
target
	​

=atan2(v
y
	​

,v
x
	​

)
Angular error:
𝑒
=
atan2
(
sin
⁡
(
𝜃
𝑡
𝑎
𝑟
𝑔
𝑒
𝑡
−
𝜃
)
,
cos
⁡
(
𝜃
𝑡
𝑎
𝑟
𝑔
𝑒
𝑡
−
𝜃
)
)
e=atan2(sin(θ
target
	​

−θ),cos(θ
target
	​

−θ))
PID control law:
𝜔
=
𝐾
𝑝
𝑒
+
𝐾
𝑖
∫
𝑒
 
𝑑
𝑡
+
𝐾
𝑑
𝑑
𝑒
𝑑
𝑡
ω=K
p
	​

e+K
i
	​

∫edt+K
d
	​

dt
de
	​


Where:

𝐾
𝑝
K
p
	​

 → proportional gain (responsiveness)

𝐾
𝑖
K
i
	​

 → integral gain (steady-state correction)

𝐾
𝑑
K
d
	​

 → derivative gain (damping)

👉 Output:

𝜔
  
(
rad/s
)
ω(rad/s)
⚠️ 3. Obstacle Avoidance (Artificial Potential Field)

The system uses a repulsive potential field:

Distance to obstacle:
𝑑
=
(
𝑥
−
𝑥
𝑜
)
2
+
(
𝑦
−
𝑦
𝑜
)
2
d=
(x−x
o
	​

)
2
+(y−y
o
	​

)
2
	​

Repulsive force:
𝐹
𝑟
𝑒
𝑝
=
{
𝑘
(
1
𝑑
−
1
𝑑
𝑠
𝑎
𝑓
𝑒
)
𝑑
⃗
𝑑
3
,
	
𝑑
<
𝑑
𝑠
𝑎
𝑓
𝑒


0
,
	
𝑑
≥
𝑑
𝑠
𝑎
𝑓
𝑒
F
rep
	​

={
k(
d
1
	​

−
d
safe
	​

1
	​

)
d
3
d
	​

,
0,
	​

d<d
safe
	​

d≥d
safe
	​

	​


Where:

𝑑
𝑠
𝑎
𝑓
𝑒
d
safe
	​

 → influence radius

𝑘
k → repulsion gain

👉 Behavior:

Strong repulsion when close

No effect when far

🤖 4. Inter-Robot Collision Avoidance

Each robot maintains a minimum distance from others:

𝐹
𝑟
𝑜
𝑏
𝑜
𝑡
=
𝑘
𝑟
𝑒
𝑝
⋅
𝑑
⃗
𝑑
3
F
robot
	​

=k
rep
	​

⋅
d
3
d
	​


Activated when:

𝑑
<
𝑑
𝑚
𝑖
𝑛
d<d
min
	​


To avoid singularities:

𝑑
=
max
⁡
(
𝑑
,
𝜖
)
d=max(d,ϵ)

👉 Ensures:

Safe spacing

Collision-free motion

📐 5. Formation Control
5.1 Normal Mode (V-Shape Formation)
𝑡
𝑎
𝑟
𝑔
𝑒
𝑡
2
=
[
𝑥
1
−
𝑑
cos
⁡
𝜃
1
+
𝑠
sin
⁡
𝜃
1


𝑦
1
−
𝑑
sin
⁡
𝜃
1
−
𝑠
cos
⁡
𝜃
1
]
target
2
	​

=[
x
1
	​

−dcosθ
1
	​

+ssinθ
1
	​

y
1
	​

−dsinθ
1
	​

−scosθ
1
	​

	​

]
𝑡
𝑎
𝑟
𝑔
𝑒
𝑡
3
=
[
𝑥
1
−
𝑑
cos
⁡
𝜃
1
−
𝑠
sin
⁡
𝜃
1


𝑦
1
−
𝑑
sin
⁡
𝜃
1
+
𝑠
cos
⁡
𝜃
1
]
target
3
	​

=[
x
1
	​

−dcosθ
1
	​

−ssinθ
1
	​

y
1
	​

−dsinθ
1
	​

+scosθ
1
	​

	​

]

Where:

𝑑
d → longitudinal spacing

𝑠
s → lateral offset

5.2 Obstacle Mode (Single-File Formation)
𝑡
𝑎
𝑟
𝑔
𝑒
𝑡
2
=
[
𝑥
1
−
𝑑
cos
⁡
𝜃
1


𝑦
1
−
𝑑
sin
⁡
𝜃
1
]
target
2
	​

=[
x
1
	​

−dcosθ
1
	​

y
1
	​

−dsinθ
1
	​

	​

]
𝑡
𝑎
𝑟
𝑔
𝑒
𝑡
3
=
[
𝑥
1
−
2
𝑑
cos
⁡
𝜃
1


𝑦
1
−
2
𝑑
sin
⁡
𝜃
1
]
target
3
	​

=[
x
1
	​

−2dcosθ
1
	​

y
1
	​

−2dsinθ
1
	​

	​

]

👉 Enables safe navigation in narrow environments.

🎮 6. Follower Control Strategy

Followers use proportional heading control:

𝜃
𝑡
𝑎
𝑟
𝑔
𝑒
𝑡
=
atan2
(
𝐹
𝑦
,
𝐹
𝑥
)
θ
target
	​

=atan2(F
y
	​

,F
x
	​

)
𝑒
=
angle difference
e=angle difference
𝜔
=
𝐾
𝑝
⋅
𝑒
ω=K
p
	​

⋅e

With saturation:

𝜔
∈
[
−
2
,
  
2
]
ω∈[−2,2]

👉 Simpler than PID → computationally efficient.

📊 7. Key Parameters (with Units)
⏱ Time & Simulation
Parameter	Value	Unit	Description
dt	0.05	s	Time step
steps	900	-	Simulation iterations
Total time	45	s	Simulation duration
🚗 Motion
Parameter	Value	Unit	Description
v	8	m/s	Leader speed
v_f	8	m/s	Follower speed
📏 Formation
Parameter	Value	Unit	Description
d	5	m	Inter-robot distance
side	4	m	Lateral offset
⚠️ Obstacle Avoidance
Parameter	Value	Unit	Description
safe_dist	8	m	Obstacle influence radius
k_rep_obs	20	-	Repulsion gain
obs_r	1	m	Obstacle radius
🤖 Robot Safety
Parameter	Value	Unit	Description
safe_robot	4	m	Minimum robot distance
k_rep_robot	25	-	Inter-robot repulsion gain
🎛 Control Gains
Parameter	Value	Unit	Description
Kp	2.5	-	Proportional gain
Ki	0.01	1/s	Integral gain
Kd	0.5	s	Derivative gain
Kp_f	2.0	-	Follower proportional gain
🔥 8. System Characteristics
✔ Advantages

Fully decentralized follower behavior

Real-time obstacle avoidance

Adaptive formation switching

Computationally efficient

⚠️ Limitations

No global path planning (can get stuck in local minima)

Fixed velocity (no adaptive speed control)

Sensitive to parameter tuning

🧠 9. Classification

This system falls under:

Hybrid Reactive Multi-Robot Control System

Combining:

Kinematic control

Potential field methods

Behavior-based formation

🚀 10. Future Improvements

Model Predictive Control (MPC)

Dynamic velocity adaptation

Graph-based swarm coordination

SLAM integration

📌 Conclusion

This implementation demonstrates a robust and modular approach to multi-robot navigation by integrating:

PID-based control for stability

Potential fields for safety

Formation logic for coordination

It provides a strong baseline for extending toward real-world autonomous multi-robot systems.
