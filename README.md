


# 🚀 3D Leader–Follower Multi-Robot Navigation System

## 📌 Overview

This project implements a 3D simulation of a **leader–follower multi-robot system** using a reactive control approach. The system combines trajectory tracking, obstacle avoidance, formation control, and inter-robot collision avoidance.

The leader robot navigates through predefined waypoints, while follower robots maintain formation and avoid both obstacles and each other using Artificial Potential Fields (APF).

---

## 🧠 System Model (Kinematics)

Each robot follows a unicycle kinematic model:

```
x(k+1) = x(k) + v * cos(theta) * dt
y(k+1) = y(k) + v * sin(theta) * dt
theta(k+1) = theta(k) + omega * dt
```

Where:
- x, y → position (meters)
- theta → heading angle (radians)
- v → linear velocity (m/s)
- omega → angular velocity (rad/s)
- dt → time step (seconds)

---

## 🎯 Leader Navigation

### Waypoint Tracking

The leader moves toward a target waypoint using a normalized direction vector:

```
d = (x_wp - x, y_wp - y) / norm(...)
vx = d_x * V
vy = d_y * V
```

Waypoint switching occurs when:

```
distance_to_waypoint < threshold
```

---

## 🎯 PID Heading Control

The leader adjusts its heading using a PID controller.

Target heading:

```
theta_target = atan2(vy, vx)
```

Heading error:

```
e = atan2(sin(theta_target - theta), cos(theta_target - theta))
```

Control law:

```
omega = Kp * e + Ki * integral(e) + Kd * derivative(e)
```

---

## ⚠️ Obstacle Avoidance (Artificial Potential Field)

Distance to obstacle:

```
d = sqrt((x - xo)^2 + (y - yo)^2)
```

Repulsive force:

```
F_rep = k * (1/d - 1/d_safe) * (d_vec / d^3),   if d < d_safe
F_rep = 0                                      otherwise
```

This force pushes the robot away from nearby obstacles.

---

## 🤖 Inter-Robot Collision Avoidance

To prevent collisions between robots:

```
F_robot = k_rep * (d_vec / d^3)
```

Activated when:

```
distance_between_robots < safe_robot
```

To avoid division by zero:

```
d = max(d, epsilon)
```

---

## 📐 Formation Control

### V-Shape Formation (Normal Mode)

```
target2 = [x1 - d*cos(theta1) + side*sin(theta1),
           y1 - d*sin(theta1) - side*cos(theta1)]

target3 = [x1 - d*cos(theta1) - side*sin(theta1),
           y1 - d*sin(theta1) + side*cos(theta1)]
```

---

### Single-File Formation (Obstacle Mode)

```
target2 = [x1 - d*cos(theta1),
           y1 - d*sin(theta1)]

target3 = [x1 - 2*d*cos(theta1),
           y1 - 2*d*sin(theta1)]
```

---

## 🎮 Follower Control

Each follower combines attractive and repulsive forces:

```
F_total = F_attractive + F_repulsive
```

Heading update:

```
theta_target = atan2(Fy, Fx)
omega = Kp_f * error
```

With saturation:

```
omega in [-2, 2]
```

---

## 📊 Key Parameters

### Simulation Parameters

| Parameter | Value | Unit | Description |
|----------|------|------|------------|
| dt | 0.05 | s | Time step |
| steps | 900 | - | Number of iterations |
| total time | 45 | s | Total simulation time |

---

### Motion Parameters

| Parameter | Value | Unit | Description |
|----------|------|------|------------|
| v | 8 | m/s | Leader velocity |
| v_f | 8 | m/s | Follower velocity |

---

### Formation Parameters

| Parameter | Value | Unit | Description |
|----------|------|------|------------|
| d | 5 | m | Distance from leader |
| side | 4 | m | Lateral spacing |

---

### Obstacle Avoidance

| Parameter | Value | Unit | Description |
|----------|------|------|------------|
| safe_dist | 8 | m | Obstacle influence radius |
| k_rep_obs | 20 | - | Obstacle repulsion gain |
| obs_r | 1 | m | Obstacle radius |

---

### Robot Safety

| Parameter | Value | Unit | Description |
|----------|------|------|------------|
| safe_robot | 4 | m | Minimum robot distance |
| k_rep_robot | 25 | - | Robot repulsion gain |

---

### Control Gains

| Parameter | Value | Unit | Description |
|----------|------|------|------------|
| Kp | 2.5 | - | Proportional gain (leader) |
| Ki | 0.01 | 1/s | Integral gain |
| Kd | 0.5 | s | Derivative gain |
| Kp_f | 2.0 | - | Follower heading gain |

---

## 🔥 System Characteristics

### Advantages
- Real-time reactive navigation
- Adaptive formation switching
- Computationally efficient
- Modular design

### Limitations
- No global path planning
- Potential local minima (APF)
- Constant velocity (no speed adaptation)

---

## 🚀 Future Improvements

- Model Predictive Control (MPC)
- Dynamic speed adaptation
- Swarm/distributed control
- SLAM integration

---

## 📌 Conclusion

This project demonstrates a robust multi-robot coordination strategy combining:

- PID control for stability
- Artificial Potential Fields for obstacle avoidance
- Formation control for coordinated movement

It provides a strong foundation for real-world autonomous multi-robot systems.
