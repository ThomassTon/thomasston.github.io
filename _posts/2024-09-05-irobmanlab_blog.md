---
layout: distill
title: Multi-Robot Pick and Place
date: 2024-09-05 
# description: an example of a blog post with some math
tags: lab project
# categories: sample-posts
related_posts: false
featured: true


toc:
  - name: Introduction
  - name: Background
    # - name: optimization-based methods
    subsections:
        - name: sampling-based motion planning
        - name: optimization-based motion planning
        - name: KOMO and ST-RRT*
  - name: Implementation
---



## 1. Introduction

In certain robotic applications, there are scenarios where a single robotic arm may struggle to perform tasks, such as handling a large object or a flexible item. In such cases, two or more robotic arms need to collaborate in order to accomplish the task efficiently.

In this work, we assume the use of two robotic arms working collaboratively to perform motion planning based on the given initial and target positions of the object to be grasped. Additionally, due to limitations of the simulation framework, the grasping is accomplished through a linking mechanism. In the environmental setup, we also assume that the obstacles remain stationary.

---

## 2. Background
Motion planning is the process of determining a feasible path or sequence of movements for a robot (or other agents) to achieve a specific task while avoiding obstacles and respecting constraints like joint limits, collision avoidance, and smoothness of the motion.

Motion planning can be broadly categorized into two types: optimization-based methods and sampling-based methods.

### 2.1 sampling-based motion planning
sampling-based motion planning constructs feasible paths for a robot by randomly sampling points in the robot's configuration space (C-space) and connecting these points to form a valid path from the start to the goal, common approaches include RRT, RRT*, and Bi-RRT.

- **RRT** incrementally builds a tree-like structure that explores the space by randomly sampling points and connecting them to the existing tree. Thereby finding a collision-free path. With a sufficient number of samples, it's possible to find a path, though it may not necessarily be the optimal one.

- **RRT\*** is an optimized version of RRT. After a vertex has been connected to the cheapest neighbor, the neighbors are again examined. Neighbors are checked if being rewired to the newly added vertex will make their cost decrease. If the cost does indeed decrease, the neighbor is rewired to the newly added vertex.

- **(Bi-RRT)** is an enhanced version of the RRT algorithm. Bi-RRT grows two trees simultaneously,i.e., One tree starts from the initial position of the robot, the other tree starts from the goal position. 

### 2.2 optimization-based motion planning

Instead of searching for a path first (like in sampling-based methods), the optimization-based motion planning optimizes the motion directly by minimizing or maximizing a specific objective function, such as minimizing travel time, energy, or avoiding obstacles. Common optimization-based methods include KOMO, CHOMP, and STOMP. 

- **KOMO** means k-order markov optimization. KOMO is a way to formulate path optimization problems.[1] 

- **CHOMP (Covariant Hamiltonian Optimization for Motion Planning)** is a method for trajectory optimization invariant to reparametrization. CHOMP uses functional gradient techniques to iteratively improve the quality of an initial trajectory, optimizing a functional that trades off between a smoothness and an obstacle avoidance component.[2]

- **STOMP** is a stochastic trajectory optimazation framework. The approach relies on generating nosiy trajectories to explore the space around an initial trajectory, which are then combined to produced an updated trajectory wit lower cost.[3]




### 2.3 KOMO and ST-RRT*

In this project, we utilized two motion planning methods: one is the optimization-based KOMO, and the other is the sampling-based ST-RRT*.


### 2.3.1 KOMO
**k-order Markov Optimization** is a method used in decision-making processes where the current decision depends not just on the immediate previous state (as in a first-order Markov process) but on a sequence of previous states, up to k steps in the past:

$$
\min_{x_{0:T}} \sum_{t=0}^{T} f_t(x_{t-k:t})^\top f_t(x_{t-k:t}) + \sum_{t,t'} k(t,t') x_t^\top x_{t'}
$$ 

$$
\text{s.t.} \quad \forall t : g_t(x_{t-k:t}) \leq 0, \quad h_t(x_{t-k:t}) = 0. 
$$

where $$x_{t-k:t} = (x_{t-k},..., x_{t-1}, x_{t})$$ are $$k+1$$ tuples of consecutive states. And the the term $$k(t,t^{'})$$ is an optional kernel measuring the desired correlation between time steps $$t$$ and $$t^{'}$$, which we explored but in practice hardly used.

To compute the inverse kinematics of a robotic arm, we typically need to define the following parameters:

$$
\begin{aligned}
q &\in \mathbb{R}^n &\text{vector of joint angles (robot configuration)} \\
\dot{q} &\in \mathbb{R}^n &\text{vector of joint angular velocities} \\
\phi : q &\mapsto y \in \mathbb{R}^d &\text{feature (or forward kinematic)} \text{e.g. position} \in \mathbb{R}^3 \text{ or vector} \in \mathbb{R}^3 \\
J(q) &= \frac{\partial \phi}{\partial q} \in \mathbb{R}^{d \times n} &\text{Jacobian}
\end{aligned}
$$

To apply KOMO (K-order Markov Optimization) to inverse kinematics with $$k=2$$:

$$
\begin{aligned}
J_{\text{pos}} &= \sum_{t=1}^{T} \| \mathbf{y}_t^{\text{desired}} - \mathbf{y}_t(\mathbf{q}_t) \|^2 &\text{Error of the end effector} \\
J_{\text{smooth}} &= \sum_{t=3}^{T} \left( \lambda_1 \| \theta_t - \theta_{t-1} \|^2 + \lambda_2 \| \theta_t - 2\theta_{t-1} + \theta_{t-2} \|^2 \right) &\text{Smoothness Constraint}\\
\end{aligned}
$$

Specifically:

- $$\lambda_{1}$$ controls the first-order smoothness (to prevent excessive changes in joint angles between consecutive time steps).

- $$\lambda_{2}$$ controls second-order smoothness (to avoid abrupt changes in acceleration or angular velocity).

- $$\theta_{t} - 2\theta_{t-1} + \theta_{t-2}$$ approximately describes the change in acceleration of the joint angles.

The final total cost function is:

$$
\begin{aligned}
J &= J_{pos} + J_{smooth} + \text{other constraint terms}
\end{aligned}
$$

We need to minimize this objective function $$J$$, .i.e,

$$
\min_{\theta_{t}, \theta_{t-1}, \theta_{t-2}} J
$$

### 2.3.2 ST-RRT*
ST-RRT* [4]is an advanced motion planning algorithm specifically designed for dynamic environments, where both spatial and temporal dimensions need to be considered. Its primary goal is to find paths that satisfy velocity constraints while minimizing arrival time. 

Unlike traditional methods that only plan in a configuration space (Q), ST-RRT* adds a time dimension, forming a space-time state space denoted as $$X = Q \times T$$, where $$Q$$ represents the configuration space and $$T$$ represents the time dimension.

ST-RRT* builds on the dual-tree RRT-Connect framework but introduces several key modifications to handle unbounded time spaces and optimize arrival time: 

- **Progressive Goal Region Expansion:** ST-RRT* uses a progressive expansion strategy that gradually increases the sampled time range while ensuring sufficient sample density through batch sampling. The initial time range **B.timeRange** and batch size **B.batchSize** are set. As the algorithm progresses, the time range is expanded by a factor (P.rangeFactor) to include a larger time horizon, allowing the planner to explore more of the time dimension.

- **Conditional Sampling:** Only the intersection of start and goal velocity cones is sampled. This greatly improves efficiency by reducing unnecessary exploration of infeasible areas.

- **Simplified Rewiring:** Like RRT*, ST-RRT* optimizes paths by rewiring nodes in the tree. After extending the tree with a new node x_new, the goal tree is rewired to ensure that the path to the goal minimizes arrival time.

As shown in the figure below, the orange area represents the goal region, and the blue dashed line is an initial estimate of the feasible goal time:
- (a) Using the initial batch of samples, no solution was found.
- (b) The upper bound of the time space (represented by the dashed line) is expanded, allowing more goal nodes to be sampled, and both trees continue to grow.
- (c) An initial solution is found (shown in orange), and the upper time bound is reduced accordingly.
- (d) Tree branches that can no longer contribute to an improved solution are pruned (shown with lower opacity), leading to the final solution after convergence.

<div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/st-rrt.png" class="img-fluid rounded z-depth-1" %}
</div>

---

## 3. Implementation

In this section, we will introduce how to achieve dual-arm collaborative object transportation. The process can be divided into three main steps: scene setup, task planning, and motion planning.

### 3.1 Scene Setup
Here, we will set the initial position and target location of the object to be grasped, as well as the fixed positions of any obstacles. Additionally, the positions of the two robots need to be defined. Typically, the robots are placed at a slightly greater distance from each other to prevent collisions while grasping the same object.

In this case, the first three elements of Q represent the displacement coordinates along the x, y, and z axes, while the last four elements represent the rotation angles using quaternions. Additionally, setting `contact` to 1 enables collision detection.

```PHP
_obstacle (bin1){ type:ssBox, size:[0.2 0.2 0.1 .01], contact:1 Q:<[ -0, 0, 0.5, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}

goal1 (bin1){ joint:rigid type:ssBox, size:[0.1 0.1 0.1 .01], contact:0 Q:<[  0, 0.0, 0.13, 1., 0., .0, 0]> color:[0.4, 1, 1, 0.2]}
goal2 (bin1){ joint:rigid type:ssBox, size:[0.1 0.1 0.1 .01], contact:0 Q:<[  0, 0.0, 0.23, 1., 0., .0, 0]> color:[0.4, 1, 1, 0.2]}

obj2 (bin2){ joint:rigid type:ssBox, size:[0.1 0.1 0.1 .01], contact:1 Q:<[  -0., -0.15, 0.03, 1, 0, .0, 0]> color:[0.4, 1, 1, 1]}
obj1 (bin2){ joint:rigid type:ssBox, size:[0.1 0.1 0.1 .01], contact:1 Q:<[  -0., -0.01, 0.03, 1, 0, .0, 0]> color:[0.4, 1, 1, 1]}
```


### 3.2 Task Planning
Task planning refers to the process of determining the sequence of high-level actions or tasks that a robot (or a group of robots) must perform to achieve a specific goal.

Common methods for Task Sequence Planning include Greedy Search, Random Search, and Simulated Annealing Search. **Greedy search** is a locally optimal strategy that, at each step, selects the best immediate option without considering future consequences. **Random search** generates task sequences randomly, evaluates their performance, and selects the best-performing sequence. It does not rely on selecting the best option at each step, instead exploring different combinations of task sequences randomly. **Simulated annealing** is inspired by the physical process of annealing in metallurgy, the algorithm initially allows the acceptance of worse solutions (higher “temperature”) to escape local optima. As the search progresses, the “temperature” is gradually lowered, and the algorithm becomes more likely to accept only better solutions, converging toward a global optimum.


However, in our work, there is no need for complex task planning. Instead, we directly use the sequence of objects as the task sequence. Here, a task represents the movement of a robotic arm from position A to position B. For example, moving the arm from the initial position to the position where it can grasp an object constitutes a task. The content of the task is determined by the joint configurations that correspond to the target positions of the robot's end effector. For example, if two objects need to be transported, this results in four tasks.


### 3.3 Motion Planning


In this project, we use the **KOMO** optimizer to solve the inverse kinematics:

We can configure the optimizer using the skeleton framework: {1., 1., SY_touch, {pen_tip_0, obj}}: **‘1.’** represents the timestep, **'SY_touch'** indicates contact with the target, can be replaced with other types of constraints, such as **SY_stable**, which indicates being stationary relative to the target, **'pen_tip'** represents the robot's end-effector, **'obj'** represents the target.

Additionally, we can use **addObjective** to add extra constraints, such as ensuring the distance between the end-effector and the object is zero, or enforcing that the end-effector remains perpendicular to the object's surface.

```c++

Skeleton S = {
    {1., 1., SY_touch, {pen_tip_0, obj}},
    {1., 1., SY_touch, {pen_tip_1, obj}},
    {1., 2., SY_stable, {pen_tip_0, obj}},
    {1., 2., SY_stable, {pen_tip_1, obj}},
    {2., 2., SY_poseEq, {obj, goal}},
};
komo.setSkeleton(S);
// komo.addObjective({1.}, FS_vectorZ, {STRING(robots[0] << "pen")}, OT_sos, {1e1}, {0., 0., -1.});
komo.addObjective({1.,1.}, FS_distance, {obj, STRING(robots[0] << "pen_tip")}, OT_ineq, {1e1},{-0.0}); 
komo.addObjective({1.,1.}, FS_distance, {obj, STRING(robots[1] << "pen_tip")}, OT_ineq, {1e1},{-0.0}); 
```


**Path planning** is implemented using two methods: one based on the KOMO optimizer, and the other using bi-RRT.

The **KOMO optimizer** solves the problem by using the provided initial and target positions, while setting velocity and acceleration constraints.
```c++
OptOptions options;
options.stopIters = 100;  // Set the maximum number of iterations.
options.damping = 1e-3;
options.stopLineSteps = 5;
komo.setConfiguration(-2, q0); // By configuring KOMO for second-order optimization, it preserves position information from time step t-2 to t.
komo.setConfiguration(-1, q0);
komo.setConfiguration(0, q0); 
komo.add_collision(true, .001, 1e1);  //set collision detection
komo.addObjective({1}, FS_qItself, {}, OT_eq, {1e2}, q1); // set goal position q1
komo.addObjective({0.95, 1.0}, FS_qItself, {}, OT_eq, {1e1}, {},1); // speed slow at end
komo.addObjective({0.95, 1.0}, FS_qItself, {}, OT_eq, {1e1}, {}, 2); // acceleration slow at end

komo.run(options);
// get the path from KOMO
arr path(ts.N, q0.N);
for (uint j = 0; j < ts.N; ++j) {
path[j] = komo.getPath_q(j);
};

```

The RRT path planner generates the path based on the initial and target positions provided by the KOMO optimizer.
```c++
TaskPart rrt_path = plan_in_animation_rrt(A, C, t0, q0, q1, time_lb, prefix); // q0 is start configuration of joints, qi is goal configuration of joints
```

By using these two planners, a robot-arm's movement path can be obtained in a straightforward and efficient manner.

However, because the simulation framework defines object connections based on a tree structure where each node can only have one parent and one child, this restricts each object to being linked to only one end-effector at a time. For example, place the object on the table,

```c++
auto to = CPlanner[obj];
auto from = CPlanner["table_base"];
to->unLink();
to->linkFrom(from, true);
```


Therefore, our solution is to use the target positions generated by the KOMO optimizer to determine the relative positions of the two robotic arms, and let second arm perform as pseudo-grasp.

$$
\begin{aligned}
^{w}P = ^{w}r_{r1} + ^{w}R_{r1}\ ^{r1}P
\end{aligned}
$$

Let $$^{r1}P$$ represent the relative displacement between two end effectors. By obtaining the coordinates $$^{w}r_{r1}$$ and rotation matrix $$^{w}R_{r1}$$ of the first end effector, we can determine the coordinates of the second end effector $$^{w}P$$.

<!-- <div class="col-sm mt-3 mt-md-0"> -->
{% include figure.liquid loading="eager" path="assets/img/transformation.png" class="img-fluid rounded z-depth-1" %}
<!-- </div> -->

```c++
if(i==0){ // get the grasp position of robot 1
    to->unLink();
    to->linkFrom(from, true);
    r0_b =  CPlanner[pen_tip]->getPosition();
    rotationmantix = CPlanner[pen_tip]->getRotationMatrix();
    }
    else{ // get the grasp position of robot 2
    r0_1 = get_r_0_1(r0_b,rotationmantix, CPlanner[pen_tip]->getPosition()); // obtain the relative positions
    }

```


Once the relative positions of the two end-effectors are determined, we can calculate the position of the second end-effector based on the path of the first one. Then, by using KOMO to compute the inverse kinematics, we obtain the angles for each joint.

```c++
uint size_of_path =  paths[sequence[0].first].back().path.N /7;   // het the size of path from first robot
arr t_a1;
arr path_a1(0u,paths[sequence[0].first].back().path.d1);   // define the path for second robot
CPlanner.setJointState(paths[robot][0].path[-1]);
for(uint i = 0; i<size_of_path; i++){
    auto r0b = paths[sequence[0].first][1].path[i];
    auto t = paths[sequence[0].first].back().t(i);
    CTest.setJointState(r0b);
    const auto pen_tip =  STRING(sequence[0].first << "pen_tip");
    auto _r0_b = CTest[pen_tip]->getPosition();
    auto rotationmatrix = CTest[pen_tip]->getRotationMatrix();
    auto _goal_pose = get_trans_position(_r0_b,rotationmatrix,r0_1);
    auto goal_pose_= get_position(CPlanner,robot,_goal_pose);     // use KOMO to compute the inverse kinematics
    CPlanner.setJointState(goal_pose_);
    t_a1.append(t);
    path_a1.append(goal_pose_);

}
TaskPart path_(t_a1,path_a1);
path_.has_solution=true;
```
## Result

In our tests, we found that when both RRT and KOMO can find a path, the path generated by KOMO is usually shorter,

| Method      | path1 |path2 | path3 |path4 
|-----------|-----|-----------|-------|
| KOMO     | 30  | 111  | 153 | 182
| RRT      | 31  | 136 | 171 | 185



In the animation below, the left side shows the path generated by KOMO, while the right side shows the path generated by RRT. We can observe that the path generated by KOMO is faster than the one generated by RRT.
<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/blog1.mp4" class="img-fluid rounded z-depth-1" controls=true autoplay=true %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/blog2.mp4" class="img-fluid rounded z-depth-1" controls=true %}
    </div>
</div>


In addition, we also tested the obstacle avoidance functionality of this path planner and found that it effectively avoids obstacles during path planning. Below are some of our simulation test videos.

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/stacking_co_3.mp4" class="img-fluid rounded z-depth-1" controls=true autoplay=true %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include video.liquid path="assets/video/cooperation_4.mp4" class="img-fluid rounded z-depth-1" controls=true %}
    </div>
</div>

## Conclusion

The KOMO solver can efficiently compute inverse solutions and plan paths, but there are still some scenarios where it fails to find a valid solution. Additionally, while our method allows both robotic arms to grasp an object simultaneously, the second arm is performing a pseudo-grasp, which causes rotation between the end-effector and the object during movement. This aspect can be improved in future work.
