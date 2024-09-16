---
layout: post
title: Multi-Robot Pick and Place
date: 2024-09-05 
# description: an example of a blog post with some math
tags: lab project
# categories: sample-posts
related_posts: false
---

For this project, we need to implement the simultaneous manipulation of two objects using two Franka robotic arms in a simulation environment. The project utilizes the KOMO optimizer and bi-RRT for planning the motion paths of the robotic arms.

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

$$\lambda_{1}$$ controls the first-order smoothness (to prevent excessive changes in joint angles between consecutive time steps).

$$\lambda_{2}$$ controls second-order smoothness (to avoid abrupt changes in acceleration or angular velocity).

$$\theta_{t} - 2\theta_{t-1} + \theta_{t-2}$$ approximately describes the change in acceleration of the joint angles.

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

**Bi-directional Rapidly-exploring Random Tree (Bi-RRT)** is an enhanced version of the Rapidly-exploring Random Tree (RRT) algorithm. 
Unlike the standard RRT, which grows a single tree from the start to explore the space, Bi-RRT grows two trees simultaneously,i.e., One tree starts from the initial position of the robot, the other tree starts from the goal position. 
Both trees expand alternately by randomly sampling points in the space and attempting to extend the trees towards those points. Once the two trees are connected, a valid path is formed that goes from the start, through the nodes of the first tree, to the connecting point, and then through the nodes of the second tree to the goal.

<div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/rrt_img.png" class="img-fluid rounded z-depth-1" %}
</div>

**Setting up the simulation environment:**

Typically, we need to configure the initial position of the objects to be grasped, the target positions, and also the placement of the robotic arms.

For the **objects** to be grasped, we need to define the coordinates and the rotational angles using quaternions. For the initial position, "contact" should be set to 1 for collision detection, while for the target position, it should be set to 0.

```PHP
goal1 (bin1){ joint:rigid type:ssBox, size:[0.1 0.1 0.1 .01], contact:0 Q:<[  0, 0.0, 0.13, 1., 0., .0, 0]> color:[0.4, 1, 1, 0.2]}
goal2 (bin1){ joint:rigid type:ssBox, size:[0.1 0.1 0.1 .01], contact:0 Q:<[  0, 0.0, 0.23, 1., 0., .0, 0]> color:[0.4, 1, 1, 0.2]}
goal3 (bin2){ joint:rigid type:ssBox, size:[0.20 0.20 0.1 .01], contact:0 Q:<[  0, 0.05, 0.03, 1., 0., .0, 0]> color:[0.4, 1, 1, 0.2]}

obj3 (bin1){ joint:rigid type:ssBox, size:[0.20 0.20 0.1 .01], contact:1 Q:<[ 0, -0., 0.03, 1, 0, .0, 0]> color:[0.4, 1, 1, 1]}
obj2 (bin2){ joint:rigid type:ssBox, size:[0.1 0.1 0.1 .01], contact:1 Q:<[  -0., -0.15, 0.03, 1, 0, .0, 0]> color:[0.4, 1, 1, 1]}
obj1 (bin2){ joint:rigid type:ssBox, size:[0.1 0.1 0.1 .01], contact:1 Q:<[  -0., -0.01, 0.03, 1, 0, .0, 0]> color:[0.4, 1, 1, 1]}
```

For the robotic arms, we need to configure their placement positions as well as the initial position of each joint. Typically, the distance between the two arms should be set large enough to avoid joint collisions when grasping the same object.
```c++

const arrA basePos = {
    {-0.8, 0.0, 0.00},
    {0.8, 0.0, 0.0}
};

const arrA baseQuat = {
    {1, 0, 0, 0},
    {0, 0, 0, 1},
    {0.924, 0, 0, -0.383},
    {-0.383, 0, 0, 0.924},
};

for (uint i = 0; i < 2; i++)
{
    auto *a = C.addFile("./in/franka.g");
    C.reconfigureRoot(a, true);
    a->linkFrom(C["table"]);
    const rai::String prefix = STRING('a' << i << '_');
    a->prefixSubtree(prefix);
    const rai::String agentBase = STRING(prefix << "base");
    C[agentBase]->setRelativePosition(basePos(i));
    C[agentBase]->setQuaternion(baseQuat(i));
    setActive(C, std::string(prefix.p));
    arr state = C.getJointState();
    C.setJointState(state);
}
```


In the **task plan**, since this project involves two robotic arms completing a task simultaneously, there is no need to account for the Traveling Salesman Problem. The tasks can simply be assigned in sequence, i.e., {task_{1}(robot_{1}, robot_{2}), task_{2}(robot_{1}, robot_{2})}.  

Here, a task represents the movement of a robotic arm from position A to position B. For example, moving the arm from the initial position to the position where it can grasp an object constitutes a task. The content of the task is determined by the joint configurations that correspond to the target positions of the robot's end effector.

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


Therefore, our solution is to use the target positions generated by the KOMO optimizer to determine the relative positions of the two robotic arms,

**picture**

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

From the animation below, we can also see that the path generated by KOMO is faster than that of RRT.

| KOMO|RRT|
|-----|------|
![演示动画](/home/haolei/thomasston.github.io/assets/video/blog1.gif)|<img src="assets/video/blog2.gif" alt="005" style="zoom: 70%;" />