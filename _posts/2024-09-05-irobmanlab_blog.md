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


**Komo:**

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

**Bi-RRT**:

Bi-directional Rapidly-exploring Random Tree (Bi-RRT) is an enhanced version of the Rapidly-exploring Random Tree (RRT) algorithm. 
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
```c++
  const arrA basePos = {\{-0.8, 0.0, 0.00\}, \{0.8, 0.0, 0.0\}};

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
    // state(1) -= 0.25;
    // state(3) += 0.25;
    C.setJointState(state);
  }
```
