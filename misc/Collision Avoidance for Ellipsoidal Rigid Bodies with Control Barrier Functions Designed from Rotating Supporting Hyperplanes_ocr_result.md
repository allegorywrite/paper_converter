# Collision Avoidance for Ellipsoidal Rigid Bodies with Control Barrier Functions Designed from Rotating Supporting Hyperplanes 

Riku Funada ${ }^{1}$, Koju Nishimoto ${ }^{1}$, Tatsuya Ibuki ${ }^{2}$, and Mitsuji Sampei ${ }^{1}$


#### Abstract

This paper proposes a collision avoidance method for ellipsoidal rigid bodies, which utilizes a control barrier function (CBF) designed from a supporting hyperplane. We formulate the problem in the Special Euclidean Group $S E(2)$ and $S E(3)$, where the dynamics are described as rigid body motion (RBM). Then, we consider the condition for separating two ellipsoidal rigid bodies by employing a signed distance from a supporting hyperplane of a rigid body to the other rigid body. Although the positive value of this signed distance implies that two rigid bodies are collision-free, a naively prepared supporting hyperplane yields a smaller value than the actual distance. To avoid such a conservative evaluation, the supporting hyperplane is rotated so that the signed distance from the supporting hyperplane to the other rigid body is maximized. We prove that the maximum value of this optimization problem is equal to the actual distance between two ellipsoidal rigid bodies, hence eliminating excessive conservativeness. We leverage this signed distance as a CBF to prevent collision while the supporting hyperplane is rotated via a gradient-based input. The designed CBF is integrated into a quadratic programming (QP) problem, where each rigid body calculates its collision-free input in a distributed manner, given communication among rigid bodies. The proposed method is demonstrated with simulations. Finally, we exemplify our method can be extended to a vehicle having nonholonomic dynamics.


Index Terms-Collision avoidance, Constrained control, Mobile robotics, Cooperative control

## I. INTRODUCTION

Collision avoidance is one of the fundamental requirements for ensuring the safe operation of multi-robot systems in many application fields, including precision agriculture [1], autonomous transportation [2], [3], and environmental monitoring [4]. In such challenging and complex domains, it is paramount important to integrate robots having different capabilities, sizes, and shapes into a system to complete the task [5]. In order to embrace such heterogeneity, collision avoidance methods are required to guide robots not to collide with each other while considering their shapes.

Real-time collision avoidance has been actively explored in the fields of multi-robot systems, but most work approximates the shape of a robot as a circle or sphere, as will be discussed

[^0]in Section I-A. Although such approaches can be utilized for any robots by overestimating the original form of robots to a sphere enclosing them, this approach could result in too conservative evasion if they have a nonspherical, especially elongated, body. To model such heterogeneous shapes with a higher fidelity model and still a small set of parameters, an ellipsoidal approximation of the shape has been utilized in SLAM and path planning fields.

The work [6] proposes the path planning method, where each quadcopter is modeled as an ellipsoid to account for a downwash wind they generate. The approach to model a rigid body as an ellipsoid is also suitable for many SLAM or environmental monitoring techniques, e.g., [7]-[9], where these works represent the objects as an ellipsoid and infer their sizes from visual information.

Despite the advantages of modeling the shape of rigid bodies as ellipsoids, the difficulties in deriving the distance between two separated ellipsoids hinder the development of collision avoidance controllers for ellipsoidal rigid bodies. In general, the analytical solutions of the distance between two ellipses/ellipsoids are difficult to obtain in simple analytical form, and the algorithms calculating the numerical solutions of the distance are utilized in the computational science field instead [10], [11]. For two-dimensional ellipses, the method in [12] detects whether two ellipses overlap by evaluating the discriminant of a cubic characteristic equation. However, because the value of the discriminant is not necessarily in a proportional relationship with the distance, rigid bodies could take unreasonable motion if we employ it in a collision avoidance method, as will be shown in the comparative studies with the work [13] later. In addition, it is difficult to extend this method to ellipsoids in 3D environments because one is required to solve the discriminant of higher dimensional characteristic equations, which solution is divided by case.

This paper presents a collision avoidance method for elliptical/ellipsoidal rigid bodies in 2D/3D environments shown in Fig. 1, the dynamics of which are described by rigid body motion (RBM). To circumvent the difficulty of directly deriving analytical form of the distance between two ellipsoids, this paper proposes a novel CBF that utilizes a signed distance from a supporting hyperplane of an ellipsoid to the other ellipsoid, as depicted in Fig. 2. Because a naively prepared supporting hyperplane could render a smaller distance than the actual distance between two ellipsoidal rigid bodies (Fig. 2(a)), we design a gradient-ascent-based update law, where the supporting hyperplane is rotated so that the


[^0]:    *This work is supported by JSPS KAKENHI Grant Number 22K14275. (Corresponding Author: Riku Funada)
    ${ }^{1}$ R. Funada, K. Nishimoto, and M. Sampei are with the Department of Systems and Control Engineering, Tokyo Institute of Technology, Tokyo 152-8550, Japan \{funada, sampei\}@sc.e.titech.ac.jp, nishimoto@sl.sc.e.titech.ac.jp
    ${ }^{2}$ T. Ibuki is with the Department of Electronics and Bioinformatics, Meiji University, Kanagawa 214-8571, Japan ibuki@meiji.ac.jp

![img-0.jpeg](img-0.jpeg)

Fig. 1. Proposed scenario. The rigid bodies characterized as ellipses or ellipsoids with heterogeneous shapes avoid collisions with each other.
signed distance from the supporting hyperplane to the other ellipsoidal rigid body is maximized (Fig. 2(b)). We prove that the maximum signed distance derived from this optimization problem is equal to the actual distance between two ellipsoidal rigid bodies. A novel CBF integrating the rotating supporting hyperplane is incorporated into the quadratic programming (QP) problem, which allows a distributed computation under communication. The simulation studies demonstrate that the proposed method can successfully achieve collision avoidance for elliptical/ellipsoidal rigid bodies in 2D/3D environments. Finally, we exemplify the proposed method can be extended to a vehicle having nonholonomic dynamics.

## A. Related Work

Among the various methodologies for preventing collisions, artificial potential fields (APFs) are one of the most traditional methods, which were first presented in [14]. APFs have been utilized for multi-robot (multi-agent) systems to attain cooperative behaviors [15], such as flocking [16], [17] and formation control [18]. As a repulsive potential field is a key element for designing collision avoidance behaviors, several functions are proposed, including [19], which is activated only when any other robots is in the sensing region of the robot. The work [20] designs a vector field for navigating a circular robot in an environment cluttered with convex obstacles by utilizing the idea of Voronoi diagrams. Still, such a Voronoibased approach cannot be readily extendable for the robots with ellipsoidal shapes because it is known to be challenging to generate the Voronoi diagram for ellipsoids [21].

Control barrier functions (CBFs) are another popular method to guarantee the safety of the system by formulating a quadratic programming (QP) problem [22]-[24]. Extensive studies utilize a CBF-based framework to achieve collision avoidance among multi-robot systems. For example, the work [25] achieves a distributed collision-free coordination in multirobot systems with heterogeneity in the control input range. The robots with limited sensing ranges are also considered in [26] with employing the hybrid CBFs.

The work [27] provides the comparative analysis between CBFs and APFs, which proves that one can obtain CBFs from a given APF. The authors also claim that CBFs designed from APFs have additional beneficial properties, such as mitigation of oscillation, compared with its counterpart of APFs. In
![img-1.jpeg](img-1.jpeg)

Fig. 2. The supporting hyperplane $l_{i j}$ separating two elliptical rigid bodies $\mathcal{E}_{i}$ and $\mathcal{E}_{j}$. Both (a) and (b) show the distance $h_{i j}\left(\boldsymbol{z}_{i j}\right)$ between the ellipse $\mathcal{E}_{j}$ and a supporting hyperplane $l_{i j}$. The tangent point is denoted as $\boldsymbol{m}_{i j}$, which is specified by a unit vector $\boldsymbol{z}_{i j}$ as detailed in Section IV.
addition, CBFs have the capacity to embrace different types of safety requirements, including collision avoidance, prevention of battery depletion [28], and connectivity maintenance [29], as one synthesized controller. Because of these virtues, this paper opts for the CBF-based approach. Still, most of the papers mentioned above model the robot as a point or a sphere.

The work [30] addresses the flocking for ellipsoidal rigid bodies, where the APF-based method is employed for collision avoidance. However, the condition utilized for designing the repulsive potential function takes complicated form and is not straightforwardly extendable to CBFs. The authors in [31] develop a collision avoidance methodology by utilizing the result in the computer graphics fields, which provides the separating conditions between ellipsoids. Still, the metric used in the separating condition does not intuitively align with the distance, and its physical interpretation might be difficult to understand. The work [32] considers planar robots having quadratic surfaces, where a condition calculated from robots' relative velocities, collision cone, is utilized for avoiding a collision. However, the condition requires several procedures to derive and is not easy to employ as CBFs.

The work [33] presents the extent-compatible CBF, which can enforce the safety of the robot having volume. The proposed method relies on the sum-of-squares (SOS) based optimization method and could be applied to ellipsoidal rigid bodies. Still, the computational burden in the SOS problem might hinder the application to a team composed of many robots. The work [34], which coincidentally was presented around the same time as our preliminary work [35], considers collision avoidance for polygonal robots, where a nonsmooth CBF is utilized. The authors in [34] developed this approach to handle general convex robots in [36], where the input is calculated in a centralized manner. The paper [37] also considers collision avoidance for a general convex shape by employing a scaling factor instead of the distance. Both [36] and [37] require solving an additional optimization problem other than a QP yielding a collision-free input, hence necessitating an extra computational effort.

Shifting the focus to spacecraft navigation fields, several studies consider collision avoidance between a spacecraft, modeled as a point or a circle, and debris represented as circles or ellipses [38], [39]. The work [39] sets a rotating supporting

hyperplane on the ellipsoidal debris to prevent collisions. Since the supporting hyperplane linearizes the constraints for collision avoidance, this formulation reduces the computation time of a navigation controller. Still, typical methodologies for preparing supporting hyperplanes require a user-specified constant angular velocity for rotating hyperplanes or a predefined spacecraft trajectory for setting multiple hyperplanes along a path [39]-[41]. In addition, the spacecraft is modeled as a point or sphere; hence this approach cannot be readily extendable for collision avoidance for ellipsoidal rigid bodies.

This paper presents a novel CBF that achieves collision avoidance for ellipsoidal rigid bodies while leveraging the simplicities of the rotating supporting hyperplane method. The update rule for the supporting hyperplane is newly developed by providing theoretical results eliminating conservativeness and guaranteeing safety. The proposed method only requires solving a QP to obtain a collision-free input; hence its computational effort is low, as same as traditional CBFs modeling a robot as a circle. Furthermore, we formulate the problem in the Special Euclidean Group to represent a collision avoidance law for the rigid bodies' poses in 2D and 3D environments. More detailed explanations of the contributions are shown below.

## B. Contributions

This paper develops our preliminary work [35], which only considered collision avoidance for elliptical robots in a 2D environment, where the dynamics of the robots were modeled as a single integrator. In addition, the presented collision avoidance law only allowed centralized computation. The contributions of this paper are as follows.

- We formulate the collision avoidance problem in the Special Euclidean Group, where the dynamics of rigid bodies are modeled as rigid body motion (RBM) to design a unified collision avoidance law for the robot's pose (position and attitude) in 2D and 3D environments. With this extension, we newly derive the time derivative of the CBF along with the dynamics represented as RBM, which is presented in Lemma 1.
- A novel CBF utilizing a rotating supporting hyperplane is developed, which modifies the one in our conference work [35] to account for a 3D case. We prove that the maximum of the optimization problem considered in the update rule of the supporting hyperplane is equal to the actual distance between two rigid bodies and ensure the forward invariance of the set representing no collisions.
- We show that the QP yielding a collision-free input can be calculated in a distributed manner by assuming the communication between rigid bodies. Furthermore, we prove the validity of the proposed CBF, namely the feasibility of the presented QP in both 2D and 3D scenarios.
- More comprehensive simulation studies are presented to demonstrate the effectiveness of the proposed method.
- We extend the proposed method to achieve collision avoidance for nonholonomic systems by employing a vehicle as a case study, where the update rule of the supporting hyperplane is modified.


## II. Preliminary: CONtrol Barrier Function

This section introduces a CBF, which will be utilized to guarantee the collision avoidance of the robots. Together with a basic CBF, we explain an approach to guarantee safety described by the high-relative degree constraints, which will be utilized when we extend our method to a vehicle having the nonholonomic dynamics in Section VI. Note that this paper considers rigid bodies in the Special Euclidean Group, whose dynamics are expressed by a different form with (1). Still, the same approach can be utilized by calculating the time derivatives of CBFs along the trajectories of the system described by RBM, as will be shown in Lemma 1.

Let us consider the control affine system

$$
\dot{\boldsymbol{x}}=f(\boldsymbol{x})+g(\boldsymbol{x}) \boldsymbol{u}
$$

where $f$ and $g$ are locally Lipschitz, $\boldsymbol{x} \in \mathbb{R}^{n}$ and $\boldsymbol{u} \in \mathbb{R}^{m}$. We also introduce a set defined as the zero super-level set of a continuously differentiable function $h(\boldsymbol{x})$, namely, $\mathcal{S}=\{\boldsymbol{x} \in$ $\mathbb{R}^{n} \mid h(\boldsymbol{x}) \geq 0\}$. Then, a CBF is defined as follows.

Definition 1. [22, Def. 5] The function $h$ is a control barrier function (CBF) defined on a set $\overline{\mathcal{S}}$ with $\mathcal{S} \subseteq \overline{\mathcal{S}} \subset \mathbb{R}^{n}$, if there exists an extended class $\mathcal{K}$ function $\alpha$, such that for the control system (1)

$$
\begin{aligned}
\sup _{\boldsymbol{u}} \dot{h}(\boldsymbol{x}, \boldsymbol{u})=\sup _{\boldsymbol{u}}\left[L_{f} h(\boldsymbol{x})+L_{g} h(\boldsymbol{x}) \boldsymbol{u}\right] & \geq-\alpha(h(\boldsymbol{x})) \\
& \forall \boldsymbol{x} \in \overline{\mathcal{S}}
\end{aligned}
$$

where $L_{f} h(\boldsymbol{x})$ and $L_{g} h(\boldsymbol{x})$ are the Lie derivatives of $h$ along $f(\boldsymbol{x})$ and $g(\boldsymbol{x})$, respectively.

The forward invariance of the set $\mathcal{S}$, defined just below (1), can be achieved through the following proposition.

Proposition 1. [22, Cor. 2] Given a set $\mathcal{S}$, if $h$ is a CBF on $\overline{\mathcal{S}}$, then any Lipschitz continuous controller $u(\boldsymbol{x}): \overline{\mathcal{S}} \rightarrow \mathbb{R}^{m}$ such that

$$
\dot{h}(\boldsymbol{x}, \boldsymbol{u})=L_{f} h(\boldsymbol{x})+L_{g} h(\boldsymbol{x}) \boldsymbol{u}(\boldsymbol{x}) \geq-\alpha(h(\boldsymbol{x}))
$$

will render the set $\mathcal{S}$ forward invariant.
The condition (3) can be integrated into the control law to ensure the forward invariance of the set by leveraging Quadratic Programming (QP). Let us denote the nominal input as $\boldsymbol{u}_{\text {nom }}$, and wish to modify it minimally invasive way so that the condition (3) is satisfied. This objective can be achieved by employing the input $\boldsymbol{u}^{*}$ obtained from the following QP.

$$
\begin{aligned}
\boldsymbol{u}^{*}= & \underset{\boldsymbol{u}}{\arg \min }\left\|\boldsymbol{u}-\boldsymbol{u}_{\mathrm{nom}}(\boldsymbol{x})\right\|^{2} \\
& \text { s.t. } L_{f} h(\boldsymbol{x})+L_{g} h(\boldsymbol{x}) \boldsymbol{u} \geq-\alpha(h(\boldsymbol{x}))
\end{aligned}
$$

The previous discussion describes how CBFs can be utilized to ensure the safety constraints for a control affine system (1). However, so far, we presume that the control input appears in the first derivative of the CBF with respect to time, as in (3). This property can be formally expressed as having a relative degree one, which is defined as follows.

Definition 2. [24, Def. 5.2] The system $\dot{\boldsymbol{x}}=f(\boldsymbol{x})+g(\boldsymbol{x}) \boldsymbol{u}$, with output $y=h(\boldsymbol{x})$, has relative degree, $r \in \mathbb{Z}_{+}$, at $\boldsymbol{x}_{0}$, if

$$
L_{g} L_{f}^{\delta} h(\boldsymbol{x})=0, \forall \delta \leq r-2
$$

$\forall \boldsymbol{x}$ in a neighborhood of $\boldsymbol{x}_{0}$, and

$$
L_{g} L_{f}^{r-1} h\left(\boldsymbol{x}_{0}\right) \neq 0
$$

The QP shown in (4) cannot be employed if the safety constraint is not of relative degree one, because the control input does not appear in the first derivative of CBFs [28], [42], [43]. To overcome this issue, the framework of a basic CBF needs to be augmented. The rest of this section introduces the approach presented in [28] to grant safety for a higher relative degree system, which will be utilized in Section VI.

Let us first consider when the relative degree of the system is two. To guarantee the forward invariance of the safe set $\mathcal{S}_{1}=\left\{\boldsymbol{x} \in \mathbb{R}^{n} \mid h_{1}(\boldsymbol{x}) \geq 0\right\}$, the first derivative of the CBF $h_{1}(\boldsymbol{x})$ should satisfy the following condition.

$$
\dot{h}_{1}(\boldsymbol{x})+\alpha_{1}\left(h_{1}(\boldsymbol{x})\right)=L_{f} h_{1}(\boldsymbol{x})+\alpha_{1}\left(h_{1}(\boldsymbol{x})\right) \geq 0
$$

To ensure the condition (7), let us define an additional CBF, where we set $\alpha_{1}\left(h_{1}(\boldsymbol{x})\right)=\gamma_{1} h_{1}(\boldsymbol{x})$ for simplicity, as

$$
\begin{aligned}
h_{2}(\boldsymbol{x}) & =\dot{h}_{1}(\boldsymbol{x})+\gamma_{1} h_{1}(\boldsymbol{x}) \\
& =L_{f} h_{1}(\boldsymbol{x})+\gamma_{1} h_{1}(\boldsymbol{x})
\end{aligned}
$$

whose zero superlevel set is $\mathcal{S}_{2}=\left\{\boldsymbol{x} \in \mathbb{R}^{n} \mid h_{2}(\boldsymbol{x}) \geq 0\right\}$. If there exists a positive constant $\gamma_{1}$ and a locally Lipschitz extended class $\mathcal{K}$ function $\alpha_{2}$ such that

$$
\begin{aligned}
& \sup _{\boldsymbol{u}}\left[\dot{h}_{2}(\boldsymbol{x}, \boldsymbol{u})+\alpha_{2}\left(h_{2}(\boldsymbol{x})\right)\right] \\
& =\sup _{\boldsymbol{u}}\left[L_{f}^{g} h_{1}(\boldsymbol{x})+L_{g} L_{f} h_{1}(\boldsymbol{x}) \boldsymbol{u}\right. \\
& \left.\quad+\gamma_{1} L_{f} h_{1}(\boldsymbol{x})+\alpha_{2}\left(h_{2}(\boldsymbol{x})\right)\right] \geq 0
\end{aligned}
$$

then $h_{2}$ is a valid CBF. This implies that by employing

$$
\dot{h}_{2}(\boldsymbol{x}, \boldsymbol{u})+\alpha_{2}\left(h_{2}(\boldsymbol{x})\right) \geq 0
$$

as the constraint of the QP, instead of (4b), the resultant input $\boldsymbol{u}^{*}$ renders the set $\mathcal{S}_{2}$ forward invariant and, in turn, the set $\mathcal{S}_{1}$ too. The following proposition generalizes this technique.
Proposition 2. [28, Thm. 1] Given a dynamical system (1), a sufficiently smooth CBF $h_{1}(\boldsymbol{x})$ with relative degree $r$ and a CBF $h_{r}(\boldsymbol{x})$ that can be evaluated recursively starting from $h_{1}(\boldsymbol{x})$ using the following equation:

$$
h_{\delta+1}(\boldsymbol{x})=\dot{h}_{\delta}(\boldsymbol{x})+\alpha_{\delta}\left(h_{\delta}(\boldsymbol{x})\right), 1 \leq \delta<r
$$

with $\alpha_{\delta}$ continuously differentiable extended class $\mathcal{K}$ functions, we define the set $K_{r}(\boldsymbol{x})$ as

$$
\begin{aligned}
& K_{r}(\boldsymbol{x})=\left\{\boldsymbol{u} \in \mathbb{R}^{m} \mid L_{f}^{r} h_{1}(\boldsymbol{x})+L_{g} L_{f}^{r-1} h_{1}(\boldsymbol{x}) \boldsymbol{u}\right. \\
& \left.+\sum_{i=1}^{r-1} \sum_{J \in\left(r_{-1}^{r-1}\right)} \prod_{j \in J} \frac{\partial \alpha_{j}}{\partial h_{j}} L_{f}^{r-i} h_{1}(\boldsymbol{x})+\alpha_{r}\left(h_{r}(\boldsymbol{x})\right) \geq 0\right\}
\end{aligned}
$$

where $\binom{r_{-1}}{i}$ is the set of $i$ combinations from the set $\{1 \cdots r-1\} \subset \mathbb{N}$ and $\alpha_{r}$ is a locally Lipschitz extended class $\mathcal{K}$ function. Then, any Lipschitz continuous controller $u \in K_{r}(\boldsymbol{x})$ will render the set $\mathcal{S}_{1}=\left\{\boldsymbol{x} \in \mathbb{R}^{n} \mid h_{1}(\boldsymbol{x}) \geq 0\right\}$ forward invariant.

## III. Problem Formulation

This paper considers a collision avoidance method for rigid bodies, the shape of which can be modeled by an ellipse or an ellipsoid. The rigid bodies, labeled through the index set $\mathcal{N}=\{1 \cdots n\}$, are distributed in the Euclidean space $\mathbb{R}^{d}$, as illustrated in Fig. 1. Note that this paper considers the scenario with $d=2$ or $d=3$. We denote the world coordinate frame as $\Sigma_{w}$. The coordinate frame of rigid body $i$ is defined as $\Sigma_{i}$, which is arranged at the center of rigid body $i$ so that its axes aligned with each axis of an ellipse or an ellipsoid. The relative pose of $\Sigma_{i}$ with respect to $\Sigma_{w}$ is described as $g_{i}=\left(\boldsymbol{p}_{i}, R_{i}\right) \in S E(d):=\mathbb{R}^{d} \times S O(d)$ with the position $\boldsymbol{p}_{i} \in \mathbb{R}^{d}$ and the orientation $R_{i} \in S O(d):=\{R \in$ $\left.\mathbb{R}^{d \times d} \mid R R^{\top}=I_{d}, \operatorname{det}(R)=1\right\}$. Hereafter, we denote a two-dimensional ellipse and three-dimensional ellipsoid as an ellipsoid all together to make notations simpler.

The area rigid body $i$ occupies is modeled as an ellipsoid described as

$$
\mathcal{E}_{i}=\left\{\boldsymbol{q} \in \mathbb{R}^{d} \mid\left(\boldsymbol{q}-\boldsymbol{p}_{i}\right)^{\top} R_{i} Q_{i}^{-2} R_{i}^{\top}\left(\boldsymbol{q}-\boldsymbol{p}_{i}\right)-1 \leq 0\right\}
$$

where $Q_{i}$ is a diagonal matrix having $q_{i m}$ as the $m$-th diagonal element corresponding with the length of the $m$-th axis of the ellipsoid. As detailed later, we assume that rigid body $i$ can obtain the pose $\left(\boldsymbol{p}_{j}, R_{j}\right)$ and shape $Q_{j}$ of other rigid bodies $j \in \mathcal{N} \backslash\{i\}$ through sensing or communications.

Let us denote the body velocity of rigid body $i$ relative to $\Sigma_{w}$ as $V_{i}^{b}=\left[\boldsymbol{v}_{i}^{\top} \boldsymbol{\omega}_{i}^{\top}\right]^{\top} \in \mathbb{R}^{d+\frac{d(d-1)}{2}}$, where $\boldsymbol{v}_{i} \in \mathbb{R}^{d}$ and $\boldsymbol{\omega}_{i} \in \mathbb{R}^{\frac{d(d-1)}{2}}$ are the translational and angular body velocity, respectively ${ }^{1}$. We also introduce the operator $\wedge$, which renders a skew-symmetric matrix, i.e., an element of $s o(d):=\left\{S \in\right.$ $\left.\mathbb{R}^{d \times d} \mid S+S^{\top}=O_{d}\right\}$. More specifically, $\wedge$ renders $s o(2)$ from $a \in \mathbb{R}$ and $s o(3)$ from $\boldsymbol{a}=\left[\begin{array}{lll}a_{1} & a_{2} & a_{3}\end{array}\right]^{\top} \in \mathbb{R}^{3}$ as

$$
\hat{a}=\left[\begin{array}{cc}
0 & -a \\
a & 0
\end{array}\right], \hat{\boldsymbol{a}}=\left[\begin{array}{ccc}
0 & -a_{3} & a_{2} \\
a_{3} & 0 & -a_{1} \\
-a_{2} & a_{1} & 0
\end{array}\right]
$$

We also define the inverse operator of $\wedge$ as $\vee$. By employing $\wedge$, the pose $g_{i}$ and the body velocity $V_{i}^{b}$ can be described by the following homogeneous representation.

$$
\begin{aligned}
g_{i} & =\left[\begin{array}{ll}
R_{i} & \boldsymbol{p}_{i} \\
0 & 1
\end{array}\right] \in \mathbb{R}^{(d+1) \times(d+1)} \\
\hat{V}_{i}^{b} & =\left[\begin{array}{cc}
\boldsymbol{\omega}_{i} & \boldsymbol{v}_{i} \\
0 & 0
\end{array}\right] \in \mathbb{R}^{(d+1) \times(d+1)}
\end{aligned}
$$

Then, the dynamics of rigid body $i$ can be modeled as the rigid body motion [44]

$$
\dot{g}_{i}=g_{i} \hat{V}_{i}^{b}
$$

Except for Section VI exemplifying the proposed CBF can be extendable to a nonholonomic system, the dynamics of rigid body $i$ is governed by (16), where the body velocity $V_{i}^{b}$ constitutes a control input for rigid body $i$.

This paper proposes a collision avoidance method for a team of rigid bodies described with (13). If we can obtain the minimum distance between $\mathcal{E}_{i}$ and $\mathcal{E}_{j}$ as $w_{i j}^{*}\left(g_{i}, g_{j}\right)$, the

[^0]
[^0]:    ${ }^{1}$ While an angular body velocity is a scalar for $d=2$, making a bold letter unsuitable, we use $\boldsymbol{u}$ for both $d=2$ and $d=3$ for notational simplicity.

![img-2.jpeg](img-2.jpeg)
(a) $h_{i j}\left(g_{i}, g_{j}, \boldsymbol{z}_{i j}\right)<0$
![img-3.jpeg](img-3.jpeg)
(b) $h_{i j}\left(g_{i}, g_{j}, \boldsymbol{z}_{i j}\right)>0$

Fig. 3. The separation condition evaluated by $h_{i j}$, the minimum signed distance from the supporting hyperplane $l_{i j}$. $h_{i j}$ returns zero on $l_{i j}$ while it takes the larger value as a point to be evaluated moves to the upper direction specified by the green normal vector at a point $\boldsymbol{m}_{i j}$. Note that if the supporting hyperplane $l_{i j}$ is not adequately prepared, $h_{i j}$ could become negative even when two rigid bodies are separated, as shown in (a).
safe set preventing collisions between rigid bodies $i$ and $j$ can be described as

$$
\mathcal{S}_{i j}=\left\{\left(g_{i}, g_{j}\right) \in S E(d) \times S E(d) \mid w_{i j}^{*}\left(g_{i}, g_{j}\right) \geq 0\right\}
$$

where this set has to be rendered forward invariant.

## IV. CBFs FOR ELLIPSOIDAL RIGID BODIES

This section presents a novel CBF, which enforces the forward invariance of the set $\mathcal{S}_{i j}$, namely preventing collisions between ellipsoidal rigid bodies $i$ and $j$. If $w_{i j}^{*}$ in (17) can be derived in analytical form, we can employ it as a CBF. However, as mentioned in Section I and [10]-[12], it is difficult to obtain the analytical solution of the distance between two ellipsoids (for both $d=2$ and $d=3$ ). Moreover, the numerical solutions of $w_{i j}^{*}$ cannot be utilized as a CBF either since the CBF-constraint in (4) requires to calculate the derivative of the CBF. To circumvent the difficulties, we formulate a novel CBF that employs a surrogate distance that can be derived in analytical form. More specifically, we utilize the signed distance from a supporting hyperplane of an ellipsoid to the other one, depicted as $h_{i j}$ in Fig. 3. Because $h_{i j}$ could take a shorter length than $w_{i j}^{*}$ with a naively prepared supporting hyperplane, we propose the framework that drives $h_{i j}$ to $w_{i j}^{*}$ based on the gradient of a signed distance, as shown in Fig. 2.

## A. Supporting Hyperplanes Separating Two Ellipsoids

We first define a supporting hyperplane $l_{i j}$ of an ellipsoid, which touches $\mathcal{E}_{i}$ at the point $\boldsymbol{m}_{i j}$, as shown in Fig. 4. Let us define the point $\boldsymbol{m}_{i j}$ as

$$
\begin{aligned}
\boldsymbol{m}_{i j}\left(g_{i}, \boldsymbol{z}_{i j}\right) & =\bar{Q}_{i}\left(R_{i}\right) \boldsymbol{z}_{i j}+\boldsymbol{p}_{i} \\
\left\|\boldsymbol{z}_{i j}\right\| & =1
\end{aligned}
$$

where $\bar{Q}_{i}\left(R_{i}\right)=R_{i} Q_{i} R_{i}^{\top}$ is a positive definite matrix. Hereafter, for notational simplicity, we will denote $\bar{Q}_{i}\left(R_{i}\right)$ as $\bar{Q}_{i}$. Note that $\bar{Q}_{i}^{\dagger} \bar{Q}_{i}=\bar{Q}_{i}^{2}=R_{i} Q_{i}^{2} R_{i}^{\top}$ holds since $Q_{i}$ is a diagonal matrix. The unit vector $\boldsymbol{z}_{i j} \in \mathbb{S}^{(d-1)}$ specifies a point on the boundary of the ellipsoid $\mathcal{E}_{i}$ as shown in Fig. 4. Then, the supporting hyperplane $l_{i j}$ is expressed as

$$
l_{i j}=\left\{\boldsymbol{q} \in \mathbb{R}^{d} \mid \boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1} \boldsymbol{q}-\left(1+\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1} \boldsymbol{p}_{i}\right)=0\right\}
$$

which is determined by $g_{i}$ and $\boldsymbol{z}_{i j}$.
![img-4.jpeg](img-4.jpeg)

Fig. 4. Relationship between a unit vector $\boldsymbol{z}_{i j}$ and a point $\boldsymbol{m}_{i j}$ on an ellipse characterized by $\bar{Q}_{i} . \boldsymbol{z}_{i j}$ in (19) specifies a point on the unit circle. Then, $\boldsymbol{z}_{i j}$ is transformed with the positive definite matrix $\bar{Q}_{i}$ to designate the point on the ellipse, which is adopted as a tangent point of a supporting hyperplane $l_{i j}$. Here, we omit $\boldsymbol{p}_{i}$ by setting the center of the ellipse $\boldsymbol{p}_{i}=\left[\begin{array}{ll}0 & 0\end{array}\right]^{-}$.

Let us utilize the supporting hyperplane $l_{i j}$ to derive a collision-free condition, in which two ellipsoids are separated by the supporting hyperplane. For this goal, we calculate the signed distance from the supporting hyperplane $l_{i j}$, which yields a positive value to a point in a different half-space with $\mathcal{E}_{i}$, and a negative value otherwise, as in Fig. 3. This signed distance from $l_{i j}$ to $\mathcal{E}_{j}$ renders the minimum value when the distance is evaluated with the point $\boldsymbol{n}_{i j} \in \mathcal{E}_{j}$ as

$$
\boldsymbol{n}_{i j}\left(g_{i}, g_{j}, \boldsymbol{z}_{i j}\right)=-\frac{1}{\left\|\bar{Q}_{j} \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|} \bar{Q}_{j}^{2} \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}+\boldsymbol{p}_{j}
$$

The minimum signed distance from $l_{i j}$ is then described as

$$
h_{i j}\left(g_{i}, g_{j}, \boldsymbol{z}_{i j}\right)=\frac{-\left\|\bar{Q}_{j} \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|+\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\top} \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}-1}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}
$$

As shown in Fig. 3, $h_{i j}\left(g_{i}, g_{j}, \boldsymbol{z}_{i j}\right)$ yields a positive value if and only if $\mathcal{E}_{i}$ and $\boldsymbol{n}_{i j}$ exist in the different half-space divided by $l_{i j}$. Note that the point $\boldsymbol{n}_{i j} \in \mathcal{E}_{j}$ is not the nearest point from the supporting hyperplane $l_{i j}$ (Fig. 3(a)) because this signed distance increases along with the direction of the normal vector of $l_{i j}$, depicted as the green arrow in Fig. 3.

Since $h_{i j}\left(g_{i}, g_{j}, \boldsymbol{z}_{i j}\right)>0$ signifies $\mathcal{E}_{i}$ and $\mathcal{E}_{j}$ are separated by the supporting hyperplane defined with $\boldsymbol{z}_{i j}$, the function $h_{i j}\left(g_{i}, g_{j}, \boldsymbol{z}_{i j}\right)$ could be employed as a CBF to achieve collision avoidance. However, this condition could be a conservative if $l_{i j}$ is prepared naively as shown in Fig. 3(a), where $h_{i j}\left(g_{i}, g_{j}, \boldsymbol{z}_{i j}\right)<0$ holds even if two ellipsoids are collision-free. In other words, a naive choice of $\boldsymbol{z}_{i j}$ makes $h_{i j}\left(g_{i}, g_{j}, \boldsymbol{z}_{i j}\right)$ smaller than the actual distance $w_{i j}^{*}$, resulting in too conservative evasive motion. To alleviate this gap, let us develop the following optimization problem that intends to maximize $h_{i j}\left(g_{i}, g_{j}, \boldsymbol{z}_{i j}\right)$ by rotating the supporting hyperplane on the boundary of $\mathcal{E}_{i}$.

$$
\begin{gathered}
\max _{\boldsymbol{z}_{i j}} h_{i j}\left(g_{i}, g_{j}, \boldsymbol{z}_{i j}\right) \\
\text { s.t. }\left\|\boldsymbol{z}_{i j}\right\|=1
\end{gathered}
$$

Notice that the optimization problem (23) maximizes $h_{i j}\left(g_{i}, g_{j}, \boldsymbol{z}_{i j}\right)$ over the unit vector $\boldsymbol{z}_{i j}$ while fixing two ellipsoids. In the rest of this subsection, let us denote $h_{i j}\left(g_{i}, g_{j}, \boldsymbol{z}_{i j}\right)$ as $h_{i j}\left(\boldsymbol{z}_{i j}\right)$ for notational simplicity since we discuss the property of the optimization problem (23) that presumes fixed ellipsoids. Then, in the next subsection, we

discuss how to update $\boldsymbol{z}_{i j}$ with moving ellipsoidal rigid bodies to establish a real-time collision avoidance methodology.
To elucidate the meaning of the optimization problem (23), let us introduce the optimization problem

$$
\begin{gathered}
\min _{\boldsymbol{x}, \boldsymbol{y}, \boldsymbol{w}}\|\boldsymbol{w}\| \\
\text { s.t. } f_{i}(\boldsymbol{x}) \leq 0, f_{j}(\boldsymbol{y}) \leq 0 \\
\boldsymbol{y}-\boldsymbol{x}=\boldsymbol{w}
\end{gathered}
$$

where $f_{i}(\boldsymbol{x}):=\left(\boldsymbol{x}-\boldsymbol{p}_{i}\right)^{\top} \bar{Q}_{i}^{-2}\left(\boldsymbol{x}-\boldsymbol{p}_{i}\right)-1 \leq 0$ signifies the condition $\boldsymbol{x} \in \mathcal{E}_{i}$. Therefore, the problem (24) returns the shortest distance between two ellipsoids, namely $w_{i j}^{*}$, as its optimal solution $\left\|\boldsymbol{w}^{*}\right\|$. Then, the following theorem formalizes the relationship between $w_{i j}^{*}$ and $h_{i j}\left(\boldsymbol{z}_{i j}\right)$.
Theorem 1. Suppose that two ellipsoids $\mathcal{E}_{i}$ and $\mathcal{E}_{j}$ have no overlap, namely $\mathcal{E}_{i} \cap \mathcal{E}_{j}=\emptyset$ holds. Then, the optimization problem (23) is the dual of the problem (24). Furthermore, the strong duality holds between the optimization problems (24) and (23), namely the following condition holds.

$$
w_{i j}^{*}=h_{i j}\left(\boldsymbol{z}_{i j}^{*}\right) \geq h_{i j}\left(\boldsymbol{z}_{i j}\right)
$$

Proof. See Appendix A. Note that our preliminary work [35] proves the case of $d=2$. The proof in Appendix A modifies the one in [35] to suit the formulation of this paper.

Theorem 1 signifies that the proposed update law, maximizing $h_{i j}\left(\boldsymbol{z}_{i j}\right)$ by rotating the supporting hyperplane $l_{i j}$ as in Fig. 2, renders $h_{i j}\left(\boldsymbol{z}_{i j}\right)$ the actual distance $w_{i j}^{*}$ between two ellipsoids. In addition, the equation (25) implies $h_{i j}\left(\boldsymbol{z}_{i j}\right)>0$ serves as a sufficient condition for avoiding collisions, even if $\boldsymbol{z}_{i j}$ does not converge to the maximizer of (23), $\boldsymbol{z}_{i j}^{*}$.

## B. CBFs Integrating Rotating Supporting Hyperplanes

This subsection first presents the collision avoidance method for two ellipsoidal rigid bodies. We then extend the result to allow distributed implementation for $n$ rigid bodies, assuming each rigid body can communicate its state. Note that we again regard $h_{i j}$ as a function of $g_{i}, g_{j}$, and $\boldsymbol{z}_{i j}$, although we have omitted the dependency of $h_{i j}$ for notational simplicity.

As presented in the optimization problem (23) and Fig. 2, the evaluation of $h_{i j}$ requires a supporting hyperplane $l_{i j}$ between rigid bodies $i$ and $j$. Without loss of generality, let us suppose that a rigid body with a lower ID equips a supporting hyperplane. Since the supporting hyperplane should be rotated to minimize the gap between $h_{i j}$ and $w_{i j}^{*}$, we regard $\boldsymbol{z}_{i j}$ as an additional state variable of rigid body $i(i<j)$. In other words, to avoid collisions between rigid bodies $i$ and $j$, we need to control $\left(g_{i}, g_{j}, \boldsymbol{z}_{i j}\right)$.

In our proposed method, $\boldsymbol{z}_{i j}$ is updated based on the following dynamics

$$
\dot{\boldsymbol{z}}_{i j}=\left(I_{d}-\boldsymbol{z}_{i j} \boldsymbol{z}_{i j}^{\top}\right) \boldsymbol{u}_{\boldsymbol{z}_{i j}}
$$

with the input $\boldsymbol{u}_{\boldsymbol{z}_{i j}} \in \mathbb{R}^{d}$ rendered by a QP for collision avoidance, as will be presented in (29). Note that the update rule (26) restricts $\boldsymbol{z}_{i j}$ on the unit ball. Since the pose of rigid bodies $i$ and $j$ follow the dynamics shown in (16), the
combined control input to be considered for two rigid bodies $i$ and $j$ can be denoted as $\boldsymbol{u}_{i j}=\left[\boldsymbol{v}_{i}^{\top} \boldsymbol{\omega}_{i}^{\top} \boldsymbol{u}_{\boldsymbol{z}_{i j}}^{\top} \boldsymbol{v}_{j}^{\top} \boldsymbol{\omega}_{j}^{\top}\right]^{\top}$.

Let us denote the nominal input of $\boldsymbol{z}_{i j}$ as $\boldsymbol{u}_{\mathrm{nom}, \boldsymbol{z}_{i j}}$ that will be integrated into the CBF-based framework, as discussed in Section II. Then, $\boldsymbol{u}_{\mathrm{nom}, \boldsymbol{z}_{i j}}$ that intends to maximize $h_{i j}$ can be derived from the gradient ascent law as

$$
\boldsymbol{u}_{\mathrm{nom}, \boldsymbol{z}_{i j}}=k_{z} \frac{\partial h_{i j}}{\partial \boldsymbol{z}_{i j}}, \quad k_{z}>0
$$

which drives $h_{i j}$ to the local maximum. Note that the maximum point of $h_{i j}$ changes as two rigid bodies move around during the operation. Hence, the update rule should make $h_{i j}$ converge fast enough to keep up with their motion. This requirement is easily satisfied by setting a large enough $k_{z}$ since $\boldsymbol{z}_{i j}$ is a virtual variable that does not depend on the physical dynamics of the rigid bodies. The calculated result of $\partial h_{i j} / \partial \boldsymbol{z}_{i j}$ will be shown in (31c), where we derive $h_{i j}$.

Having defined the state to be considered, we are now ready to present a novel collision avoidance strategy for ellipsoidal rigid bodies. Let us introduce a new safe set $\overline{\mathcal{S}}_{i j}$ that integrates an angle of the supporting hyperplane, namely $\boldsymbol{z}_{i j}$, as follows.

$$
\overline{\mathcal{S}}_{i j}=\left\{\left(g_{i}, g_{j}, \boldsymbol{z}_{i j}\right) \in S E(d) \times S E(d) \times \mathbb{S}^{(d-1)} \mid h_{i j} \geq 0\right\}
$$

Since $w_{i j}^{*}>h_{i j}$ holds from Theorem 1, the original safe set $\mathcal{S}_{i j}$ can be rendered forward invariant if the proposed CBF guarantees the forward invariance of $\overline{\mathcal{S}}_{i j}$. To achieve this goal, we propose the following QP, which synthesizes the nominal input and the proposed CBF $h_{i j}$ as

$$
\begin{aligned}
& \boldsymbol{u}_{i j}^{*}=\underset{\boldsymbol{u}_{i j}}{\arg \min }\left\|\boldsymbol{u}_{i j}-\boldsymbol{u}_{\mathrm{nom}, i j}\right\|^{2} \\
& \text { s.t. } \dot{h}_{i j} \geq-\alpha\left(h_{i j}\right)
\end{aligned}
$$

The following lemma presents $\dot{h}_{i j}$ in both $d=3$ and $d=2$.
Lemma 1. The time-derivative of the CBF (22) along with the dynamics represented as RBM (16) in $d=3$ is derived as

$$
\begin{aligned}
\dot{h}_{i j} & =\boldsymbol{\zeta}_{i j} R_{i} \boldsymbol{\omega}_{i}+\boldsymbol{\eta}_{i j} R_{i} \boldsymbol{v}_{i}+\boldsymbol{\mu}_{i j}\left(I_{d}-\boldsymbol{z}_{i j} \boldsymbol{z}_{i j}^{\top}\right) \boldsymbol{u}_{z_{i j}} \\
& +\boldsymbol{\nu}_{i j} R_{j} \boldsymbol{\omega}_{j}+\boldsymbol{\xi}_{i j} R_{j} \boldsymbol{v}_{j}
\end{aligned}
$$

where each coefficient can be derived as

$$
\begin{aligned}
& \overline{\boldsymbol{\zeta}}_{i j}=\frac{\rho}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|^{3}} \overline{\boldsymbol{z}}_{i j}^{\top} \bar{Q}_{i}^{-2} \dot{\boldsymbol{z}}_{i j} \\
& +\frac{1}{\sigma}\left(\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1} \bar{Q}_{j}^{2}\right)\left(\left(\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right)^{\wedge}-\bar{Q}_{i}^{-1} \dot{\boldsymbol{z}}_{i j}\right) \\
& +\frac{1}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}\left(\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\top} \bar{Q}_{i}^{-1} \dot{\boldsymbol{z}}_{i j}-\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1}\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\wedge}\right) \\
& \boldsymbol{\eta}_{i j}=-\frac{1}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|} \boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1} \\
& \boldsymbol{\mu}_{i j}=\frac{\rho}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|^{3}} \boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-2}+\frac{1}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\top} \bar{Q}_{i}^{-1} \\
& -\frac{1}{\sigma} \boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1} \bar{Q}_{j}^{2} \bar{Q}_{i}^{-1} \\
& \boldsymbol{\nu}_{i j}=-\frac{1}{\sigma} \boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1} \bar{Q}_{j}^{2}\left(\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right)^{\wedge} \\
& \boldsymbol{\xi}_{i j}=\frac{1}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|} \boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1}
\end{aligned}
$$

with

$$
\begin{aligned}
\rho & =\left(1-\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\top} \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}+\left\|\bar{Q}_{j} \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|\right) \\
\sigma & =\left\|\bar{Q}_{j} \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|
\end{aligned}
$$

Furthermore, $\dot{h}_{i j}$ in $d=2$ is calculated as

$$
\begin{aligned}
\dot{h}_{i j} & =\tilde{\boldsymbol{\epsilon}}_{i j} \boldsymbol{\omega}_{i}+\boldsymbol{\eta}_{i j} R_{i} \boldsymbol{v}_{i}+\boldsymbol{\mu}_{i j}\left(I_{d}-\boldsymbol{z}_{i j} \boldsymbol{z}_{i j}^{\top}\right) \boldsymbol{u}_{z_{i j}} \\
& +\tilde{\boldsymbol{\nu}}_{i j} \boldsymbol{\omega}_{j}+\boldsymbol{\xi}_{i j} R_{j} \boldsymbol{v}_{j}
\end{aligned}
$$

with

$$
\begin{aligned}
& \tilde{\boldsymbol{\epsilon}}_{i j}=-\frac{\rho}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|^{3}} \boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-2} \hat{1} \boldsymbol{z}_{i j} \\
& -\frac{1}{\sigma}\left(\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1} \bar{Q}_{j}^{2}\right)\left(\hat{1} \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}-\bar{Q}_{i}^{-1} \hat{1} \boldsymbol{z}_{i j}\right) \\
& -\frac{1}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}\left(\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\top} \bar{Q}_{i}^{-1} \hat{1} \boldsymbol{z}_{i j}+\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1} \hat{1}\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)\right) \\
& \tilde{\boldsymbol{\nu}}_{i j}=\frac{1}{\sigma} \boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1} \bar{Q}_{j}^{2} \hat{1}\left(\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right)
\end{aligned}
$$

Proof. See Appendix B.
Let us emphasize that, as stated in the contributions in Section I-B, this paper newly derives $\dot{h}_{i j}$ along with the dynamics expressed as RBM (16) to develop a collision avoidance controller for both 2D and 3D environments. Note that the difference between (30) and (33) mainly stems from the operator $\wedge$ that returns a slightly different result in $d=3$ and $d=2$, as in (14). Appendix B also provides more detailed explanations of this difference. The discussion in the rest of this section proceeds with the QP (29) with $\dot{h}_{i j}$ in (30) because the results are consistent for both $d=2$ and $d=3$.

Although the QP (29) rectifies the nominal input to avoid collisions, (29b) cannot be evaluated in a distributed fashion because $\dot{h}_{i j}$ in (30) requires the information of both rigid bodies' control inputs and a supporting hyperplane. Nevertheless, the proposed method can be distributed by assuming the communication between rigid bodies as follows.

Assumption 1. Rigid body $i \in \mathcal{N}$ can acquire the pose and the shape information of other rigid bodies, namely $\left(\boldsymbol{p}_{j}, R_{j}\right)$ and $Q_{j}, \forall j \in \mathcal{N} \backslash\{i\}$. In addition, rigid body $j$ can receive $\boldsymbol{z}_{i j}$ from rigid bodies $i, \forall i \in\{1 \cdots j-1\}$.

Assumption 1 implies each rigid body can acquire other rigid bodies' poses and shapes. As presented in [8], [9], [45], it is realizable by equipping some sensors on rigid bodies, such as a visual sensor or LiDAR. In contrast, $\boldsymbol{z}_{i j}$ with $i<j$ needs to be communicated because $\boldsymbol{z}_{i j}$ is a virtual variable of rigid body $i$ and cannot be estimated by rigid body $j$ from its sensor data. Nevertheless, the dimension of $\boldsymbol{z}_{i j}$ is only $d$ and easy to transfer. Note that although we assume a complete graph in Assumption 1 to make a discussion simpler, the proposed method can be extendable to a distance-based graph, such as the $\Delta$-disk proximity graph [46], as conducted in [13], [47].

With Assumption 1, let us consider evaluating the QP (29) in a distributed manner, where rigid bodies $i$ and $j$ with $i<$ $j$ are responsible for determining $\boldsymbol{u}_{i}=\left[\boldsymbol{v}_{i}^{\top} \boldsymbol{\omega}_{i}^{\top} \boldsymbol{u}_{\boldsymbol{z}_{i j}}^{\top}\right]^{\top} \in$ $\mathbb{R}^{2 d+\frac{d(d-1)}{2}}$ and $\boldsymbol{u}_{j}=\left[\boldsymbol{v}_{j}^{\top} \boldsymbol{\omega}_{j}^{\top}\right]^{\top} \in \mathbb{R}^{d+\frac{d(d-1)}{2}}$, respectively.

Then, the CBF condition (29b) and (30) can be decomposed into the following two conditions.

$$
\begin{gathered}
\boldsymbol{\zeta}_{i j} R_{i} \boldsymbol{\omega}_{i}+\boldsymbol{\eta}_{i j} R_{i} \boldsymbol{v}_{i}+\boldsymbol{\mu}_{i j}\left(I_{d}-\boldsymbol{z}_{i j} \boldsymbol{z}_{i j}^{\top}\right) \boldsymbol{u}_{z_{i j}} \geq-\frac{1}{2} \alpha\left(h_{i j}\right) \\
\boldsymbol{\nu}_{i j} R_{j} \boldsymbol{\omega}_{j}+\boldsymbol{\xi}_{i j} R_{j} \boldsymbol{v}_{j} \geq-\frac{1}{2} \alpha\left(h_{i j}\right)
\end{gathered}
$$

Note that both rigid bodies $i$ and $j$ can calculate $h_{i j}$ and coefficients as they have all the information needed to evaluate (22) and (31) under Assumption 1. Therefore, the conditions (35a) and (35b) can be evaluated by rigid bodies $i$ and $j$, respectively. In summary, the QP (29) is distributed with the following two QPs.

$$
\begin{aligned}
& \boldsymbol{u}_{i}^{*}=\underset{\boldsymbol{u}_{i}}{\arg \min} \quad\left\|\boldsymbol{u}_{i}-\boldsymbol{u}_{\mathrm{nom}, i}\right\|^{2} \\
& \text { s.t. } \boldsymbol{\zeta}_{i j} R_{i} \boldsymbol{\omega}_{i}+\boldsymbol{\eta}_{i j} R_{i} \boldsymbol{v}_{i}+\boldsymbol{\mu}_{i j}\left(I_{d}-\boldsymbol{z}_{i j} \boldsymbol{z}_{i j}^{\top}\right) \boldsymbol{u}_{z_{i j}} \geq-\frac{1}{2} \alpha\left(h_{i j}\right)
\end{aligned}
$$

for rigid body $i$ and

$$
\begin{aligned}
& \boldsymbol{u}_{j}^{*}=\underset{\boldsymbol{u}_{j}}{\arg \min} \quad\left\|\boldsymbol{u}_{j}-\boldsymbol{u}_{\mathrm{nom}, j}\right\|^{2} \\
& \text { s.t. } \boldsymbol{\nu}_{i j} R_{j} \boldsymbol{\omega}_{j}+\boldsymbol{\xi}_{i j} R_{j} \boldsymbol{v}_{j} \geq-\frac{1}{2} \alpha\left(h_{i j}\right)
\end{aligned}
$$

for rigid body $j$. Note that each QP is only responsible for calculating the control input of one rigid body. This is in contrast to the QP (29), which calculates both $\boldsymbol{u}_{i}$ and $\boldsymbol{u}_{j}$ in a centralized manner.

The existence of the control input satisfying the constraint in (36b) and (37b), also called as the validity of the CBF [23], [28], is guaranteed by the following theorem.

Theorem 2. The function $h_{i j}$ in (22) is a valid CBF with the distributed QPs in (36) and (37) for both $d=2$ and $d=3$ when $\boldsymbol{u}_{i} \in \mathbb{R}^{2 d+\frac{d(d-1)}{2}}, \boldsymbol{u}_{j} \in \mathbb{R}^{d+\frac{d(d-1)}{2}}$.

Proof. Let us first discuss the case of $d=3$. To guarantee both QPs (36) and (37) have a solution for any $\left(\boldsymbol{p}_{i}, R_{i}\right),\left(\boldsymbol{p}_{j}, R_{j}\right) \in$ $S E(3)$ and $\boldsymbol{z}_{i j} \in \mathbb{S}^{2}$, we need to prove the coefficients of the constraints in (36b) and (37b) never become zero vecotors, namely $\left[\boldsymbol{\zeta}_{i j}^{\top} \boldsymbol{\eta}_{i j}^{\top} \boldsymbol{\mu}_{i j}^{\top}\right]^{\top} \neq \mathbf{0}$ and $\left[\boldsymbol{\nu}_{i j}^{\top} \boldsymbol{\xi}_{i j}^{\top}\right]^{\top} \neq \mathbf{0}$. These two conditions are always satisfied because $\left\|\boldsymbol{\eta}_{i j}\right\|=\left\|\boldsymbol{\xi}_{i j}\right\|=1$ holds from (31b) and (31e). Therefore, each QP can find a solution satisfying the constraints by appropriately selected $\boldsymbol{v}_{i}$ and $\boldsymbol{v}_{j}$. The above discussion also applies with $d=2$ because $\dot{h}_{i j}$ in $d=2$ also has $\boldsymbol{\eta}_{i j}$ and $\boldsymbol{\xi}_{i j}$ as the coefficients of $\boldsymbol{v}_{i}$ and $\boldsymbol{v}_{j}$, as shown in (33). This completes the proof.

Theorem 2 means that, for any $\left(\boldsymbol{p}_{i}, R_{i}\right),\left(\boldsymbol{p}_{j}, R_{j}\right) \in S E(d)$ and $\boldsymbol{z}_{i j} \in \mathbb{S}^{(d-1)}$, both QPs (36) and (37) can find the control input that renders the set $\dot{\mathcal{S}}_{i j}$ forward invariant.

The above discussion can be easily extended to the case of $n$ rigid bodies. Because the rigid body with a smaller ID among any two rigid bodies has a supporting hyperplane, rigid body $i$ owns $(n-i)$ supporting hyperplanes. Let us introduce the vector combining states of all supporting hyperplanes rigid body $i$ has as $\boldsymbol{z}_{i}=\left[\boldsymbol{z}_{i i+1}^{\top} \boldsymbol{z}_{i i+2}^{\top} \cdots \boldsymbol{z}_{i n}^{\top}\right]^{\top}$ with $\boldsymbol{z}_{n}=\emptyset$, which has the following dynamics as (26)

$$
\dot{\boldsymbol{z}}_{i}=\left(I_{(n-i)} \otimes\left(I_{d}-\boldsymbol{z}_{i j} \boldsymbol{z}_{i j}^{\top}\right)\right) \boldsymbol{u}_{\boldsymbol{z}_{i}}
$$

where $\otimes$ represents the Kronecker product and $\boldsymbol{u}_{\boldsymbol{z}_{i}}=$ $\left[\boldsymbol{u}_{\boldsymbol{z}_{i+1}}^{T} \cdots \boldsymbol{u}_{\boldsymbol{z}_{i-1}}^{T}\right]^{\top}$. Then, the ensemble state of rigid body $i$ becomes $X_{i}=\left(\boldsymbol{p}_{i}, R_{i}, \boldsymbol{z}_{i}\right)$. Rigid body $i$ calculates the control input $\boldsymbol{u}_{i}=\left[\boldsymbol{v}_{i}^{\top} \boldsymbol{\omega}_{i}^{\top} \boldsymbol{u}_{\boldsymbol{z}_{i}}^{\top}\right]^{\top}$ for this ensemble state $X_{i}$ by solving the QP

$$
\begin{aligned}
\boldsymbol{u}_{i}^{*} & =\underset{\boldsymbol{u}_{i}}{\arg \min \left\|\boldsymbol{u}_{i}-\boldsymbol{u}_{\mathrm{nom}, i}\right\|_{W}^{2}} \\
\text { s.t. } \boldsymbol{\zeta}_{i j} R_{i} \boldsymbol{\omega}_{i}+\boldsymbol{\eta}_{i j} R_{i} \boldsymbol{v}_{i} & +\boldsymbol{\mu}_{i j}\left(I_{d}-\boldsymbol{z}_{i j} \boldsymbol{z}_{i j}^{\top}\right) \boldsymbol{u}_{\boldsymbol{z}_{i j}} \\
& \geq-\frac{1}{2} \alpha\left(h_{i j}\right), \forall j>i \\
\boldsymbol{\nu}_{i j} R_{j} \boldsymbol{\omega}_{j}+\boldsymbol{\xi}_{i j} R_{j} \boldsymbol{v}_{j} & \geq-\frac{1}{2} \alpha\left(h_{i j}\right), \forall j<i
\end{aligned}
$$

where $\|\boldsymbol{a}\|_{W}=\boldsymbol{a}^{\top} W \boldsymbol{a}$ with a diagonal matrix $W=$ $\operatorname{diag}\left(\beta_{\boldsymbol{v}} I_{d}, \beta_{\boldsymbol{\omega}} I_{d(d-1) / 2}, I_{d(n-i)}\right)$ that adjusts the unit difference and can also reflect a user preference on which translational and angular velocity is modified more by $\beta_{\boldsymbol{v}}, \beta_{\boldsymbol{\omega}}>0$.

## V. SimULATION ReSultS WITH RIGID BODY MOTION

This section presents simulation studies, where their movies, including the one in Section VI, can be found in the supplementary material. The proposed method only requires solving (39) to generate the collision-free input for each rigid body. The computational time of (39) is short, within 1-2 ms in most cases for all simulation studies, where we utilize quadprog in MATLAB and a PC equipped with Intel i7-10700 CPU.

## A. Two-Dimensional Case

This subsection presents simulation results with two elliptical rigid bodies in a 2D environment, which demonstrates the proposed CBF and the input for the supporting hyperplane successfully prevent collisions between two elliptical rigid bodies. The length of the major and the minor axis of two rigid bodies are randomly selected from the range from 1.0 m to 2.0 m and from 0.4 m to 0.8 m , respectively. The initial positions of the two rigid bodies are $\boldsymbol{p}_{1}=[-3-3]^{\top}$ and $\boldsymbol{p}_{2}=\left[\begin{array}{ll}2 & 0\end{array}\right]^{\top}$, as shown in Fig. 5(a). Figure 5(a) also illustrates the supporting hyperplane incorporated in the proposed CBF as a black line together with the shortest line connecting the supporting hyperplane and the other rigid body, namely its length corresponding with $h_{12}$. We employ $\alpha(h)=10 h$ as an extended class $\mathcal{K}$ function. We set $\beta_{\boldsymbol{v}}=1$ and $\beta_{\boldsymbol{\omega}}=0.5$. The nominal input to $\boldsymbol{z}_{12}$ is set to $\boldsymbol{u}_{\mathrm{nom}, \boldsymbol{z}_{12}}=20\left(\partial h_{12} / \partial \boldsymbol{z}_{12}\right)$. Note that the initial value of $\boldsymbol{z}_{12}$ for the supporting hyperplane is set randomly within a range that separates two rigid bodies.

The snapshots of the simulation are shown in Fig. 5, where a supporting hyperplane rotates on rigid body 1, adapting to the motion of the rigid bodies. As a result, the length of the green line, $h_{12}$, takes almost the same value as the actual distance between rigid bodies, $w_{12}^{*}$. Figure 6 depicts the evolution of $h_{12}$, shown as a blue line, together with the actual distance $w_{12}^{*}$ calculated from the optimization problem (24), as a red dashed line. Note that $w_{12}^{*}$ is calculated just for comparison, and the proposed method does not necessitate solving (24). By comparing $h_{12}$ and $w_{12}^{*}$ in Fig. 6, $h_{12}$ follows $w_{12}^{*}$ fast enough to eliminate the conservativeness, namely the error between $h_{12}$ and $w_{12}^{*}$. In addition, because $h_{12}$ is always larger than 0 , collision avoidance is successfully achieved.
![img-5.jpeg](img-5.jpeg)

Fig. 5. Snapshots of the simulation with the proposed collision avoidance method, where rigid body 1 depicted in blue has a supporting hyperplane. The supporting hyperplane is rotated to maximize the signed distance $h_{12}$, which corresponds with the length of the green line.
![img-6.jpeg](img-6.jpeg)

Fig. 6. Evolution of the CBF $h_{12}$ and the actual minimum distance between two elliptical rigid bodies, $w_{12}^{*}$. The CBF $h_{12}$ always takes a smaller value than that of $w_{12}^{*}$ while keeping a close value with it.

## B. Comparative Simulation in Two-Dimension

This subsection provides the comparative study between our proposed CBF and the one employing the discriminant of a cubic characteristic equation [13], which showcases the proposed method exhibits more favorable avoidance trajectories. To make it easier to understand the difference, this subsection considers a scenario, where an elliptical robot avoids a fixed elliptical obstacle. If we index the elliptical robot and the obstacle as rigid bodies 1 and 2 , the QP to be solved by rigid body 1 , namely, the robot, is written as

$$
\begin{aligned}
& \boldsymbol{u}_{1}^{*}=\underset{\boldsymbol{u}_{1}}{\arg \min \left\|\boldsymbol{u}_{1}-\boldsymbol{u}_{\mathrm{nom}, 1}\right\|_{W}^{2}} \\
& \text { s.t. } \dot{h}_{12} \geq-\alpha\left(h_{12}\right)
\end{aligned}
$$

with

$$
\dot{h}_{12}=\tilde{\boldsymbol{\zeta}}_{12} \boldsymbol{\omega}_{1}+\boldsymbol{\eta}_{12} R_{1} \boldsymbol{v}_{1}+\boldsymbol{\mu}_{12}\left(I_{d}-\boldsymbol{z}_{12} \boldsymbol{z}_{12}^{\top}\right) \boldsymbol{u}_{\boldsymbol{z}_{12}}
$$

Note that since there are no control inputs for the obstacle, the terms having $\omega_{2}$ and $\boldsymbol{v}_{2}$ in (33) are removed in (41).

![img-7.jpeg](img-7.jpeg)

Fig. 7. Simulation results of the proposed CBF with $\gamma=1,10,100$. (a): All three cases change their attitudes in the direction smooth avoidance is achieved. (b): The value of the proposed CBF $h$ corresponds with the actual distance between the supporting hyperplane and the obstacle.

Let us introduce the CBF presented in [13], with which we will compare the performance of our proposed CBF. The CBF in [13] is based on the following cubic characteristic equation

$$
\begin{aligned}
J(x) & =\operatorname{det}\left(x T_{i}\left(g_{i}\right)-T_{j}\left(g_{j}\right)\right) \\
T_{i}\left(g_{i}\right) & =\left(g_{i}^{-1}\right)^{\top}\left[\begin{array}{cc}
Q_{i}^{-2} & 0 \\
0 & -1
\end{array}\right] g_{i}^{-1}
\end{aligned}
$$

where $g_{i}$ is defined in (15a). Let us denote the discriminant of (42) as $D_{i j}(x)$. The discriminant $D_{i j}(x)$ has two properties: (i) if ellipses $i$ and $j$ do not overlap, $D_{i j}(x)>0$ holds and (ii) if ellipses $i$ and $j$ contact with each other without overlapping, $D_{i j}(x)=0$ holds. The work [13] presents a CBF

$$
h_{\text {discr }}=D_{i j}(x)
$$

to leverage the above property of $D_{i j}(x)$ for collision avoidance. For more detail, please see [12], [13]. Note that, differently from the proposed CBF $h_{i j}$, the CBF $h_{\text {discr }}=$ $D_{i j}(x)$ cannot be extended to three-dimensional ellipsoidal rigid bodies because it requires solving the higher-dimensional characteristic equations, the solution of which is divided by cases or not obtained in explicit form.

In the following comparative studies, we consider a scenario in which the robot, rigid body 1 , moves toward a goal while avoiding an obstacle, rigid body 2 . The shapes of rigid bodies are modeled as ellipses, which sizes are specified as $Q_{1}=$ $\operatorname{diag}(0.5,0.2)$ and $Q_{2}=\operatorname{diag}(3,2)$, respectively. The initial and the goal pose of the robot are $\left(\boldsymbol{p}_{\text {ini }}, R_{\text {ini }}\right)=\left(\left[\begin{array}{ll}12 & 0\end{array}\right]^{\top}, I_{2}\right)$ and $\left(\boldsymbol{p}_{\text {goal }}, R_{\text {goal }}\right)=\left(\left[\begin{array}{ll}0 & 0\end{array}\right]^{\top}, I_{2}\right)$. The nominal input to $\boldsymbol{z}_{12}$ is set to $u_{\text {nom }, \boldsymbol{z}_{12}}=10\left(\partial h_{12} / \partial \boldsymbol{z}_{12}\right)$. We set $\beta_{\boldsymbol{v}}=1$ and $\beta_{\boldsymbol{\omega}}=0.3$ for a weighting matrix $W$. As an extended class $\mathcal{K}$ function, we utilize $\alpha(h)=\gamma h$ for both CBFs, where we analyze how the result changes in each CBF with $\gamma=1,10,100$.

Figures 7(a) and 8(a) show the trajectories of the robot with the proposed CBF $h_{12}$ and one with $h_{\text {discr }}=D_{12}$ in [13], respectively. Along with each trajectory, we plot a pose of the robot every 0.2 s . In all six trajectories, the robot successfully avoids the collision with the obstacle shown in black. Nevertheless, the results of the CBF in [13] exhibit unreasonable rotational inputs when the robot approaches the obstacle, as shown in red ellipses in Fig. 8(a). More concretely, the robot changes its attitude in the direction not to contribute to collision avoidance. This unreasonable change
![img-8.jpeg](img-8.jpeg)

Fig. 8. Simulation results with the CBF proposed in [13], where the discriminant of the cubic characteristic equation is employed as a CBF. (a): With $\gamma=10$ shown in red, the robot shows an unreasonable change of its attitude, which does not contribute to securing a margin between the obstacle. The result with $\gamma=1$ shows a sharper change in its attitude with a larger detour than that of the proposed method in Fig. 7. (b): A CBF in [13] does not correspond with the distance, resulting in enormous values.
in the attitude appears because the discriminant utilized in $h_{\text {discr }}$ does not necessarily have a proportional relationship with the Euclidean distance between ellipses. In addition, since the physical interpretation of $h_{\text {discr }}$ is difficult to understand, the tuning of parameters is prone to be complicated, as seen in Fig. 8(a), in which all three trajectories vary greatly. The evolution of $h_{\text {discr }}$, shown in Fig. 8(b), also exhibits difficulties in understanding its physical interpretation because its value takes a significantly larger value than that of the distance. In contrast, the robots with the proposed CBF $h_{12}$ change their attitudes so that the distance from the obstacle increases. Figure 7(b) also verifies that the value of CBF takes almost the same value as the distance.

## C. Three-Dimensional Case

This subsection demonstrates the proposed collision avoidance method (39) with the rigid body network in a 3D space. The rigid body network is composed of sixteen ellipsoidal rigid bodies, eight of which are located at the lower-left corner and the others at the upper-right corner at the initial time, as shown in Fig. 9(a). Each group moves toward the other corner of the space while rearranging the positions of the rigid bodies in each group. The rearranged positions in the goal configuration and the ID of rigid bodies are randomly set. The attitude of each ellipsoidal rigid body is determined so that its major axis directs to the goal position. The length of the major axis of all rigid bodies is set to 1 m , while the length of the other axes is determined randomly from 0.3 m to 0.7 m . With denoting the goal pose for rigid body $i$ as $\left(\boldsymbol{p}_{i}^{\text {goal }}, R_{i}^{\text {goal }}\right)$, the nominal inputs for the translational and angular body velocities are calculated as $\boldsymbol{v}_{\text {nom }, i}=k_{\boldsymbol{v}} R_{i}^{\top}\left(\boldsymbol{p}_{i}^{\text {goal }}-\boldsymbol{p}_{i}\right)$ and $\boldsymbol{\omega}_{\text {nom }, i}=\frac{k_{\boldsymbol{\omega}}}{2}\left(R_{i}^{\top} R_{i}^{\text {goal }}-\left(R_{i}^{\top} R_{i}^{\text {goal }}\right)^{\top}\right)^{\vee}$, where $k_{\boldsymbol{v}}=3$, $k_{\boldsymbol{\omega}}=0.5$, and $\vee$ is the inverse operator of $\wedge$ as defined below (14). The weighting matrix $W$ is set with $\beta_{\boldsymbol{v}}=0.1$ and $\beta_{\boldsymbol{\omega}}=1$. The gain for the nominal input (27) for the supporting hyperplane is set to $k_{\boldsymbol{z}}=20 /\left(1+h_{i j}^{2}\right)$, which is designed to take a smaller value when two ellipsoids are located far away. The extended class $\mathcal{K}$ function is $\alpha(h)=10 h$.

Figure 9 shows the snapshots of the simulation result. At $t=0.3 \mathrm{~s}$ and $t=0.5 \mathrm{~s}$, two groups meet at the center of the

![img-9.jpeg](img-9.jpeg)
(a) $t=0 \mathrm{~s}$
![img-10.jpeg](img-10.jpeg)
(c) $t=0.5 \mathrm{~s}$
![img-11.jpeg](img-11.jpeg)
(d) $t=0.9 \mathrm{~s}$
![img-12.jpeg](img-12.jpeg)
(e) $t=1.2 \mathrm{~s}$
![img-13.jpeg](img-13.jpeg)
(f) $t=4.0 \mathrm{~s}$

Fig. 9. Snapshots of the simulation with the proposed collision avoidance method. The sixteen ellipsoidal rigid bodies are separated into two groups, which are located at the lower-left and the upper-right corner, respectively.
![img-14.jpeg](img-14.jpeg)

Fig. 10. Evolution of CBFs $h_{i j}, \forall j \in \mathcal{N} \backslash\{i\}, \forall i \in \mathcal{N} . h_{i j}$ taking the minimum value is highlighted as a red line, which keeps positive values.
field, where the proposed CBF rectifies the nominal input to prevent collision. During this time period, the CBFs, depicted in Fig. 10, take smaller values but remain positive, achieving collision avoidance. After $t=0.9 \mathrm{~s}$, each group approaches the final destination, where the position of each rigid body in the group is shuffled from the one in the initial coordination. Still, all the rigid bodies smoothly converge to their final destinations while preventing collisions.
![img-15.jpeg](img-15.jpeg)

Fig. 11. Vehicle model. The control input is the acceleration of the coordinate frame $\Sigma_{i}$, denoted as $\dot{\mathrm{v}}_{i}=u_{a i}$, and steering angular velocity $\dot{\varphi}_{i}=u_{\omega i}$.

## VI. Extension to the 2D Vehicle Model

This section demonstrates that the proposed method achieves collision avoidance for a vehicle with nonholonomic dynamics by applying the methodology introduced in Section II to the proposed CBF. The considered vehicle model is described in Fig. 11, where $\left(\boldsymbol{p}_{i}, R_{i}\right)$ is the relative pose of $\Sigma_{i}$ with respect to $\Sigma_{w} . \varphi_{i}$ is the steering angle and $\mathrm{v}_{i}$ signifies the translational velocity of the center of mass. The length of the wheelbase is $\ell$, and the center of mass is located at a distance of $\varsigma \ell$ from the rear axle. The control input is the acceleration, namely $\dot{\mathrm{v}}_{i}:=u_{a i}$, and the steering angular velocity $\dot{\varphi}_{i}:=u_{\omega i}$. With assuming there is no wheel slip and applying the bicycle model as discussed in [48, Chapter 2], the dynamics of the vehicles are expressed as

$$
\begin{aligned}
\dot{g}_{i} & =g_{i} \dot{V}_{i}^{b}, \dot{V}_{i}^{b}=\left[\begin{array}{cc}
\boldsymbol{\omega}_{i} & \boldsymbol{v}_{i} \\
0 & 0
\end{array}\right] \\
\boldsymbol{v}_{i} & =\frac{\mathrm{v}_{i}}{\sqrt{1+\left(\varsigma \tan \varphi_{i}\right)^{2}}}\left[\begin{array}{c}
1 \\
\varsigma \tan \varphi_{i}
\end{array}\right] \\
\boldsymbol{\omega}_{i} & =\frac{\mathrm{v}_{i} \tan \varphi_{i}}{\ell \sqrt{1+\left(\varsigma \tan \varphi_{i}\right)^{2}}} \\
\dot{\mathrm{v}}_{i} & =u_{a i} \\
\dot{\varphi}_{i} & =u_{\omega i}
\end{aligned}
$$

where we expressed the dynamics in [48] by employing (45a) to suit this paper.

Differently from the dynamics (16) considered in the previous section, the vehicle dynamics (45) have relative degree two, i.e., the input $\left[u_{a i} u_{\omega i}\right]^{\top}$ shows up in $\bar{h}_{i j}$ but not in $\bar{h}_{i j}$. To match the relative degree of the supporting hyperplane with the vehicle dynamics, we modify the dynamics of the supporting hyperplane as

$$
\left[\begin{array}{l}
\dot{\boldsymbol{z}}_{i j} \\
\dot{\boldsymbol{r}}_{i j}
\end{array}\right]=\left[\begin{array}{cc}
0 & \left(I_{d}-\boldsymbol{z}_{i j} \boldsymbol{z}_{i j}^{\top}\right) \\
0 & 0
\end{array}\right]\left[\begin{array}{l}
\boldsymbol{z}_{i j} \\
\boldsymbol{r}_{i j}
\end{array}\right]+\left[\begin{array}{l}
0 \\
I_{d}
\end{array}\right] \boldsymbol{u}_{\boldsymbol{r}_{i j}}
$$

The nominal input for the supporting hyperplane is designed as

$$
\boldsymbol{u}_{\mathrm{nom}, \boldsymbol{r}_{i j}}=k_{z} \frac{d}{d t}\left(\frac{\partial h_{i j}}{\partial \boldsymbol{z}_{i j}}\right), k_{z}>0
$$

with the intention that the input (47) integrated through the dynamics (46) resembles the one in (27), hence maximizing the distance between the supporting hyperplane and the other vehicle. Similar to $\boldsymbol{z}_{i}=\left[\boldsymbol{z}_{i i+1}^{\top} \boldsymbol{z}_{i i+2}^{\top} \cdots \boldsymbol{z}_{i n}^{\top}\right]^{\top}$ introduced in the

last paragraph of Section IV-B, we introduce the augmented vector $\boldsymbol{r}_{i}=\left[\boldsymbol{r}_{i i+1}^{\top} \cdots \boldsymbol{r}_{i n}^{\top}\right]^{\top}$ and $\boldsymbol{u}_{\boldsymbol{r}_{i}}=\left[\boldsymbol{u}_{\boldsymbol{r}_{i i+1}}^{\top} \cdots \boldsymbol{u}_{\boldsymbol{r}_{i n}}^{\top}\right]^{\top}$. Hereafter, we consider $X_{i}=\left(\boldsymbol{p}_{i}, R_{i}, v_{i}, \varphi_{i}, \boldsymbol{z}_{i}, \boldsymbol{r}_{i}\right)$ and $\boldsymbol{u}_{i}=$ $\left[\boldsymbol{v}_{i}^{\top} \boldsymbol{\omega}_{i}^{\top} \boldsymbol{u}_{\boldsymbol{r}_{i}}^{\top}\right]^{\top}$ as the state and input of rigid body $i$.

The change of the dynamics of the supporting hyperplane replaces $\boldsymbol{u}_{\boldsymbol{z}_{i j}}$ in $\dot{h}_{i j}$, shown in (33) in the case of $d=2$, into $\boldsymbol{r}_{i j}$ as follows, while other parts remain in the same value.

$$
\begin{aligned}
\dot{h}_{i j} & =\tilde{\boldsymbol{\zeta}}_{i j} \boldsymbol{\omega}_{i}+\boldsymbol{\eta}_{i j} R_{i} \boldsymbol{v}_{i}+\boldsymbol{\mu}_{i j}\left(I_{d}-\boldsymbol{z}_{i j} \boldsymbol{z}_{i j}^{\top}\right) \boldsymbol{r}_{i j} \\
& +\tilde{\boldsymbol{\nu}}_{i j} \boldsymbol{\omega}_{j}+\boldsymbol{\xi}_{i j} R_{j} \boldsymbol{v}_{j}
\end{aligned}
$$

Because the equation (48) contains neither the input $\boldsymbol{u}_{i}$ nor $\boldsymbol{u}_{j}, \dot{h}_{i j}$ cannot be served as a constraint for the input. Hence, following the concept in Section II, we employ the following CBF for the considered vehicle model.

$$
h_{i j, v e h}=\dot{h}_{i j}+h_{i j}
$$

Then, the constraint to achieve collision avoidance between rigid bodies $i$ and $j$ can be expressed as follows.

$$
\dot{h}_{i j, v e h}=\ddot{h}_{i j}+\dot{h}_{i j} \geq-h_{i j, v e h}
$$

Let us take a closer look of (50) to confirm the condition (50) can be evaluated in a distributed manner as in (36) and (37). While $\dot{h}_{i j}$ is given in (48), $\ddot{h}_{i j}$ is calculated as

$$
\begin{aligned}
\ddot{h}_{i j} & =\tilde{\boldsymbol{\zeta}}_{i j} \dot{\boldsymbol{\omega}}_{i}+\boldsymbol{\eta}_{i j} R_{i} \dot{\boldsymbol{v}}_{i}+\boldsymbol{\mu}_{i j}\left(I_{d}-\boldsymbol{z}_{i j} \boldsymbol{z}_{i j}^{\top}\right) \dot{\boldsymbol{r}}_{i j} \\
& +\tilde{\boldsymbol{\nu}}_{i j} \dot{\boldsymbol{\omega}}_{j}+\boldsymbol{\xi}_{i j} R_{j} \dot{\boldsymbol{v}}_{j}+A\left(X_{i}, X_{j}\right) \\
A\left(X_{i}, X_{j}\right) & =\dot{\tilde{\boldsymbol{\zeta}}}_{i j} \boldsymbol{\omega}_{i}+\dot{\boldsymbol{\eta}}_{i j} R_{i} \boldsymbol{v}_{i}+\boldsymbol{\eta}_{i j} \dot{R}_{i} \boldsymbol{v}_{i} \\
& +\tilde{\boldsymbol{\mu}}_{i j}\left(I_{d}-\boldsymbol{z}_{i j} \boldsymbol{z}_{i j}^{\top}\right) \boldsymbol{r}_{i j}+\boldsymbol{\mu}_{i j}\left(-2 \boldsymbol{z}_{i j} \boldsymbol{z}_{i j}^{\top}\right) \boldsymbol{r}_{i j} \\
& +\tilde{\boldsymbol{\nu}}_{i j} \boldsymbol{\omega}_{j}+\boldsymbol{\xi}_{i j} R_{j} \boldsymbol{v}_{j}+\boldsymbol{\xi}_{i j} \dot{R}_{j} \boldsymbol{v}_{j}
\end{aligned}
$$

where the term $A\left(X_{i}, X_{j}\right)$ only depends on the states of rigid bodies $i$ and $j$ and is independent of their control inputs. Then, let us divide (50) into the following two conditions, where we substitute (51) into (50).

$$
\begin{aligned}
\tilde{\boldsymbol{\zeta}}_{i j} \dot{\boldsymbol{\omega}}_{i} & +\boldsymbol{\eta}_{i j} R_{i} \dot{\boldsymbol{v}}_{i}+\boldsymbol{\mu}_{i j}\left(I_{d}-\boldsymbol{z}_{i j} \boldsymbol{z}_{i j}^{\top}\right) \dot{\boldsymbol{r}}_{i j} \\
& \geq-\frac{1}{2}\left(A\left(X_{i}, X_{j}\right)+\dot{h}_{i j}+h_{i j, v e h}\right) \\
\tilde{\boldsymbol{\nu}}_{i j} \dot{\boldsymbol{\omega}}_{j}+\boldsymbol{\xi}_{i j} R_{j} \dot{\boldsymbol{v}}_{j} \geq-\frac{1}{2}\left(A\left(X_{i}, X_{j}\right)+\dot{h}_{i j}+h_{i j, v e h}\right)
\end{aligned}
$$

Because $\left(\dot{\boldsymbol{\omega}}_{i}, \dot{\boldsymbol{v}}_{i}\right)$ and $\left(\dot{\boldsymbol{\omega}}_{j}, \dot{\boldsymbol{v}}_{j}\right)$ can be derived by differentiating (45b) and (45c) with time, the control inputs $\boldsymbol{u}_{i}$ and $\boldsymbol{u}_{j}$ appear in the left-hand side of (53a) and (53b), respectively. Because each rigid body needs to calculate $A\left(X_{i}, X_{j}\right)$ and $\dot{h}_{i j}$, we need a similar requirement for communicated information to Assumption 1 to achieve distributed computation.
Assumption 2. Rigid body $i \in \mathcal{N}$ can acquire $\left(\boldsymbol{p}_{j}, R_{j}, \mathrm{v}_{j}, \varphi_{j}\right)$ and $Q_{j}, \forall j \in \mathcal{N} \backslash\{i\}$. In addition, rigid body $j$ can receive $\left[\boldsymbol{z}_{i j}^{\top} \boldsymbol{r}_{i j}^{\top}\right]^{\top}$ from rigid bodies $i, \forall i \in\{1 \cdots j-1\}$.

Assumption 2 requires each vehicle to communicate the state describing its dynamics (45). This is a similar requirement to Assumption 1, which necessitates the communication of $\left(\boldsymbol{p}_{i}, R_{i}\right)$, the state of RBM (16).
![img-16.jpeg](img-16.jpeg)

Fig. 12. Snapshots of the simulation with the two vehicles having the nonholonomic dynamics (45). Each vehicle traces a dashed line depicted in the same color as itself. Note that the snapshots (b) and (c) are zoomed in to detail how the vehicles avoid a collision.
![img-17.jpeg](img-17.jpeg)

Fig. 13. Evolution of CBF $h_{12}$ and the actual distance between vehicles $w_{12}^{*}$.

With Assumption 2, the QP evaluated by rigid body $i$ is represented as

$$
\begin{aligned}
\boldsymbol{u}_{i}^{*}= & \underset{\boldsymbol{u}_{i}}{\arg \min }\left\|\boldsymbol{u}_{i}-\boldsymbol{u}_{\mathrm{nom}, i}\right\|_{W}^{2} \\
\text { s.t. } & \tilde{\boldsymbol{\zeta}}_{i j} \dot{\boldsymbol{\omega}}_{i}+\boldsymbol{\eta}_{i j} R_{i} \dot{\boldsymbol{v}}_{i}+\boldsymbol{\mu}_{i j}\left(I_{d}-\boldsymbol{z}_{i j} \boldsymbol{z}_{i j}^{\top}\right) \dot{\boldsymbol{r}}_{i j} \\
& \geq-\frac{1}{2}\left(A\left(X_{i}, X_{j}\right)+\dot{h}_{i j}+h_{i j, v e h}\right), \forall j>i \\
\tilde{\boldsymbol{\nu}}_{i j} \dot{\boldsymbol{\omega}}_{j} & +\boldsymbol{\xi}_{i j} R_{j} \dot{\boldsymbol{v}}_{j} \\
& \geq-\frac{1}{2}\left(A\left(X_{i}, X_{j}\right)+\dot{h}_{i j}+h_{i j, v e h}\right), \forall j<i
\end{aligned}
$$

with $W=\operatorname{diag}\left(\beta_{\mathrm{v}}, \beta_{\varphi}, I_{2 d(n-i)}\right)$.
Having extended the proposed method to achieve collision avoidance for the rigid bodies with the vehicle dynamics (45), we are now ready to conduct a simulation to demonstrate the effectiveness of the method. Figure 12(a) shows the initial configurations of the two vehicles, where each vehicle traces a dashed line depicted in the same color as itself. The parameters

of the vehicle shown in Fig. 11 are set as $\ell=2.7 \mathrm{~m}$ and $\varsigma=0.5$, respectively. We model these vehicles as elliptical rigid bodies defined with $Q_{1}=Q_{2}=\operatorname{diag}(4,2)$. We set $\beta_{s}=1$ and $\beta_{\varphi}=10$. The gain for the nominal input (47) for the supporting hyperplane is set to $k_{s}=1000 /\left(1+h_{12}^{2}\right)$, which takes smaller value when two vehicles are far away. We limit the angular velocity of the unit vector $\boldsymbol{z}_{i j}$ in the range of $\left[-\frac{\pi}{3}, \frac{\pi}{3}\right]$ to avoid too fast rotational motion of the supporting hyperplane. The nominal control input for the vehicle is set as $u_{\text {nom,ai }}=-\left(\mathrm{v}_{i}-5\right)$ and $u_{\text {nom,wi }}=$ $-0.1 e_{1}-e_{2}-1.5 \varphi_{i}$. Here, $e_{1}$ is the lateral position error between the center of mass of the vehicle and the line to be traced. $e_{2}=\frac{1}{2}\left(R_{i}^{\top} R_{i}^{\text {line }}-\left(R_{i}^{\top} R_{i}^{\text {line }}\right)^{\top}\right)^{\vee}$ evaluates the error between the attitude of the vehicle and the direction of the line expressed by $R_{i}^{\text {line }}$.

Figure 12 shows the snapshots of the simulation. As the two vehicles approach each other at $t=2 \mathrm{~s}$, the supporting hyperplane starts to rotate, and each vehicle turns the steering wheel to avoid a collision. At $t=4 \mathrm{~s}$, collision avoidance is successfully achieved, and vehicles return to the line to be traced at $t=8 \mathrm{~s}$. Figure 13 shows the value of $h_{12}$, the length of the green line in Fig. 12, where its value keeps positive. Figure 13 also depicts the actual distance between two vehicles, i.e. $w_{12}^{*}$. We can confirm that $h_{12}$ tracks $w_{12}^{*}$ fast enough to eliminate the error between them.

## VII. CONCLUSION

This paper presented a collision avoidance method for ellipsoidal rigid bodies in $S E(2)$ and $S E(3)$, where we leverage the CBF employing the supporting hyperplane on the rigid body. We first formulated the problem with the rigid bodies governed by the rigid body motion. Then, we introduced a signed distance from the supporting hyperplane of an ellipsoid to the other ellipsoid to derive the condition that makes the ellipsoidal rigid bodies collision-free. However, we observed that the derived condition could render a smaller value than the actual distance between two rigid bodies if the supporting hyperplane is naively prepared. To prevent such a conservative evaluation, we designed the optimization problem that rotates the supporting hyperplane to maximize the signed distance from the supporting hyperplane to the other rigid body. We then proved that the maximum value of this optimization problem is equivalent to the actual distance between two ellipsoidal rigid bodies. This signed distance is leveraged as a CBF to achieve collision avoidance, where the supporting hyperplane is updated by a gradient-based input to eliminate conservativeness. The proposed CBF is implemented in a QP that allows each rigid body to compute the collision-free input in a distributed fashion under communication. The simulation studies demonstrated the validity of the proposed framework in both 2D and 3D environments. Finally, we showcased the presented CBF can be extended to the nonholonomic system.

## APPENDIX A

## Proof of Theorem 1

This appendix proves Theorem 1. The dual function of the optimization problem (24) is expressed as

$$
\begin{aligned}
& g\left(\lambda_{i}, \lambda_{j}, \boldsymbol{s}\right)=\inf _{\boldsymbol{x}, \boldsymbol{y}, \boldsymbol{w}}\left(\|\boldsymbol{w}\|+\lambda_{i} f_{i}(\boldsymbol{x})+\lambda_{j} f_{j}(\boldsymbol{y})\right. \\
& \left.+\boldsymbol{s}^{\top}(\boldsymbol{y}-\boldsymbol{x}-\boldsymbol{w})\right) \\
& =\left\{\begin{array}{cc}
\inf _{\boldsymbol{x}}\left(\lambda_{i} f_{i}(\boldsymbol{x})-\boldsymbol{s}^{\top} \boldsymbol{x}\right) & \|\boldsymbol{s}\| \leq 1 \\
+\inf _{\boldsymbol{y}} \inf _{\boldsymbol{y}}\left(\lambda_{j} f_{j}(\boldsymbol{y})+\boldsymbol{s}^{\top} \boldsymbol{y}\right) & \lambda_{i}, \lambda_{j} \geq 0 \\
-\infty & \text { otherwise }
\end{array}\right.
\end{aligned}
$$

with the Lagrange multipliers $\lambda_{i}, \lambda_{j}$, and $\boldsymbol{s}$. Let us calculate the first term of (55b), i.e., $\inf _{\boldsymbol{x}}\left(\lambda_{i} f_{i}(\boldsymbol{x})-\boldsymbol{s}^{\top} \boldsymbol{x}\right)$. Here, we introduce $\bar{x}_{i}=\bar{Q}_{i}^{-1} \boldsymbol{x}, \bar{\boldsymbol{p}}_{i}=\bar{Q}_{i}^{-1} \boldsymbol{p}_{i}$, and $\bar{s}_{i}=\bar{Q}_{i} \boldsymbol{s}$ to simplify the equation as

$$
\begin{aligned}
& \inf _{\boldsymbol{x}}\left(\lambda_{i} f_{i}(\boldsymbol{x})-\boldsymbol{s}^{\top} \boldsymbol{x}\right) \\
= & \inf _{\boldsymbol{x}}\left(\lambda_{i}\left(\boldsymbol{x}-\boldsymbol{p}_{i}\right)^{\top} \bar{Q}_{i}^{-2}\left(\boldsymbol{x}-\boldsymbol{p}_{i}\right)-\lambda_{i}-\boldsymbol{s}^{\top} \boldsymbol{x}\right) \\
= & \inf _{\boldsymbol{x}}\left(\lambda_{i}\left\|\bar{x}_{i}-\left(\bar{p}_{i}+\frac{1}{2 \lambda_{i}} \bar{s}_{i}\right)\right\|^{2}-\bar{s}_{i}^{\top} \bar{p}_{i}-\frac{1}{4 \lambda_{i}}\left\|\bar{s}_{i}\right\|^{2}-\lambda_{i}\right) \\
= & -\bar{s}_{i}^{\top} \bar{p}_{i}-\frac{\left\|\bar{s}_{i}\right\|^{2}+4 \lambda_{i}^{2}}{4 \lambda_{i}}
\end{aligned}
$$

Similarly, the second term in (55b) can be transformed as

$$
\inf _{\boldsymbol{y}}\left(\lambda_{j} f_{j}(\boldsymbol{y})+\boldsymbol{s}^{\top} \boldsymbol{y}\right)=\bar{s}_{j}^{\top} \bar{p}_{j}-\frac{\left\|\bar{s}_{j}\right\|^{2}+4 \lambda_{j}^{2}}{4 \lambda_{j}}
$$

with $\bar{p}_{j}=\bar{Q}_{j}^{-1} \boldsymbol{p}_{j}$ and $\bar{s}_{j}=\bar{Q}_{j} \boldsymbol{s}$. From (55b), (56), and (57), the dual problem can be expressed as

$$
\begin{gathered}
\max _{\boldsymbol{s}, \lambda_{i}, \lambda_{j}}-\bar{s}_{i}^{\top} \bar{p}_{i}-\frac{\left\|\bar{s}_{i}\right\|^{2}+4 \lambda_{i}^{2}}{4 \lambda_{i}}+\bar{s}_{j}^{\top} \bar{p}_{j}-\frac{\left\|\bar{s}_{j}\right\|^{2}+4 \lambda_{j}^{2}}{4 \lambda_{j}} \\
\text { s.t. } \bar{s}_{i}=\bar{Q}_{i} \boldsymbol{s}, \bar{s}_{j}=\bar{Q}_{j} \boldsymbol{s},\|\boldsymbol{s}\| \leq 1, \lambda_{i}, \lambda_{j} \geq 0
\end{gathered}
$$

We will next evaluate the second and fourth terms in (58a). For this goal, let us define a function $M(a, \tau)$ as

$$
M(a, \tau)=-\frac{a+4 \tau^{2}}{4 \tau} \quad(a, \tau \geq 0)
$$

and consider $\max _{a, \tau} M(a, \tau)$.
In the case of $a>0$, the gradients of $M$ are

$$
\frac{\partial M}{\partial \tau}=\frac{(\sqrt{a}-2 \tau)(\sqrt{a}+2 \tau)}{4 \tau^{2}}, \frac{\partial M}{\partial a}=-\frac{1}{4 \tau}
$$

This implies that the function $M(a, \tau)$ has no extremum for all $a>0$, and for $\tau \geq 0$ it has the maximum value at $\tau^{*}(a)=$ $\sqrt{a} / 2$. Hence, the following equation holds.

$$
\max _{a, \tau} M(a, \tau)=\max _{a}-\sqrt{a} \quad(a>0)
$$

When $a=0$ holds, the maximum value of $M(a, \tau)$ is

$$
\max _{a, \tau} M(a, \tau)=\max _{a}-\tau=0
$$

Given that (62) is equivalent to the result of substituting $a=0$ into (61), we can unify them as

$$
\max _{a, \tau} M(a, \tau)=\max _{a}-\sqrt{a} \quad(a \geq 0)
$$

Since the second and fourth terms in (58a) are equal to $M\left(\left\|\bar{s}_{i}\right\|^{2}, \lambda_{i}\right)$ and $M\left(\left\|\bar{s}_{j}\right\|^{2}, \lambda_{j}\right)$, respectively, the problem (58) can be simplified by employing (63) as

$$
\begin{gathered}
\max _{\boldsymbol{s}}-\overline{\boldsymbol{s}}_{i}^{\top} \overline{\boldsymbol{p}}_{i}-\left\|\overline{\boldsymbol{s}}_{i}\right\|+\overline{\boldsymbol{s}}_{j}^{\top} \overline{\boldsymbol{p}}_{j}-\left\|\overline{\boldsymbol{s}}_{j}\right\| \\
\text { s.t. }\|\boldsymbol{s}\| \leq 1 \\
\overline{\boldsymbol{s}}_{i}=\bar{Q}_{i} \boldsymbol{s}, \overline{\boldsymbol{s}}_{j}=\bar{Q}_{j} \boldsymbol{s}
\end{gathered}
$$

Let us parameterize $s$ as

$$
\boldsymbol{s}=\frac{\varrho}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|} \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}
$$

with $0 \leq \varrho \leq 1$ to satisfy the constraint (64b). By substituting (65), $\overline{\boldsymbol{s}}=\bar{Q}_{i} \boldsymbol{s}$ and $\overline{\boldsymbol{s}}_{j}=\bar{Q}_{j} \boldsymbol{s}$, the dual problem (64) can be expressed as

$$
\max _{\varrho, \boldsymbol{z}_{i j}} \varrho \frac{-\left\|\bar{Q}_{j} \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|+\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\top} \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}-1}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|} \overline{h_{i j}\left(\boldsymbol{z}_{i j}\right)}
$$

s.t. $\left\|\boldsymbol{z}_{i j}\right\|=1$,

$$
0 \leq \varrho \leq 1
$$

Notice that the objective function (66a) can be expressed as $\varrho h_{i j}\left(\boldsymbol{z}_{i j}\right)$. When two ellipsoids $\mathcal{E}_{i}$ and $\mathcal{E}_{j}$ have no overlap, there always exists $\boldsymbol{z}_{i j}$, namely a supporting hyperplane, realizing $h_{i j}\left(\boldsymbol{z}_{i j}\right)>0$. Because the variable $\varrho$ is independent of $\boldsymbol{z}_{i j}, \varrho=1$ should hold to maximize (66a). Substituting $\varrho=1$ into (66a) leads to the optimization problem (23). This result concludes that the problem (23) is the dual of the optimization problem (24) if $\mathcal{E}_{i} \cap \mathcal{E}_{j}=\emptyset$. Furthermore, because the optimization problem (24) satisfies the Slater's Condition [49, Sec. 5.2.3], the solution of (24) is equal to the solution of (23), which completes the proof.

## APPENDIX B

## Proof of Lemma 1

This appendix first derives the time derivatives of the proposed CBF in the case of $d=3$, then briefly discusses how $h_{i j}$ differs in $d=2 . h_{i j}\left(g_{i}, g_{j}, \boldsymbol{z}_{i j}\right)$ is recalled here for ease of reference.

$$
\begin{aligned}
& h_{i j}\left(g_{i}, g_{j}, \boldsymbol{z}_{i j}\right)= \\
& -\frac{\left\|\bar{Q}_{j} \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}+\frac{\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\top} \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}-\frac{1}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}
\end{aligned}
$$

Note that $\bar{Q}_{i}=R_{i} Q_{i} R_{i}^{\top}$ is a positive definite matrix since $Q_{i}$ is a diagonal matrix. Also, $\bar{Q}_{i}^{\top} \bar{Q}_{i}=\bar{Q}_{i}^{2}=R_{i} Q_{i}^{2} R_{i}^{\top}$ holds.

From (16), the dynamics of $\boldsymbol{p}_{i}$ and $R_{i}$ are expressed as

$$
\begin{aligned}
\dot{\boldsymbol{p}}_{i} & =R_{i} \boldsymbol{v}_{i} \\
\dot{R}_{i} & =R_{i} \dot{\boldsymbol{\omega}}_{i}
\end{aligned}
$$

$\boldsymbol{z}_{i j}$ specifying a supporting hyperplane is governed by the following dynamics.

$$
\dot{\boldsymbol{z}}_{i j}=\left(I_{d}-\boldsymbol{z}_{i j} \boldsymbol{z}_{i j}^{\top}\right) \boldsymbol{u}_{z_{i j}}
$$

In the case of $d=3$, we use the following properties the operator $\wedge$ and the rotation matrix $R \in S O(3)$ satisfy for any skew symmetric matrix $\dot{\boldsymbol{\omega}}$ and $\boldsymbol{a}, \boldsymbol{b} \in \mathbb{R}^{3}$.

$$
\begin{aligned}
\dot{\boldsymbol{a}} \boldsymbol{b} & =-\dot{\boldsymbol{b}} \boldsymbol{a} \\
R \dot{\boldsymbol{\omega}} R^{\top} & =(R \boldsymbol{\omega})^{\wedge}
\end{aligned}
$$

We first derive the time derivative of the third term in (67).

$$
\begin{aligned}
& \frac{d}{d t}\left(\frac{1}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}\right)=\frac{d}{d t}\left(\frac{1}{\sqrt{\left(R_{i}^{\top} \boldsymbol{z}_{i j}\right)^{\top} Q_{i}^{-2}\left(R_{i}^{\top} \boldsymbol{z}_{i j}\right)}}\right) \\
& =-\frac{1}{2\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|^{3}}\left(2\left(R_{i}^{\top} \boldsymbol{z}_{i j}\right)^{\top} Q_{i}^{-2} \frac{d}{d t}\left(R_{i}^{\top} \boldsymbol{z}_{i j}\right)\right) \\
& =-\frac{1}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|^{3}}\left(\left(R_{i}^{\top} \boldsymbol{z}_{i j}\right)^{\top} Q_{i}^{-2}\left(R_{i} \dot{\boldsymbol{\omega}}_{i}\right)^{\top} \boldsymbol{z}_{i j}\right. \\
& \left.+\left(R_{i}^{\top} \boldsymbol{z}_{i j}\right)^{\top} Q_{i}^{-2} R_{i}^{\top}\left(I_{d}-\boldsymbol{z}_{i j} \boldsymbol{z}_{i j}^{\top}\right) \boldsymbol{u}_{z_{i j}}\right)
\end{aligned}
$$

The equations (68) and (69) were substituted into (73). The first term of (73) can be expressed as

$$
\begin{aligned}
& \boldsymbol{z}_{i j}^{\top} R_{i} Q_{i}^{-2}\left(R_{i} \dot{\boldsymbol{\omega}}_{i}\right)^{\top} \boldsymbol{z}_{i j}=-\boldsymbol{z}_{i j}^{\top} R_{i} Q_{i}^{-2} \dot{\boldsymbol{\omega}}_{i} R_{i}^{\top} \boldsymbol{z}_{i j} \\
& =-\boldsymbol{z}_{i j}^{\top} R_{i} Q_{i}^{-2} R_{i}^{\top} R_{i} \dot{\boldsymbol{\omega}}_{i} R_{i}^{\top} \boldsymbol{z}_{i j} \\
& =-\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-2}\left(R_{i} \boldsymbol{\omega}_{i}\right)^{\wedge} \boldsymbol{z}_{i j} \\
& =\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-2} \dot{\boldsymbol{z}}_{i j} R_{i} \boldsymbol{\omega}_{i}
\end{aligned}
$$

Note that $\dot{\boldsymbol{\omega}}_{i}=-\dot{\boldsymbol{\omega}}_{i}^{\top}$ is utilized in (74a). In (74b), $R_{i}^{\top} R_{i}=I_{d}$ is substituted. In (74c) and (74d), the properties (72) and (71) are utilized, respectively. By substituting (74d) into (73), we obtain

$$
\begin{aligned}
& \frac{d}{d t}\left(\frac{1}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}\right) \\
& =-\frac{\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-2} \dot{\boldsymbol{z}}_{i j} R_{i} \boldsymbol{\omega}_{i}+\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-2}\left(I_{d}-\boldsymbol{z}_{i j} \boldsymbol{z}_{i j}^{\top}\right) \boldsymbol{u}_{z_{i j}}}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|^{3}}
\end{aligned}
$$

We next derive the time derivative of the second term in (67).

$$
\begin{aligned}
& \frac{d}{d t}\left(\frac{\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\top} \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}\right) \\
& =\frac{d}{d t}\left(\frac{1}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}\right)\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\top} \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j} \\
& +\frac{1}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}\left(\frac{d}{d t}\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\top}\right) \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j} \\
& +\frac{1}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\top} \frac{d}{d t}\left(R_{i} Q_{i}^{-1} R_{i}^{\top} \boldsymbol{z}_{i j}\right)
\end{aligned}
$$

The derivative term of (76a) has been obtained in (75). By substituting (68), the equation (76b) can be expressed as

$$
\begin{aligned}
& \frac{1}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}\left(R_{j} \boldsymbol{v}_{j}-R_{i} \boldsymbol{v}_{i}\right)^{\top} \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j} \\
& =\frac{1}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|} \boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1}\left(R_{j} \boldsymbol{v}_{j}-R_{i} \boldsymbol{v}_{i}\right)
\end{aligned}
$$

The third term (76c) can be calculated as

$$
\begin{aligned}
& \frac{1}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\top} \frac{d}{d t}\left(R_{i} Q_{i}^{-1} R_{i}^{\top} \boldsymbol{z}_{i j}\right) \\
& =\frac{1}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\top}\left(\left(R_{i} \hat{\boldsymbol{\omega}}_{i}\right) Q_{i}^{-1} R_{i}^{\top} \boldsymbol{z}_{i j}\right. \\
& \left.+R_{i} Q_{i}^{-1}\left(R_{i} \hat{\boldsymbol{\omega}}_{i}\right)^{\top} \boldsymbol{z}_{i j}+R_{i} Q_{i}^{-1} R_{i}^{\top}\left(I_{d}-\boldsymbol{z}_{i j} \boldsymbol{z}_{i j}^{\top}\right) \boldsymbol{u}_{z_{i j}}\right)
\end{aligned}
$$

The first and the second term in (78b) can be expressed as follows, where we omit $\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|^{-1}$.

$$
\begin{aligned}
& \left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\top} R_{i} \hat{\boldsymbol{\omega}}_{i} Q_{i}^{-1} R_{i}^{\top} \boldsymbol{z}_{i j}-\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\top} R_{i} Q_{i}^{-1} \hat{\boldsymbol{\omega}}_{i} R_{i}^{\top} \boldsymbol{z}_{i j} \\
& =\boldsymbol{z}_{i j}^{\top} R_{i} Q_{i}^{-1} R_{i}^{\top} R_{i} \hat{\boldsymbol{\omega}}_{i}^{\top} R_{i}^{\top}\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right) \\
& \quad-\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\top} R_{i} Q_{i}^{-1} R_{i}^{\top} R_{i} \hat{\boldsymbol{\omega}}_{i} R_{i}^{\top} \boldsymbol{z}_{i j} \\
& =-\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1}\left(R_{i} \boldsymbol{\omega}_{i}\right)^{\wedge}\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right) \\
& \quad-\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\top} \bar{Q}_{i}^{-1}\left(R_{i} \boldsymbol{\omega}_{i}\right)^{\wedge} \boldsymbol{z}_{i j} \\
& =\left(\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1}\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\wedge}+\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\top} \bar{Q}_{i}^{-1} \hat{\boldsymbol{z}}_{i j}\right) R_{i} \boldsymbol{\omega}_{i}
\end{aligned}
$$

Note that we transposed the first term and substituted $R_{i}^{\top} R_{i}$ into both terms in (79a). In (79b), $\bar{Q}_{i}^{-1}=R_{i} Q_{i}^{-1} R_{i}^{\top}$ and (72) are used. Lastly, the property (71) is applied in (79c). By substituting (77), (78b) and (79c) into (76), we obtain

$$
\begin{aligned}
& \frac{d}{d t}\left(\frac{\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\top} \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}\right) \\
& =\frac{d}{d t}\left(\frac{1}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}\right)\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\top} \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j} \\
& +\frac{1}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|} \boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1}\left(R_{j} \boldsymbol{v}_{j}-R_{i} \boldsymbol{v}_{i}\right) \\
& +\frac{1}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}\left(\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\top} \bar{Q}_{i}^{-1}\left(I_{d}-\boldsymbol{z}_{i j} \boldsymbol{z}_{i j}^{\top}\right) \boldsymbol{u}_{z_{i j}}\right. \\
& \left.+\left(\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1}\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\wedge}+\left(\boldsymbol{p}_{j}-\boldsymbol{p}_{i}\right)^{\top} \bar{Q}_{i}^{-1} \hat{\boldsymbol{z}}_{i j}\right) R_{i} \boldsymbol{\omega}_{i}\right)
\end{aligned}
$$

where $d / d t\left(\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|^{-1}\right)$ is given from (75).
Finally, we derive the time derivative of the first term in (67). Here, we introduce $\sigma:=\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|\left\|\bar{Q}_{j} \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|$ to make the notations simple.

$$
\begin{aligned}
& \frac{d}{d t}\left(\frac{\left\|\bar{Q}_{j} \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}\right)=\left\|\bar{Q}_{j} \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\| \frac{d}{d t}\left(\frac{1}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}\right) \\
& +\frac{1}{\sigma}\left(\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1} \bar{Q}_{j}^{2}\right)\left(\frac{d}{d t}\left(R_{i} Q_{i}^{-1} R_{i}^{\top} \boldsymbol{z}_{i j}\right)\right) \\
& +\frac{1}{2 \sigma}\left(\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1}\right)\left(\frac{d}{d t}\left(R_{j} Q_{j}^{2} R_{j}^{\top}\right)\right)\left(\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right)
\end{aligned}
$$

Because the derivative term of the second term (81b) is the same as the one in (78a), the second term (81b) can be calculated as follow, where we utilize the same techniques utilized in (78b) and (79).

$$
\begin{aligned}
& \frac{1}{\sigma}\left(\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1} \bar{Q}_{j}^{2}\right)\left(\frac{d}{d t}\left(R_{i} Q_{i}^{-1} R_{i}^{\top} \boldsymbol{z}_{i j}\right)\right) \\
& =\frac{1}{\sigma}\left(\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1} \bar{Q}_{j}^{2}\right)\left(\left(R_{i} \hat{\boldsymbol{\omega}}_{i}\right) Q_{i}^{-1} R_{i}^{\top} \boldsymbol{z}_{i j}\right. \\
& \left.+R_{i} Q_{i}^{-1}\left(R_{i} \hat{\boldsymbol{\omega}}_{i}\right)^{\top} \boldsymbol{z}_{i j}+R_{i} Q_{i}^{-1} R_{i}^{\top}\left(I_{d}-\boldsymbol{z}_{i j} \boldsymbol{z}_{i j}^{\top}\right) \boldsymbol{u}_{z_{i j}}\right) \\
& =\frac{1}{\sigma}\left(\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1} \bar{Q}_{j}^{2}\right)\left(-\left(\bar{Q}_{i}^{-1} \hat{\boldsymbol{z}}_{i j}\right)^{\wedge}+\bar{Q}_{i}^{-1} \hat{\boldsymbol{z}}_{i j}\right) R_{i} \boldsymbol{\omega}_{i} \\
& +\frac{1}{\sigma}\left(\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1} \bar{Q}_{j}^{2}\right) \bar{Q}_{i}^{-1}\left(I_{d}-\boldsymbol{z}_{i j} \boldsymbol{z}_{i j}^{\top}\right) \boldsymbol{u}_{z_{i j}}
\end{aligned}
$$

The third term (81c) can be calculated as

$$
\begin{aligned}
& \frac{1}{\sigma}\left(\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1}\right)\left(R_{j} Q_{j}^{2}\left(R_{j} \hat{\boldsymbol{\omega}}_{j}\right)^{\top}\right)\left(\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right) \\
& =\frac{1}{\sigma}\left(\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1}\right)\left(R_{j} Q_{j}^{2} R_{j}^{\top} R_{j} \hat{\boldsymbol{\omega}}_{j}^{\top} R_{j}^{\top}\right)\left(\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right) \\
& =-\frac{1}{\sigma}\left(\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1}\right) \bar{Q}_{j}^{2}\left(R_{j} \boldsymbol{\omega}_{j}\right)^{\wedge}\left(\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right) \\
& =\frac{1}{\sigma}\left(\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1}\right) \bar{Q}_{j}^{2}\left(\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right)^{\wedge} R_{j} \boldsymbol{\omega}_{j}
\end{aligned}
$$

By substituting (82c) and (83d) into (81), we obtain

$$
\begin{aligned}
& \frac{d}{d t}\left(\frac{\left\|\bar{Q}_{j} \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}\right)=\left\|\bar{Q}_{j} \bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\| \frac{d}{d t}\left(\frac{1}{\left\|\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right\|}\right) \\
& +\frac{1}{\sigma}\left(\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1} \bar{Q}_{j}^{2}\right)\left(-\left(\bar{Q}_{i}^{-1} \hat{\boldsymbol{z}}_{i j}\right)^{\wedge}+\bar{Q}_{i}^{-1} \hat{\boldsymbol{z}}_{i j}\right) R_{i} \boldsymbol{\omega}_{i} \\
& +\frac{1}{\sigma}\left(\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1} \bar{Q}_{j}^{2}\right) \bar{Q}_{i}^{-1}\left(I_{d}-\boldsymbol{z}_{i j} \boldsymbol{z}_{i j}^{\top}\right) \boldsymbol{u}_{z_{i j}} \\
& +\frac{1}{\sigma}\left(\boldsymbol{z}_{i j}^{\top} \bar{Q}_{i}^{-1}\right) \bar{Q}_{j}^{2}\left(\bar{Q}_{i}^{-1} \boldsymbol{z}_{i j}\right)^{\wedge} R_{j} \boldsymbol{\omega}_{j}
\end{aligned}
$$

The time derivative of the proposed CBF in $d=3$ can be obtained by combining (75), (80), and (84) together. Note that $\dot{h}_{i j}$ presented in (30) and (31) divides the terms of $\dot{h}_{i j}$ so that the coefficients of each control input, namely $\boldsymbol{\omega}_{i}, \boldsymbol{v}_{i}, \boldsymbol{u}_{z_{i j}}, \boldsymbol{\omega}_{j}$, and $\boldsymbol{v}_{j}$, are easily understandable.

Finally, we briefly discuss the time derivative of $h_{i j}$ in the case of $d=2$. Because most of the equations are the same as those of $d=3$ shown above, we only explain what causes the difference between $d=3$ and $d=2$.

The disagreement of $\dot{h}_{i j}$ between $d=2$ and $d=3$ mainly stems from the operator $\wedge$, which renders a different result, as shown in (14). As a consequence, with $d=2$, the properties of the operator $\wedge$ are distinct from that of $d=3$ in (71) and (72), as shown below with $a \in \mathbb{R}, \boldsymbol{b} \in \mathbb{R}^{2}$, and $R \in S O(2)$.

$$
\begin{aligned}
\hat{a} \boldsymbol{b} & =\hat{1} \boldsymbol{b} a \\
R \hat{\omega} R^{\top} & =\hat{\omega}
\end{aligned}
$$

Hence, $\dot{h}_{i j}$ in $d=2$ takes a bit different form from (30), as shown in (33). More concretely, the difference between (71) and (85) alters $(\cdot)$ in (31a) and (31c) into $-\hat{1}(\cdot)$ in (34a) and (34b), where $(\cdot)$ denotes a vector with the length of $d$. In addition, because of the difference between (72) and (86), $R_{i}$ and $R_{j}$ preceding $\boldsymbol{\omega}_{i}$ and $\boldsymbol{\omega}_{j}$ in (30) disappear in (33). This completes the proof.

## REFERENCES

[1] C. Zhang and J. M. Kovacs, "The application of small unmanned aerial systems for precision agriculture: A review," Precision Agriculture, vol. 13, pp. 693-712, 2012.
[2] T. Miyano, J. Romberg, and M. Egerstedt, "Primal-dual gradient dynamics for cooperative unknown payload manipulation without communication," in Proc. Amer. Control Conf., 2020, pp. 2061-2067.
[3] G.-B. Dai and Y.-C. Liu, "Distributed coordination and cooperation control for networked mobile manipulators," IEEE Trans. Industrial Electronics, vol. 64, no. 6, pp. 5065-5074, 2017.
[4] R. Funada, M. Santos, T. Gencho, J. Yamauchi, M. Fujita, and M. Egerstedt, "Visual coverage maintenance for quadcopters using nonsmooth barrier functions," in Proc. IEEE Int. Conf. Robot. Autom., 2020, pp. 3255-3261.
[5] Y. Rizk, M. Awad, and E. W. Tunstel, "Cooperative heterogeneous multirobot systems: A survey," ACM Comput. Surv., vol. 52, no. 2, pp. 1-31, 2019.
[6] J. A. Preiss, W. Hönig, N. Ayanian, and G. S. Sukhatme, "Downwashaware trajectory planning for large quadrotor teams," in Proc. IEEE/RSJ Int. Conf. Intell. Robots. Syst., 2017, pp. 250-257.
[7] T. Hatanaka, R. Funada, G. Gezer, and M. Fujita, "Distributed visual 3-D localization of a human using pedestrian detection algorithm: A passivity-based approach," in Proc. 6th IFAC Workshop Distrib. Estimation Control Netw. Syst., 2016, pp. 210-215.
[8] L. Nicholson, M. Milford, and N. Sünderhauf, "QuadricSLAM: Dual quadrics from object detections as landmarks in object-oriented SLAM," IEEE Robot. Autom. Lett., vol. 4, no. 1, pp. 1-8, 2019.
[9] K. Ok, K. Liu, K. Frey, J. P. How, and N. Roy, "Robust object-based SLAM for high-speed autonomous navigation," in Proc. IEEE Int. Conf. Robot. Autom., 2019, pp. 669-675.
[10] M. G. Choi, "Computing the closest approach distance of two ellipsoids," Symmetry, vol. 12, no. 8, p. 1302, 2020.
[11] I. Girault, M.-A. Chadil, and S. Vincent, "Comparison of methods computing the distance between two ellipsoids," J. Comput. Physics, vol. 458, p. 111100, 2022.
[12] Y.-K. Choi, W. Wang, Y. Liu, and M.-S. Kim, "Continuous collision detection for two moving elliptic disks," IEEE Trans. Robotics, vol. 22, no. 2, pp. 213-224, 2006.
[13] T. Ibuki, T. Hirano, R. Funada, and M. Sampei, "Optimization-based distributed safety control with applications to collision avoidance for mobile robotic networks," Advanced Robotics, vol. 37, no. 1-2, pp. 8798, 2023.
[14] O. Khatib, "Real-time obstacle avoidance for manipulators and mobile robots," in Autonomous Robot Vehicles. Springer New York, 1986, pp. 396-404.
[15] M. Hoy, A. S. Matveev, and A. V. Savkin, "Algorithms for collisionfree navigation of mobile robots in complex cluttered environments: A survey," Robotica, vol. 33, no. 3, pp. 463-497, 2015.
[16] R. Olfati-Saber, "Flocking for multi-agent dynamic systems: Algorithms and theory," IEEE Trans. Autom. Control, vol. 51, no. 3, pp. 401-420, 2006.
[17] V. Gazi and K. M. Passino, "A class of attractions/repulsion functions for stable swarm aggregations," Int. J. Control, vol. 77, no. 18, pp. 1567-1579, 2004.
[18] D. V. Dimarogonas and K. H. Johansson, "On the stability of distancebased formation control," in Proc. 47th IEEE Conf. Decis. Control, 2008, pp. 1200-1205.
[19] D. M. Stipanović, P. F. Hokayem, M. W. Spong, and D. D. Siljak, "Cooperative avoidance control for multiagent systems," J. Dynamic Syst., Measurement, and Control, vol. 129, no. 5, pp. 699-707, 2007.
[20] O. Arslan and D. E. Koditschek, "Sensor-based reactive navigation in unknown convex sphere worlds," Int. J. Robotics Research, vol. 38, no. 2-3, pp. 196-223, 2019.
[21] I. Z. Emiris and G. M. Tzoumas, "A real-time and exact implementation of the predicates for the Voronoi diagram of parametric ellipses," in Proc. ACM Symp. Solid and Physical Modeling, 2007, pp. 133-142.
[22] A. D. Ames, X. Xu, J. W. Grizzle, and P. Tabuada, "Control barrier function based quadratic programs for safety critical systems," IEEE Trans. Autom. Control, vol. 62, no. 8, pp. 3861-3876, 2017.
[23] A. D. Ames, S. Coogan, M. Egerstedt, G. Notomista, K. Sreenath, and P. Tabuada, "Control barrier functions: Theory and applications," in Proc. Eur. Control Conf., 2019, pp. 3420-3431.
[24] M. Egerstedt, Robot Ecology: Constraint-Based Design for LongDuration Autonomy. Princeton University Press, 2021.
[25] L. Wang, A. D. Ames, and M. Egerstedt, "Safety barrier certificates for collisions-free multirobot systems," IEEE Trans. Robotics, vol. 33, no. 3, pp. 661-674, 2017.
[26] P. Glotfelter, I. Buckley, and M. Egerstedt, "Hybrid nonsmooth barrier functions with applications to provably safe and composable collision avoidance for robotic systems," IEEE Robot. Autom. Lett., vol. 4, no. 2, pp. 1303-1310, 2019.
[27] A. Singletary, K. Klingebiel, J. Bourne, A. Browning, P. Tokumaru, and A. Ames, "Comparative analysis of control barrier functions and artificial potential fields for obstacle avoidance," in Proc. IEEE/RSJ Int. Conf. Intell. Robots. Syst., 2021, pp. 8129-8136.
[28] G. Notomista and M. Egerstedt, "Persistification of robotic tasks," IEEE Trans. Control Syst. Technol., vol. 29, no. 2, pp. 756-767, 2021.
[29] P. Ong, B. Capelli, L. Sabattini, and J. Cortés, "Network connectivity maintenance via nonsmooth control barrier functions," in Proc. 60th IEEE Conf. Decis. Control, 2021, pp. 4786-4791.
[30] K. D. Do, "Flocking for multiple ellipsoidal agents with limited communication ranges," Int. Scholarly Research Notices, vol. 2013, p. 13, 2013.
[31] C. K. Verginis and D. V. Dimarogonas, "Closed-form barrier functions for multi-agent ellipsoidal systems with uncertain Lagrangian dynamics," IEEE Control Syst. Lett., vol. 3, no. 3, pp. 727-732, 2019.
[32] K. Dhal, A. Kashyap, and A. Chakravarthy, "Collision avoidance and rendezvous of quadric surfaces moving on planar environments," in Proc. 60th IEEE Conf. Decis. Control, 2021, pp. 3569-3575.
[33] M. Srinivasan, M. Abate, G. Nilsson, and S. Coogan, "Extent-compatible control barrier functions," Syst. Control Lett., vol. 150, p. 104895, 2021.
[34] A. Thirugnanam, J. Zeng, and K. Sreenath, "Duality-based convex optimization for real-time obstacle avoidance between polytopes with control barrier functions," in Proc. Amer. Control Conf., 2022, pp. 22392246.
[35] K. Nishimoto, R. Funada, T. Ibuki, and M. Sampei, "Collision avoidance for elliptical agents with control barrier function utilizing supporting lines," in Proc. Amer. Control Conf., 2022, pp. 5147-5153.
[36] A. Thirugnanam, J. Zeng, and K. Sreenath, "Nonsmooth control barrier functions for obstacle avoidance between convex regions," 2023, arXiv:2306.13259.
[37] B. Dai, R. Khorrambakht, P. Krishnamurthy, V. Gonçalves, A. Tzes, and F. Khorrami, "Safe navigation and obstacle avoidance using differentiable optimization based control barrier functions," 2023, arXiv:2304.08586.
[38] H. Park, S. Di Cairano, and I. Kolmanovsky, "Model predictive control for spacecraft rendezvous and docking with a rotating/tumbling platform and for debris avoidance," in Proc. Amer. Control Conf., 2011, pp. 19221927.
[39] H. Park, C. Zagaris, J. V. Llop, R. Zappulla, I. Kolmanovsky, and M. Romano, "Analysis and experimentation of model predictive control for spacecraft rendezvous and proximity operations with multiple obstacle avoidance," in Proc. AIAA/AAS Astrodynamics Specialist Conf., 2016, AIAA 2016-5273.
[40] C. Zagaris, H. Park, J. Virgili-Llop, R. Zappulla, M. Romano, and I. Kolmanovsky, "Model predictive control of spacecraft relative motion with convexified keep-out-zone constraints," J. Guidance, Control, and Dynamics, vol. 41, no. 9, pp. 2054-2062, 2018.
[41] D. Malyuta, Y. Yu, P. Elango, and B. Açıkmeye, "Advances in trajectory optimization for space vehicle control," Annual Reviews in Control, vol. 52, pp. 282-315, 2021.
[42] Q. Nguyen and K. Sreenath, "Exponential control barrier functions for enforcing high relative-degree safety-critical constraints," in Proc. Amer. Control Conf., 2016, pp. 322-328.
[43] W. Xiao and C. Belta, "High-order control barrier functions," IEEE Trans. Autom. Control, vol. 67, no. 7, pp. 3655-3662, 2022.
[44] T. Hatanaka, N. Chopra, M. Fujita, and M. W. Spong, Passivity-Based Control and Estimation in Networked Robotics. Springer, 2015.
[45] M. Srinivasan, A. Dabholkar, S. Coogan, and P. A. Vela, "Synthesis of control barrier functions using a supervised machine learning approach," in Proc. IEEE/RSJ Int. Conf. Intell. Robots. Syst., 2020, pp. 7139-7145.
[46] M. Mesbahi and M. Egerstedt, Graph Theoretic Methods in Multiagent Networks. Princeton University Press, 2010.
[47] T. Ibuki, S. Wilson, J. Yamauchi, M. Fujita, and M. Egerstedt, "Optimization-based distributed flocking control for multiple rigid bodies," IEEE Robot. Autom. Lett., vol. 5, no. 2, pp. 1891-1898, 2020.
[48] R. Rajamani, Vehicle Dynamics and Control. Springer Science \& Business Media, 2011.
[49] S. Boyd and L. Vandenberghe, Convex Optimization. Cambridge University Press, 2004.