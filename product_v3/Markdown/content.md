# 単調環境における協調自己位置推定 のためのCollaborative Stein Particle Filter

○有田 朋樹，滑川 徹（慶應義塾大学）

## 単調環境における協調自己位置推定 のためのCollaborative Stein Particle Filter

## *T. Arita, T. Namerikawa (Keio University)


#### Abstract
Key Words: Collaborative localization, Stein particle filter, Visual inertial system

## 1. introduction
多くの自律移動可能なロボットは，動作の基盤となる自己位置推定を必要とする．また，自律移動ロボットの社会への浸透に伴って，過酷環境や山間地域等GPS/GNSSが利用できない環境での自己位置推定技術が数多く研究されている．自己位置推定には，力学制御や局所の動作計画などに利用する局所の自己位置推定の階層と，一貫した環境地図の作成や広域の経路計画に利用する大域の自己位置推定の階層が存在するが，GPS/GNSSが利用できない環境における大域の自己位置推定では，主にLiDARやカメラ等によって得られた点群及び画像データを種々のマッチング手法で構造化することにより自己位置推定を達成する手法が一般的である．実用的には，カメラとIMU(慣性計測ユニット)を組み合わせるVisual Inertial System(VINS)が安価にハードウェアを実装できる点から多く利用されている\cite{VINS-Mono}．

更に，自律移動ロボットの普及によって，複数のエージェントが同時に動作するマルチエージェント環境が近年主要な研究分野となっている．GPS/GNSSが使用できない過酷環境や山間地域においても，システム全体としての対故障性やそれぞれの目的におけるスケーラビリティの観点から，いくつかの研究でマルチエージェントによる協調作業が試みられている\cite{Chen2023OverviewOM}\cite{Ego-Plannerv2}．

これらの背景から，GPS/GNSSを用いない大域自己位置推定手法が求められているところであるが，農地や森林など単調かつ広大な野外環境下では，誤差の蓄積と外れ値の影響により画像のマッチングによる大域の自己位置推定が十分に機能しない．
特に複数のエージェントによる協調推定においては，各エージェントの誤った状態推定が全エージェントに影響する為，各エージェントの外れ値割合に伴ってシステム全体が誤った推定をする確率は指数関数的に上昇する．
そこで本研究では，Stein Particle Filterを用いることで，従来の画像特徴量のマッチングに位置情報による尤度を考慮しつつ，複数のエージェント間で推定状態を合意するVINS手法を提案する．

![[simple_model.png]]
Fig. 1: Collaborative Visual Inertial System
### A. Related Work
deep research
### B. Contributions
本論文の主要な貢献は以下の通りである．
・SE(3)上でパーティクルフィルタによる協調自己位置推定を定式化した．
・ADMMの導入により，大域(複数エージェントでも一貫した)での収束が可能なStein Particle Filterのアルゴリズムを提案した．
・協調自己位置推定における曖昧性表現の獲得により，これまで単調環境において根本的な問題であった外れ値にロバストな推定手法を提案した．
## 2. Preliminary

### A. Stein Variational Gradient Descent
以下のようなカルバックライブラー情報量の最小化問題を考える．

$$
 \begin{aligned}
 q^{*}=\underset{q \in \mathcal{Q}}{\text{arg min}}\left\{D_{KL}(q \| p) \equiv \mathbb{E}_{q}[\log q(x)]-\mathbb{E}_{q}[\log {p}(x)]\right\}，
\end{aligned}
$$

このとき以下の定理が成り立つ．

**Theorem. KLダイバージェンスとStein Operatorの関係**
$\boldsymbol{T}(x)=x+\epsilon \boldsymbol{\Phi}(x)$のような変換において，$x \sim q(x)$のとき$z=\boldsymbol{T}(x)$の確率分布を$q_{[\boldsymbol{T}]}(z)$とすると

$$
\begin{aligned}
\left.\nabla_{\epsilon} D_{KL}\left(q_{[\boldsymbol{T}]} \| p\right)\right|_{\epsilon=0}=-\mathbb{E}_{x \sim q}\left[\mathcal{A}_{p} \boldsymbol{\Phi}(x)\right]，
\end{aligned}
$$

が成り立つ．ただし$\mathcal{A}_{p} \boldsymbol{\Phi}(x)=\nabla_{x} \log p(x) \boldsymbol{\Phi}(x)^{\top}+\nabla_{x} \boldsymbol{\Phi}(x)$はStein Operator, $\boldsymbol{\Phi}$は再生核ヒルベルト空間$\mathcal{H}^d$に属する汎関数である．

ここで，Kernelized Stein Discrepancy(KSD)を

$$
\begin{aligned}
\mathbb{D}(q,p)=\max_{\boldsymbol{\Phi}\in \mathcal{H}^d}\mathbb{E}_{x \sim q}\left[\mathcal{A}_{p} \boldsymbol{\Phi}(x)\right], \quad \text{s.t.} \quad \|\boldsymbol{\Phi}\|_{\mathcal{H}^d} \leq 1，
\end{aligned}
$$

のように定義すると，この問題の解は

$$
\begin{aligned}
\boldsymbol{\Phi}_{q, p}^{*}(\cdot)=\mathbb{E}_{x \sim q}\left[k(x, \cdot) \nabla_{x} \log p(x)+\nabla_{x} k(x, \cdot)\right]，
\end{aligned}
$$

で与えられる．
### B. Relaxed ADMM

以下のような制約付き最適化問題

$$
\begin{aligned}
&\min_{x,y} f(x)+g(y),\\
&\text{s.t.}\: Ax+By=b，
\end{aligned}
$$

は，次のような拡張ラグランジアン

$$
\begin{aligned}
\mathcal{ L}_{f,\gamma}(z) &= f(x) - \langle z, Ax \rangle + \frac \gamma 2 \|Ax\|^2，\\
\mathcal{ L}_{g,\gamma}(z) &=  g(y) - \langle z, By-b \rangle + \frac \gamma 2 \|By-b\|^2，
\end{aligned}
$$

を用いて

$$
\begin{aligned}
y^+&= \underset{y}{\text{arg min}}\: \mathcal{ L}_{g,\gamma}(z)，\\
\omega_g &=  z - \gamma (By^+-b)， \\
x^+&= \underset{x}{\text{arg min}}\: \mathcal{ L}_{f,\gamma}(2\omega_g - z)，\\
\omega_f &= 2\omega_g - z - \gamma Ax^+，\\
z^+&=z+\eta(\omega_f-\omega_g)，
\end{aligned}
$$

の様に解くことができる．
## 3 Problem Formulation

本文では，ユークリッド空間$\mathbb{R}^d$に存在するエージェントの協調自己位置推定を扱う．
時間ステップ$t$におけるエージェント$i$の状態を$x_i^t=(\mathbf{t}^t_i, R^t_i)\in \mathrm{SE}(d):=\mathbb{R}^d\times \mathrm{SO}(d)$，ここで座標$\mathbf{p}^t_i\in\mathbb{R}^d$及び回転行列$R^t_i\in \mathrm{SO}(d)$である．また，手法の検証においては$d=2$, $d=3$の場合で検証を行う．協調自己位置推定は，各エージェントの観測を$z^t=[z_1^t, \cdots, z_N^t]$とすると，
以下のような最大事後確率推定問題(MAP推定問題)

$$
\begin{aligned}
\underset{x^{t+1}_1, \cdots, x^{t+1}_N}{\text{max}} \: \sum^N_{i=1} P(x^{t+1}_i|Z_i^{t}) Q(z_i^{t+1}|x_i^{t+1})，
\end{aligned}
$$

として定式化できる．ここで，分布$P(x^{t+1}_i|Z_i^{t})$は時間ステップ$t+1$において，観測情報$z_i^{t+1}$が与えられる前のエージェント$i$の状態に関する確率分布を表し，分布$Q(z_i^{t+1}|x_i^{t+1})$は状態$x_i^{t+1}$において観測情報$z_i^{t+1}$が得られる尤度分布を表す．また，$Z_i^t = [z_i^0 \cdots z_i^t]$である．

3次元特殊ユークリッド群$\mathrm{SE(3)}$は以下の形式

$$
\begin{aligned}
T
 = 
\left(\begin{array}{@{}c|c@{}}
\mathbf{R}
& \mathbf{t} \\
\hline
\begin{matrix}
0 & 0 & 0
\end{matrix} & {\mbox{0}}
\end{array}\right)
=
\left(\begin{array}{@{}c|c@{}}
\begin{matrix}
\mathbf{d}_{c1} & \mathbf{d}_{c2} & \mathbf{d}_{c3} 
\end{matrix} 
& \mathbf{t} \\
\hline
\begin{matrix}
0 & 0 & 0
\end{matrix} & {\mbox{0}}
\end{array}\right)\in \mathbb{R}^{4\times4}，
\end{aligned}
$$

で表現される．ここで，特殊直交群$\mathrm{SO(d)}$及び特殊ユークリッド群$\mathrm{SE(d)}$における以下の演算子を導入する．

$$
\begin{aligned}
(\cdot)^{\wedge}&: \mathbb{R}^\frac{d(d-1)}{2} \rightarrow  \mathfrak{so}(d)，\mathbb{R}^{d+\frac{d(d-1)}{2}} \rightarrow  \mathfrak{se}(d)，\\
\boldsymbol{\omega}&=\begin{bmatrix}
    \:x\: \\
    \:y\: \\
    \:z\: 
\end{bmatrix}\in\mathbb{R}^3,
\quad \boldsymbol{\omega}^{\wedge}=\begin{pmatrix}
    0&-z&y \\
    z&0&x \\
    -y&x&0 
\end{pmatrix}\in \mathfrak{so}(3)\\
\mathbf{v}&=\begin{bmatrix}
    \:\mathbf{t}\: \\
    \:\boldsymbol{\omega}\: 
\end{bmatrix}\in\mathbb{R}^6,\quad \mathbf{v}^\wedge=\begin{pmatrix}
    {\boldsymbol{\omega}^\wedge}& \mathbf{t}\\
    0&1
\end{pmatrix}\in \mathfrak{se}(3)\\
\exp&:  \mathfrak{so}(d) \rightarrow  \text{SO}(d)，\mathfrak{se}(d) \rightarrow  \text{SE}(d)，\\
\quad e^{\boldsymbol{\omega}} &\equiv \exp({\boldsymbol{\omega}}^\wedge) = \mathbf{I}_3 + \frac{\sin \theta}{\theta } {\boldsymbol{\omega}}^\wedge+\frac{1-\cos \theta }{\theta^2}({\boldsymbol{\omega}}^\wedge)^2 \in \text{SO}(3)\\
e^{\mathbf{v}}&\equiv \exp({\mathbf{v}^\wedge}) = 
\begin{pmatrix}
    e^{{\boldsymbol{\omega}}^\wedge}& \mathbf{Vt}\\
    0&1
\end{pmatrix}\in\text{SE}(3),\quad  \mathbf{V}=\mathbf{I}_3+\frac{1-\cos\theta}{\theta^2}{\boldsymbol{\omega}}^\wedge +\frac{\theta-\sin \theta}{\theta^3}({\boldsymbol{\omega}}^\wedge)^2\\
\end{aligned}
$$


更に，$()^\vee$，$\log$はそれぞれ$()^\wedge$，$\exp$の逆演算を表す．
SPFにおける一般的な手法として，\Eqref{map}をカルバック・ライブラー情報量(KLダイバージェンス)$D_{KL}$の最小化問題

$$
\begin{aligned}
&\underset{x_i^{t+1}}{\text{max}} \:  P(x_i^{t+1}|Z_i^{t}) Q(z_i^{t+1}|x_i^{t+1})\simeq \underset{x_i^{t+1}}{\text{max}} \: \underset{P_{[T]}}{\text{arg min}} \: D_{KL}(P(x_i^{t+1}|Z_i^{t})_{[T]} \| P_Q)，\\
\end{aligned}
$$

に置き換える．ここで目標分布$P_Q$は尤度分布$Q$と同型の状態$x_i^{t+1}$についての確率分布であり，平均座標$\mu\in\mathrm{SE}(d)$及び重み対角行列$\Sigma \in \mathbb{R}^{(d+\frac{d(d-1)}{2})\times (d+\frac{d(d-1)}{d})}$を用いて，$\mathrm{SE(3)}$上におけるカーネル

$$
\begin{aligned}
\mathcal{K}_j&=\exp \left(-(d_j^\vee)^{\top} \Sigma d_j^\vee \right)\in \mathbb{R},\\
d_j&=\log \left((\mu_j^{-1}) x_i^{t+1} \right)\in \mathfrak{se}(d)
\end{aligned}
$$

の重ね合わせで表現される．これらの背景から，時間ステップtにおいてエージェントiからみたエージェントjの状態$x_{i,j}^t$をとし，推定する状態量を$\hat x_i^t= \left\{ x_{i,j}^t \in \mathrm{SE(d)}\: | \: j \in \mathcal{N}_i \right\}$のように修正することで解くべき問題は
$$
\begin{aligned}
\max _{\hat x_{1}^{t}, \cdots, \hat x_{N}^{t}} & \sum_{i=1}^{N} \arg \min _{{P_{i}}_{[T]}} D_{K L}\left(P_{i}\left(\hat x_{i}^{t} \mid Z_{i}^{t-1}\right)_{\left[T_{i}]\right.} \| P_{Q_{i}} \right) \\
\text { s.t. } & \hat x_{i}^{t}=\hat x_{j}^{t}, \quad\forall(i, j) \in \mathcal{E}
\end{aligned}
$$
のように定式化できる．この式1における制約は，エージェント間の通信を示すすべてのエッジ$\mathcal{E}$において，ドローンの推定状態が一致していることを示す．
## 4. SPF For Collaborative Localization

このセクションでは，式１の解を求めるためのSPF(Stein Particle Filter)を用いた数学的アルゴリズムを提案する．確率分布におけるKLダイバージェンスの最小化は，SPFを用いることで数値計算可能である．一方で，マルチエージェント問題におけるKLダイバージェンスの最小化は絶対座標に固定された目標分布が存在せず，目標分布$P_{Q_i}$自体が推定したい状態量の従属変数となってしまうため非常に困難である．更に，式１のような制約の無い安易なSPFアルゴリズムを適用した場合，フィルタリングアルゴリズムは部分的な状態量に対してPartialにのみ最適化されるため，
後述するように，エージェント全体で統一された相対位置のグラフを作ることができず推定が破綻する．これらの問題を解決するため，我々は明示的なパーティクル間の相対位置を用いた尤度関数，およびRelaxed ADMMによる合意アルゴリズムを用いることでCollaborative Localizationに適用可能なSPFアルゴリズムを定式化した．

### A. パーティクル同士の勾配効果(仮)

まず，確率分布を$P_{i}\left( x_{i}^{t} \mid Z_{i}^{t-1}\right)\simeq \left\{ p_{i,l}\in \mathrm{SE(3)} \right\}_{l=1:m}$のようにパーティクルで近似し，これらを式1に基づき目標分布$P_{Q_i}$にそって勾配降下させることを考える．

$$
\begin{aligned}
&\underset{P_{[T]}}{\text{min}} \: \nabla_\epsilon \left . D_{KL}(P(x_i^{t+1}|Z^{t})_{[T]} \| P_{Q_i})\right |_{\epsilon = 0}，\\
&\Rightarrow p_{i,l}:= p_{i,l} \exp(\boldsymbol{\Phi}^*)，\\
&\quad \:\: \boldsymbol{\Phi}^*(p_{i,l}) = \frac{1}{m}\sum_{r=1}^m(\nabla_{p_{i,r}}\log P_Q(p_{i,r})k(p_{i,l},p_{i,r}) + \nabla_{p_{i,j}}k(p_{i,l},p_{i,r}))，
\end{aligned}
$$
ここで$k(p_{i,l},p_{i,r})=\exp \left(-(d_j^\vee)^{\top} \Sigma d_j^\vee \right)\in \mathbb{R}$は$\mathrm{SE(3)}$上のカーネルである．

$$
\begin{aligned}
    \log P_{Q_i}(p_{i,r}) &= \log \prod_{k\in \mathcal{N}} \mathcal{K}_k
    = 
    -\sum_{k\in \mathcal{N}} (d_k^\vee)^{\top} \Sigma_k d_k^\vee \in \mathbb{R}，\\
d_k(p_{i,r})&=\log \left(p_{j,k}^{-1} (p_{i,r}T_{i\rightarrow j}) \right)\in \mathfrak{se}(d)\\
\end{aligned}
$$
ここで$k\in\mathcal{N} := \mathcal{N}_{j}(p_{i,l}T_{i\rightarrow j})$であり，エージェントjが持つパーティクルの内，$T_{i\rightarrow j}\in \mathrm{SE(3)}$で変換したエージェントiが持つr番目のパーティクル$p_{i,l}$にとって近傍のパーティクルの集合を表す．このように得られた相対位置$T_{i\rightarrow j}$にそってパーティクルを変換した場合の予測位置近傍のパーティクルについて明示的に$\mathrm{SE(3)}$上の誤差を計算し，近傍でKDEにより真の目標分布を推定することで，固定された目標分布が存在しない場合においてもKLダイバージェンスを最小化することが可能である．
更に収束を高速化させるため，目標分布の勾配$\nabla_{p_{i,r}}\log P_Q(p_{i,r})$はガウスニュートン法を用いて
$$
\begin{aligned}
\nabla_{p_{i,r}}\log P_Q(p_{i,r})&:=
(\mathbf{H}^r)^{-1}b^r,\\
\mathbf{H}^r &= (\mathbf{J}^r)^\top \mathbf{J}^r, \\
b^r&=(\mathbf{J}^r)^\top d_k(p_{i,r})^\vee,\\
\mathbf{J}^r&=\left. \frac{\partial(d_k(p_{i,r}e^\boldsymbol{\varepsilon})^\vee
)}{\partial \boldsymbol{\varepsilon}}\right|_{\boldsymbol{\varepsilon}=\mathbf{0}}
\end{aligned}
$$
の様に計算を行う．

### B. SPF integrating Relaxed ADMM

このサブセクションでは，サブセクションAで定式化したSPFによる勾配降下法をもとに，式1の合意制約付きKLダイバージェンスの最小化を行うアルゴリズムの定式化を行う．まず，単純(straightforward)な解法として
$$
\begin{aligned}
&\underset{P_{[T]}}{\text{min}} \: \nabla_\epsilon \left . D_{KL}(P(x_i^{t+1}|Z^{t})_{[T]} \| P_{Q_i})\right |_{\epsilon = 0}, \quad 
\forall i\in \mathcal{A}
\end{aligned}
$$
のように，すべてのエージェントが相対位置を受け取った時点で各エージェントペアに対し相対位置パーティクルの更新を行う場合を考える．この場合，相対位置の送り元であるエージェントがグローバルに正しい位置に収束している保証が無いため，Fig1(a)に示すように特定のエージェントペアでのみ最適化された位置関係に収束してしまい，エージェント全体で一貫した相対位置のグラフを構築することができない．フィルタリングによる種々の最適化は，複数の状態量を同時に最適化することができず，単一の状態量をpartialに最適化する必要がある．ここで，本手法では，Fig1(b)のようにADMMによるconsensus filterを用いることで相対位置を取得できる順番に関わらず大域での収束を可能にするアルゴリズムを定式化する．

![[parallel_update1.png]]
式1はスラック変数$y_{i,j}$を用いて
$$
\begin{aligned}
\max _{\hat x_{1}^{t}, \cdots, \hat x_{N}^{t}} & \sum_{i=1}^{N} \arg \min _{{P_{i}}_{[T]}} D_{K L}\left(P_{i}\left(\hat x_{i}^{t} \mid Z_{i}^{t-1}\right)_{\left[T_{i}]\right.} \| P_{Q_{i}} \right) \\
\text { s.t. } & \hat x_{i}^{t}=y_{i,j}, \:\hat x_{j}^{t}=y_{i,j}, \quad\forall(i, j) \in \mathcal{E}
\end{aligned}
$$
のように緩和することにより，式7の問題に帰着できる．

**Definition. 確率分布に対する拡張ラグランジアン**
ここで，Relaxed ADMMにおける拡張ラグランジアン\Eqref{relaxed_lagrangian}として，確率分布を引数にもつ拡張ラグランジアンを
$$
\begin{aligned}
 \mathcal{ L}_{F,\gamma}(x_i, \zeta) 
 = F_i(x_i) - \int_{\mathcal{X}_i}
\zeta dx_i + \frac{\gamma}{2}\int_{\mathcal{X}_i} dx_i^2，
\end{aligned}
$$
のように再定義する．これにより\Eqref{kl_consensus_relaxed}は以下のアルゴリズム
$$
\begin{aligned}
x^+_i&= \underset{x_i}{\text{arg min}}\: D_{KL} ({P_i}_{[T]}\|{P_Q}_i) + \sum_{r\in \mathcal{E}_i} \int_{P_i}  z_{ir,r} dx_i + \frac{\gamma}{2}|\mathcal{E}_i|\int_{P_i} dx_i^2，\\
&= \underset{x_i}{\text{arg min}}\: D_{KL} ({P_i}_{[T]}\|{P_Q}_i) + \frac{\gamma}{2} \:\mathbb{E}_{x_i\sim P_i}\left[\sum_{\gamma \in \mathcal{E}_i} \|x_i + z^+_{ir,i}/\gamma \|^2\right]
\\
z^+_{ir,i} &=z_{ir,i}+\eta((z_{ir,i} + z_{ir,r})/2 + \gamma x_{i}).\quad  \forall r \in \mathcal{E}_i，
\end{aligned}
$$
に変換できる．

**Theorem. 指数的なペナルティとしての追加項**
以下の最適化問題
$$
\begin{aligned}
&\min_{P_i} \; D_{KL}(P_i \,\|\, P_{Q_i}) 
\;+\; \sum_{r \in \mathcal{E}_i} \int P_i(x_i) z_{ir,r}(x_i)\,dx_i +\; \frac{\gamma}{2}|\mathcal{E}_i| \int P_i(x_i) x_i^2\, dx_i
\end{aligned}
$$
を考える．ただし，$\int P_i(x_i)dx_i=1$ であり，$D_{KL}(P_i \,\|\, P_{Q_i})$ は
$$
\begin{aligned}
D_{KL}(P_i \,\|\, P_{Q_i}) = \int P_i(x_i)\log\frac{P_i(x_i)}{P_{Q_i}(x_i)}\,dx_i，
\end{aligned}
$$
で定義される．このとき，上記の最適化問題に対する最適分布 $P_i^*$ は，ある正規化定数 $Z$ を用いて
$$
\begin{aligned}
P_i^*(x_i) = \frac{P_{Q_i}(x_i)\exp\left(- \sum_{r \in \mathcal{E}_i} z_{ir,r}(x_i) 
\;-\; \frac{\gamma}{2}|\mathcal{E}_i| x_i^2\right)}{Z}，
\end{aligned}
$$
と表せる.すなわち，追加のペナルティ項 
$$
\begin{aligned}
\sum_{r \in \mathcal{E}_i} z_{ir,r}(x_i) \;+\; \frac{\gamma}{2}|\mathcal{E}_i|x_i^2，
\end{aligned}
$$
は，基準分布 $P_{Q_i}(x_i)$ に対して指数的な重み（ペナルティ）を掛ける形で最適解を特徴づける．

したがって\Eqref{variational_inference}, \Eqref{prob_relaxed_admm}から，パーティクルの更新則は

$$
\begin{aligned}
&\underset{x_i}{\text{min}}\: D_{KL} ({P_i}_{[T]}\|{P_Q}_i) + \sum_{r\in \mathcal{E}_i} \int_{P_i} z_{ir,r} dx_i + \frac{\gamma}{2}|\mathcal{E}_i|\int_{P_i} dx_i^2，\\
&\Rightarrow p_{i,l}:= p_{i,l} \exp(\boldsymbol{\Phi}^*)，\\
&\quad \:\: \boldsymbol{\Phi}^*(p_{i,l}) = \frac{1}{m}\sum_{r=d1}^m(\nabla_{p_{i,r}}\log {{P}^*_i}(p_{i,r})k(p_{i,l},p_{i,r}) + \nabla_{p_{i,j}}k(p_{i,l},p_{i,r}))，\\
&\quad \:\:  {{P}^*_i}(p_{i,r}) \propto {P}_{Q_i}(p_{i,r})\exp\left(- \sum_{r \in  \mathcal{E}_i} z_{ir,r}(p_{i,l}) - \frac{\gamma}{2}|\mathcal{E}_i| p_{i,l}^2\right)，
\end{aligned} 
$$

のように修正できる．これらの定式化により，\Eqref{map}を解くためのアルゴリズムは以下のように表すことができる．

\begin{algorithm}[h]
\caption{SPF For Collaborative Localization}\label{alg:dspf}

\begin{algorithmic}[1]
\STATE \textbf{Input:} $n$ UAVs, $m$ particles $\{x^t_{i,j}\}^m_{j=1}$, Target distribution ${P_Q}_i$

\FOR{$k = 1$ to $K$}
\STATE preintegration by (\ref{predict})
\ENDFOR
\STATE $\forall_{j=1:m}\; p^{t+1}_{i,j} = p^{t}_{i,j} \oplus T^{t \rightarrow {t+1}}$  // prediction (\ref{step})
\FOR{$l = 1$ to $L$}
\STATE $\forall_{j=1:m}\;$ update particle by (\ref{modified_SVGD})
\STATE $x_i = \underset{x_i}{\text{arg min}}\: {P_i(x_i)}$ // local MAP estimation
\STATE consensus update by (\ref{prob_relaxed_admm_compile})
\ENDFOR
% \STATE \textbf{Output:} Updated particles $x^i_{t}$
\end{algorithmic}
\end{algorithm}

## 5. Simulation Result with Collaborative Localization

提案したアルゴリズムに基づき，2次元と3次元の場合において協調自己位置推定のシミュレーションを行った．アルゴリズムの実装においては，計算負荷の削減のため，全アルゴリズムをCUDA上に実装し，各パーティクルの更新をGPUで並列処理してシミュレーションを行った．Gauss Newton 法の求解にはcuSolverを用いた．ハードウェアはintel core ultra 9, NVIDIA GeForce RTX 4060を搭載したpcを用いた．
### A. Two-dimensional case

提案手法の検証として，2Dにおける協調自己位置推定シナリオのシミュレーションを行った．

\begin{table}[H]
\caption{シミュレーション条件}
  \centering
  \begin{tabular}{l|c} \hline
    項目 & 値  \\ \hline
    エージェント数 & 3  \\
    エージェントごとのパーティクル数 & 50  \\ 
    時間ステップ数 & 250 \\
    総マッチング数 & 250 \\ 
    誤マッチング数 & 52 \\
    外れ値の割合 & $\simeq$ 0.2 \\ \hline
  \end{tabular}
  \label{tb:sim1}
\end{table}

シミュレーションでは，各ステップランダムなエージェントペアの相対位置(擬似的なクロージャーループ, \Figref{simulation_1_step_0}内黒点線)がそれぞれ得られると仮定し，その相対情報を用いて各エージェントの位置を推定する．また，約0.2の割合で誤った相対位置(\Figref{simulation_1_step_0}内赤点線)が得られると仮定し，matplotlibを用いて協調自己位置推定シミュレーションを行った．\Figref{simulation_1_step_0}内左上図が真の各エージェント位置，右上図，左下図，右下図がエージェント1,2,3の推定画面である．
誤った相対位置は，シミュレーション領域内のランダムな座標と各エージェントの真の位置から生成した．

\Figref{simulation_1_step_0}, \Figref{simulation_1_step_150}はそれぞれ0ステップと150ステップにおける各エージェントの推定状態の様子である．これらから，0ステップにおいては一様に設定されていた確率分布(パーティクル)が，150ステップにおいてはすべてのエージェント推定において妥当な位置に収束していることが分かる．
また，\Figref{plot_1}はシミュレーションにおける全パーティクルの座標の時間遷移である．
\Figref{plot_1}からも，150ステップほどで合意及び収束が達成されていることが分かる．

img1.
img2.
img3.

### B. Three-dimensional case

SE(3)における提案手法の検証として，3Dにおける協調自己位置推定シナリオのシミュレーションを行った．

\begin{table}[H]
\caption{シミュレーション条件}
  \centering
  \begin{tabular}{l|c} \hline
    項目 & 値  \\ \hline
    エージェント数 & 3  \\
    エージェントごとのパーティクル数 & 50  \\ 
    時間ステップ数 & 250 \\
    総マッチング数 & 250 \\ 
    誤マッチング数 & 48 \\
    外れ値の割合 & $\simeq$ 0.2 \\ \hline
  \end{tabular}
  \label{tb:sim2}
\end{table}

## 6. Extension to the Visual Inertial System

提案手法を現実の協調自己位置推定システムに適用するための部分的な検証として，BMI270 IMUとIntel RealSense D435カメラを搭載したUAVを用いた単機での実機実験を行った．  
この実験では，6DoF状態推定においてSVGDに基づくStein Particle Filterが誤差蓄積を低減し，ベンチマーク手法(D2SLAM等)と比較して精度向上している様子が見られた．  
複数機での完全な協調効果は未検証であるが，本研究で提案するSPFベースのフレームワークが実環境下でも有用である可能性を示す初期的結果が得られた．
行った実機実験の条件を\Tblref{tb:sim2}に示す．

予測ステップでは，\Eqref{map}の$P(x_{t+1}|z_t)$を計算する必要がある．これは$t$ステップまでの情報から$t+1$ステップの状態を予測する操作であり，VINSでは次のようにIMUから得られる加速度$a_m$，角速度$\omega_m$を数値積分することで得られる．SE(3)上での数値積分は文献\cite{Forster_2017}などと類似した枠組みを用いて

$$
\begin{aligned}
p_{k+1} &=p_{k} + v_{k}\Delta t \\
&\quad+ \int\!\!\!\int_{t\in [t_k,t_{k+1}]} \underbrace{\left \{ R_{k}(a_m-b_a-\eta_{ad})+g\right \}}_{\hat a}dt^2，\\
v_{k+1} &=v_{k} + \int_{t\in [t_k,t_{k+1}]} \hat a dt，\\
R_{k+1} &=R_{k} \otimes \exp \left( \int_{t\in [t_k,t_{k+1}]} \underbrace{(\omega_m-b_g-\eta_{gd})}_{\hat \omega} dt \right)，
\end{aligned}
$$

のように計算できる．ここで，$p$は位置，$v$は速度，$R$は姿勢を表す回転行列，$b_a, b_g$はそれぞれ加速度，角速度のバイアス, $\eta_{ad}, \eta_{gd}$は白色ノイズである．
数値積分によって得られた変換$T_{t\rightarrow t+1}\in \text{SE}(3)$によって各パーティクルは

$$
\begin{aligned}
\hat x_{t+1}^i = x_{t}^i T_{t\rightarrow t+1}, \forall i，
\end{aligned}
$$

のように更新される．

\begin{table}[htbp]
\caption{実機実験条件}
  \centering
  \begin{tabular}{l|c} \hline
    項目 & 値  \\ \hline
    エージェント数 & 1  \\
    パーティクル数 & 20 /agent  \\
    外れ値の割合 & 〜10\% \\
    Ground Truth & OptiTrack Flex13, Naturalpoint \\ \hline
  \end{tabular}
  \label{tb:sim2}
\end{table}

\Figref{ros_demo}はROSを用いて自己位置推定を行った様子である．
\Figref{benchmark}は各パーティクルとベンチマーク手法におけるGround Truthとの誤差遷移を示している．ここから，時系列の大部分で提案手法がベンチマーク手法より高精度に推定できていることが確認できる．本実験によって6 DoFにおけるパーティクルの勾配更新法の有効性が検証できたが，今後の研究としては複数機を用いた合意手法の検証を行いたい．

## 7. Conclusion

本稿では，Stein Particle FilterとRelaxed ADMMを組み合わせた新たなフレームワークを提案し，単調な広域環境におけるUAVの協調自己位置推定問題に取り組んだ．  
提案手法は，複数エージェント間の合意制約と多峰性分布の扱いを同時に可能にし，従来手法では困難であった外れ値混在環境下での安定的な位置合意を実現する．  
シミュレーションおよび実機実験(単機)結果から，SPFを用いることによって，分布近似と勾配情報を活用し，既存手法よりもロバストかつ柔軟な自己位置推定が可能であることが示唆された．  
今後は，複数UAVによる実機レベルの大規模実験を通じて，提案手法の有効性とスケーラビリティをさらに検証していく予定である．