# Stein Particle Filterを用いた単調環境における UAVの協調自己位置推定 

○有田 朋樹，滑川 徹（慶應義塾大学）

## Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

## *T. Arita, T. Namerikawa (Keio University)


#### Abstract

In extensive and monotonous outdoor environments such as farmlands and forest areas, global self-localization through image matching often fails due to error accumulation and outlier effects. Particularly in cooperative estimation involving multiple agents, there is a need to efficiently solve multi-modal combination problems to construct a unified relative position graph. In this study, we propose a Cooperative Visual Inertial System (CoVINS) method that incorporates position-based likelihood into conventional image feature matching using the Stein Particle Filter.


Key Words: Collaborative localization, Stein particle filter, Visual inertial system

## 1 はじめに

多くの自律移動可能なロボットは，動作の基盤となる自己位置推定を必要とする。また，自律移動ロボッ トの社会への浸透に伴って，過酷環境や山間地域等 GPS/GNSS が利用できない環境での自己位置推定技術 が数多く研究されている。自己位置推定には，力学制御や局所の動作計画などに利用する局所の自己位置推定の階層と，一貫した環境地図の作成や広域の経路計画に利用する大域の自己位置推定の階層が存在するが， GPS/GNSS が利用できない環境における大域の自己位置推定では，主に LiDAR やカメラ等によって得られた点群及び画像データを種々のマッチング手法で構造化す ることにより自己位置推定を達成する手法が一般的であ る。実用的には，カメラと IMU(慣性計測ユニット)を組 み合わせる Visual Inertial System(VINS) が安価にハードウェアを実装できる点から多く利用されている ${ }^{1)}$ 。

更に，自律移動ロボットの普及によって，複数のエージェントが同時に動作するマルチエージェント環境が近年主要な研究分野となっている。GPS/GNSS が使用で きない過酷環境や山間地域においても，システム全体と しての対故障性やそれぞれの目的におけるスケーラビリ ティの観点から，いくつかの研究でマルチエージェント による協調作業が試みられている ${ }^{213)}$ 。

これらの背景から，GPS/GNSS を用いない大域自己位置推定手法が求められているところであるが，農地や森林など単調かつ広大な野外環境下では，誤差の蓄積と外れ値の影響により画像のマッチングによる大域の自己位置推定が十分に機能しない。特に複数のエージェン トによる協調推定においては，各エージェントの誤った状態推定が全エージェントに影響する為，各エージェン トの外れ値割合に伴ってシステム全体が誤った推定を する確率は指数関数的に上昇する。そこで本研究では， Stein Particle Filterを用いることで，従来の画像特徴量 のマッチングに位置情報による尤度を考慮しつつ，複数 のエージェント間で推定状態を合意するVINS 手法を提案する。
![img-0.jpeg](img-0.jpeg)

Fig. 1: Collaborative Visual Inertial System

## 2 Collaborative Visual Inertial System

協調自己位置推定の為の Collaborative Visual Inertial System(CoVINS) は Fig 1 のように構成される。カメラ と IMU(慣性計測ユニット)を備えたエージェントが N機存在し，それぞれのエージェントがカメラで観測した画像情報を隣接したエージェントに送信する。画像情報 を受け取ったエージェントは，各エージェントが持つ データベース内を探索し，同じランドマークを観測した画像を検出して 2 つの画像におけるカメラフレーム間の相対位置を送り返す。グラフ最適化の文脈においては，各エージェントは得られた相対位置情報を用いてポーズ グラフと呼ばれる相対位置のグラフを作成する ${ }^{4)}$ 。最適化の結果，得られた各相対位置が再現されるようなポー ズグラフを生成できれば，矛盾のない位置推定が達成で きたと言える。

一方農園や森林等の不整地環境では，最適化による大域の自己位置推定が十分に機能しない。この原因は，単調環境において類似画像のマッチングが機能せず，ポー ズグラフ上で誤ったマッチングが行われてしまうことで あると考えられる。これらの背景から，Fig 2 のような単

純化されたモデルを考えることができる。Fig 2 内の赤丸，黒丸が各エージェントの位置を表すノード，黒線が エッジである。Fig 2(a) の理想的な環境では，エッジの生成によってグラフを最適化する事ができるが，Fig 2(b) のような正しいエッジと誤ったエッジが存在する場合 においては，最適化の結果 2 つの内分点が選ばれてし まい，これは問題の設定を考えると望ましくない結果で ある。ここで，パーティクルフィルタを用いることで， Fig 2(c) のように正しいエッジと誤ったエッジによって生成され得る 2 つのグラフを同時に探索できるのではな いかという動機が発生する。ここから本稿では，ファク ターグラフをパーティクルフィルタの視点から再定式化 することを試みる。
![img-1.jpeg](img-1.jpeg)
(a) case 1
![img-2.jpeg](img-2.jpeg)
(b) case 2
![img-3.jpeg](img-3.jpeg)
(c) case 3

Fig. 2: primitive model of factor graph

## 3 数学的準備

## 3.1 三次元回転の表現

本稿では， 6 自由度状態の推定を行うため，三次元の回転についての演算が伴う。三次元の回転を表す三次元特殊直交群 $\mathrm{SO}(3)$ や並進を追加した三次元特殊ユーク リッド群 $\mathrm{SE}(3)$ 及びその積 $\otimes$ はベクトル空間上で計算 することができないため，微分その他の演算を可能にす るための適応が必要である。ここでは，文献 ${ }^{5)}$ に倣い $\mathrm{SO}(3), \mathrm{SE}(3)$ におけるベクトル空間への写像，逆写像お よび，任意の演算における演算子の定義を行う。SO(3) は Lie 群をなし，以下の指数，対数写像によって対応す る Lie 代数 $\mathrm{so}(3)$ 及び線形空間上のベクトルへと変換で きる。

$$
\begin{aligned}
& \log : \mathrm{SO}(3) \rightarrow \mathbb{R}^{3} \\
& \exp : \mathbb{R}^{3} \rightarrow \mathrm{SO}(3) \\
& \left(\cdot\right)^{\circ}: \mathrm{so}(3) \rightarrow \mathbb{R}^{3} \\
& \left(\cdot\right)^{\wedge}: \mathbb{R}^{3} \rightarrow \mathrm{so}(3)
\end{aligned}
$$

$\mathrm{SE}(3)$ についても同様に

$$
\begin{aligned}
& \log : \mathrm{SE}(3) \rightarrow \mathbb{R}^{6} \\
& \exp : \mathbb{R}^{6} \rightarrow \mathrm{SE}(3)
\end{aligned}
$$

が定義できる。また $T \in \mathrm{SE}(3)$ は並進を表す $\mathbf{t} \in \mathbb{R}^{3}$ と回転を表す $\mathbf{R} \in \mathrm{SO}(3)$ を用いて以下の行列分解

$$
T=\left(\begin{array}{c|c|c}
\mathbf{R} & \mathbf{t} \\
\hline 0 & 0 & 0
\end{array}\right) \simeq\left(\begin{array}{ccc}
\mathbf{d}_{1} & \mathbf{d}_{2} & \mathbf{d}_{3} \\
\hline 0 & 0 & 0
\end{array}\right\lvert\, \begin{aligned}
\mathbf{t} \\
0
\end{aligned}
$$

$\oplus: \mathrm{SE}(3) \times \mathbb{R}^{3} \rightarrow \mathbb{R}^{3}$,

$$
\theta, p \mapsto \mathbf{R}(\theta) p+\mathbf{t}(\theta)
$$

を定義する。

## 3.2 Relaxed ADMM ${ }^{8}$

本稿で扱う協調自己位置推定は，合意制約付き凸最適化問題として定式化できる。このとき，Relaxed ADMM （Alternating Direction Method of Multipliers） ${ }^{8)}$ を用いて，元問題を双対空間上の近似問題として解くことが可能と なる。

以下のような制約付き最適化問題

$$
\begin{aligned}
& \min _{x, y} f(x)+g(y) \\
& \text { s.t. } A x+B y=b
\end{aligned}
$$

は，次のようなラグランジアン

$$
\mathcal{L}(x, y, z)=f(x)+g(y)+z^{T}(b-A x-B y)
$$

を最小化する緩和問題に変換することで解くことができ る. ここで， $z$ は双対変数である。一方で，ラグランジ アンが直接最小化できない場合には，ラグランジュ双対関数

$$
\begin{aligned}
d(z) & =\inf _{x, y}\left\{f(x)+g(y)+z^{T}(b-A x-B y)\right\} \\
& =\inf \{f(x)-\langle z, A x\rangle\}+\inf _{y}\{f(x)-\langle z, B y-b\rangle\} \\
& =\inf \mathcal{L}_{f}(z)+\inf _{y} \mathcal{L}_{g}(z)
\end{aligned}
$$

を最大化することで元の問題を解くことができる。Relaxed ADMM では式（7）の文脈から

$$
\begin{aligned}
& y^{+}=\underset{y}{\arg \min } \mathcal{L}_{g, y}(z) \\
& \omega_{g}=z-\gamma\left(B y^{+}-b\right) \\
& x^{+}=\underset{x}{\arg \min } \mathcal{L}_{f, y}\left(2 \omega_{g}-z\right) \\
& \omega_{f}=2 \omega_{g}-z-\gamma A x^{+} \\
& z^{+}=z+\eta\left(\omega_{f}-\omega_{g}\right)
\end{aligned}
$$

のように計算することで，式（5）の解を求めることがで きる ${ }^{8)}$ 。ただし，この時 $\mathcal{L}_{f, y}, \mathcal{L}_{g, y}$ はそれぞれ拡張ラグ ランジアン

$$
\begin{aligned}
& \mathcal{L}_{f, y}(z)=f(x)-\langle z, A x\rangle+\frac{\gamma}{2}\|A x\|^{2} \\
& \mathcal{L}_{g, y}(z)=g(y)-\langle z, B y-b\rangle+\frac{\gamma}{2}\|B y-b\|^{2}
\end{aligned}
$$

である.

## 3.3 Stein Variational Gradient Descent ${ }^{9}$ )

以下のようなカルバックライブラー情報量の最小化問題を考える。

$$
q^{*}=\underset{q \in Q}{\arg \min }\left\{D_{K L}(q \| p) \equiv \mathbb{E}_{q}[\log q(x)]-\mathbb{E}_{q}[\log p(x)]\right\}
$$

このとき以下の定理が成り立つ.
Theorem 1 (KL ダイバージェンスと Stein Operator の関係 ${ }^{9}$ )。 $\boldsymbol{T}(x)=x+\epsilon \phi(x)$ のような変換において， $x \sim q(x)$ のとき $z=\boldsymbol{T}(x)$ の確率分布を $q_{(\boldsymbol{T})}(z)$ とすると

$$
\nabla_{x} D_{K L}\left(\left.q_{(\boldsymbol{T})} \| p\right)\right|_{\epsilon=0}=-\mathbb{E}_{x \sim q}\left\{\mathcal{A}_{p} \phi(x)\right\}
$$

が成り立つ. ただし $\mathcal{A}_{p} \phi(x)=\nabla_{x} \log p(x) \phi(x)^{\mathrm{T}}+$ $\nabla_{x} \phi(x)$ は Stein Operator, $\phi$ は再生核ヒルベルト空間 $\mathcal{H}^{d}$ に属する沉関数である。

ここで，Kernelized Stein Discrepancy(KSD)を

$$
\mathbb{D}(q, p)=\max _{\phi \in \mathcal{H}^{d}} \mathbb{E}_{x \sim q}\left[\mathcal{A}_{p} \phi(x)\right], \quad \text { s.t. } \quad|\psi| \mid \eta_{P} \leq 1
$$

のように定義すると，この問題の解は

$$
\phi_{q, p}^{*}(\cdot)=\mathbb{E}_{x \sim q}\left[k(x, \cdot) \nabla_{x} \log p(x)+\nabla_{x} k(x, \cdot)\right]
$$

で与えられる。

## 4 問題設定

協調自己位置推定は，時間ステップ $t$ におけるエー ジェント $i$ の状態を $x_{i}^{t}$ ，各エージェントの観測を $z_{t}=$ $\left\{z_{t}^{1}, \cdots, z_{t}^{N}\right\}$ とすると，以下のような最大事後確率推定問題（MAP 推定問題）

$$
\max _{x_{i+1}^{t} \cdots x_{i+1}^{N}} \sum_{t=1}^{N} P\left(x_{t+1}^{t} \mid Z_{t}\right) Q\left(z_{t+1} \mid x_{t+1}\right)
$$

として定式化できる。ここで，分布 $P$ は時間ステップ $t$ において，観測情報 $z_{t}$ が与えられる前のエージェン ト $i$ の状態に関する確率分布を表し，分布 $Q$ は状態 $x_{i}$ において観測情報 $z_{t}$ が得られる尤度分布を表す。また， $Z_{t}=\left[z_{0} \cdots z_{t}\right]$ である。

## 5 Stein Particle Filter

Stein Particle Filter(SPF) は，非ガウス・非線形な確率的状態推定問題を数値解析する手法であり，基本的な枠組みは文献 ${ }^{7)}$ によって提案された。SPF では確率分布をパーティクル集合によって表現し，それらを Stein Variational Gradient Descent(SVGD) に基づく勾配法で更新する。

従来のパーティクルフィルタでは，リサンプリング によるパーティクル劣化が問題となり得るが，SPF は SVGD を用いてパーティクル分布を滑らかに変形させ，目標分布へと収束させる。SVGD は，カルバック・ライ ブラー情報量（KL ダイバージェンス）の最小化を，Stein演算子と呼ばれる作用素を用いて実行する。ここで，

$$
q^{*}=\underset{q}{\arg \min } D_{K L}(q \| p)
$$

を考えると， $q$ から $p$ へ収束させるための変分推論を式（11）のように Stein 演算子を用いて定式化すること で，パーティクルを勾配に従って移動させ， $p$ へ近付け ることができる。

式（14）は， $P$ がガウス分布であると仮定できない場合，カルマンフィルタなど解析的な手法で解くことが困難である。Stein Particle Filter では，分布 $P\left(x_{i} \mid Z_{t}\right)$ を

$$
P\left(x_{i} \mid Z_{t}\right)=\frac{1}{m} \sum_{j=1}^{m} P\left(x_{j}^{t} \mid Z_{t}\right)
$$

のようにパーティクルの集合 $\mathcal{X}=\left\{x_{i}^{t}\right\}_{i=1}^{m}$ で近似し，各 パーティクルを勾配法に従って移動させることで，確率分布の代数的近似を行う事なく MAP 推定問題を解くこ とができる。Stein Particle Filter は種々のフィルタリン グ手法と同様に，後述するような $P\left(x_{t+1} \mid z_{t}\right)$ の計算を行 う予測ステップと， $Q\left(z_{t+1} \mid x_{t+1}\right)$ の計算を行う更新ステッ プに分けられる。

## 5.1 $P\left(x_{t+1} \mid z_{t}\right)$ の計算 (予測ステップ)

予測ステップでは，式（14）の $P\left(x_{t+1} \mid z_{t}\right)$ を計算する必要がある。これは $t$ ステップまでの情報から $t+1$ ステッ プの状態を予測する操作であり，VINSでは次のように IMU から得られる加速度 $a_{m}$ ，角速度 $\omega_{m}$ を数値積分す ることで得られる。 $\mathrm{SE}(3)$ 上での数値積分は文献 ${ }^{6)}$ など と類似した枠組みを用いて

$$
\begin{aligned}
p_{k+1}= & p_{k}+v_{k} \Delta t \\
& +\int \int_{i \in\left\{z_{t}, i_{t+1}\right\}} \frac{\left\{R_{k}\left(a_{m}-b_{n}-\eta_{m d}\right)+g\right\} d t^{2}}{\dot{a}} \\
v_{k+1}= & v_{k}+\int_{i \in\left\{z_{t}, i_{t+1}\right\}} \dot{a} d t \\
& R_{k+1}= & R_{k} \otimes \exp \left(\int_{i \in\left\{z_{t}, i_{t+1}\right\}} \frac{\left(\omega_{m}-b_{g}-\eta_{g d}\right) d t}{\dot{\omega}}\right)
\end{aligned}
$$

のように計算できる。ここで， $p$ は位置， $v$ は速度， $R$ は姿勢を表す回転行列， $b_{n}, b_{g}$ はそれぞれ加速度，角速度のバイアス， $\eta_{m d}, \eta_{g d}$ は白色ノイズである。数値積分に よって得られた変換 ${ }^{t} T_{t+1} \in \mathrm{SE}(3)$ によって各パーティ クルは

$$
\bar{x}_{t+1}^{t}=x_{i}^{t} \otimes^{t} T_{t+1}, \forall i
$$

のように更新される。

## 5.2 $Q\left(z_{t+1} \mid x_{t+1}\right)$ の計算 (更新ステップ)

更新ステップでは，観測の尤もらしさを表す尤度で ある $Q\left(z_{t+1} \mid x_{t+1}\right)$ を計算し，何らかの方法で予測分布 $P\left(x_{t+1} \mid z_{t}\right)$ を変換する必要がある。Stein Particle Filter で は，式（14）をカルバック・ライブラー情報量（KL ダイ バージェンス） $D_{K L}$ の最小化問題

$$
\begin{aligned}
& \max _{x_{t+1}^{t}} P\left(x_{t+1}^{t} \mid Z_{t}\right) Q\left(z_{t+1} \mid x_{t+1}\right) \\
& =\max _{x_{t+1}^{t}} \underset{P_{Q T}}{\arg \min } D_{K L}\left(P\left(x_{t+1}^{t} \mid Z_{t}\right) \mid T_{1} \| P_{Q}\right)
\end{aligned}
$$

に置き換える。更に，式（18）の定式化を変分推論に変更 することで，

$$
\begin{aligned}
\min _{P_{i}(x)} \nabla_{x} D_{K L}\left(P\left(x_{i+1}^{j} \mid Z_{i}\right)_{\left[T_{i}\right.}| | P_{Q_{i}}\right)_{\mid \kappa=0} \\
\Rightarrow T:=T \equiv \phi^{*} \\
\phi^{*}(x)=\frac{1}{m} \sum_{j=1}^{m}\left(\nabla_{x_{j}} \log p\left(x_{j}\right) k\left(x, x_{j}\right)\right. \\
\left.+\nabla_{x_{j}} k\left(x, x_{j}\right)\right)
\end{aligned}
$$

のようにMAP 解を求めることができる ${ }^{7)}$ 。ここで $k(\cdot, \cdot)$ はSE(3) 上のカーネル（一般化内積）

$$
\begin{aligned}
k\left(x_{i}, x_{j}\right) & =\exp \left(-d_{i j}^{2} W d_{i j}\right) \\
d_{i j} & =x_{j} \boxminus x_{i}
\end{aligned}
$$

である。

## 6 Stein Relaxed ADMM

本稿では，Stein Particle Filter と Relaxed ADMM を組 み合わせて，外れ値が多発する環境下での協調 VINS 問題を解く Stein Relaxed ADMM フレームワークを提案 する。これにより，複数のエージェント間で整合する相対位置関係を，Stein 変分勾配と ADMM による制約合意を同時に実現できる。詳細な導出は本文中で示した通 り，KL ダイバージェンス最小化問題と Relaxed ADMM を組み合わせることで，多峰性分布を扱う確率的合意問題が Stein 勾配法で解釈可能となる。

式（14），式（18）より協調自己位置推定問題は以下のよ うな問題

$$
\begin{aligned}
\max _{x_{i}^{\prime}, \cdots, x_{i}^{0}} & \sum_{i=1}^{N} \arg \min _{P_{i}(x)} D_{K L}\left(P_{i}\left(x_{i}^{j} \mid z_{i-1}^{j}\right)_{\left[T_{i}\right.}\right) \mid P_{Q_{i}} \mid) \\
\text { s.t. } & x_{i}^{j}=x_{i}^{j}, \forall(i, j) \in \mathcal{E}
\end{aligned}
$$

として定式化できる。
式（21）は

$$
\max _{x_{i}^{\prime}, \cdots, x_{i}^{0}} \sum_{i=1}^{N} \arg \min _{P_{i}(x)} D_{K L}\left(P_{i}\left(x_{i}^{j} \mid z_{i-1}^{j}\right)_{\left[T_{i}\right.}\right) \mid P_{Q_{i}} \mid)
$$

s.t. $\quad x_{i}^{j}=y_{i j}, x_{i}^{j}=y_{i j}, \forall(i, j) \in \mathcal{E}$,

のように緩和すると式（5）の形式に変換できる。
Definition 1. ここで，Relaxed ADMM における拡張ラ グランジアン式（9）として，確率分布を引数にもつ拡張 ラグランジアンを

$$
\mathcal{L}_{F, y}\left(x_{i}, \zeta\right)=F_{i}\left(x_{i}\right)-\int_{X} \zeta d x_{i}+\frac{\gamma}{2} \int_{X} d x_{i}^{2}
$$

のように再定義する。
これにより式（22）は以下のアルゴリズム

$$
\begin{aligned}
y_{i j}^{*} & =\arg \min _{y_{i j}} \mathcal{L}_{q, y}\left(z_{i j, i}+z_{i j, j}\right) \\
& =\arg \min _{y_{i j}}\left\langle z_{i j, i}+z_{i j, j}, y_{i j}\right\rangle+\frac{\gamma}{2}\left\|y_{i j}\right\|^{2} \\
\left(\omega_{g}\right)_{i j, i} & =z_{i j, i}-\gamma y_{i j}^{*} \\
x_{i}^{*} & =\arg \min _{x_{i}} \mathcal{L}_{F, y}\left(x_{i}, z_{i}\right) \\
& =\arg \min _{x_{i}} D_{K L}\left(P_{i}\left(T_{i}\right) \mid P_{Q_{i}}\right) \\
& \quad-\sum_{j \in \mathcal{E}(i)} \int_{P_{i}}\left\{2\left(\omega_{g}\right)_{i j, i}-z_{i j, i}\right\} d x_{i}+\frac{\gamma}{2}|\mathcal{E}(i)| \int_{P_{i}} d x_{i}^{2}
\end{aligned}
$$

$\left(\omega_{f}\right)_{i j, i}=2\left(\omega_{g}\right)_{i j, i}-z_{i j, i}-\gamma x_{i}^{*}$

$$
z_{i j, i}^{*}=z_{i j, i}+\eta\left(\left(\omega_{f}\right)_{i j, i}-\left(\omega_{g}\right)_{i j, i}\right)
$$

に変換できる。式（24）は計算すると

$$
\begin{aligned}
x_{i}^{*}= & \arg \min _{x_{i}} D_{K L}\left(P_{i}\left(T_{i}\right) \mid P_{Q_{i}}\right) \\
& +\sum_{x \in \mathcal{E}(i)} \int_{P_{i}} z_{i t, r} d x_{i}+\frac{\gamma}{2}|\mathcal{E}(i)| \int_{P_{i}} d x_{i}^{2} \\
z_{i t, i}^{*}= & z_{i t, i}+\eta\left(\left(z_{i t, i}+z_{i t, r}\right) / 2+\gamma x_{i}\right), \quad \forall r \in \mathcal{E}(i)
\end{aligned}
$$

のように表せる。
Theorem 2 (指数的なペナルティとしての追加項)。考え る最適化問題を

$$
\begin{aligned}
& \min _{P_{i}} D_{K L}\left(P_{i} \| P_{Q_{i}}\right)+\sum_{i \in \mathcal{E}(i)} \int P_{i}\left(x_{i}\right) z_{i t, r}\left(x_{i}\right) d x_{i} \\
& +\frac{\gamma}{2}|\mathcal{E}(i)| \int P_{i}\left(x_{i}\right) x_{i}^{2} d x_{i}
\end{aligned}
$$

とする. ただし， $\int P_{i}\left(x_{i}\right) d x_{i}=1$ であり， $D_{K L}\left(P_{i} \| P_{Q_{i}}\right)$ は

$$
D_{K L}\left(P_{i} \| P_{Q_{i}}\right)=\int P_{i}\left(x_{i}\right) \log \frac{P_{i}\left(x_{i}\right)}{P_{Q_{i}}\left(x_{i}\right)} d x_{i}
$$

で定義される. このとき, 上記の最適化問題に対する最適分布 $P_{i}^{*}$ は，ある正規化定数 $Z$ を用いて

$$
P_{i}^{*}\left(x_{i}\right)=\frac{P_{Q_{i}}\left(x_{i}\right) \exp \left(-\sum_{r \in \mathcal{E}(i)} z_{i r, r}\left(x_{i}\right)-\frac{\gamma}{2}|\mathcal{E}(i)| x_{i}^{2}\right)}{Z}
$$

と表せる. すなわち, 追加のペナルティ項

$$
\sum_{i \in \mathcal{E}(i)} z_{i t, r}\left(x_{i}\right)+\frac{\gamma}{2}|\mathcal{E}(i)| x_{i}^{2}
$$

は，基準分布 $P_{Q_{i}}\left(x_{i}\right)$ に対して指数的な重み（ペナルティ） を掛ける形で最適解を特徴づける。

Proof. まず，目的関数を $P_{i}$ について書き下すと，

$$
\begin{aligned}
\mathcal{J}\left(P_{i}\right)= & \int P_{i}\left(x_{i}\right) \log \frac{P_{i}\left(x_{i}\right)}{P_{Q_{i}}\left(x_{i}\right)} d x_{i} \\
& +\sum_{r \in E(i)} \int P_{i}\left(x_{i}\right) z_{i r, r}\left(x_{i}\right) d x_{i}+\frac{\gamma}{2}|\mathcal{E}(i)| \int P_{i}\left(x_{i}\right) x_{i}^{2} d x_{i}
\end{aligned}
$$

これを一つの積分にまとめると，

$$
\begin{aligned}
\mathcal{J}\left(P_{i}\right)= & \int P_{i}\left(x_{i}\right)\left(\log P_{i}\left(x_{i}\right)-\log P_{Q_{i}}\left(x_{i}\right)\right. \\
& \left.+\sum_{r \in E(i)} z_{i r, r}\left(x_{i}\right)+\frac{\gamma}{2}|\mathcal{E}(i)|\left|x_{i}^{2}\right\rangle\right) d x_{i}
\end{aligned}
$$

$P_{i}$ に対して最適化し， $\int P_{i}\left(x_{i}\right) d x_{i}=1$ の制約付き変分問題を考えると，ラグランジュ乗数 $\lambda$ を用いて

$$
\begin{aligned}
& \delta_{P_{i}}\left[\int P_{i}\left(x_{i}\right) \log P_{i}\left(x_{i}\right) d x_{i}\right. \\
& -\int P_{i}\left(x_{i}\right) \log P_{Q_{i}}\left(x_{i}\right) d x_{i} \\
& +\int P_{i}\left(x_{i}\right)\left(\sum_{r} z_{i r, r}\left(x_{i}\right)+\frac{\gamma}{2}|\mathcal{E}(i)| x_{i}^{2}\right) d x_{i} \\
& \left.-\lambda\left(\int P_{i}\left(x_{i}\right) d x_{i}-1\right)\right]=0
\end{aligned}
$$

変分微分をとると,

$$
\begin{aligned}
& \log P_{i}\left(x_{i}\right)+1-\log P_{Q_{i}}\left(x_{i}\right) \\
& +\sum_{r \in E(i)} z_{i r, r}\left(x_{i}\right)+\frac{\gamma}{2}|\mathcal{E}(i)| x_{i}^{2}-\lambda=0
\end{aligned}
$$

よって,

$$
\begin{aligned}
& \log P_{i}\left(x_{i}\right)=\log P_{Q_{i}}\left(x_{i}\right) \\
& -\sum_{r \in E(i)} z_{i r, r}\left(x_{i}\right)-\frac{\gamma}{2}|\mathcal{E}(i)| x_{i}^{2}+\lambda-1
\end{aligned}
$$

$\lambda-1$ は定数（正規化のための定数）であるから，これを $-\log Z$ と置くと,

$$
\begin{aligned}
& \log P_{i}\left(x_{i}\right)=\log P_{Q_{i}}\left(x_{i}\right) \\
& -\sum_{r \in E(i)} z_{i r, r}\left(x_{i}\right)-\frac{\gamma}{2}|\mathcal{E}(i)| x_{i}^{2}-\log Z
\end{aligned}
$$

両辺の指数をとると,

$$
P_{i}\left(x_{i}\right)=\frac{P_{Q_{i}}\left(x_{i}\right) \exp \left(-\sum_{r \in E(i)} z_{i r, r}\left(x_{i}\right)-\frac{\gamma}{2}|\mathcal{E}(i)| x_{i}^{2}\right)}{Z}
$$

ここで，

$$
Z=\int P_{Q_{i}}\left(x_{i}\right) \exp \left(-\sum_{r \in E(i)} z_{i r, r}\left(x_{i}\right)-\frac{\gamma}{2}|\mathcal{E}(i)| x_{i}^{2}\right) d x_{i}
$$

は正規化定数である。これにより，与えられた追加項は基準分布 $P_{Q_{i}}\left(x_{i}\right)$ に対して指数的なペナルティとして効 いていることが明らかである。よって定理は証明され た。

したがって式（19），式（24）から，パーティクルの更新則は

$$
\begin{aligned}
& \min _{x_{i}} D_{K L}\left(P_{i}\left[\mid T\right]\left\|P_{Q_{i}}\right)+\sum_{r \in E(i)} \int_{P_{i}} z_{i r, r} d x_{i}+\frac{\gamma}{2}|\mathcal{E}(i)| \int_{P_{i}} d x_{i}^{2} \\
& \Rightarrow x:=x \oplus \phi^{*}, \\
& \phi^{*}=\frac{1}{n} \sum_{j=1}^{n}\left(\nabla_{x_{j}} \log P_{i}^{*}\left(x_{j}\right) k\left(x, x_{j}\right)+\nabla_{x_{j}} k\left(x, x_{j}\right)\right), \\
& P_{i}^{*}\left(x_{j}\right) \propto P_{Q_{i}}\left(x_{j}\right) \exp \left(-\sum_{r \in E(i)} z_{i r, r}\left(x_{i}\right)-\frac{\gamma}{2}|\mathcal{E}(i)| x_{i}^{2}\right),
\end{aligned}
$$

のように修正できる。
これらの操作により，式（14）を解くためのアルゴリズ ムは以下のように表すことができる。

## Algorithm 1 Distributed Stein Particle Filter

1: Input: $n$ UAVs, $m$ particles $\left\{x_{i}^{j}\right\}_{j=1}^{m}$, Target distribution $P_{Q_{i}}$
2: for $k=1$ to $K$ do
3: preintegration by (16)
4: end for
5: $\forall_{j=1: m} x_{i+1}^{j}=x_{i}^{j} \otimes^{\prime} T_{i+1} / /$ prediction (17)
6: for $l=1$ to $L$ do
7: $\forall_{j=1: m}$ update particle by (38)
8: $\quad x_{i}=\underset{x_{i}}{\arg \min } P_{i}\left(x_{i}\right) / /$ local MAP estimation
9: consensus update by (25)
10: end for

## 7 階層型尤度とその微分

![img-4.jpeg](img-4.jpeg)

Fig. 3: 階層型尤度
Stein Particle Filterによる勾配法を行うには，式（38） における尤度 $p$ とその微分 $\frac{\partial p}{\partial T}$ が計算可能である必要が ある。

VINS における広域の観測情報はカメラ情報のみであ るが，画像から沉用性のある尤度を抽出するのは容易で はない。そのため本研究では，Fig 3 のように広域では NetVLAD 特徴量 ${ }^{12)}$ ，局所では Superpoint 特徴量 ${ }^{13)}$ を用いた階層型の尤度を用いることとする。

まず，計算量削減のため尤度関数は

$$
\log p\left(T_{j}\right)=\sum_{k} e_{k}^{T} \Omega_{k} e_{k}
$$

のように二乗誤差で近似する。したがって，各階層にお いてなんらかの微分可能な誤差 $e^{k}$ を取得すれば式（38） は計算可能であると言える。式（39）より尤度関数の微分は Gauss-Newton 法を用いて

$$
\begin{gathered}
\nabla_{T_{j}} \log p\left(T_{j}\right)=-\boldsymbol{\Psi}^{-1} \mathbf{b} \\
\boldsymbol{\Psi}=\mathbf{J}_{j}^{T} \mathbf{J}_{j}, \mathbf{b}=\mathbf{J}_{j}^{T} \mathbf{r}
\end{gathered}
$$

のように計算できる ${ }^{(1)}$ 。ここでヤコビアン $\mathbf{J}_{r}$ は

$$
\begin{aligned}
\mathbf{J}_{r} & =\left.\frac{\partial h\left(\varepsilon \boxplus T_{j}\right)}{\partial \varepsilon}\right|_{\varepsilon=0} \\
& =\left.\frac{\partial h}{\partial \varepsilon} \frac{\partial \varepsilon\left(\varepsilon \boxplus T_{j}\right)}{\partial \varepsilon}\right|_{\varepsilon=0} \\
& =\left.\sum_{k} \Omega_{k} e_{k}\left(T_{j}\right) \frac{\partial e_{k}\left(\varepsilon \boxplus T_{j}\right)}{\partial \varepsilon}\right|_{\varepsilon=0}
\end{aligned}
$$

として計算できる.

### 7.1 1 層目: NetVLAD 特徴量を用いた尤度

Fig 4 はカメラ軌道上の画像を NetVLAD によって特徴量化し，近似度の高い画像のペアを KNN で検出した ものである。
![img-5.jpeg](img-5.jpeg)

Fig. 4: 特徴量による画像のマッチング
1層目ではこのアルゴリズムを用いて，各ステップで観測されたカメラ画像と，データベースに登録された過去の画像の近似度（距離）を利用する。NetVLAD 特徴量 の近似度を $\omega_{j k}$ ，合意アルゴリズムによって得られた目標姿勢を $T_{k}$ として

$$
e_{k}=\left(T_{k}\right)^{-1} \otimes T_{j}, \quad \Omega_{k}=\omega_{j k}
$$

で誤差を定義する。SE(3) 上の微分 ${ }^{10)}$ より誤差の微分は

$$
\begin{aligned}
\left.\frac{\partial e_{k}\left(\varepsilon \oplus T_{j}\right)}{\partial \varepsilon}\right|_{\varepsilon=0} & =\left.\frac{\partial\left(T_{k}\right)^{-1} e^{e} T_{j}}{\partial \varepsilon}\right|_{\varepsilon=0} \\
& =\left[I_{4} \otimes R\left(\left(T_{k}\right)^{-1}\right)\right]\left.\frac{\partial e^{e} T_{j}}{\partial \varepsilon}\right|_{\varepsilon=0} \\
& =\left(\begin{array}{cc}
\mathbf{0}_{2 \times 3} & -\mathbf{R}\left(T_{k}^{-1}\right) \mathbf{d}_{k}^{\wedge}\left(T_{j}\right) \\
\mathbf{0}_{2 \times 3} & -\mathbf{R}\left(T_{k}^{-1}\right) \mathbf{d}_{k}^{\wedge}\left(T_{j}\right) \\
\mathbf{0}_{2 \times 3} & -\mathbf{R}\left(T_{k}^{-1}\right) \mathbf{d}_{k}^{\wedge}\left(T_{j}\right) \\
\mathbf{R}\left(T_{k}^{-1}\right) & -\mathbf{R}\left(T_{k}^{-1}\right) \mathbf{d}_{k}^{\wedge}\left(T_{j}\right)
\end{array}\right)
\end{aligned}
$$

として計算できる。

### 7.22 層目: 特徴点群を用いた尤度

2層目では Fig 4 のように対応する点同士をマッチン グし，各ペアの距離を最小化させることで勾配を計算す る. フレーム $i$ における Superpoint 特徴量の座標を $p_{i}$ として

$$
e_{k}=T_{j} p_{j}-T_{k} p_{k}, \quad \Omega_{k}=\left(T_{j} \Sigma_{j}\left(T_{j}\right)^{\top}+T_{k} \Sigma_{k}\left(T_{k}\right)^{\top}\right)^{-1}
$$

で誤差を定義する。SE(3) 上の微分 ${ }^{10)}$ より誤差の微分は

$$
\begin{aligned}
\left.\frac{\partial e_{k}\left(\varepsilon \oplus T_{j}\right)}{\partial \varepsilon}\right|_{\varepsilon=0} & =\left.\frac{\partial\left(e^{e} T_{j}\right) \oplus p_{j}}{\partial \varepsilon}\right|_{\varepsilon=0} \\
& =\left(\mathbf{I}_{3}-\left[T_{j} \oplus p_{j}\right]^{\wedge}\right)
\end{aligned}
$$

のように計算できる。

## 8 シミュレーション

提案手法の検証の為，Algorithm 1 のシミュレーショ ンを行った。シミュレーションの条件を表1に示す。

Table 1: シミュレーション条件

| 項目 | 値 |
| :-- | :--: |
| アルゴリズム | Algorithm 1 |
| エージェント数 | 3 |
| エージェントごとのパーティクル数 | 50 |
| 時間ステップ数 | 250 |
| 総マッチング数 | 250 |
| 誤マッチング数 | 52 |
| 外れ値の割合 | $\simeq 0.2$ |

シミュレーションでは，各ステップランダムなエー ジェントペアの相対位置（擬似的なクロージャーループ， Fig 5 内黒点線）がそれぞれ得られると仮定し，その相対情報を用いて各エージェントの位置を推定する。また，約 0.2 の割合で誤った相対位置（Fig 5 内赤点線）が得ら れると仮定し，matplotlib を用いて協調自己位置推定シ ミュレーションを行った。Fig 5 内左上図が真の各エー ジェント位置，右上図，左下図，右下図がエージェント $1,2,3$ の推定画面である。誤った相対位置は，シミュレー ション領域内のランダムな座標と各エージェントの真の位置から生成した。

Fig 5, Fig 6 はそれぞれ 0 ステップと 150 ステップに おける各エージェントの推定状態の様子である。これら から， 0 ステップにおいては一様に設定されていた確率分布（パーティクル）が，150ステップにおいてはすべて のエージェント推定において妥当な位置に収束している ことが分かる。 また，Fig 7 はシミュレーションにおけ る全パーティクルの座標の時間遷移である。Fig 7 から も， 150 ステップほどで合意及び収束が達成されている ことが分かる。

![img-6.jpeg](img-6.jpeg)

Fig. 5: シミュレーション (外れ値の割合：0.2，step: 0 )
![img-7.jpeg](img-7.jpeg)

Fig. 6: シミュレーション (外れ値の割合：0.2, step: 150)
![img-8.jpeg](img-8.jpeg)

Fig. 7: 各パーティクルの状態遷移 (外れ値の割合：0.2)

次に，ストレステストとして誤った相対位置が得られ る確率（外れ値割合）を大きくしてシミュレーションを行った。Fig 8, Fig 9 はそれぞれ外れ値の割合が $0.5,0.8$ の場合の，る全パーティクルの座標の時間遷移である。 ここから， 0.5 の場合には推定が可能だが， 0.8 の場合に は間違った座標に収束していることがわかる。
![img-9.jpeg](img-9.jpeg)

Fig. 8: 各パーティクルの状態遷移 (外れ値の割合：0.5)
![img-10.jpeg](img-10.jpeg)

Fig. 9: 各パーティクルの状態遷移 (外れ値の割合：0.8)

## 9 実機実験

提案手法の一部検証として，BMI270 IMU と Intel RealSense D435 カメラを搭載したUAV を用いた単機で の実機実験を行った。この実験では， 6 DoF 状態推定に おいて SVGD に基づく Stein Particle Filter が誤差蓄積 を低減し，ベンチマーク手法（D2SLAM 等）と比較して精度向上している様子が見られた。複数機での完全な協調効果は未検証であるが，本研究で提案するSPF ベース のフレームワークが実環境下でも有用である可能性を示 す初期的結果が得られた。行った実機実験の条件を表 2 に示す。

Table 2: 実機実験条件

| 項目 | 値 |
| :-- | :--: |
| エージェント数 | 1 |
| パーティクル数 | $20 /$ agent |
| 外れ値の割合 | $\sim 10 \%$ |
| Ground Truth | OptiTrack Flex3, Naturalpoint |

Fig 10 は ROS を用いて自己位置推定を行った様子で ある。Fig 11 は各パーティクルとベンチマーク手法に おける Ground Truth との誤差遷移を示している。ここ から，時系列の大部分で提案手法がベンチマーク手法よ り高精度に推定できていることが確認できる。本実験に よって 6 DoF におけるパーティクルの勾配更新法の有

効性が検証できたが，今後の研究としては複数機を用い た合意手法の検証を行いたい。
![img-11.jpeg](img-11.jpeg)

Fig. 10: ROS による Stein Particle Filter の検証
![img-12.jpeg](img-12.jpeg)

Fig. 11: Stein Particle Filter と D2SLAM ${ }^{14)}$ のベンチマー ク

## 10 おわりに

本稿では，Stein Particle Filter と Relaxed ADMM を組み合わせた新たなフレームワークを提案し，単調な広域環境におけるUAVの協調自己位置推定問題に取り組 んだ。提案手法は，複数エージェント間の合意制約と多峰性分布の扱いを同時に可能にし，従来手法では困難で あった外れ値混在環境下での安定的な位置合意を実現す る。シミュレーションおよび実機実験（単機）結果から， SPF を用いることによって，分布近似と勾配情報を活用 し，既存手法よりもロバストかつ柔軟な自己位置推定が可能であることが示唆された。今後は，複数UAVによ る実機レベルの大規模実験を通じて，提案手法の有効性 とスケーラビリティをさらに検証していく予定である。

## 参考文献

1) T. Qin, P. Li, S. Shen: VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator, IEEE Transactions on Robotics, 34(4), 1004/1020 (2018)
2) W. Chen, X. Wang, S. Gao, G. Peng Shang, C. Zhou, Z. Li, C. Xu, K. Hu: Overview of Multi-Robot Collaborative SLAM from the Perspective of Data Fusion, Machines, (2023)
3) X. Zhou, X. Wen, Z. Wang, Y. Gao, H. Li, Q. Wang, T. Yang, H. Lu, Y. Cao, C. Xu, F. Gao: Swarm of mi-
cro flying robots in the wild, Science Robotics, 7(66), eabm5954 (2022)
4) F. Dellaert, M. Kaess: Factor Graphs for Robot Perception, Foundations and Trends in Robotics, (2017)
5) M. Bloesch, M. Burri, S. Omari, M. Hutter, R. Siegwart: Iterated extended Kalman filter based visualinertial odometry using direct photometric feedback, The International Journal of Robotics Research, 36(9), 1053/1072 (2017)
6) C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza: On-Manifold Preintegration for Real-Time VisualInertial Odometry, IEEE Transactions on Robotics, 33(1), 1/21 (2017)
7) F. A. Maken, F. Ramos, L. Ott: Stein Particle Filter for Nonlinear, Non-Gaussian State Estimation, IEEE Robotics and Automation Letters, 7(2), 5421/5428 (2022)
8) N. Bastianello, M. Todescato, R. Carli, L. Schenato: Distributed Optimization over Lossy Networks via Relaxed Peaceman-Rachford Splitting: a Robust ADMM Approach, Proc. of the 2018 European Control Conference (ECC), 477/482 (2018)
9) Q. Liu, D. Wang: Stein variational Gradient descent: a general purpose Bayesian inference algorithm, Proc. of the 30th Int. Conf. on Neural Information Processing Systems, 2378/2386 (2016)
10) J.-L. Blanco: A tutorial on SE(3) transformation parameterizations and on-manifold optimization, (2012) [Online tutorial note]
11) K. Koide, S. Oishi, M. Yokozuka, A. Banno: MegaParticles: Range-based 6-DoF Monte Carlo Localization with GPU-Accelerated Stein Particle Filter, Proc. of IEEE International Conference on Robotics and Automation (ICRA), (2024)
12) R. Arandjelovic, P. Gronat, A. Torii, T. Pajdla, J. Sivic: NetVLAD: CNN Architecture for Weakly Supervised Place Recognition, IEEE Transactions on Pattern Analysis and Machine Intelligence, 40(6), 1437/1451 (2018)
13) D. DeTone, T. Malisiewicz, A. Rabinovich: SuperPoint: Self-Supervised Interest Point Detection and Description, Proc. of the 2018 IEEE/CVF Conference on Computer Vision and Pattern Recognition Workshops (CVPRW), 337-33712 (2018)
14) H. Xu, P. Liu, X. Chen, S. Shen: $D^{2}$ SLAM: Decentralized and Distributed Collaborative Visual-Inertial SLAM System for Aerial Swarm, IEEE Transactions on Robotics, 40, 3445/3464 (2024)