# Stein Particle Filter を用いた単調環境における協調自己位置推定 

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters
![img-0.jpeg](img-0.jpeg)

O有田 朋樹 滑川徹（慶應義塾大学）
計測自動制御学会 第 12 回制御部門マルチシンポジウム（MSCS2025） March 5, 2024

# 近年、交通環境・日常生活環境・過酷環境等あらゆる領域で <br> 自律ロボットの導入が進んでいる 

ロボットの自律移動には基盤となる自己位置の推定が必須
複数エージェント環境では自己位置推定を協調して行う必要がある
![img-1.jpeg](img-1.jpeg)

Fig.1. tier IV "autoware"
https://github.com/tier4/
AutowareArchitectureProposal.proj
![img-2.jpeg](img-2.jpeg)

Fig.2. preferred networks "カチャカ"
https://speakerdeck.com/watanabe0710/katiyakalmilan-xi
![img-3.jpeg](img-3.jpeg)

Fig.3. uber eats 配送ロボット https://www.sankei.com/article/20240305XLAXOTEI35IONFPJAHFHZPITOE/

![img-4.jpeg](img-4.jpeg)

# Collaborative Visual Inertial System 

![img-5.jpeg](img-5.jpeg)

Fig. 4. CoVINS
$\checkmark$ カメラとIMUを備えた複数エージェント
$\checkmark$ グラフ最適化ベースの手法 (D2SLAM[Xu+, TRO2024] など) がトップ性能

Fig. 5. 特徴量抽出による協調自己位置推定

# Collaborative Visual Inertial System 

![img-6.jpeg](img-6.jpeg)

# * グラフ最適化ベースの手法は単調環境において推定が破綻 <br> $\times$ 根本的に多峰性の強い問題に対処できない [Koide+, ICRA2024] 

![img-7.jpeg](img-7.jpeg)

# 目的と定式化 

## 目的：確率分布の状態での協調自己位置推定

(1)と(2)のグラフを同時に探索

単調環境及び多峰性の強い環境で も
推定可能な曖昧表現性の獲得

Fig. 7
(1)正しいグラフ
(2)誤ったグラフ

## 定式化：KL ダイバージェンスの複数同時最

$\max _{x_{1}^{t}, \cdots, x_{N}^{t}} \sum_{i=1}^{N} \underset{P_{i}[T]}{\arg \min } D_{K L}\left(P_{i}\left(x_{i}^{t} \mid z_{i}^{t-1}\right)_{\left[T_{i}\right]} \| P_{Q_{i}}\right)$,
$x_{i}^{t} \quad$ エージェント i の位置・姿勢
$P_{i}\left(x_{i}^{t} \mid z_{i}^{t-1}\right) \quad$ 位置・姿勢に関する確率分布
![img-8.jpeg](img-8.jpeg)

エージェント 1 の分布
![img-9.jpeg](img-9.jpeg)

エージェント 2 の分布

Fig. 9. KL 最小化

# 確率分布のパーティクル近似および KL 最小化 

## エージェント 1

確率分布
![img-10.jpeg](img-10.jpeg)

相対情報
![img-11.jpeg](img-11.jpeg)

エージェント 2
目標分布
![img-12.jpeg](img-12.jpeg)

Fig. 9. KL 最小化
$\mathrm{D}_{\mathrm{KL}}$

エージェント 2 の分布

# 確率分布のパーティクル近似および KL 最小化 

エージェント 1
確率分布
![img-13.jpeg](img-13.jpeg)

E- 0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
SE(3) 変換
![img-14.jpeg](img-14.jpeg)

E- 0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
E- 0
0
0
0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 0
E- 

# 確率分布のパーティクル近似および KL 最小化 

![img-15.jpeg](img-15.jpeg)

# 確率分布のパーティクル近似および KL 最小化 

![img-16.jpeg](img-16.jpeg)

# 確率分布のパーティクル近似および KL 最小化 

![img-17.jpeg](img-17.jpeg)

# 確率分布のパーティクル近似および KL 最小化 近似（パーティクルフィル タ） 

$$
p\left(x^{t} \mid y^{1: t}\right) \approx \sum_{i=1}^{N} \omega_{i}^{t} \delta\left(x^{t}-x_{i}^{t}\right)
$$

$$
\begin{aligned}
& \omega_{i}^{t} \text { 各パーティクルの重み（尤 } \\
& \text { 度) } \\
& \delta\left(x^{t}-x_{i}^{t}\right)= \begin{cases}0, & x^{t} \neq x_{i}^{t}, \\
\infty, & x^{t}=x_{i}^{t},\end{cases} \text { ディラックのデルタ } \\
& x_{i}^{t}=\left[\begin{array}{cc}
R & t \\
0 & 1
\end{array}\right] \in \operatorname{SE}(3), \quad \text { 6DoF パーティクル }
\end{aligned}
$$

![img-18.jpeg](img-18.jpeg)

Fig. 10. 確率分布の近似

# 確率分布のパーティクル近似および KL 最小化 近似（パーティクルフィル タ） 

$$
p\left(x^{t} \mid y^{1: t}\right) \approx \sum_{i=1}^{N} \omega_{i}^{t} \delta\left(x^{t}-x_{i}^{t}\right)
$$

$\omega_{i}^{t} \quad$ 各パーティクルの重み（尤度)

$$
\begin{aligned}
& \delta\left(x^{t}-x_{i}^{t}\right)=\left\{\begin{array}{ll}
0, & x^{t} \neq x_{i}^{t}, \\
\infty, & x^{t}=x_{i}^{t},
\end{array} \quad \text { ディラックのデルタ }\right. \\
& x_{i}^{t}=\left[\begin{array}{ll}
R & t \\
0 & 1
\end{array}\right] \in \operatorname{SE}(3), \quad 6 \text { DoF パーティクル }
\end{aligned}
$$

![img-19.jpeg](img-19.jpeg)

Fig. 10. 確率分布の近似

SE(3) 変換
![img-20.jpeg](img-20.jpeg)

行列演算
![img-21.jpeg](img-21.jpeg)

Fig. 12. パーティクルの変換

# SE(3) カーネル密度推定 (KDE) 

SE(3) 上のカーネル

$$
\begin{aligned}
k\left(x_{i}, x_{j}\right) & =\exp \left(-d_{i j}^{\top} W d_{i j}\right) \\
d_{i j} & =\log \left(x_{j} \ominus x_{i}\right)
\end{aligned}
$$

カーネルの重ね合わせで SE(3) 状態（勾配）を推定

$$
\Psi^{*}=\frac{\sum_{r \in \mathcal{N}_{i, l}}\left(\mathcal{A}_{j \rightarrow i}\right) \omega_{j, r} k\left(x_{i, l}, x_{i j, r}\right) \Psi^{r}}{\sum_{m \in \mathcal{N}_{i, l}}\left(\mathcal{A}_{j \rightarrow i}\right) \omega_{j, r} k\left(x_{i, l}, x_{j, r}\right)}
$$

![img-22.jpeg](img-22.jpeg)

Fig. 13. カーネル密度推定

# 確率分布のパーティクル近似および KL 最小化 

## SE(3) カーネル密度推定 (KDE)

SE(3) 上のカーネル

$$
\begin{aligned}
k\left(x_{i}, x_{j}\right) & =\exp \left(-d_{i j}^{\top} W d_{i j}\right) \\
d_{i j} & =\log \left(x_{j} \ominus x_{i}\right)
\end{aligned}
$$

カーネルの重ね合わせで SE(3) 状態（勾配）を推定

$$
\Psi^{*}=\frac{\sum_{r \in N_{i, l}\left(\mathcal{A}_{j \rightarrow i}\right)} \omega_{j, r} k\left(x_{i, l}, x_{i j, r}\right) \Psi^{r}}{\sum_{m \in N_{i, l}\left(\mathcal{A}_{j \rightarrow i}\right)} \omega_{j, r} k\left(x_{i, l}, x_{j, r}\right)}
$$

![img-23.jpeg](img-23.jpeg)

Fig. 13. カーネル密度推定

## KL 最小化 (Stein Variational Gradient Descent[Liu+, NIPS2016])

$$
\nabla_{\epsilon} D_{K L}\left\{\left.q_{[T]} \| p\right)\right|_{\epsilon=0}=-\mathbb{E}_{x \sim q}\left[\mathcal{A}_{p} \phi(x)\right]
$$

where $\mathcal{A}_{p} \phi(x)=\nabla_{x} \log p(x) \phi(x)^{\top}+\nabla_{x} \phi(x)$
![img-24.jpeg](img-24.jpeg)
$x_{i, l}^{+}:=x_{i, l} \oplus \operatorname{Exp}\left(\Phi_{i, l}^{*}\right) \quad$ KDE による目標分布への勾配

$$
\Phi_{i, l}^{*}=\frac{\sum_{m \in N_{i, l}\left(\mathcal{A}_{i}\right)} k\left(x_{i, l}, x_{i, m} \|\Psi^{*}\right)+\nabla_{x_{i, m}} k\left(x_{i, l}, x_{i, m}\right)}{\sum_{m \in N_{i, l}\left(\mathcal{A}_{i}\right)} k\left(x_{i, l}, x_{i, m}\right)} \text { パーティクル間 }
$$

の斥力
![img-25.jpeg](img-25.jpeg)

Fig. 14. SVGD

・6自由度姿勢空間（6DoF）におけるSVGD(Stein 変分勾配降下）にリー群 SE(3) 上での計算を適用。
・勾配の計算には Gauss Newton 法を利用

SVGD on SE(3)
$\operatorname{Exp}\left(\boldsymbol{\Phi}_{i, l}^{\mathrm{VA}}\right)$
目標分布への勾配
$\boldsymbol{\Phi}_{i, l}^{*}=\frac{\sum_{m \in N_{i, l}\left(\mathcal{A}_{i}\right)} k\left(x_{i, l}, x_{i, m}\right) \overline{\boldsymbol{\Psi}^{*}}+\nabla_{x_{i, m}} k\left(x_{i, l}, x_{i, m}\right)}{\sum_{m \in N_{i, l}\left(\mathcal{A}_{i}\right)} k\left(x_{i, l}, x_{i, m}\right)}$
$\boldsymbol{\Psi}^{*}=\frac{\sum_{r \in N_{i, l}\left(\mathcal{A}_{j \rightarrow i}\right)} \omega_{j, r} k\left(x_{i, l}, x_{i j, r}\right) \overline{\boldsymbol{\Psi}^{r}}}{\sum_{m \in N_{i, l}\left(\mathcal{A}_{j \rightarrow i}\right)} \omega_{j, r} k\left(x_{i, l}, x_{j, r}\right)}$

## Gauss Newton Method on SE(3)

$\boldsymbol{\Psi}^{r}=\left(\mathbf{H}^{r}\right)^{-1} b^{r}, \quad \mathbf{H}^{r}=\left(\mathbf{J}^{r}\right)^{\top} \mathbf{J}^{r}, \quad b^{r}=\left(\mathbf{J}^{r}\right)^{\top} e^{r}$,
![img-26.jpeg](img-26.jpeg)

勾配による
勾配による
状態更新
線形演算
(KDE\&SVGD)
Fig. 11. SE(3) 上の変換

# 喧率分布のパーティクル近似および変換 

## KL 最小化の検証

エージェント数：3エージェント
$\checkmark$ 各ステップランダムなエージェントペアの相対位置が得られる

## Ground Truth

右のような
整列状態を仮定
![img-27.jpeg](img-27.jpeg)

Fig. 12. シナリオ 1
![img-28.jpeg](img-28.jpeg)
$\times$ 各パーティクルが収束しない
× 3エージェントには 6 DoF×3 の自由度に対し得られるのは 6 DoF×2（相対情報）のみ

# 確率分布のパーティクル近似および変換 

![img-29.jpeg](img-29.jpeg)

## $\times$ 各パーティクルが収束しない

## $\times 3$ エージェントには 6 DoF×3 の自由度に対し得られるのは 6 DoF×2（相対情報）のみ

## KL 最小化 + 合意 (Stein ADMM)

$\max _{x_{1}^{t}, \cdots, x_{N}^{t}} \sum_{i=1}^{N} \underset{P_{i\{T\}}}{\arg \min } D_{K L}\left(P_{i}\left(x_{i}^{t} \mid z_{i}^{t-1}\right)_{\left[T_{i}\right|} \| P_{Q_{i}}\right)$,
s.t. $\quad x_{i}^{t}=x_{j}^{t}, \quad \forall(i, j) \in \mathcal{E}$
$x_{i}^{t}:=\left\{x_{i, j}^{t} \mid j \in \mathcal{N}_{i}\right\} \quad$ パーティクルの組
![img-30.jpeg](img-30.jpeg)

Fig. 16. Stein ADMM
すべてのパーティクルは他エージェントに対応す るパーティクルの組を持つと仮定する

# 問題設定 

$$
\max _{x_{1}^{t}, \cdots, x_{N}^{t}} \sum_{i=1}^{N} \underset{P_{i}[T]}{\arg \min } D_{K L}\left(P_{i}\left(x_{i}^{t} \mid z_{i}^{t-1}\right)_{\left[T_{i}\right]} \| P_{Q_{i}}\right)
$$

s.t. $\quad x_{i}^{t}=y_{i j}, x_{j}^{t}=y_{i j}, \forall(i, j) \in \mathcal{E}$,

$$
\begin{aligned}
& x_{i}^{t}:=\left\{x_{i, j}^{t} \mid j \in N_{i}\right\} \quad \text { パーティクルの組 } \\
& P_{i}\left(x_{i}^{t} \mid z_{i}^{t-1}\right) \quad \text { エージェントiの事前分布 } \\
& P_{Q_{i}} \text { 自標分布 } \quad y_{i j} \text { エージェントi,j間の } \\
& \text { スラック変数 }
\end{aligned}
$$

## 4ADMM[Boyd + , 2010] を適用

## 双対上昇法

$x_{i}^{+}=\underset{x_{i}}{\arg \min } D_{K L}\left(P_{i[T]} \| P_{i}^{*}\right)$
$P_{i}^{*}\left(x_{j}\right) \propto P_{Q_{i}}\left(x_{j}\right.$ 表 $\left(-\sum_{r \in \mathcal{E}_{i}} z_{i r, r}\left(x_{i}\right)-\frac{\gamma}{2}\left|\mathcal{E}_{i}\right| x_{i}^{2}\right)$,
![img-31.jpeg](img-31.jpeg)

合意ステップ
$\forall r \in \mathcal{E}_{i}$,
$z_{i r, i}^{+}=z_{i r, i}+\eta\left(\left(z_{i r, i}+z_{i r, r}\right) / 2+\gamma x_{i}\right)$.

# GPU 上への実装 

## 10^3 以上のパーティクル

を
リアルタイムで動作させる必要性
・では計算量大
オーダー $O\left(n^{2}\right)$
![img-32.jpeg](img-32.jpeg)

全パーティクルを
GPU で並列計算

Fig. 17. パーティクルごとの処理アルゴリズム（Stein
![img-33.jpeg](img-33.jpeg)

シナリオ 1 ：合意 +KL 最小化の検証
エージェント数： 3 エージェント
パーティクル数： $1024 \times 3$ (エージェント数)
$\checkmark$ 各ステップランダムなエージェントペアの
相対位置が得られる
$\checkmark$ 相対位置には一定の確率 e で外れ値が含まれ る
![img-34.jpeg](img-34.jpeg)

Fig. 12. シナリオ 1
![img-35.jpeg](img-35.jpeg)

外れ値割合
$e=0.0$

合意係数
$\gamma=1.0$

外れ値割合
$e=0.0$

合意係数
$\gamma=0.5$

外れ値へのロバスト性を検証
(3)
![img-36.jpeg](img-36.jpeg)

外れ値割合
$e=0.1$
合意係数
$\gamma=1.0$

## 追加シナリオ2：自己位置推定に適用

$\checkmark$ 単機エージェントの内部ループのみで合意なし
$\checkmark$ 現実環境で KL 最小化のみ検証
$\checkmark$ モーションキャプチャで Ground Truth 取得

結果
![img-37.jpeg](img-37.jpeg)

Fig. 18. シナリオ 2

![img-38.jpeg](img-38.jpeg)

外れ値へのロバスト性を検証
(3)
![img-39.jpeg](img-39.jpeg)

外れ値割合
$e=0.1$
合意係数
$\gamma=1.0$

## 追加シナリオ2：自己位置推定に適用

$\checkmark$ 単機エージェントの内部ループのみで合意なし
$\checkmark$ 現実環境で KL 最小化のみ検証
$\checkmark$ モーションキャプチャで Ground Truth 取得

結果
![img-40.jpeg](img-40.jpeg)

Fig. 18. シナリオ 2

# まとめ・今後の展望 

## 本発表：Stein Particle Filter を用いた単調環境における協調自己位置推定

Relaxed ADMM を用いた Stein Particle Filter の合意問題を定式化し、強力な曖昧表現性を有する協調的な自己位置推定を提案
6 自由度姿勢空間における協調最適化手法の定式化により、CoVINS など
現実環境のシステムに適用できる位置推定手法を提案
GPU の利用により、複数機の大量パーティクルを同時に並列計算できるアルゴリズムを実現
提案手法が外れ値に対し一定のロバスト性を有することを確認

## 今後の展望

口 動的な環境下における予測ステップを含めた手法の検証
口 複数実機を用いた提案手法の検証
口 より外れ値にロバスト＆収束の速いアルゴリズムの探索

# ご清聴ありませんか 

Collaborative Localization of UAVs in $\mid$ onot $t$ eavironments Using Stein Particle Filters

[1] X. Zhou, et al., Swarm of micro flying robots in the wild, Science Robotics, 2022.
[2] H. Xu, et al., D2SLAM: Decentralized and Distributed Collaborative Visual-Inertial SLAM System for Aerial Swarm, Transaction on Robotics, 2024.
[3] K. Koide, et al., MegaParticles: Range-based 6-DoF Monte Carlo Localization with GPUAccelerated Stein Particle Filter, 2024.
[4] Q. Liu, et al., Stein Variational Gradient descent: a general purpose Bayesian inference algorithm, In Proceedings of the $30^{\text {th }}$ International Conference on Neural Information Processing Systems, 2016.
[5] S. Boyd, et al., Distributed Optimization and Statistical Learning via the Alternating Direction Method of Multipliers, Foundation and Trends in Machine Learning, 2010.

# 外れ値へのロバスト性を検証 

(5)
![img-41.jpeg](img-41.jpeg)

外れ値割合
$e=0.5$

合意係数
$\gamma=1.0$

Fig. 13. シナリオ 2

- 近傍エージェントのパーティクル分布に対して尤度評価，指数ペナルティの表現をする必要がある。
・尤度の推定にはカーネル密度推定を利用。
・近傍パーティクルの重みを

$$
\hat{\omega}_{i}=\omega_{i} \exp \left(-\sum_{r \in \mathcal{E}_{i}} z_{i r, r}\left(x_{i}\right)-\frac{\gamma}{2}\left|\mathcal{E}_{i}\right| x_{i}^{2}\right) \quad \text { に従って変化させることで擬似的に指数ペナルティを表現 }
$$

![img-42.jpeg](img-42.jpeg)

エージェント1パーティクル
![img-43.jpeg](img-43.jpeg)

指数ペナルティで
重みを変化させて $P_{i}^{\prime}\left(x_{i}\right)$ を得る

Fig7. Evaluation of target distribution and consensus penalty

・ 6 自由度姿勢空間（6DoF）における最適化にリー群 SE(3) 上での計算を適用。
・勾配の計算（Gauss Newton 法）は MegaParticles[Koide+, ICRA2024] の手法を利用

修正 SVGD

$$
\begin{aligned}
& x_{i, l}^{+}:=x_{i, l} \boxplus \boldsymbol{\Phi}_{i, l}^{*} \quad \begin{array}{l}
\text { Multiplication } \\
\text { by Exp map }
\end{array} \\
& \boldsymbol{\Phi}_{i, l}^{*}=\frac{\sum_{m \in \mathcal{N}_{i, l}\left(\mathcal{A}_{i}\right)} k\left(x_{i, l}, x_{i, m}\right) \boldsymbol{\Psi}^{*}+\nabla_{x_{i, m}} k\left(x_{i, l}, x_{i, m}\right)}{\sum_{m \in \mathcal{N}_{i, l}\left(\mathcal{A}_{i}\right)} k\left(x_{i, l}, x_{i, m}\right)} \\
& \boldsymbol{\Psi}^{*}=\frac{\sum_{r \in \mathcal{N}_{i, l}\left(\mathcal{A}_{j \rightarrow i}\right)} \omega_{j, r} k\left(x_{i, l}, x_{i j, r}\right) \boldsymbol{\Psi}^{r}}{\sum_{m \in \mathcal{N}_{i, l}\left(\mathcal{A}_{j \rightarrow i}\right)} \omega_{j, r} k\left(x_{i, l}, x_{j, r}\right)}
\end{aligned}
$$

Gauss Newton Method on SE(3)

$$
\begin{aligned}
& \boldsymbol{\Psi}^{r}=\left(\mathbf{H}^{r}\right)^{-1} b^{r}, \quad \mathbf{H}^{r}=\left(\mathbf{J}^{r}\right)^{\top} \mathbf{J}^{r}, \quad b^{r}=\left(\mathbf{J}^{r}\right)^{\top} e^{r} \\
& e^{r}=\left[\frac{\log \left(\left(x_{i, l} \oplus T_{i \rightarrow j}\right) \ominus x_{j, r}\right)}{\mathbf{J}^{r}}=\frac{\frac{\partial\left(\left(x_{i, l} \oplus e^{\varepsilon} \oplus T_{i \rightarrow j}\right) \ominus x_{j, r}\right)}{\partial \varepsilon}\right]_{\varepsilon=0}
\end{aligned}
$$

![img-44.jpeg](img-44.jpeg)

Fig8. Exponential map and multiplication[10]
![img-45.jpeg](img-45.jpeg)

Fig9. Exponential map and $\log \operatorname{map}[10]$

![img-46.jpeg](img-46.jpeg)

# 問題設定 

$$
\max _{x_{1}^{t}, \cdots, x_{N}^{t}} \sum_{i=1}^{N} \underset{P_{i}[T]}{\arg \min } D_{K L}\left(P_{i}\left(x_{i}^{t} \mid z_{i}^{t-1}\right)_{\left[T_{i}\right]} \| P_{Q_{i}}\right), \quad \text { s.t. } \quad x_{i}^{t}=y_{i j}, x_{j}^{t}=y_{i j}, \forall(i, j) \in \mathcal{E}
$$

## ラグランジアンの最小化

$$
\mathcal{L}_{F_{i}, \gamma}\left(x_{i}, \zeta\right)=D_{K L}\left(P_{i} \| P_{Q_{i}}\right)+\frac{\gamma}{2} \mathbb{E}_{x_{i} \sim P_{i}}\left[\left\|x_{i}-\zeta / \gamma\right\|^{2}\right]
$$

## 双対上昇法

$$
\begin{aligned}
& x_{i}^{+}=\underset{x_{i}}{\arg \min } D_{K L}\left(P_{i[T]} \| P_{i}^{*}\right) \\
& \text { where } \quad P_{i}^{*}\left(x_{j}\right) \propto P_{Q_{i}}\left(x_{j}\right) \exp \left(-\sum_{r \in \mathcal{E}_{i}} z_{i r, r}\left(x_{i}\right)-\frac{\gamma}{2}\left|\mathcal{E}_{i}\right| x_{i}^{2}\right)
\end{aligned}
$$

$$
\begin{aligned}
& \forall r \in \mathcal{E}_{i} \\
& z_{i r, i}^{+}=z_{i r, i}+\eta\left(\left(z_{i r, i}+z_{i r, r}\right) / 2+\gamma x_{i}\right)
\end{aligned}
$$