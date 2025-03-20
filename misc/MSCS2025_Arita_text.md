## Slide 1

Stein Particle Filterを用いた単調環境における協調自己位置推定

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

## Slide 2

2

背景

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

✔ 近年、交通環境・日常生活環境・過酷環境等あらゆる領域で
　自律ロボットの導入が進んでいる
✔ ロボットの自律移動には基盤となる自己位置の推定が必須
✔ 複数エージェント環境では自己位置推定を協調して行う必要がある

Fig.3. uber eats 配送ロボット
https://www.sankei.com/article/20240305-XLAXOTEI35IONFPJAHFHZPITOE/

Fig.2. preferred networks “カチャカ”
https://speakerdeck.com/watanabe0710/katiyakallmlian-xi

Fig.1. tier IV “autoware”
https://github.com/tier4/AutowareArchitectureProposal.proj

## Slide 3

3

背景

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

✔ 近年、交通環境・日常生活環境・過酷環境等あらゆる領域で
　自律ロボットの導入が進んでいる
✔ ロボットの自律移動には基盤となる自己位置の推定が必須
✔ 複数エージェント環境では自己位置推定を協調して行う必要がある

Fig3. uber eats 配送ロボット
https://www.sankei.com/article/20240305-XLAXOTEI35IONFPJAHFHZPITOE/

Fig2. preferred networks “カチャカ”
https://speakerdeck.com/watanabe0710/katiyakallmlian-xi

Fig1. tier IV “autoware”
https://github.com/tier4/AutowareArchitectureProposal.proj

[Zhou+, Science Robotics, 2022]

## Slide 4

4

Collaborative Visual Inertial System

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

✔ グラフ最適化ベースの手法
(D2SLAM[Xu+, TRO2024]など)
がトップ性能

✔ カメラとIMUを備えた複数エージェント

Fig. 4. CoVINS

Fig. 5. 特徴量抽出による協調自己位置推定

## Slide 5

✔ グラフ最適化ベースの手法
(D2SLAM[Xu+, TRO2024]など)
がトップ性能

✔ カメラとIMUを備えた複数エージェント

Fig. 1. CoVINS

5

Collaborative Visual Inertial System

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

Fig. 5. 特徴量抽出による協調自己位置推定

[Xu+, TRO2024]

## Slide 6

6

課題点

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

〇 グラフ最適化成功例

✖ グラフ最適化失敗例

屋内(廊下)

✘ グラフ最適化ベースの手法は単調環境において推定が破綻

✘ 根本的に多峰性の強い問題に対処できない[Koide+, ICRA2024]

単調環境の例

屋外(農園)

Fig. 6. 単調環境の例

誤ったマッチング
= 外れ値
によって最適化
が破綻

Fig. 7. グラフ最適化例１

Fig. 8. グラフ最適化例２

エージェント1の(推定)軌道

エージェント2の(推定)軌道

## Slide 7

7

目的と定式化

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

目的：確率分布の状態での協調自己位置推定

✔ ①と②のグラフを同時に探索

✔ 単調環境及び多峰性の強い環境でも
　推定可能な曖昧表現性の獲得

①正しいグラフ

②誤ったグラフ

定式化：KLダイバージェンスの複数同時最小化

エージェント i の位置・姿勢

位置・姿勢に関する確率分布

Fig. 9. KL最小化

Fig. 7

Fig. 8

## Slide 8

8

確率分布のパーティクル近似およびKL最小化

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

相対情報

✖

✖

エージェント1
確率分布

エージェント2
目標分布

エージェント2
確率分布

Fig. 9. KL最小化

## Slide 9

9

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

相対情報

✖

近似

SE(3)変換

✖

エージェント1
確率分布

エージェント2
目標分布

エージェント2
確率分布

Fig. 9. KL最小化

確率分布のパーティクル近似およびKL最小化

## Slide 10

10

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

相対情報

✖

近似

SE(3)変換

✖

近似

✖

エージェント1
確率分布

エージェント2
目標分布

エージェント2
確率分布

Fig. 9. KL最小化

確率分布のパーティクル近似およびKL最小化

## Slide 11

11

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

相対情報

✖

近似

SE(3)変換

✖

近似

✖

カーネル
密度推定

エージェント1
確率分布

エージェント2
目標分布

エージェント2
確率分布

Fig. 9. KL最小化

確率分布のパーティクル近似およびKL最小化

## Slide 12

12

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

相対情報

✖

近似

SE(3)変換

✖

近似

✖

カーネル
密度推定

エージェント1
確率分布

エージェント2
目標分布

エージェント2
確率分布

最尤推定

エージェント2
推定状態

Fig. 9. KL最小化

確率分布のパーティクル近似およびKL最小化

## Slide 13

13

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

近似

近似(パーティクルフィルタ)

Fig. 10. 確率分布の近似

Agent 1
確率分布

Agent 1
パーティクル

確率分布のパーティクル近似およびKL最小化

## Slide 14

14

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

近似

近似(パーティクルフィルタ)

SE(3)変換

SE(3)変換

行列演算

線形演算

Fig. 11. SE(3)上の変換

Fig. 12. パーティクルの変換

Agent 1
確率分布

Agent 1
パーティクル

Agent 1
パーティクル

Agent 2
パーティクル
(推定)

確率分布のパーティクル近似およびKL最小化

Fig. 10. 確率分布の近似

## Slide 15

15

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

Fig. 13. カーネル密度推定

確率分布のパーティクル近似およびKL最小化

カーネル密度推定

Agent 2
パーティクル
(推定)

Agent 2
目標分布

SE(3)カーネル密度推定(KDE)

カーネルの重ね合わせでSE(3)状態(勾配)を推定

近傍探索
(遠方はいずれにせよ0)

## Slide 16

16

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

KL最小化
(Stein Variational Gradient Descent[Liu+, NIPS2016])

カーネル密度推定

KDEによる目標分布への勾配

パーティクル間
の斥力

Fig. 13. カーネル密度推定

Fig. 14. SVGD

Agent 2
パーティクル
(推定)

Agent 2
目標分布

Agent 2
目標分布

Agent 2
パーティクル

確率分布のパーティクル近似およびKL最小化

SE(3)カーネル密度推定(KDE)

カーネルの重ね合わせでSE(3)状態(勾配)を推定

近傍探索
(遠方はいずれにせよ0)

## Slide 17

17

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

・6自由度姿勢空間(6DoF)におけるSVGD(Stein変分勾配降下)にリー群 SE(3) 上での計算を適用．
・勾配の計算にはGauss Newton法を利用

Gauss Newton Method on SE(3)

SVGD on SE(3)

Exp map

Log map

確率分布のパーティクル近似およびKL最小化

目標分布への勾配

カーネル密度推定

→ SE(3)上の微分

勾配による
状態更新

線形演算
(KDE&SVGD)

Fig. 11. SE(3)上の変換

## Slide 18

18

確率分布のパーティクル近似および変換

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

✘ 各パーティクルが収束しない

✘ 3エージェントには 6 DoF×3 の自由度に対し
　得られるのは 6 DoF×2 (相対情報)のみ

✔ 各ステップランダムなエージェントペアの
　 相対位置が得られる

Ground Truth

右のような
整列状態を仮定

２ｍ

２ｍ

エージェント数：3 エージェント

Fig. 12. シナリオ1

KL最小化の検証

## Slide 19

19

確率分布のパーティクル近似および変換

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

KL最小化+合意
(Stein ADMM)

✘ 各パーティクルが収束しない

すべてのパーティクルは他エージェントに対応するパーティクルの組を持つと仮定する

パーティクルの組

合意

Fig. 16. Stein ADMM

Fig. 15. CoSVGD

✘ 3エージェントには 6 DoF×3 の自由度に対し
　得られるのは 6 DoF×2 (相対情報)のみ

## Slide 20

20

Stein ADMM

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

問題設定

双対上昇法

where

主問題の最適化

指数ペナルティ

合意ステップ

目標分布

エージェントi,j 間の
スラック変数

パーティクルの組

エージェントiの事前分布

ADMM[Boyd＋, 2010]を適用

## Slide 21

21

GPU上への実装

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

全パーティクルを
GPUで並列計算

SE(3)状態

重み

予測ステップ

近傍探索

勾配計算

パーティクル更新

IMUデータ

相対情報

CPU上では計算量大

オーダー

※ パーティクルごとのオーダー

10^3以上のパーティクルを
リアルタイムで動作させる
必要性

Fig. 17. パーティクルごとの処理アルゴリズム（Stein ADMM)

## Slide 22

22

シミュレーション

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

シナリオ１：合意+KL最小化の検証

✔ 各ステップランダムなエージェントペアの
　 相対位置が得られる
✔ 相対位置には一定の確率eで外れ値が含まれる

Ground Truth

右のような
整列状態を仮定

合意係数

２ｍ

２ｍ

エージェント数：3 エージェント
パーティクル数：1024 × 3(エージェント数)

外れ値割合

①

②

合意係数

外れ値割合

Fig. 12. シナリオ1

## Slide 23

23

結果

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

追加シナリオ２：自己位置推定に適用

外れ値へのロバスト性を検証

精度高

✔ 現実環境でKL最小化のみ検証

✔ 単機エージェントの内部ループのみで
　合意なし

✔ モーションキャプチャでGround Truth取得

結果

Fig. 18. シナリオ2

③

④

## Slide 24

合意係数

外れ値割合

合意係数

外れ値割合

③

④

24

結果

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

追加シナリオ２：自己位置推定に適用

外れ値へのロバスト性を検証

精度高

✔ 現実環境でKL最小化のみ検証

✔ 単機エージェントの内部ループのみで
　合意なし

✔ モーションキャプチャでGround Truth取得

結果

Fig. 18. シナリオ2

Ground　Truth

Benchmark

パーティクル

画像特徴量
マッチング

## Slide 25

25

結果

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

追加シナリオ２：自己位置推定に適用

外れ値へのロバスト性を検証

精度高

✔ 現実環境でKL最小化のみ検証

✔ 単機エージェントの内部ループのみで
　合意なし

✔ モーションキャプチャでGround Truth取得

結果

Fig. 18. シナリオ2

③

④

## Slide 26

26

まとめ・今後の展望

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

本発表：Stein Particle Filter を用いた単調環境における協調自己位置推定

👍 Relaxed ADMMを用いたStein Particle Filterの合意問題を定式化し、強力な曖昧表現性を有する
　  協調的な自己位置推定を提案
👍 6自由度姿勢空間における協調最適化手法の定式化により、CoVINSなど
　  現実環境のシステムに適用できる位置推定手法を提案
👍 GPUの利用により、複数機の大量パーティクルを同時に並列計算できるアルゴリズムを実現
👍 提案手法が外れ値に対し一定のロバスト性を有することを確認

今後の展望

動的な環境下における予測ステップを含めた手法の検証
複数実機を用いた提案手法の検証
より外れ値にロバスト&収束の速いアルゴリズムの探索

## Slide 27

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

27

## Slide 28

28

参考文献

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

[1] X. Zhou, et al., Swarm of micro flying robots in the wild, Science Robotics, 2022.
[2] H. Xu, et al., D2SLAM: Decentralized and Distributed Collaborative Visual-Inertial SLAM System for Aerial Swarm, Transaction on Robotics, 2024.
[3] K. Koide, et al., MegaParticles: Range-based 6-DoF Monte Carlo Localization with GPU-Accelerated Stein Particle Filter, 2024.
[4] Q. Liu, et al., Stein Variational Gradient descent: a general purpose Bayesian inference algorithm, In Proceedings of the 30th International Conference on Neural Information Processing Systems, 2016. 
[5] S. Boyd, et al., Distributed Optimization and Statistical Learning via the Alternating Direction Method of Multipliers, Foundation and Trends in Machine Learning, 2010.

## Slide 29

29

結果

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

外れ値へのロバスト性を検証

合意係数

外れ値割合

Fig. 13. シナリオ2

⑤

## Slide 30

30

指数ペナルティ

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

・近傍エージェントのパーティクル分布に対して尤度評価，指数ペナルティの
　表現をする必要がある．
・尤度の推定にはカーネル密度推定を利用．
・近傍パーティクルの重みを

に従って変化させることで擬似的に指数ペナルティを表現

Fig7. Evaluation of target distribution and consensus penalty

## Slide 31

31

Stein ADMM on SE(3)

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

・6自由度姿勢空間(6DoF)における最適化にリー群 SE(3) 上での計算を適用．
・勾配の計算(Gauss Newton法)はMegaParticles[Koide+, ICRA2024]の手法を利用

Gauss Newton Method on SE(3)

修正SVGD

Multiplication 
by Exp map

Log map

Fig8. Exponential map and multiplication[10]

Fig9. Exponential map and Log map[10]

[10] F. Dellaert and GTSAM Contributors, borglab/gtsam, Georgia Tech Borg Lab, 2022.

## Slide 32

32

モチベーション

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

## Slide 33

33

Stein ADMM

2025/03/05 MSCS2025

Collaborative Localization of UAVs in Monotone Environments Using Stein Particle Filters

問題設定

ラグランジアンの最小化

双対上昇法

where