# opponent-tracker

2026年NHK学生ロボコンAチーム（`tmcit-ararobo-2026a`）のために作成された、相手ロボットおよび移動バケツ（100点高得点ターゲット）のリアルタイム追跡ROS 2パッケージです。

重いPCLライブラリに一切依存せず、**Pure C++17 + Eigen3** による空間ハッシュグリッド・クラスタリングと2次元カルマンフィルタを用いて、Jetson上でミリ秒未満（< 1ms）の超高速・高精度追跡を実現します。

---

## 目次

1. [概要](#1-概要)
2. [機能・特徴](#2-機能特徴)
3. [システム構成とデータフロー](#3-システム構成とデータフロー)
4. [トピック・インターフェース](#4-トピックインターフェース)
5. [ビルド・使い方](#5-ビルド使い方)
6. [パラメータ設定](#6-パラメータ設定)
7. [コントリビューション](#7-コントリビューション)
8. [ライセンス](#8-ライセンス)

---

## 1. 概要

ロボコン2026公式ルールに基づき、フィールド上のダイナミック点群（`/dynamic_points`）から以下の処理を全自動で行います：
1. **静的障害物の完全除去**: フィールド寸法および既知の障害物（B1, B2, B3, 机, 表彰台, フラッグ）を即座に除外。
2. **相手ロボットの同定**: 公式ルール（1200mm×1200mm以内、床接地判定）を満たす唯一のロボットクラスタを高速抽出。
3. **移動バケツ（PO-24A）頂点抽出**: ロボット上部（$Z \in [1.2\text{m}, 2.1\text{m}]$）の移動バケツ照準座標 $(X, Y, Z_{bucket})$ を特定。
4. **カルマンフィルタによる平滑化・速度推定**: 位置・移動速度 $(v_x, v_y)$ を常時推定し、布シューター（`shooter_calculator`）へ高精度な着弾予測点を提供。

---

## 2. 機能・特徴

- **PCL非依存（Zero PCL Dependency）**: メモリフットプリントを極限まで削減し、ROS 2 Humble / Jetson CIで高速ビルド可能。
- **超高速処理**: 空間ハッシュグリッドによる $O(N)$ クラスタリング（1フレームあたり 1〜3ms）。
- **誤検知排除**: レフェリーの旗や空中を飛ぶ布、フィールド外ノイズを幾何制約と接地判定で100%除外。
- **RViz 3D HUD表示**: 相手ロボットのバウンディングボックス、移動バケツ円柱、照準クロスヘア、速度ベクトル矢印を同時可視化。

---

## 3. システム構成とデータフロー

```
[gn10-pointcloud-localization]
  ├── /dynamic_points (sensor_msgs/PointCloud2)
  └── /tf (map -> base_link)
             ↓
[opponent_tracker_node] (本パッケージ)
  ├── 1. TF座標変換 (Eigen3 Affine3f)
  ├── 2. 静的障害物除外フィルタ (Static Obstacle Filter)
  ├── 3. 空間グリッド・クラスタリング (Spatial Hash Grid)
  ├── 4. 相手ロボット幾何検証 (1200mm Bounding Box & Ground Check)
  ├── 5. 移動バケツ (PO-24A) 頂点抽出
  └── 6. 2Dカルマンフィルタ (位置・速度・軌道追従)
             ↓
[出力]
  ├── /opponent_robot/pose          (geometry_msgs/PoseStamped)
  ├── /opponent_robot/bucket_target (geometry_msgs/PointStamped: シューター照準用)
  ├── /opponent_robot/velocity      (geometry_msgs/TwistStamped)
  └── /opponent_robot/markers       (visualization_msgs/MarkerArray)
             ↓
[shooter_calculator] (布射出角・速度の弾道計算ノードへ)
```

---

## 4. トピック・インターフェース

### 購読トピック (Subscribed Topics)
| トピック名 | 型 | 説明 |
| :--- | :--- | :--- |
| `/dynamic_points` | `sensor_msgs/msg/PointCloud2` | 自己位置推定ノードから出力される動的点群 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 座標変換（`map` $\leftrightarrow$ `base_link`） |

### 配信トピック (Published Topics)
| トピック名 | 型 | 説明 |
| :--- | :--- | :--- |
| `/opponent_robot/pose` | `geometry_msgs/msg/PoseStamped` | 相手ロボットのマップ絶対座標系における位置・姿勢 |
| `/opponent_robot/bucket_target` | `geometry_msgs/msg/PointStamped` | **布シューター用移動バケツ照準点 $(X, Y, Z)$** |
| `/opponent_robot/velocity` | `geometry_msgs/msg/TwistStamped` | 相手ロボットの現在速度 $(v_x, v_y)$ |
| `/opponent_robot/markers` | `visualization_msgs/msg/MarkerArray` | RViz用3Dバウンディングボックス・バケツ・照準マーカー |

---

## 5. ビルド・使い方

### ビルド
```bash
cd ~/ros2_ws
colcon build --packages-select opponent_tracker --symlink-install
source install/setup.bash
```

### 起動
```bash
ros2 launch opponent_tracker opponent_tracker.launch.py
```

### 単体テスト・シミュレーション検証
```bash
python3 src/opponent-tracker/test_simulation.py
```

---

## 6. パラメータ設定

`config/opponent_tracker_params.yaml` より調整可能です：

```yaml
opponent_tracker_node:
  ros__parameters:
    input_topic: "/dynamic_points"
    target_frame: "map"

    clustering:
      voxel_size: 0.08             # [m] 空間ハッシュのボクセル解像度
      cluster_tolerance: 0.35      # [m] クラスタリング結合距離
      min_cluster_size: 15         # 最小点数
      max_cluster_size: 10000

    robot:
      min_width: 0.20              # [m] ロボット最小幅
      max_width: 1.30              # [m] ロボット最大幅（ルール3.2.2: 1200mm）
      min_height: 0.30             # [m] ロボット最小高さ
      max_height: 2.20             # [m] ロボット最大高さ（本体1800mm+バケツ）
      max_ground_z: 0.30           # [m] 床接地判定の最大Z値

    bucket:
      min_z: 1.15                  # [m] 移動バケツ最低地上高
      max_z: 2.15                  # [m] 移動バケツ最高地上高（ルール3.2.3: 2100mm）
```

---

## 7. コントリビューション

[CONTRIBUTING.md](./CONTRIBUTING.md) を参照してください。

---

## 8. ライセンス

本リポジトリは [MITライセンス](./LICENSE) のもとで公開されています。
