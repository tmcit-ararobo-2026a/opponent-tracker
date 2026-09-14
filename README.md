# opponent_tracker

NHK学生ロボコン2026に向けた、**相手ロボットおよび移動バケツのリアルタイム追跡 ROS 2 パッケージ**です。

自己位置推定ノード（`gn10_pointcloud_localization`）から出力される動的点群（`/dynamic_cloud`）とTF（`map -> base_link`）を受け取り、相手ロボットの位置・速度・移動バケツ座標を特定します。

重い PCL ライブラリに一切依存せず、**Pure C++17 + Eigen3** による空間ハッシュグリッド・クラスタリングと 2次元カルマンフィルタを用いて、Jetson 上でミリ秒未満（< 1.5ms）の超高速・安定追跡を実現します。

---

## 1. システム構成とデータフロー

```
[ gn10_pointcloud_localization ] (自己位置推定ノード)
  ├── /dynamic_cloud (sensor_msgs/msg/PointCloud2: 動的点群)
  └── /tf (map -> base_link: 自己位置)
            ↓
[ opponent_tracker_node ] (本パッケージ: Pure C++17 / Eigen3)
  ├── 1. 自機近傍・フィールド外の除外フィルタ
  ├── 2. 空間グリッド・クラスタリング (O(N))
  ├── 3. ロボコン公式ルール幾何検証 (1200mm寸法 & 床接地判定)
  ├── 4. 移動バケツ頂点抽出
  └── 5. 2Dカルマンフィルタ (位置・速度・ヘディング平滑化)
            ↓
[ 出力トピック ]
  ├── /opponent_robot/pose          (geometry_msgs/msg/PoseStamped)   : 相手位置・姿勢
  ├── /opponent_robot/bucket_target (geometry_msgs/msg/PointStamped)  : 移動バケツ座標
  ├── /opponent_robot/velocity      (geometry_msgs/msg/TwistStamped)  : 相手速度
  └── /opponent_robot/markers       (visualization_msgs/msg/MarkerArray: RViz3D描画)
```

---

## 2. 必要環境・依存パッケージ

- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble
- **C++ 標準**: C++17
- **必須ライブラリ**:
  - `Eigen3`
  - ROS 2 標準パッケージ (`rclcpp`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`, `tf2_ros`, `tf2_eigen`, `tf2_geometry_msgs`)
  - ※ **PCL (Point Cloud Library) への依存はありません**。

---

## 3. ビルド手順

```bash
# 依存ライブラリのインストール
sudo apt update
sudo apt install -y libeigen3-dev ros-humble-tf2-eigen ros-humble-tf2-geometry-msgs

# ビルド
cd ~/ros2_ws
colcon build --symlink-install --packages-select opponent_tracker
source install/setup.bash
```

---

## 4. 実行手順（実機 / 収録済み rosbag）

### ターミナル 1（相手追跡ノード + RViz2 起動）
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch opponent_tracker visualize_opponent_tracker.launch.py input_topic:=/dynamic_cloud
```

### ターミナル 2（自己位置推定 または rosbag 再生）
```bash
cd ~/ros2_ws
source install/setup.bash

# 実機（Jetson）で自己位置推定ノードを動かす場合
ros2 launch gn10_pointcloud_localization localization.launch.py

# 開発PCで収録済み rosbag を再生する場合
ros2 bag play src/gn10-pointcloud-localization/rosbag/rosbag2_2026_09_12-22_24_07 --loop
```

---

## 5. 単体テスト・動作確認（LiDAR実機や自己位置推定ノードがない場合）

LiDAR実機や `gn10` ノードが手元にない場合でも、付属のテストツールで相手追跡ノードの単体動作を確認できます：

```bash
cd ~/ros2_ws
source install/setup.bash

# ターミナル 1: 相手追跡ノード + RViz2 起動
ros2 launch opponent_tracker visualize_opponent_tracker.launch.py input_topic:=/dynamic_cloud use_sim_time:=false

# ターミナル 2: テスト用擬似点群を流す
python3 src/opponent-tracker/test/test_feed_pointcloud.py
```
> **確認できること**: 360度周回する相手ロボットの点群に対して、赤い追跡ボックスとバケツがピタッと追従し、トピックが正しく出力されることを確認できます。

---

## 6. トピック仕様

### 購読トピック (Subscribed)
| トピック名 | 型 | 説明 |
| :--- | :--- | :--- |
| `/dynamic_cloud` | `sensor_msgs/msg/PointCloud2` | 自己位置推定ノードから出力される動的点群 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 自己位置座標変換（`map` $\leftrightarrow$ `base_link`） |

### 配信トピック (Published)
| トピック名 | 型 | 説明 |
| :--- | :--- | :--- |
| `/opponent_robot/pose` | `geometry_msgs/msg/PoseStamped` | 相手ロボットのマップ絶対座標 $(X, Y, Z)$ と姿勢 |
| `/opponent_robot/bucket_target` | `geometry_msgs/msg/PointStamped` | 移動バケツ頂点座標 $(X, Y, Z)$ |
| `/opponent_robot/velocity` | `geometry_msgs/msg/TwistStamped` | 相手ロボットの現在移動速度 $(v_x, v_y)$ |
| `/opponent_robot/markers` | `visualization_msgs/msg/MarkerArray` | 相手ロボット（赤色ソリッド枠）・バケツ（黄色円柱）の3D描画 |
| `/our_robot/markers` | `visualization_msgs/msg/MarkerArray` | 自機ロボット（シアン色車体）・進行方向矢印 |

---

## 7. パラメータ設定

`config/opponent_tracker_params.yaml` より変更可能です：

```yaml
opponent_tracker_node:
  ros__parameters:
    input_topic: "/dynamic_cloud"
    target_frame: "map"

    clustering:
      voxel_size: 0.08             # [m] 空間ハッシュのボクセル解像度
      cluster_tolerance: 0.20      # [m] クラスタ結合距離
      min_cluster_size: 15         # 最小点数
      max_cluster_size: 10000

    robot:
      min_width: 0.20              # [m] ロボット最小幅
      max_width: 1.30              # [m] ロボット最大幅（ルール3.2.2: 1200mm）
      min_depth: 0.20              # [m] ロボット最小奥行き
      max_depth: 1.30              # [m] ロボット最大奥行き
      min_height: 0.30             # [m] ロボット最小高さ
      max_height: 2.20             # [m] ロボット最大高さ
      max_ground_z: 0.25           # [m] 床接地判定の最大Z値

    bucket:
      min_z: 1.15                  # [m] 移動バケツ最低地上高
      max_z: 2.15                  # [m] 移動バケツ最高地上高

    field:
      min_x: -6.0                  # [m] フィールドX範囲
      max_x: 6.0
      min_y: -6.5                  # [m] フィールドY範囲
      max_y: 6.5

    court:
      opponent_side: "auto"        # 相手コート設定: "auto" (自機位置から自動判定), "side_a" (領域A), "side_b" (領域B)
      court_y_divider: 0.0         # [m] コート境界Y座標
      center_barrier_margin: 0.30  # [m] 中央教壇エリア除外マージン

    kalman:
      process_noise_pos: 2.0       # 位置プロセスノイズ
      process_noise_vel: 5.0       # 速度プロセスノイズ
      measurement_noise_pos: 0.02  # LiDAR計測ノイズ
      max_association_dist: 2.0    # データ対応付け距離 [m]
```

---

## 8. ライセンス

本リポジトリは [MITライセンス](./LICENSE) のもとで公開されています。
