# toio_free_fleet

[Open-RMF](https://github.com/open-rmf/rmf) の [`free_fleet`](https://github.com/open-rmf/free_fleet) 経由で
[toio コア キューブ](https://toio.io/) を fleet として制御する統合実装。

## アーキテクチャ

upstream の `Nav2RobotAdapter` を **改造なしで** 使う。本リポジトリの
`toio_free_fleet_client` は ROS 2 ノードとして動き、Nav2 互換の topic / action を
expose する:

| ROS 2 リソース | 内容 |
|---|---|
| `/<cube>/tf` (TFMessage)            | mat→RMF 変換後の cube 姿勢 (マット左上=原点の RMF メートル) |
| `/<cube>/battery_state` (BatteryState) | ダミー (100%) — toio は電池残量を BLE で取れるので将来差し替え |
| `/<cube>/navigate_to_pose` (NavigateToPose action) | RMF メートルで届く目的地を `rmf_to_mat_xy` でマット単位に戻して `motor_control_target` に流す |

`zenoh-bridge-ros2dds` が DDS↔Zenoh 変換を担う。free_fleet 側で CDR エンコードを
自前実装する必要はない。

すべて同一ホストで動かせる。cube 側と RMF 側で `ROS_DOMAIN_ID` を分けるだけで
上下が独立する:

```
ROS 2 Jazzy + cyclonedds, BLE が触れるマシン
├── ROS_DOMAIN_ID=0 (default)
│   ├── toio_free_fleet_client      (rclpy node, BLE→ MultipleToioCoreCubes)
│   │     /<cube>/{tf, battery_state, navigate_to_pose}
│   ├── zenoh-bridge-ros2dds        (上記 DDS topics を Zenoh に橋渡し)
│   └── zenohd                      (Zenoh router)
└── ROS_DOMAIN_ID=55
    ├── free_fleet_adapter          (Nav2RobotAdapter × N, Zenoh subscriber)
    └── rmf_core / rmf_demos_tasks
```

## 動作環境

- Ubuntu 24.04
- ROS 2 Jazzy + `rmw-cyclonedds-cpp`
- Python 3.10 以上

## 前提

以下が `~/ff_ws` にセットアップ済みであること。手順は各 upstream を参照。

- ROS 2 Jazzy ([docs.ros.org](https://docs.ros.org/en/jazzy/Installation.html))
- Open-RMF (apt の `ros-jazzy-rmf-dev` でも、ソースビルドでも可)
- `free_fleet` のビルド + `zenohd` / `zenoh-bridge-ros2dds` のインストール
  ([open-rmf/free_fleet README](https://github.com/open-rmf/free_fleet))

`source ~/ff_ws/install/setup.bash` した状態で `ros2 pkg list | grep free_fleet` が
3 パッケージ (`free_fleet`, `free_fleet_adapter`, `free_fleet_examples`) を返せば前提 OK。

BLE を使うために以下も必要:

```bash
pip3 install 'toio-py>=1.0' --break-system-packages
sudo apt install -y bluez ros-jazzy-nav2-msgs
```

## Setup

### 1. 本リポジトリのクローンとビルド

```bash
cd ~/ff_ws/src
git clone https://github.com/remix-yh/toio_free_fleet.git

cd ~/ff_ws
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -yr
colcon build --packages-select toio_free_fleet_client toio_free_fleet_rmf \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/ff_ws/install/setup.bash
```

### 2. cube ID の確認と `client.yaml` への登録

物理 cube と論理名 (`cube_0`, `cube_1`, ...) のマッピングを固定しないと、
起動するたびにロボットの役割が入れ替わる。BLE local name の末尾 3 文字
(例: `H7p`) を **cube ID** として `client.yaml` に書く。

cube 底面シールには ID が直接印字されていない世代もあるので、**BLE スキャンで確認**:

```bash
# 推奨: 同梱の scan_cubes (cube を 1 台ずつ電源 ON にして個別特定するのに便利)
source ~/ff_ws/install/setup.bash
ros2 run toio_free_fleet_client scan_cubes
# found 2 cube(s):
#   cube_id=H7p   local_name=toio Core Cube-H7p
#   cube_id=j3F   local_name=toio Core Cube-j3F

# 代替: bluetoothctl でも見える
bluetoothctl scan on
# ... [NEW] Device XX:XX:XX:XX:XX:XX toio Core Cube-H7p
bluetoothctl scan off
```

`toio_free_fleet_client/config/client.yaml` の `robots:` を書き換える:

```yaml
fleet:
  name: toio
  robots:
    - name: cube_0
      cube_id: H7p              # ← cube 底面シール末尾 3 文字
      led_color: [0xFF, 0x00, 0x00]
    - name: cube_1
      cube_id: j3F
      led_color: [0x00, 0x00, 0xFF]
```

`led_color` は省略可。指定 ID の cube が見つからなければ `RuntimeError` で停止する。

### 3. nav graph

`maps/toio_map/toio_map.building.yaml` から **build 時に自動生成** される (`CMakeLists.txt`
の `add_custom_command` が `building_map_generator nav` を呼ぶ)。
`traffic_editor` で waypoint / lane を編集したら:

```bash
cd ~/ff_ws
colcon build --packages-select toio_free_fleet_rmf
# install/toio_free_fleet_rmf/share/toio_free_fleet_rmf/maps/toio_map/nav_graphs/0.yaml
# が更新される
```

> `toio_map.building.yaml` の vertex/lane を編集するときは `traffic_editor toio_map.building.yaml` を開く。

## 起動

ターミナル 2 つで完了。cube 側と RMF 側で `ROS_DOMAIN_ID` を分けるのは
upstream nav2 例と同じパターン。

### Terminal 1: cube 側 (zenohd + bridge + client)

```bash
source ~/ff_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch toio_free_fleet_rmf cube_side.launch.xml
```

各 cube の LED 色がコンフィグ通りに点灯すれば接続成功。
zenoh-bridge-ros2dds のバイナリ位置を変えたい場合は
`zenoh_bridge_bin:=<path>` を渡す。

### Terminal 2: RMF 側

```bash
source ~/ff_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=55

ros2 launch toio_free_fleet_rmf fleet_adapter.launch.xml
```

### タスク投入 (同じくドメイン 55)

```bash
source ~/ff_ws/install/setup.bash
export ROS_DOMAIN_ID=55

ros2 run rmf_demos_tasks dispatch_patrol \
  -p waypoint_a waypoint_b -n 3 --use_sim_time false
```

`waypoint_a` 等は `toio_map.building.yaml` で打った頂点名と合わせる。

---

## パッケージ構成

| パス | 内容 | ビルド |
|---|---|---|
| `toio_free_fleet_client/` | rclpy ノード。BLE で N 台の cube を 1 プロセス管理し、Nav2 互換 topic / action を expose | ament_python |
| `toio_free_fleet_rmf/` | RMF 側のマップ、fleet adapter 設定、zenoh-bridge 設定、launch。nav_graph は build 時自動生成 | ament_cmake |

## 主要な設計決定

### スケール `METERS_PER_MAT_UNIT = 0.05` (×50)

簡易プレイマット (TMD01SS) の物理 30 × 22 cm は RMF で扱うには小さすぎるため、
**マット 1 unit = 0.05 m** として仮想 15.2 × 10.8 m の "倉庫" に拡大する。

スケール変換 (`mat_to_rmf_xy`) は client 側で適用し、TF はマット左上=(0, 0) の
RMF メートル系で publish する。一方 traffic_editor は背景画像の凡例分のオフセットを
持つため、その小さなズレは `toio_config.yaml` の `reference_coordinates` 4 点
(`rmf` 側 = 画像上の placement, `robot` 側 = (0,0)〜(15.2, 10.8)) から `nudged`
が学習して upstream adapter 側で吸収する。

### 原点はマット左上

`reference_coordinates` の 4 点をマット 4 隅にそのまま書ける:

| mat unit | RMF [m] |
|---|---|
| (98, 142) 左上 | (0.0, 0.0) |
| (402, 142) 右上 | (15.2, 0.0) |
| (402, 358) 右下 | (15.2, 10.8) |
| (98, 358) 左下 | (0.0, 10.8) |

Y 反転は行わない。RMF map フレームは image 系と同じ Y-down となるが、
traffic_editor の操作感と一致するため可視化が直感的。

### 速度系: 仕様準拠で導出

[toio 仕様](https://toio.github.io/toio-spec/docs/) の以下を直接利用し、環境ごとの計測は不要:

| 量 | 出典 | 値 |
|---|---|---|
| 直進最高速度 | hardware_other | 350 mm/s |
| 回転最高速度 | hardware_other | 1500 °/s |
| cube 外形 | hardware_shape | 31.8 × 31.8 × 19.3 mm |
| `motor_control_target` 最小速度値 | ble_motor | 8 |
| 速度指示値→mm/s | ble_motor (グラフ線形領域) | N ≈ N mm/s |

`toio.speed_max_value = 20` で実機 20 mm/s、RMF 世界では 1.0 m/s に見える。
demo 用に倉庫 AGV 相応の速度感を出す設定。

### 1 プロセス N 台

BLE セントラルが PC に 1 つしかないため、`MultipleToioCoreCubes` で
複数 cube を 1 プロセスから扱う。ROS 2 namespace で `/<cube_name>/` に分割。

### upstream `Nav2RobotAdapter` を改造しない

cube 側を Nav2 互換に "成りすませる" ことで、free_fleet 側は標準の
`navigation_stack: 2` 設定でそのまま動く。これにより:

- 自前で CDR エンコード / Zenoh queryable を書かなくてよい
- free_fleet upstream の更新に追随できる
- 通信仕様は ROS 2 標準メッセージのみ

## 開発

```bash
# client 側の単体テスト
cd ~/ff_ws/src/toio_free_fleet/toio_free_fleet_client
PYTHONPATH=. pytest tests/

# 個別ビルド
cd ~/ff_ws
colcon build --packages-select toio_free_fleet_client
colcon build --packages-select toio_free_fleet_rmf
```

## ライセンス

Apache-2.0
