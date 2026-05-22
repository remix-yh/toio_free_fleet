# toio_free_fleet

[Open-RMF](https://github.com/open-rmf/rmf) の [`free_fleet`](https://github.com/open-rmf/free_fleet) 経由で
[toio コア キューブ](https://toio.io/) を fleet として制御する統合実装。

## 動作環境

- Ubuntu 24.04
- ROS 2 Jazzy
- `rmw-cyclonedds-cpp` (free_fleet upstream の推奨に従う)
- Python 3.10 以上

free_fleet 本体は Jazzy + cyclonedds 構成でテストされているため、本リポジトリも
それを前提とする。Humble 等の他ディストリでは動作未確認。

## Setup

### 1. ROS 2 Jazzy と Open-RMF

ROS 2 Jazzy 公式手順でインストールしたあと、Open-RMF と cyclonedds を追加する。

```bash
sudo apt update
sudo apt install \
  ros-jazzy-desktop \
  ros-jazzy-rmf-dev \
  ros-jazzy-rmw-cyclonedds-cpp \
  python3-pip python3-colcon-common-extensions python3-rosdep
```

### 2. Zenoh

free_fleet は zenoh を通信層に使う。router と ROS 2 bridge を導入する。

```bash
# zenohd: https://zenoh.io/docs/getting-started/installation/
echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" \
  | sudo tee /etc/apt/sources.list.d/zenoh.list
sudo apt update && sudo apt install zenoh

# zenoh-bridge-ros2dds の standalone バイナリ (v1.5.0)
export ZENOH_VERSION=1.5.0
wget -O /tmp/zenoh-plugin-ros2dds.zip \
  https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/$ZENOH_VERSION/zenoh-plugin-ros2dds-$ZENOH_VERSION-x86_64-unknown-linux-gnu-standalone.zip
sudo unzip /tmp/zenoh-plugin-ros2dds.zip -d /usr/local/bin/
```

### 3. free_fleet と toio_free_fleet のビルド

ROS 2 ワークスペースを 1 つ作り、free_fleet と本リポジトリを並べて colcon build する。

```bash
mkdir -p ~/ff_ws/src
cd ~/ff_ws/src
git clone https://github.com/open-rmf/free_fleet
git clone https://github.com/remix-yh/toio_free_fleet.git

cd ~/ff_ws
rosdep install --from-paths src --ignore-src --rosdistro jazzy -yr

# free_fleet が要求する Python 依存
pip3 install nudged 'eclipse-zenoh==1.5.0' pycdr2 rosbags --break-system-packages

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/ff_ws/install/setup.bash
```

### 4. toio クライアント (BLE 側)

cube に BLE で接続する Python クライアントを pip でインストールする。
**ROS は不要**で、cube を直接動かすマシン (Raspberry Pi 等でも可) に入れる。

```bash
cd ~/ff_ws/src/toio_free_fleet/toio_free_fleet_client
pip3 install -e . --break-system-packages
```

### 5. cube のペアリング

toio Core Cube を電源 ON にして簡易プレイマット (TMD01SS) の上に置く。
ペアリングは `toio-py` が起動時に自動でスキャンするため、特別な事前設定は不要。

## 起動

3 つのターミナルを開く。

### Terminal 1: zenoh router

```bash
zenohd
```

### Terminal 2: toio クライアント (BLE)

```bash
cd ~/ff_ws/src/toio_free_fleet/toio_free_fleet_client
toio-free-fleet-client -c config/client.yaml
```

`config/client.yaml` で接続する cube の名前と台数を指定する。
起動後、各 cube の LED 色がコンフィグ通りに点灯すれば接続成功。

### Terminal 3: RMF fleet adapter

```bash
source ~/ff_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch toio_free_fleet_rmf fleet_adapter.launch.xml
```

タスクの投入は `rmf_demos_tasks` 等の通常の RMF タスク dispatcher 経由で行う。

```bash
ros2 run rmf_demos_tasks dispatch_patrol \
  -p waypoint_a waypoint_b -n 3 --use_sim_time false
```

---

## アーキテクチャ

```
   ┌─────────────────────────────────────────────────┐
   │  RMF host (ROS 2 Jazzy)                         │
   │  ┌──────────────┐  ┌─────────────────────────┐  │
   │  │ rmf_core     │←→│ free_fleet_adapter      │  │
   │  └──────────────┘  └──────────┬──────────────┘  │
   └─────────────────────────────── │ Zenoh ─────────┘
                                    ▼
   ┌────────────────────────────────────────────────┐
   │  toio host PC (ROS 不要)                       │
   │  toio_free_fleet_client (Python)               │
   │    cube0  cube1  …   ← BLE (toio-py)           │
   └────────────────────────────────────────────────┘
```

## パッケージ構成

| パス | 内容 | 依存 |
|---|---|---|
| `toio_free_fleet_client/` | BLE で N 台の cube を 1 プロセス管理、free_fleet 互換の Zenoh メッセージを pub/sub | pip (ROS 不要) |
| `toio_free_fleet_rmf/` | RMF 側のマップ、fleet adapter 設定、launch | ROS 2 (ament_python) |

## 主要な設計決定

### スケール `METERS_PER_MAT_UNIT = 0.05` (×50)

簡易プレイマット (TMD01SS) の物理 30 × 22 cm は RMF で扱うには小さすぎるため、
**マット 1 unit = 0.05 m** として仮想 15.2 × 10.8 m の "倉庫" に拡大する。

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
複数 cube を 1 プロセスから扱い、free_fleet 上は "N 台のロボットを持つ fleet" として publish する。

## 開発

```bash
# client 側の単体テスト
cd ~/ff_ws/src/toio_free_fleet/toio_free_fleet_client
pytest tests/

# RMF 側のみ再ビルド
cd ~/ff_ws
colcon build --packages-select toio_free_fleet_rmf
```

## ライセンス

Apache-2.0
