# CLAUDE.md

このリポジトリで作業する際のガイド。

## プロジェクト概要

[Open-RMF](https://github.com/open-rmf/rmf) の [`free_fleet`](https://github.com/open-rmf/free_fleet)
を経由して toio コア キューブを fleet として制御する統合実装。BLE で N 台の cube を 1 プロセス管理する
クライアント (`toio_free_fleet_client`) と、RMF 側の設定パッケージ (`toio_free_fleet_rmf`) の 2 つで構成される。

**動作環境: Ubuntu 24.04 + ROS 2 Jazzy + rmw-cyclonedds-cpp** (free_fleet upstream の前提に合わせる)。
他ディストリ対応のための条件分岐や互換コードは入れない。

詳細な背景・設計判断・セットアップ手順は [README.md](README.md) を参照。

## 構成

```
toio_free_fleet/
├── toio_free_fleet_client/                       # ament_python (ROS 2 ノード)
│   ├── toio_free_fleet_client/
│   │   ├── transform.py        # mat 座標 ↔ RMF map 座標 (m)
│   │   ├── cube_manager.py     # BLE 接続管理 (1 プロセス N 台)
│   │   ├── navigator.py        # waypoint 追従 (motor_control_target + 完了通知待ち)
│   │   ├── ros_adapter.py      # Nav2 互換 topic/action の expose
│   │   └── main.py             # rclpy entry (asyncio + rclpy executor)
│   ├── config/client.yaml
│   └── tests/
└── toio_free_fleet_rmf/                          # ament_python (RMF アセット)
    ├── config/fleet/toio_config.yaml             # navigation_stack: 2, reference_coordinates
    ├── config/zenoh/toio_zenoh_bridge_ros2dds_client_config.json5
    ├── maps/toio/{toio_map.building.yaml, toio_map.png}
    └── launch/
```

`maps/toio/nav_graphs/` は traffic_editor 編集後に
`ros2 run rmf_building_map_tools building_map_generator nav` で都度生成 (リポジトリには commit しない)。

## 不変の設計決定 (変える場合は議論してから)

| 決定 | 値 | 根拠 |
|---|---|---|
| スケール | `METERS_PER_MAT_UNIT = 0.05` | マット 30 cm を RMF で 15 m の "倉庫" に拡大 |
| 原点 | マット左上 = RMF (0, 0) | `reference_coordinates` の 4 点が矩形の 4 隅と一致 |
| Y 軸 | 反転しない (Y-down) | traffic_editor 画面と一致、変換コードがほぼ id |
| 1 プロセス N 台 | `MultipleToioCoreCubes` を 1 つ | PC の BLE セントラルが 1 つしかないため |
| cube ID で名指し | `cube_id` (BLE local name 末尾 3 文字) を `client.yaml` に書く | スキャン順任せだと再起動でロボットが入れ替わる |
| 速度系 | toio 公式仕様から導出 | 環境ごとのキャリブを排除 |
| Nav2 互換 facade | client が `<name>/tf`, `<name>/battery_state`, `<name>/navigate_to_pose` action を expose | upstream `Nav2RobotAdapter` を fork せずに使うため |
| TF はマット左上=原点の RMF メートルで publish | スケール (×50) は client で適用、`reference_coordinates` は画像オフセット吸収のみ | mat unit を TF に流すと貯まる単位の不整合を避けるため |

スケール定数 `METERS_PER_MAT_UNIT = 0.05` は `transform.py` で `mat_to_rmf_xy` /
`rmf_to_mat_xy` に直接使われ、TF publish と goal 受信時の変換に runtime で効く。
`toio_config.yaml` の `reference_coordinates.robot` は client が publish する
TF と同じ frame (マット左上=原点 RMF メートル) で、4 点はマット物理サイズの
4 隅 (0,0)/(15.2,0)/(0,10.8)/(15.2,10.8) を素直に書く。`rmf` 側は traffic_editor
上の placement (画像凡例によるオフセット等) を反映するため別 frame。
スケールを変える場合は `transform.py` の定数と `reference_coordinates.robot` の
4 点を必ず一緒に更新する (yaml だけ変えても client は知らない)。

## 仕様参照

toio コア キューブの BLE 仕様: <https://toio.github.io/toio-spec/>

特に重要なページ:
- `docs/ble_motor` — モーター制御コマンド、速度値の意味、応答コード
- `docs/hardware_position_id` — マットごとの座標範囲 (簡易マットは 98,142〜402,358)
- `docs/hardware_other` — 最高速度 (直進 350 mm/s、回転 1500 °/s)

## コーディングルール

- コメントは「なぜ」を書く。何をしているかは識別子で表現する。
- `transform.py` の定数は spec / 設計決定由来。変更時は CLAUDE.md と README の表も更新。
- `client` の BLE 側 (cube_manager / navigator / transform) は ROS 依存禁止。
  ROS 連携は `ros_adapter.py` と `main.py` にのみ集約する (テスト容易性のため)。

## よくある作業

- マット種別を増やす: `transform.py` の定数と `reference_coordinates` の 4 点を新マットの
  `hardware_position_id` 値から書き換える。スケールは据え置きで OK。
- 速度感を変える: `client.yaml` の `toio.speed_max_value` と `toio_config.yaml` の `limits.linear`
  を一緒に動かす (1 速度値 ≈ 1 mm/s × scale)。
- nav graph を編集: `traffic_editor` で `maps/toio/toio_map.building.yaml` を開いて編集。

## 何をしないか

- マットの物理 cm をそのまま RMF の m として使わない (×50 スケールを通す)。
- cube ごとに別プロセスを立てない (BLE 競合する)。
