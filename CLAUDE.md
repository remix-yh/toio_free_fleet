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
├── toio_free_fleet_client/      # pip パッケージ (ROS 不要)
│   ├── toio_free_fleet_client/
│   │   ├── transform.py         # mat 座標 ↔ RMF map 座標 (m)
│   │   ├── cube_manager.py      # BLE 接続管理 (1 プロセス N 台)
│   │   ├── navigator.py         # waypoint 追従 (motor_control_target)
│   │   ├── zenoh_bridge.py      # free_fleet Zenoh メッセージ pub/sub
│   │   └── main.py              # CLI entry
│   ├── config/client.yaml
│   └── tests/
└── toio_free_fleet_rmf/         # ament_python パッケージ
    ├── config/fleet/toio_config.yaml  # vehicle_traits, reference_coordinates
    ├── config/zenoh/client_config.json5
    ├── maps/toio/{toio.building.yaml, toio.png}
    └── launch/
```

`maps/toio/` 配下に `nav_graphs/` サブディレクトリは置かない (free_fleet_examples 準拠)。
nav graph は building.yaml から build 時に生成される。

## 不変の設計決定 (変える場合は議論してから)

| 決定 | 値 | 根拠 |
|---|---|---|
| スケール | `METERS_PER_MAT_UNIT = 0.05` | マット 30 cm を RMF で 15 m の "倉庫" に拡大 |
| 原点 | マット左上 = RMF (0, 0) | `reference_coordinates` の 4 点が矩形の 4 隅と一致 |
| Y 軸 | 反転しない (Y-down) | traffic_editor 画面と一致、変換コードがほぼ id |
| 1 プロセス N 台 | `MultipleToioCoreCubes` を 1 つ | PC の BLE セントラルが 1 つしかないため |
| 速度系 | toio 公式仕様から導出 | 環境ごとのキャリブを排除 |

スケール定数は `transform.py` (client 側) と `toio_config.yaml` の `reference_coordinates` (RMF 側)
の 2 箇所に書かれる意図的な冗長性がある。値を変える場合は両方を必ず同時に更新すること。

## 仕様参照

toio コア キューブの BLE 仕様: <https://toio.github.io/toio-spec/>

特に重要なページ:
- `docs/ble_motor` — モーター制御コマンド、速度値の意味、応答コード
- `docs/hardware_position_id` — マットごとの座標範囲 (簡易マットは 98,142〜402,358)
- `docs/hardware_other` — 最高速度 (直進 350 mm/s、回転 1500 °/s)

## コーディングルール

- コメントは「なぜ」を書く。何をしているかは識別子で表現する。
- `transform.py` の定数は spec / 設計決定由来。変更時は CLAUDE.md と README の表も更新。
- `client` 側に ROS 依存は持ち込まない (Raspberry Pi に pip だけで入れられる状態を維持する)。

## よくある作業

- マット種別を増やす: `transform.py` の定数と `reference_coordinates` の 4 点を新マットの
  `hardware_position_id` 値から書き換える。スケールは据え置きで OK。
- 速度感を変える: `client.yaml` の `toio.speed_max_value` と `toio_config.yaml` の `limits.linear`
  を一緒に動かす (1 速度値 ≈ 1 mm/s × scale)。
- nav graph を編集: `traffic_editor` で `maps/toio/toio.building.yaml` を開いて編集。

## 何をしないか

- `docs/`、`.github/`、`CONTRIBUTING.md`、Docker 関連ファイルは追加しない。
- マットの物理 cm をそのまま RMF の m として使わない (×50 スケールを通す)。
- cube ごとに別プロセスを立てない (BLE 競合する)。
