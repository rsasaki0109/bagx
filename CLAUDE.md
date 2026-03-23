# bagx 開発ガイド

## プロジェクト概要
bagxはROS2 rosbagのポスト処理CLIツール。記録・再生(ros2 bag)ではなく、評価・比較・解析を行う。
SLAM / Nav2 / Autoware / MoveIt のフレームワーク自動検出とドメイン別レコメンデーション機能を持つ。

## ビルド & テスト
```bash
pip install -e ".[dev]"       # 開発インストール
pip install -e ".[dev,mcap]"  # mcapサポート込み
pip install -e ".[dev,llm]"   # LLM(ask)サポート込み
pip install -e ".[docs]"      # ドキュメントビルド
python3 -m pytest tests/ -v   # テスト実行
python3 -m pytest tests/ --cov=bagx --cov-report=term-missing  # カバレッジ付き
ruff check bagx/              # lint
ruff check .                  # テスト含むlint (CIと同じ)
```

## コマンド一覧
eval / compare / sync / export / anomaly / scenario / ask / scene / info / batch eval / batch anomaly

CLIオプション: `--version`, `--verbose`/`-v`, `--quiet`/`-q`

## アーキテクチャ
- `reader.py`: rosbag2_py → mcap → SQLiteフォールバック の3段階読み込み
- 各コマンドは独立モジュール (eval.py, compare.py, etc.)
- CLIは `cli.py` (typer) に集約、各モジュールは遅延import
- テスト用のダミー.db3は `tests/conftest.py` でSQLiteから直接生成
- `logging` を全モジュールで使用。デシリアライズ失敗時にWARNING出力

## 設計原則
- CLI-first、GUIなし
- ros2 bagと機能を被らせない (record/play/echoは作らない)
- ROS2なしでも.db3の基本解析は動作する (SQLiteフォールバック + CDRパーサー)
- 出力は数値・JSON・Parquet・CSV
- メモリ効率: PointCloud2等の重いメッセージはメモリに保持しない (timestampのみ記録)

## reader.py の仕組み
- **マルチセグメント対応**: ディレクトリ内の複数.db3/.mcapを集約して読む
- **CDRエンディアン対応**: encapsulationヘッダーからbig/littleを判定
- **mcapフォールバック**: デコーダ失敗時にCDR basic parserへフォールバック

## CDRパーサー
`reader.py` に以下のメッセージ型のCDRバイナリパーサーがある:
- sensor_msgs/msg/NavSatFix
- sensor_msgs/msg/Imu
- sensor_msgs/msg/PointCloud2 (メタデータのみ)
- geometry_msgs/msg/PoseStamped
- geometry_msgs/msg/PoseWithCovarianceStamped
- nav_msgs/msg/Odometry
- geometry_msgs/msg/TwistStamped
- tf2_msgs/msg/TFMessage
- std_msgs/msg/Header

テスト用CDRビルダーは `tests/conftest.py` にあり、ペイロードアラインメント(`_payload_align`)に注意。

## IMU評価の仕組み
- **ノイズ推定**: `std(diff(x))/√2` — Allan Deviationのτ=dtと数学的に等価
- **静止データ自動検出**: accel magnitude ≈ 9.81 かつ std < 0.5 → Allan Variance計算
- **マルチIMU**: トピック別に評価、最良スコアを採用
- **EvalConfig**: スコアリング閾値をPython APIで設定可能
- **バイアス安定性**: 静止→Allan dev最小値、動的→windowed mean drift
- **周波数計算**: duration/count 方式 (バーストタイムスタンプ対応)

## フレームワーク自動検出
`eval.py` の `_detect_domain_recommendations()` でトピック名/型からフレームワークを判定:

### Nav2 検出
- **トピックパターン**: `/odom`, `/scan`, `/cmd_vel`, `/map`, `/amcl_pose`, `/local_costmap` 等
- **型マッチ**: `LaserScan` 型の存在
- **検出条件**: 上記2つ以上 OR (LaserScan型 + `/odom`)
- **チェック項目**: odomレート(20Hz+), LaserScanレート(10Hz+), cmd_velレート
- **パイプライン遅延**: scan→costmap, odom→localization, scan→cmd_vel(フルループ)
- **検証データ**: TurtleBot3 Walker bag (okritvik/TurtleBot3_Walker)

### Autoware 検出
- **トピックパターン**: `/sensing/*`, `/perception/*`, `/planning/*`, `/control/*`, `/localization/*`, `/vehicle/*`
- **検出条件**: 上記プレフィックスのトピック1つ以上
- **チェック項目**: カメラFPS(15Hz+), LiDARレート, GNSSの有無
- **パイプライン遅延**: sensing→perception, perception→planning, planning→control, end-to-end
- **検証データ**: AutoCore.ai Ouster OS1-64 dataset

### MoveIt 検出
- **トピックパターン**: `/joint_states`, `*joint_states*`, `/display_planned_path`, `/planning_scene`
- **型マッチ**: `JointState` 型の存在
- **検出条件**: 上記2つ以上 OR (JointState型 + 名前マッチ)
- **チェック項目**: JointStateレート(100Hz+), レートの異常値ガード(>100kHz除外)
- **パイプライン遅延**: joint_states→planned_path
- **検証データ**: Franka Panda bag (adriankobras/panda)

### ドメイン間の干渉防止
- Nav2/Autoware/MoveIt検出時、SLAM固有のレコメ（「No IMU → KISS-ICP推奨」等）を抑制
- IMU noisy警告はドメインに応じてメッセージを変更（SLAM: "LiDAR-only推奨" / 他: "IMUキャリブ確認"）

## パイプライン遅延計測
`eval.py` の `_add_pipeline_latency_recs()`:
- 出力トピックの各タイムスタンプに対し、`np.searchsorted` で直前の入力タイムスタンプを特定
- 差分 = 処理遅延 (median, P95)
- 閾値: <50ms ✔, 50-200ms ⚠ (may affect real-time), >200ms ❌ (too slow)
- 注意: 入力と出力が同レートで固定オフセットの場合にのみ正確。異なるレートの場合は近似

## anomaly検出の注意点
- **IMUトピックは専用検出器**でカバー。`rate_gap`からは除外（二重検出防止）
- **最低gap閾値**: 50ms未満のgapは報告しない（高レートトピックのジッター誤検出防止）
- **median interval < 100μs**: バーストタイムスタンプとして検出をスキップ
- **メタトピック除外**: `/rosout`, `/parameter_events`, `/tf_static` 等はscenario検出から除外

## dogfoodingで発見した主なバグ
1. IMU周波数 28306Hz → 388Hz (median → duration/count に変更)
2. anomaly 31,972件 → 640件 (IMU二重検出 + 最低gap閾値)
3. scenario /rosout偽sync_degraded (メタトピック除外)
4. 空ディレクトリ/壊れたファイルでtraceback (RuntimeErrorキャッチ追加)
5. パイプライン遅延0ms (sliding pointer → binary search)
6. Nav2 bagに「KISS-ICP推奨」(ドメイン検出時SLAM advice抑制)
7. MoveIt `/fr3_gripper/joint_states`未検出 (パターン拡張)

## 実データ検証済みデータセット

### SLAM
| データセット | Overall | IMU | Sync | 所見 |
|---|---|---|---|---|
| Newer College (handheld) | 92.3 | 84.5 | 100.0 | 最高品質、SLAM benchmarkに最適 |
| Livox MID-360 | 83.7 | 97.3 | 70.0 | IMU最良、sync 25ms要deskew |
| NTU VIRAL (drone) | 79.8 | 59.5 | 100.0 | IMUノイジー、LO推奨 |
| Ouster OS0-32 (static) | 76.6 | 79.6 | 73.6 | 50Hz IMU遅い、Allan Variance自動計算 |

### Nav2
| データセット | 検出 | 所見 |
|---|---|---|
| TurtleBot3 Walker | ✅ | odom 29Hz ✔, scan 5Hz ⚠, pipeline 1ms ✔ |

### Autoware
| データセット | 検出 | 所見 |
|---|---|---|
| AutoCore OS1-64 | ✅ | LiDAR 10Hz ✔ |

### MoveIt
| データセット | 検出 | 所見 |
|---|---|---|
| Franka Panda | ✅ | JointState検出、Duration 0sのbagはレートガードで対応 |

## リリース手順
1. `bagx/__init__.py` と `pyproject.toml` のバージョンを更新
2. `git commit` → `git push origin main`
3. `git tag vX.Y.Z` → `git push origin vX.Y.Z`
4. GitHub Actions (publish.yml) が自動でPyPIに公開 (Trusted Publisher)
5. `gh release create vX.Y.Z` でGitHub Release作成

## ドキュメント
- `mkdocs.yml` + `docs/` → GitHub Pages (`mkdocs gh-deploy`)
- `docs.yml` ワークフローでmainにpush時に自動デプロイ
- `scripts/generate_docs.py --update` でREADMEのCLI Referenceを更新
