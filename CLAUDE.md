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

CI には ROS 2 ジョブ（`ros:humble-ros-base` / `ros:jazzy-ros-base` コンテナ、`--system-site-packages` の venv）があり、.db3 を rosbag2_py 経路で読むテストとバックエンド一致テストを回す。ローカルで再現するなら Ubuntu 24.04 に `ros-jazzy-rosbag2-py` 等を apt で入れ、`source /opt/ros/jazzy/setup.bash` して `python3 -m venv --system-site-packages` の venv で `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest`（ros-base の apt pytest と ROS の pytest プラグインがテスト収集を壊すため）。通常 CI も `mcap` extra を入れる（以前は入っておらず test_mcap.py が CI で常に skip されていた）

## コマンド一覧
eval / compare / sync / tf / export / anomaly / scenario / ask / scene / info / batch eval / batch anomaly

CLIオプション: `--version`, `--verbose`/`-v`, `--quiet`/`-q`

## アーキテクチャ
- `reader.py`: rosbag2_py → mcap → SQLite → ROS1(rosbags) の4段階読み込み
- 各コマンドは独立モジュール (eval.py, compare.py, etc.)
- eval は分割済み: `eval.py`（dataclass・evaluate_bag・GNSS/IMU/sync・表示・レコメ）/ `eval_domains.py`（ドメイン別レコメ・domain score）/ `eval_topics.py`（`_select_topics` 等のトピック選択）/ `eval_workflow.py`（action/service 結果集計）/ `eval_latency.py`（パイプライン遅延）/ `eval_findings.py`（structured findings）。`bagx.eval` から `_select_topics` / `_has_*_signature` / `WorkflowMetrics` は互換のため re-export
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
- **バックエンド間の一致**: rosbag2_py / mcap でデコードした dict も `_normalize_msg_data` で SQLite/CDR と同じ形（`stamp_sec`、NavSatFix の flat な `status`）に揃える。`HEAVY_MSG_TYPES`（PointCloud2 / Image / CompressedImage）はどのバックエンドでもメタデータのみの CDR パーサーで読む（ROS デコードで画素を Python list にしない）
- ROS 2 メッセージの dict 化は `get_fields_and_field_types()` を使う（Jazzy は `__slots__` に `_check_fields` があり、旧実装は全メッセージで例外→黙って CDR フォールバックしていた）
- `tests/test_backend_parity.py`（ROS がある時だけ実行）: fixture と rosbag2_py で書いた実 bag で ROS 経路と SQLite 経路の eval 結果が一致すること、全 CDR パーサーが rclpy のシリアライズと frame_id 長 0〜11 で一致することを検証

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
- sensor_msgs/msg/LaserScan (ranges は保持せず num_ranges / num_valid_ranges に集約)
- sensor_msgs/msg/CameraInfo
- sensor_msgs/msg/Image / CompressedImage (メタデータのみ、画素は data_size だけ)
- sensor_msgs/msg/JointState

新しいパーサーは `_CdrCursor`（アラインメントは payload 先頭基準、画像ペイロードはコピーしない、空シーケンスはパディングなし）。mcap decoder（r2b 実データ135件）と rosbags シリアライザ（両エンディアン fuzz 1200件）で一致を確認済み。

Header 先頭の型 (NavSatFix / Imu / PoseStamped / PointCloud2 / Header) は `frame_id` も返す（TF の frame 照合用）。

テスト用CDRビルダーは `tests/conftest.py` にあり、ペイロードアラインメント(`_payload_align`)に注意。

## IMU評価の仕組み
- **ノイズ推定**: `std(diff(x))/√2` — Allan Deviationのτ=dtと数学的に等価
- **静止データ自動検出**: accel magnitude ≈ 9.81 かつ std < 0.5 → Allan Variance計算
- **マルチIMU**: トピック別に評価、最良スコアを採用
- **EvalConfig**: スコアリング閾値をPython APIで設定可能
- **バイアス安定性**: 静止→Allan dev最小値、動的→windowed mean drift
- **周波数計算**: duration/count 方式 (バーストタイムスタンプ対応)

## フレームワーク自動検出
`eval_domains.py` の `_detect_domain_recommendations()` でトピック名/型からフレームワークを判定:

### Nav2 検出
- **トピックパターン**: `/odom`, `/scan`, `/cmd_vel`, `/map`, `/amcl_pose`, `/local_costmap` 等
- **型マッチ**: `LaserScan` 型の存在
- **検出条件**: 上記2つ以上 OR (LaserScan型 + `/odom`)
- **チェック項目**: odomレート(20Hz+), LaserScanレート(10Hz+), cmd_velレート
- **パイプライン遅延**: scan→costmap, odom→localization, scan→cmd_vel(フルループ)
- **検証データ**: Nav2 Gazebo dogfood capture (`scripts/run_ros_dogfood.py nav2-gazebo`)。TurtleBot3 Walker (okritvik/TurtleBot3_Walker) は配布元消失

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
- **検証データ**: MoveIt dogfood capture (`scripts/run_ros_dogfood.py moveit-demo`)。Franka Panda (adriankobras/panda) は配布元消失

### ドメイン間の干渉防止
- Nav2/Autoware/MoveIt検出時、SLAM固有のレコメ（「No IMU → KISS-ICP推奨」等）を抑制
- IMU noisy警告はドメインに応じてメッセージを変更（SLAM: "LiDAR-only推奨" / 他: "IMUキャリブ確認"）

## TF ツリー検証
`bagx/tf_tree.py`（`bagx tf` / eval に統合 / MCP `check_tf`）:
- `TfAccumulator` を eval の単一 read パスに流す（2回目の読み込みなし）。frame 収集済み/frameless のトピックは早期 return（オーバーヘッド対策）
- チェック: 複数 parent / cycle / 分断ツリー / static+dynamic 重複 / 動的 edge の gap・途中停止 / stamp 0・逆行 / sensor frame_id の欠落 / `--require-frames`
- edge ごとにストリーミング統計（count/first/last/max gap/stamp 0・逆行数）のみ保持し、タイムスタンプ列はメモリに持たない。通常間隔は duration/count（burst タイムスタンプ対応）
- gap は同一 publisher の edge を time_range でグループ化して1件にまとめる
- clock チェック: トピックごとに median(header.stamp − 記録時刻) を取り、1s 以内でクラスタリング。最大クラスタが基準、同数なら `tf.clock_split` を1件
- TF findings はスコアに影響しない（findings と `TF:` 付きレコメのみ）。`report.tf` にツリー要約
- 実データ所見: bag4 は GNSS/TF が GPS time-of-week、r2b_galileo は `*_optical` が2 parent、r2b_robotarm は arm TF が5.2s停止

## メッセージ内容チェック
`bagx/msg_checks.py`（eval の単一 read パスに統合、スコアに影響しない、`Data:` 付きレコメ）:
- LaserScan: 有効 range 比率 / beam 数変化 / 設定不正、CameraInfo: K 未設定 / サイズ0 / 同 namespace の Image とサイズ不一致、Image: data ≠ step×height / 解像度・encoding 変化、JointState: joint セット変化（複数 publisher）/ 長さ不一致 / NaN、空 frame_id
- CDR フォールバック dict（num_ranges / data_size）と decoded dict（ranges / data）の両方に対応。トピックごとの小さなカウンタのみ保持

## パイプライン遅延計測
`eval_latency.py` の `_add_pipeline_latency_recs()`:
- 出力トピックの各タイムスタンプに対し、`np.searchsorted` で直前の入力タイムスタンプを特定
- 差分 = 処理遅延 (median, P95)
- 閾値: <50ms ✔, 50-200ms ⚠ (may affect real-time), >200ms ❌ (too slow)
- **header stamp 照合**: 出力メッセージの header.stamp が入力の stamp と一致する（stamp 伝播）出力が50%以上あれば、記録時刻差でメッセージ単位の正確な遅延を出す（レコメに `(header-stamp matched)`）。eval の read パスで stamp を持つトピックの (記録時刻, stamp) を `report._topic_stamps` に保持
- 照合できない場合は従来の近似: 入力と出力が同レートで固定オフセットの場合にのみ正確。異なるレートの場合は近似（例: 50Hz odom → 45ms 遅れの amcl_pose を 5ms と過小評価する）
- SQLite フォールバックで stamp が取れるのは CDR パーサーのある型のみ（Autoware 独自型は ROS / mcap デコード時のみ）

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
8. NavSatFix CDR パーサーが status(int8) の前で4バイト境界に揃えていた → frame_id 長+1 が4の倍数以外（`gnss_link` など）で fix rate を誤判定（ROS バックエンドとの一致テストで発見）
9. ROS 2 Jazzy で全メッセージの dict 化が失敗し、独自型（Autoware 等）の中身が失われていた（`_check_fields` スロット）
10. pyarrow 26 は NumPy 2 必須（未宣言）→ NumPy 1.x の ROS 環境で `bagx export` が import 失敗。`pyarrow<26` に固定
11. sync 指標が名前順で先のトピックを走査していた → 200Hz IMU ↔ 10Hz LiDAR で LiDAR 周期の1/4（約25ms）を「遅延」と誤計測し、偽の deskew 警告（MID-360 sync 70 → 100、bag4 67.7 → 87.4）。疎な側を走査する `topic_filters.nearest_neighbor_delays_ns` に統一（scenario の sync_degraded も同じ）
12. `/imu/temperature` / `/imu/magnetic_field` が名前の `imu` で sync 候補になりカメラと組まれていた（NTU VIRAL）→ Temperature / MagneticField 等の型を除外

## 実データ検証済みデータセット

### SLAM
| データセット | Overall | IMU | Sync | 所見 |
|---|---|---|---|---|
| Livox MID-360 (AIST driving, Zenodo 14841855) | 98.7 | 97.3 | 100.0 | IMU最良、LiDAR↔IMU 1.6ms（旧 sync 70 / 25ms は指標バグ） |
| NTU VIRAL eee_03 (drone, DR-NTU) | 84.0 | 72.0 | 100.0 | 飛行中の accel ノイズ 0.30 m/s²、LO推奨 |
| Newer College (handheld) | — | — | — | 申請フォーム必須で未固定、スコア削除（旧 0.4.0 は 92.3） |
| Ouster OS0-32 (static) | — | — | — | 元 bag が公開ファイルに紐付かず `source_unavailable` |

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

### ROS1
| データセット | Overall | IMU | Sync | 所見 |
|---|---|---|---|---|
| TUM VI calib-imu1 (`dataset-calib-imu1_512_16.bag`) | 95.4 | 90.8 | 100.0 | IMU 199Hz ✔, cam↔IMU sync 1.3ms ✔ |
| Synthetic GNSS (rosbags fixture) | — | — | — | NavSatFix 100msg、split bag 集約テスト済み |
| Synthetic IMU (rosbags fixture) | — | — | — | Imu 200msg、eval パイプライン通過 |

EuRoC MH_01 は `robotics.ethz.ch` が不通の環境では ETH Research Collection から取得:
`bagx eval MH_01_easy.bag`（`pip install bagx[ros1]` 必須）

## リリース手順
1. `bagx/__init__.py` と `pyproject.toml` のバージョンを更新
2. `git commit` → `git push origin main`
3. `git tag vX.Y.Z` → `git push origin vX.Y.Z`
4. GitHub Actions (publish.yml) が自動でPyPIに公開 (Trusted Publisher)
5. `gh release create vX.Y.Z` でGitHub Release作成

## MCP サーバー (P5)
- `pip install bagx[mcp]` — `mcp` Python SDK（LLM キー不要）
- `bagx mcp` — stdio MCP server
- Tools: `eval_bag`, `list_topics`, `detect_anomalies`, `check_tf`, `compare_bags`, `query_messages`
- `mcp<2` に固定（2.x で `FastMCP` が `MCPServer` に改名され import が壊れ、テストも skip される）
- `query_messages` / `list_bag_topics` は `ask.py` のヘルパーを再利用
- 設定例: `claude mcp add bagx -- bagx mcp`（`docs/guide/mcp.md`）

## スコアボード (P6)
- `benchmarks/scoreboard.json` — 30データセットの manifest（スコア・再現コマンド・bag fingerprint 付き）
  - `provenance`: `public`（配布元からDL）/ `simulator`（dogfood収録）。dogfood キャッシュへの fallback は `simulator` のみ
  - `availability: source_unavailable` の行はスコアを持たない（配布元消失）
  - `scripts/generate_scoreboard.py --check` で整合性検証（同一bagで採点された行の重複検出など、テストでも実行）
  - 公開行に dogfood bag を symlink で割り当てない（過去に TurtleBot3/Panda 行が sim の結果の複製になっていた）
- `scripts/generate_scoreboard.py` — manifest → `docs/scoreboard.md` 自動生成
- `--refresh --write-manifest` でローカル bag からスコア更新（`BAGX_SCOREBOARD_BAGS`）。タグ外のコードで採点した行は `bagx_version` が `X.Y.Z+dev`
- `.db3.zstd` 展開先: `BAGX_DB3_CACHE`（`/tmp` 満杯時は `.cache/db3-decompress` 等を指定）
- `scripts/fetch_scoreboard_bags.sh` — Autoware S3 + AutoCore + NVIDIA r2b (NGC) を一括取得
- 分析記事: `docs/scoreboard/why-*.md`（低スコアの理由解説）

## ドキュメント
- `mkdocs.yml` + `docs/` → GitHub Pages (`mkdocs gh-deploy`)
- `docs.yml` ワークフローでmainにpush時に自動デプロイ
- `scripts/generate_docs.py --update` でREADMEのCLI Referenceを更新
- `scripts/generate_scoreboard.py` でスコアボード表を更新
