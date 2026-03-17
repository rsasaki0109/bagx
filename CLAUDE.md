# bagx 開発ガイド

## プロジェクト概要
bagxはROS2 rosbagのポスト処理CLIツール。記録・再生(ros2 bag)ではなく、評価・比較・解析を行う。

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
