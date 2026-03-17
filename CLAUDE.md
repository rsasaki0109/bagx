# bagx 開発ガイド

## プロジェクト概要
bagxはROS2 rosbagのポスト処理CLIツール。記録・再生(ros2 bag)ではなく、評価・比較・解析を行う。

## ビルド & テスト
```bash
pip install -e ".[dev]"       # 開発インストール
pip install -e ".[dev,mcap]"  # mcapサポート込み
pip install -e ".[dev,llm]"   # LLM(ask)サポート込み
python3 -m pytest tests/ -v   # テスト実行
ruff check bagx/              # lint
```

## コマンド一覧
eval / compare / sync / export / anomaly / scenario / ask / scene / info / batch

## アーキテクチャ
- `reader.py`: rosbag2_py → mcap → SQLiteフォールバック の3段階読み込み
- 各コマンドは独立モジュール (eval.py, compare.py, etc.)
- CLIは `cli.py` (typer) に集約、各モジュールは遅延import
- テスト用のダミー.db3は `tests/conftest.py` でSQLiteから直接生成

## 設計原則
- CLI-first、GUIなし
- ros2 bagと機能を被らせない (record/play/echoは作らない)
- ROS2なしでも.db3の基本解析は動作する (SQLiteフォールバック + CDRパーサー)
- 出力は数値・JSON・Parquet・CSV

## CDRパーサー
`reader.py` にNavSatFix/Imu等の主要メッセージ型のCDRバイナリパーサーがある。
テスト用CDRビルダーは `tests/conftest.py` にあり、ペイロードアラインメントに注意。
