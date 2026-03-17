# bagx

rosbagデータの評価・比較・同期解析・AI向け変換を行うポスト処理CLIツール。

> **bagxはros2 bagの代替ではない。**
> ros2 bagがデータの記録・再生（I/O）を担うのに対し、bagxはデータの評価・理解・比較（Analysis）を行う。

## インストール

```bash
pip install -e .
```

ROS2環境がある場合は `rosbag2_py` を利用した完全なメッセージデシリアライズが可能。
ROS2がなくても `.db3` ファイルの基本的な解析は動作する（SQLiteフォールバック）。

## コマンド

### `bagx info` — Bag情報表示

```bash
bagx info recording.db3
```

### `bagx eval` — 品質評価

単一bagの品質を自動評価し、スコアを出力。

```bash
bagx eval recording.db3
bagx eval recording.db3 --json report.json
```

評価項目:
- **GNSS**: Fix率、HDOP統計
- **IMU**: 加速度/ジャイロノイズ、バイアス安定性、周波数
- **SYNC**: topic間の平均/最大遅延
- **総合スコア**: 0-100

### `bagx compare` — 2つのbagを比較

```bash
bagx compare A.db3 B.db3
bagx compare A.db3 B.db3 --json diff.json
```

出力:
- 各指標のA/B値と差分
- improved / degraded / unchanged 判定
- 総合的にどちらが良いか

### `bagx sync` — topic間の時間同期解析

```bash
bagx sync recording.db3 /gnss /lidar
bagx sync recording.db3 /imu /camera --json sync.json
```

出力:
- 平均遅延、最大遅延、中央値、P95
- 標準偏差、外れ値率

### `bagx export` — AI/解析用フォーマットに変換

```bash
# Parquetにエクスポート（デフォルト）
bagx export recording.db3

# AI向けモード（相対タイムスタンプ、正規化）
bagx export recording.db3 --ai --format parquet

# JSON出力、特定topicのみ
bagx export recording.db3 --format json --topics /gnss,/imu

# フラット化なし
bagx export recording.db3 --no-flatten
```

### `bagx anomaly` — 異常検知

センサーデータの異常・外れ値を自動検出。

```bash
bagx anomaly recording.db3
bagx anomaly recording.db3 --topic /gnss
bagx anomaly recording.db3 --json anomalies.json
```

検出対象:
- **GNSS**: 位置ジャンプ、Fix状態の突然の喪失、HDOPスパイク
- **IMU**: 加速度/ジャイロのスパイク（N*σ超過）、周波数低下
- **全般**: メッセージレート異常（3x中央値超のギャップ）

### `bagx scenario` — 危険シーン抽出

「興味深い」または「危険な」シナリオの時間区間を自動抽出。

```bash
bagx scenario recording.db3
bagx scenario recording.db3 --json scenarios.json
```

検出ルール:
- **GNSS消失**: 連続no-fixが閾値期間を超過
- **センサー脱落**: topicが閾値期間以上停止
- **高ダイナミクス**: IMU加速度の大きさが閾値を超過（急ブレーキ、急旋回）
- **同期劣化**: topic間遅延が持続的に閾値超過

## 設計方針

- **CLI-first**: コマンドを主役とする設計
- **再現性・バッチ処理**: スクリプトやCI/CDに組み込み可能
- **出力は数値・JSON・Parquet**: GUIではなくデータで語る
- **モジュール構造**: eval / compare / sync / export が独立

## プロジェクト構成

```
bagx/
  __init__.py
  cli.py        # typerベースのCLIエントリポイント
  reader.py     # rosbag読み込み抽象化（rosbag2_py / SQLiteフォールバック）
  eval.py       # 品質評価エンジン
  compare.py    # 2bag比較
  sync.py       # topic間同期解析
  export.py     # Parquet / JSON エクスポート
  anomaly.py    # 異常検知
  scenario.py   # 危険シーン抽出
  schema.py     # スキーマ推定・正規化
```

## 技術スタック

- **typer** + **rich**: CLI & 表示
- **rosbag2_py**: ROS2 bag読み込み（オプション）
- **numpy / pandas**: 数値解析
- **pyarrow**: Parquet出力

<!-- CLI_REFERENCE_START -->

## CLI Reference

```
Usage: bagx [OPTIONS] COMMAND [ARGS]...                                        
                                                                                
 Post-processing analysis engine for ROS2 rosbag data.                          
                                                                                
╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --install-completion          Install completion for the current shell.      │
│ --show-completion             Show completion for the current shell, to copy │
│                               it or customize the installation.              │
│ --help                        Show this message and exit.                    │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Commands ───────────────────────────────────────────────────────────────────╮
│ eval       Evaluate quality of a single bag file.                            │
│ compare    Compare quality metrics of two bag files.                         │
│ sync       Analyze time synchronization between two topics.                  │
│ export     Export bag data to AI/analytics-friendly formats.                 │
│ anomaly    Detect anomalies and outliers in sensor data.                     │
│ scenario   Identify and extract dangerous or interesting scenarios.          │
│ ask        Ask a natural language question about a bag file, answered by an  │
│            LLM.                                                              │
│ scene      Extract 3D state (position, orientation, velocity) time series.   │
│ info       Show bag file summary information.                                │
│ batch      Batch operations on multiple bags                                 │
╰──────────────────────────────────────────────────────────────────────────────╯
```

### `bagx eval`

```
Usage: bagx eval [OPTIONS] BAG                                                 
                                                                                
 Evaluate quality of a single bag file.                                         
                                                                                
 Analyzes GNSS fix rate/HDOP, IMU noise/bias, and inter-topic sync,             
 producing a composite quality score.                                           
                                                                                
╭─ Arguments ──────────────────────────────────────────────────────────────────╮
│ *    bag      TEXT  Path to the bag file (.db3 or directory) [required]      │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --json  -j      TEXT  Output JSON report to file                             │
│ --help                Show this message and exit.                            │
╰──────────────────────────────────────────────────────────────────────────────╯
```

### `bagx compare`

```
Usage: bagx compare [OPTIONS] BAG_A BAG_B                                      
                                                                                
 Compare quality metrics of two bag files.                                      
                                                                                
 Evaluates both bags and reports per-metric differences,                        
 indicating which bag is better overall.                                        
                                                                                
╭─ Arguments ──────────────────────────────────────────────────────────────────╮
│ *    bag_a      TEXT  Path to the first bag file [required]                  │
│ *    bag_b      TEXT  Path to the second bag file [required]                 │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --json  -j      TEXT  Output JSON report to file                             │
│ --help                Show this message and exit.                            │
╰──────────────────────────────────────────────────────────────────────────────╯
```

### `bagx sync`

```
Usage: bagx sync [OPTIONS] BAG TOPIC_A TOPIC_B                                 
                                                                                
 Analyze time synchronization between two topics.                               
                                                                                
 Reports mean/max/median delay, variance, P95, and outlier rate.                
                                                                                
╭─ Arguments ──────────────────────────────────────────────────────────────────╮
│ *    bag          TEXT  Path to the bag file [required]                      │
│ *    topic_a      TEXT  First topic name [required]                          │
│ *    topic_b      TEXT  Second topic name [required]                         │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --json  -j      TEXT  Output JSON report to file                             │
│ --help                Show this message and exit.                            │
╰──────────────────────────────────────────────────────────────────────────────╯
```

### `bagx export`

```
Usage: bagx export [OPTIONS] BAG                                               
                                                                                
 Export bag data to AI/analytics-friendly formats.                              
                                                                                
 Outputs one file per topic in JSON or Parquet format,                          
 with optional timestamp normalization and field flattening.                    
                                                                                
╭─ Arguments ──────────────────────────────────────────────────────────────────╮
│ *    bag      TEXT  Path to the bag file [required]                          │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --output   -o                  TEXT  Output directory [default: ./export]    │
│ --format   -f                  TEXT  Output format: parquet or json          │
│                                      [default: parquet]                      │
│ --topics   -t                  TEXT  Comma-separated topic names (default:   │
│                                      all)                                    │
│ --ai                                 Enable AI-friendly mode (relative       │
│                                      timestamps, normalized)                 │
│ --flatten      --no-flatten          Flatten nested message fields           │
│                                      [default: flatten]                      │
│ --help                               Show this message and exit.             │
╰──────────────────────────────────────────────────────────────────────────────╯
```

### `bagx anomaly`

```
Usage: bagx anomaly [OPTIONS] BAG                                              
                                                                                
 Detect anomalies and outliers in sensor data.                                  
                                                                                
 Finds GNSS position jumps, IMU spikes, message rate gaps,                      
 and other anomalous events in the bag file.                                    
                                                                                
╭─ Arguments ──────────────────────────────────────────────────────────────────╮
│ *    bag      TEXT  Path to the bag file [required]                          │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --topic  -t      TEXT  Analyze only this topic                               │
│ --json   -j      TEXT  Output JSON report to file                            │
│ --help                 Show this message and exit.                           │
╰──────────────────────────────────────────────────────────────────────────────╯
```

### `bagx scenario`

```
Usage: bagx scenario [OPTIONS] BAG                                             
                                                                                
 Identify and extract dangerous or interesting scenarios.                       
                                                                                
 Detects GNSS loss, sensor dropouts, high dynamics events,                      
 and sync degradation periods.                                                  
                                                                                
╭─ Arguments ──────────────────────────────────────────────────────────────────╮
│ *    bag      TEXT  Path to the bag file [required]                          │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --json  -j      TEXT  Output JSON report to file                             │
│ --help                Show this message and exit.                            │
╰──────────────────────────────────────────────────────────────────────────────╯
```

### `bagx ask`

```
Usage: bagx ask [OPTIONS] BAG QUESTION                                         
                                                                                
 Ask a natural language question about a bag file, answered by an LLM.          
                                                                                
 Gathers bag context (summary, eval, message samples) and sends it              
 along with your question to an LLM for analysis.                               
                                                                                
╭─ Arguments ──────────────────────────────────────────────────────────────────╮
│ *    bag           TEXT  Path to the bag file (.db3 or directory) [required] │
│ *    question      TEXT  Natural language question about the bag [required]  │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --provider  -p      TEXT  LLM provider: 'anthropic' or 'openai'              │
│                           [default: anthropic]                               │
│ --help                    Show this message and exit.                        │
╰──────────────────────────────────────────────────────────────────────────────╯
```

### `bagx scene`

```
Usage: bagx scene [OPTIONS] BAG                                                
                                                                                
 Extract 3D state (position, orientation, velocity) time series.                
                                                                                
 Auto-detects topics with scene-relevant message types (PoseStamped,            
 Odometry, Imu, NavSatFix, TFMessage) unless specific topics are given.         
                                                                                
╭─ Arguments ──────────────────────────────────────────────────────────────────╮
│ *    bag      TEXT  Path to the bag file [required]                          │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --csv     -c      TEXT  Export scene states to CSV file                      │
│ --json    -j      TEXT  Output JSON report to file                           │
│ --topics  -t      TEXT  Comma-separated topic names (default: auto-detect)   │
│ --help                  Show this message and exit.                          │
╰──────────────────────────────────────────────────────────────────────────────╯
```

### `bagx info`

```
Usage: bagx info [OPTIONS] BAG                                                 
                                                                                
 Show bag file summary information.                                             
                                                                                
╭─ Arguments ──────────────────────────────────────────────────────────────────╮
│ *    bag      TEXT  Path to the bag file [required]                          │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --help          Show this message and exit.                                  │
╰──────────────────────────────────────────────────────────────────────────────╯
```

### `bagx batch`

```
Usage: bagx batch [OPTIONS] COMMAND [ARGS]...                                  
                                                                                
 Batch operations on multiple bags                                              
                                                                                
╭─ Options ────────────────────────────────────────────────────────────────────╮
│ --help          Show this message and exit.                                  │
╰──────────────────────────────────────────────────────────────────────────────╯
╭─ Commands ───────────────────────────────────────────────────────────────────╮
│ eval      Evaluate quality of multiple bag files.                            │
│ anomaly   Run anomaly detection on multiple bag files.                       │
╰──────────────────────────────────────────────────────────────────────────────╯
```


<!-- CLI_REFERENCE_END -->
