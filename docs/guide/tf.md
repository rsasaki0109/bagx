# tf — TF Tree Validation

Rebuild the TF tree from `/tf` and `/tf_static` and check that it can actually be used
for replay: one parent per frame, no cycles, one connected tree, no pauses, and header
stamps on the same clock as the sensors.

## Usage

```bash
bagx tf recording.db3
bagx tf recording.db3 --json tf.json
bagx tf recording.db3 --require-frames map,odom,base_link --fail-on error   # CI gate
```

`bagx eval` runs the same checks in its single read pass. TF findings appear in
`findings` (category `tf_tree`), warnings and errors are listed under Recommendations
with a `TF:` prefix, and the tree summary is saved in the `tf` key of the JSON report.
TF findings do **not** change the readiness score.

## Checks

| Finding id | Severity | Meaning |
|------------|----------|---------|
| `tf.multiple_parents.<frame>` | error | A frame is published with two different parents. tf2 keeps one, so lookups flip between branches |
| `tf.cycle` | error | The parent → child graph contains a loop |
| `tf.disconnected` | warning | The frames form more than one tree, so frames in different trees cannot be looked up against each other |
| `tf.static_dynamic_conflict.<parent>.<child>` | warning | The same transform is on both `/tf` and `/tf_static` |
| `tf.edge_gap.<parent>.<child>` | warning | A dynamic transform paused longer than max(0.5 s, 5× its normal interval). Edges that pause together (one publisher) are grouped into one finding with a `time_range` |
| `tf.edge_stopped.<parent>.<child>` | warning | A dynamic transform stops ≥ 2 s before the bag ends |
| `tf.zero_stamp.<parent>.<child>` | warning | Dynamic transforms with `header.stamp == 0` |
| `tf.stamp_regression.<parent>.<child>` | warning | Header stamps go backwards (two publishers or a clock reset) |
| `tf.clock_mismatch.<topic>` | warning | One topic's header stamps use a different clock from the rest (> 1 s off) |
| `tf.clock_split` | warning | Stamped topics split into equally sized groups on different clocks |
| `tf.frame_missing.<frame>` | warning | A sensor's `header.frame_id` never appears in the TF tree |
| `tf.required_frame_missing.<frame>` | error | `--require-frames` names a frame that is not in the bag |
| `tf.required_chain_broken.<a>.<b>` | error | Two consecutive `--require-frames` entries are in different trees |
| `tf.not_recorded` | info / error | No TF at all, while ≥ 2 sensor frames exist (error when `--require-frames` is set) |

### How the clock check works

For every stamped topic (each TF topic and each sensor topic), bagx takes the median of
`header.stamp − record time`. Topics whose offsets are within 1 s of each other are
grouped. The largest group is treated as the reference clock, and every other topic is
reported. This catches drivers that stamp with device time, GNSS/INS units that stamp with
GPS time-of-week, and `use_sim_time` mismatches.

## Real-data examples

| Bag | What bagx found |
|-----|-----------------|
| Autoware all-sensors-bag4 | `ned → POS_REF` is a separate tree, and `/gnss/fix` + `/tf` are stamped ~478,519 s (GPS time-of-week) while IMU/LiDAR use Unix time |
| NVIDIA r2b_galileo / whitetunnel | Every `*_optical` camera frame has two parents (static from the camera link, dynamic from the stereo node); `/front_stereo_imu/imu` stamps ~40 s off; `segway_imu` is not in the tree |
| NVIDIA r2b_robotarm | All six arm joint transforms pause together for 5.2 s; `/tf` stamps are 3 s behind the camera stamps |

## Python API

```python
from bagx.tf_tree import analyze_tf

report = analyze_tf("recording.db3", required_frames=["map", "odom", "base_link"])
print(report.roots, len(report.edges))
for finding in report.findings:
    print(finding.severity, finding.id, finding.title)
```
