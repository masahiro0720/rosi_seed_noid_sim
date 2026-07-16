# rosi_seed_noid_sim

SEED-NoidのHRI機能を、実機driverを起動せずGazebo上で再現するためのROS Noetic
catkin packageです。

このrepositoryは、上流の
[`seed_r7_ros_pkg`](https://github.com/seed-solutions/seed_r7_ros_pkg)を直接変更せず、
simulation固有のworld、launch、認識・位置推定・動作backendをoverlayとして提供します。

## 対象環境

- Ubuntu 20.04
- ROS Noetic
- Gazebo 11
- `seed_r7_ros_pkg` type-F model

## 現在の実装範囲

Sim Phase S0として、次を提供します。

- 正面3 mにGazebo人物actorを配置したworld
- 上流`seed_r7_gazebo/seed_r7_empty_world.launch`を利用する専用launch
- catkin package metadata
- repository layout test

Sim Phase S1として、次を追加しました。

- `/camera/image_raw`を入力とするOpenCV HOG人物検出node
- `/start`、`/stop`、`/detection`と`/judge_param`の既存HRI契約
- 連続フレーム確認と検出枠付きdebug画像

人物同定、Approach、Touch、Leaveのより実機に近いbackendは、後続phaseで
このpackageへ追加します。

Sim Phase S2として、次を追加しました。

- HOG確定時の人物ROI topic
- `/camera/points`から人物領域の三次元点を抽出するDepth localization node
- Gazebo固有の平坦化PointCloud（307200×1）への画素index変換
- `camera_optical_frame`から`base_link`へのTF変換
- 固定位置ではなく実測した位置を返す`/get_position`サービス

Sim Phase S3として、次を追加しました。

- 上流`seed_r7_gazebo/seed_r7_thk_world.launch`を再利用する家具付き研究室world
- 机・椅子・棚の配置を`/scan`で認識するrolling costmap
- GlobalPlannerとTEB Local Plannerを使用する`move_base`
- 目標姿勢を受け取り、経路計画・旋回・走行・到達判定を行うNavigation HRI backend
- 失敗時に長時間迷走しない有限timeoutとrecovery無効化
- Navigation後の現在odomを毎回開始点として取り直すMove HRI backend

## 起動

```bash
roslaunch rosi_seed_noid_sim seed_noid_person_world.launch \
  robot_model:=typef GUI:=true
```

別terminalで人物検出nodeを起動します。

```bash
rosrun rosi_seed_noid_sim person_detection_hog.py \
  _detection_rate:=5.0 _required_consecutive:=3
```

nodeは`/start`が呼ばれるまで画像を判定せず、人物を指定フレーム数連続して検出した
場合だけ`/judge_param: true`をpublishします。検出結果は
`/person_detection_hog/debug_image`で確認できます。

Depth localization nodeは次で起動します。

```bash
rosrun rosi_seed_noid_sim person_localization_depth.py \
  _target_frame:=base_link
```

位置は`base_link`基準のメートル単位`[x, y, z]`で返し、
`/rosi_seed_noid_sim/person_position`にも`geometry_msgs/PointStamped`としてpublishします。

このlaunchは実機用`seed_r7_bringup`を起動しません。

家具付きworldと自律Navigationは次で起動します。

```bash
roslaunch rosi_seed_noid_sim seed_noid_furnished_navigation.launch \
  robot_model:=typef GUI:=true
rosrun rosi_seed_noid_sim navigation_hri_sim.py \
  _frame_id:=odom _navigation_timeout:=75
rosrun rosi_seed_noid_sim move_hri_sim.py _move_timeout:=20
```

標準配置はロボット `(6, 2)`（`+X`向き）、人物 `(12, 0)`（`-X`向き）です。
両者を約6.3 m離し、互いに異なる方向へ向けています。人物検出・Depth位置推定の後、
人物正面2 mの`(10, 0)`をNavigation目標にします。Navigationへ
`[x, y, z, qx, qy, qz, qw]`を渡すと、`/scan`とcostmapで机を考慮した経路を作り、
`/SEED_Noid/completed_command`へ到達結果を返します。その後の1 m Moveで人物の
約1 m手前へ進みます。

## 設計境界

HRIコンポーネントは実機・simulationで共通とし、このpackageが実機driverと同じROS
service、topic、actionを提供します。

| HRI境界 | simulation backend予定 |
|---|---|
| `/detection`, `/start`, `/stop`, `/judge_param` | Gazebo RGB画像の人物検出 |
| `/get_position` | RGB-D画像からの人物三次元位置推定 |
| `/seed_robot_action` | Gazebo台車・腕controller |
| `move_base` | Gazebo odometry、LaserScan、rolling costmapによるNavigation |

音声認識、言語理解、健康リスク判定、音声再生は`health_judge` RTCを使用し、この
repositoryには複製しません。

## 再現性

RoSIのsimulation Robot YAMLからこのrepositoryを`collect.git`で取得し、
`dependencies/dependency-lock.yaml`に固定したcommit SHAへcheckoutします。

branchの先端ではなくcommit SHAを使うことで、別workspaceでも同じsourceを取得します。

## テスト

```bash
python3 test/test_repository_layout.py
catkin build rosi_seed_noid_sim
```
