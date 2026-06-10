# tjark AGV Dynamic Obstacle Avoidance Demo

## User Manual / 使用手冊

本文件為本專案的繁體中文使用手冊與技術說明文件。

本專案由兩個 ROS package 組成，其中 `tjark_agv` 為外部取得的 AGV 模型與描述套件，`tjark_nav` 為本專案新增的導航、建圖、動態避障與展示流程。

請在正式提交或分享前，將下方來源網址替換為實際下載來源：

```text
tjark_agv download from: <這地方給我自己填網址>
```

套件來源說明：

| Package | 來源 | 說明 |
| --- | --- | --- |
| `tjark_agv` | 外部下載，網址請自行填入 | AGV 的 URDF/xacro、mesh、Gazebo plugin 與基礎模型 |
| `tjark_nav` | 本專案建立 | 建圖、定位、路徑規劃、動態避障、行人模擬與 demo launch |

---

本專案是一個以 ROS Noetic + Gazebo 11 建立的自走車導航示範系統。核心目標是讓一台差速驅動 AGV 在已知地圖中自主定位、規劃路線，並在路線上遇到多名動態行人時即時避障，最後抵達指定目標點。

整個系統包含兩個 ROS package：

| Package | 內容 |
| --- | --- |
| `tjark_agv` | 外部下載的 AGV 模型套件，包含 URDF/xacro、STL mesh、Gazebo 感測器與差速驅動 plugin |
| `tjark_nav` | 本專案新增的地圖、場景、AMCL、move_base、DWA、行人模擬、建圖與導航腳本 |

本 README 會從專案背景、系統架構、感測器、建圖、定位、導航、動態避障、行人模擬、執行方式與調參方向完整說明。

---

## 專案背景

倉儲、工廠、醫院與服務型場域中的移動機器人不只需要在靜態地圖中移動，還必須能夠和人共用空間。真實場景中，人員位置無法事先寫進地圖，也不會固定站在同一個地方。因此，一台實用的 AGV 必須同時具備以下能力：

1. **知道自己在哪裡**：在已建立的地圖中估測目前姿態。
2. **知道要去哪裡**：從起點到終點產生全域路徑。
3. **知道前方有什麼**：使用感測器即時觀察周圍障礙物。
4. **遇到動態障礙能反應**：例如行人突然走進路徑時，AGV 要能減速、繞行或從人群間穿過。
5. **不依賴手動輸入座標**：透過建圖和視覺標記取得起點與終點，讓流程更接近真實部署。

本專案用 Gazebo 模擬一個 26 m x 16 m 的避障場景。AGV 從左下角出發，依序通過黃色中繼點，最後抵達右上角紅色終點。路線中會遇到兩波行人：

- 第一波行人在起點後方通道附近分散移動。
- 第二波行人沿著往第三個目標點方向斜向分布，彼此距離較大，用來展示 AGV 在人群縫隙中穿行的效果。

---

## 系統需求

建議環境：

| 項目 | 版本 |
| --- | --- |
| Ubuntu | 20.04 LTS |
| ROS | Noetic |
| Gazebo | 11 |
| Python | 3.8 |
| Build system | catkin |

主要 ROS 套件：

- `gazebo_ros`
- `robot_state_publisher`
- `joint_state_publisher`
- `map_server`
- `amcl`
- `move_base`
- `navfn`
- `dwa_local_planner`
- `rtabmap_ros` / `rtabmap_slam`
- `cv_bridge`
- `teleop_twist_keyboard`

---

## 快速開始

第一次使用建議直接執行：

```bash
cd /home/phill/proj
./run_demo.sh setup
```

此腳本會安裝 ROS 相關套件、建立 catkin workspace、連結 `tjark_agv` 與 `tjark_nav`，並執行 build。

重新編譯：

```bash
./run_demo.sh build
```

啟動完整自主導航示範：

```bash
./run_demo.sh demo
```

或直接：

```bash
roslaunch tjark_nav demo.launch
```

啟動建圖流程：

```bash
./run_demo.sh mapping
```

鍵盤控制建圖：

```bash
./run_demo.sh teleop
```

儲存地圖：

```bash
./run_demo.sh save-map obstacle
```

查看重要 ROS topic/node 狀態：

```bash
./run_demo.sh status
```

停止 Gazebo / ROS demo 程序：

```bash
./run_demo.sh clean
```

---

## Repository 結構

```text
proj/
├── run_demo.sh
├── setup.sh
├── requirements.txt
├── README.md
├── tjark_agv/
│   ├── launch/
│   │   ├── display.launch
│   │   ├── gazebo.launch
│   │   └── tjark_agv.launch
│   ├── meshes/
│   └── urdf/
│       ├── tjark_agv.urdf
│       ├── tjark_agv.xacro
│       ├── tjark_agv.sensor.xacro
│       ├── tjark_agv.controller.xacro
│       ├── tjark_agv.motor.xacro
│       ├── tjark_agv.color.xacro
│       └── tjark_agv.controller.yaml
└── tjark_nav/
    ├── launch/
    │   ├── spawn_world.launch
    │   ├── mapping.launch
    │   ├── navigation.launch
    │   └── demo.launch
    ├── config/
    │   ├── amcl.yaml
    │   ├── costmap_common.yaml
    │   ├── global_costmap.yaml
    │   ├── local_costmap.yaml
    │   ├── move_base.yaml
    │   └── dwa_local_planner.yaml
    ├── maps/
    │   ├── map.yaml
    │   ├── map.pgm
    │   ├── cafe_map.yaml
    │   └── cafe_map.pgm
    ├── rviz/
    ├── scripts/
    │   ├── pedestrian_controller.py
    │   ├── navigation_demo.py
    │   ├── marker_detector.py
    │   ├── odom_tf_broadcaster.py
    │   └── waypoint_saver.py
    └── worlds/
        ├── obstacle_avoidance.world
        └── cafe_world.world
```

---

## Robot：tjark AGV

### 底盤形式

AGV 使用差速驅動模型：

- 左右兩個主動輪提供速度控制。
- 前後輔助輪/萬向輪用於支撐。
- 控制輸入為 `/cmd_vel`。
- Gazebo diff-drive plugin 產生 `/odom`。

主要設定位於：

- `tjark_agv/urdf/tjark_agv.urdf`
- `tjark_agv/urdf/tjark_agv.controller.xacro`
- `tjark_agv/urdf/tjark_agv.controller.yaml`

差速驅動 plugin 的角色是把 ROS velocity command 轉成左右輪速度，並發布 odometry。導航 stack 不直接控制輪子，而是只輸出 `geometry_msgs/Twist` 到 `/cmd_vel`。

### 重要 TF frames

```text
map
└── odom
    └── base_link
        ├── base_body
        │   ├── laser
        │   └── camera_link
        │       └── camera_optical_link
        ├── left_wheel
        ├── right_wheel
        ├── LF_link
        ├── LB_link
        ├── RF_link
        └── RB_link
```

各 frame 用途：

| Frame | 用途 |
| --- | --- |
| `map` | 全域地圖座標，由 map server / RTAB-Map / AMCL 使用 |
| `odom` | 連續但會漂移的里程計座標 |
| `base_link` | AGV 本體基準 frame |
| `laser` | 2D LiDAR frame |
| `camera_link` | 深度相機機械 frame |
| `camera_optical_link` | 符合 REP-103 optical convention 的相機資料 frame |

AMCL 負責發布 `map -> odom` 的修正，Gazebo diff-drive plugin 則提供 `odom -> base_link` 的運動估測。

---

## 感測器設計

### 1. 2D LiDAR

檔案：`tjark_agv/urdf/tjark_agv.sensor.xacro`

LiDAR 使用 Gazebo ray sensor 模擬：

| 參數 | 數值 |
| --- | --- |
| Topic | `/scan` |
| Frame | `laser` |
| Update rate | 10 Hz |
| Samples | 360 |
| Min range | 0.12 m |
| Max range | 20.0 m |
| Noise | Gaussian, stddev 0.01 |

LiDAR 是本專案導航和避障的主要感測器。它被用於：

- RTAB-Map 建圖時產生乾淨的 2D occupancy grid。
- AMCL 將即時 laser scan 和地圖匹配，估測 AGV 在 `map` 中的位置。
- costmap obstacle layer 標記動態障礙物，例如行人 cylinder。
- DWA local planner 依據 local costmap 產生避障速度。

### 2. RGB-D Depth Camera

檔案：`tjark_agv/urdf/tjark_agv.sensor.xacro`

深度相機使用 Gazebo OpenNI Kinect plugin：

| 參數 | 數值 |
| --- | --- |
| RGB image | `/my_camera/color/image_raw` |
| RGB camera info | `/my_camera/color/camera_info` |
| Depth image | `/my_camera/depth/image_raw` |
| Point cloud | `/my_camera/depth/points` |
| Frame | `camera_optical_link` |
| Image size | 640 x 480 |
| Plugin update rate | 10 Hz |
| Camera sensor update rate | 20 Hz |
| Depth range | 0.5 m to 3.0 m point cloud cutoff |

深度相機在本專案中有兩個用途：

1. 提供 RTAB-Map RGB-D SLAM 的 RGB/depth input。
2. 讓 `marker_detector.py` 辨識綠色起點牌與紅色終點牌，並把它們投影到 `map` frame。

### Camera optical frame 為什麼重要

Gazebo depth plugin 產生的影像和點雲遵循 optical frame convention：

- x：往右
- y：往下
- z：往前

但機器人 URDF 裡的 `camera_link` 軸向不一定符合這個慣例。如果直接把點雲發布在 `camera_link`，3D 點投影到地圖時可能會被轉到錯誤方向，甚至看起來落在地板裡。

因此專案新增固定 frame：

```xml
<link name="camera_optical_link"/>
<joint name="camera_optical_joint" type="fixed">
    <origin xyz="0 0 0" rpy="1.5707963 0 3.14159265"/>
    <parent link="camera_link"/>
    <child  link="camera_optical_link"/>
</joint>
```

深度相機 plugin 的 `frameName` 設為 `camera_optical_link`。所有使用點雲的程式都應該讀取 message header 裡的 frame，而不是硬寫 `camera_link`。

---

## 場景設計

主要場景為：

```text
tjark_nav/worlds/obstacle_avoidance.world
```

場地尺寸約 26 m x 16 m，包含：

- 四周牆壁。
- 多個貨架/箱子作為靜態障礙。
- 起點、兩個黃色中繼點、終點地面標記。
- 綠色與紅色視覺標記牌，供 RGB-D camera 在建圖階段辨識。
- 由 `pedestrian_controller.py` 動態生成的行人。

目前 obstacle 場景重點座標：

| 位置 | 座標 | 說明 |
| --- | --- | --- |
| Start | `(-10.0, -5.0)` | AGV 初始點 |
| Waypoint 1 | `(-2.8, -5.9)` | 黃色通過點，只需經過，不停車 |
| Waypoint 2 | `(1.0, -1.0)` | 中間檢查點，避開過多人群區 |
| Final goal | `(9.0, 5.0)` | 紅色終點 |
| Green sign | `(-10.0, -6.3)` | 建圖階段辨識 start |
| Red sign | `(9.0, 6.3)` | 建圖階段辨識 end |

---

## 建圖流程：RTAB-Map RGB-D SLAM

建圖 launch：

```bash
roslaunch tjark_nav mapping.launch
```

或：

```bash
./run_demo.sh mapping
```

### 建圖使用的資料

RTAB-Map node 位於 `tjark_nav/launch/mapping.launch`：

```xml
<node pkg="rtabmap_slam" type="rtabmap" name="rtabmap"
      args="--delete_db_on_start" output="screen">
```

輸入資料：

| Input | Topic |
| --- | --- |
| RGB image | `/my_camera/color/image_raw` |
| Depth image | `/my_camera/depth/image_raw` |
| Camera info | `/my_camera/color/camera_info` |
| Laser scan | `/scan` |
| Odometry | `/odom` |

重要設定：

| 參數 | 值 | 作用 |
| --- | --- | --- |
| `Reg/Force3DoF` | `true` | 限制為平面移動，符合 AGV 場景 |
| `Optimizer/Slam2D` | `true` | 使用 2D SLAM 假設 |
| `Grid/FromDepth` | `false` | occupancy grid 主要由 LiDAR 建立 |
| `Grid/RangeMax` | `12.0` | grid 使用的感測範圍 |
| `Grid/CellSize` | `0.05` | 地圖解析度 5 cm |
| `RGBD/NeighborLinkRefining` | `true` | 改善相鄰節點約束 |
| `RGBD/ProximityBySpace` | `true` | 空間接近時建立 proximity link |

### 為什麼用 RTAB-Map

RTAB-Map 是圖優化式 SLAM。它會把機器人移動過程中的 sensor observation 建成 graph：

- node：某一時刻的 robot pose 和感測資料。
- edge：odometry、visual/depth constraint、loop closure constraint。
- graph optimization：修正累積誤差，讓地圖一致。

在本專案中，地圖最後用於 2D navigation，因此 occupancy grid 主要依賴 LiDAR。RGB-D camera 的價值在於提供視覺資訊與標記辨識能力。

### 儲存地圖

當 RViz 中 `/map` 覆蓋完整後：

```bash
./run_demo.sh save-map obstacle
```

或手動：

```bash
rosrun map_server map_saver -f $(rospack find tjark_nav)/maps/map
```

`tjark_nav/maps/map.yaml`：

```yaml
image: map.pgm
resolution: 0.05
origin: [-13.0, -8.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
```

---

## 視覺標記辨識：marker_detector.py

檔案：

```text
tjark_nav/scripts/marker_detector.py
```

此節點在建圖階段運作，用來自動辨識起點與終點，不需要手動輸入座標。

流程：

1. 訂閱 RGB image。
2. 使用 HSV threshold 找出綠色或紅色區塊。
3. 找最大 contour，取得其 pixel centroid。
4. 從 organized point cloud 的同一 pixel 讀取 3D XYZ。
5. 使用 TF 將點從 `camera_optical_link` 轉換到 `map`。
6. 若連續多幀穩定，鎖定該 waypoint。
7. 在 RViz 發布 `/detected_markers`。
8. 寫入 `maps/obstacle_waypoints.yaml` 或 `maps/cafe_waypoints.yaml`。

辨識語意：

| 顏色 | 意義 |
| --- | --- |
| Green | START |
| Red | END |

這個設計模擬真實部署時「用視覺 cue 標定工作區起終點」的流程，而不是在程式中硬寫全部座標。

---

## 定位流程：AMCL

導航階段使用 AMCL：

```text
tjark_nav/config/amcl.yaml
```

AMCL 是 Adaptive Monte Carlo Localization，基於粒子濾波。它用大量粒子代表機器人可能的位置，並透過以下資訊不斷更新：

- odometry motion model：根據 `/odom` 預測粒子移動。
- laser sensor model：比較 `/scan` 和已知 `/map` 的符合程度。
- resampling：保留高機率粒子，淘汰低機率粒子。

本專案 AMCL 重要參數：

| 參數 | 值 | 說明 |
| --- | --- | --- |
| `odom_model_type` | `diff` | 差速驅動模型 |
| `min_particles` | `500` | 最少粒子 |
| `max_particles` | `3000` | 最多粒子 |
| `laser_model_type` | `likelihood_field` | 使用 likelihood field sensor model |
| `laser_max_beams` | `60` | 每次更新使用的 beam 數 |
| `update_min_d` | `0.15` | 移動超過 0.15 m 更新 |
| `update_min_a` | `0.15` | 旋轉超過 0.15 rad 更新 |

`navigation_demo.py` 會在啟動後自動發布 `/initialpose`，讓 AMCL 從已知 spawn pose 開始收斂：

```text
obstacle scene: (-10.0, -5.0, yaw=0.0)
```

---

## 全域規劃：NavfnROS

全域規劃由 `move_base` 內的 `navfn/NavfnROS` 負責：

```yaml
base_global_planner: navfn/NavfnROS
```

Navfn 會根據 static map 和 global costmap 產生一條從目前位置到目標點的全域路徑。它主要避開地圖中已知的靜態障礙，例如牆壁、貨架和箱子。

全域規劃不負責精細動態避障。動態行人會由 local costmap 和 DWA 處理。

---

## 區域規劃與動態避障：DWA

區域規劃器：

```yaml
base_local_planner: dwa_local_planner/DWAPlannerROS
```

設定檔：

```text
tjark_nav/config/dwa_local_planner.yaml
```

DWA 是 Dynamic Window Approach。它的基本想法是：

1. 根據機器人的速度與加速度限制，產生一組可行速度 `(v, w)`。
2. 對每組速度往未來模擬一小段軌跡。
3. 檢查軌跡是否撞到 local costmap 中的障礙物。
4. 依據接近全域路徑、接近目標、遠離障礙物等分數選出最佳速度。
5. 發布 `/cmd_vel` 給差速驅動 plugin。

本專案重要 DWA 參數：

| 參數 | 值 | 作用 |
| --- | --- | --- |
| `max_vel_x` | `0.42` | 最大前進速度 |
| `min_vel_x` | `-0.05` | 允許些微倒退 |
| `max_vel_theta` | `1.1` | 最大角速度 |
| `acc_lim_x` | `1.2` | 線加速度限制 |
| `acc_lim_theta` | `2.0` | 角加速度限制 |
| `sim_time` | `3.0` | 往前模擬 3 秒軌跡 |
| `vx_samples` | `20` | 線速度取樣數 |
| `vth_samples` | `48` | 角速度取樣數 |
| `path_distance_bias` | `18.0` | 偏好貼近全域路徑 |
| `goal_distance_bias` | `18.0` | 偏好朝目標前進 |
| `occdist_scale` | `0.12` | 避開障礙物成本權重 |

在動態行人場景中，DWA 的角色是讓 AGV 不只會「繞開整群人」，也能在 local costmap 允許時從人與人之間的空隙通過。

---

## Costmap 邏輯

共用設定：

```text
tjark_nav/config/costmap_common.yaml
```

### Footprint

AGV footprint：

```yaml
footprint: [[ 0.22,  0.22],
            [ 0.22, -0.22],
            [-0.22, -0.22],
            [-0.22,  0.22]]
footprint_padding: 0.03
```

這代表導航 stack 會把 AGV 當成約 0.44 m x 0.44 m 的方形底盤，再加上 padding。

### Inflation

```yaml
inflation_radius: 1.00
cost_scaling_factor: 2.0
```

行人不是只有 cylinder 半徑本身危險，因為人會移動，所以 costmap 需要在障礙周圍膨脹。`inflation_radius` 越大，AGV 越保守；越小，AGV 越敢貼近障礙物穿行。

### Obstacle layer

```yaml
observation_sources: laser_scan_sensor

laser_scan_sensor:
  sensor_frame: laser
  data_type: LaserScan
  topic: /scan
  marking: true
  clearing: true
```

local costmap 只使用 `/scan` 做 2D obstacle marking/clearing。深度相機不直接進入 navigation costmap，避免 RGB-D 噪聲影響 2D 移動。

---

## 行人模擬

檔案：

```text
tjark_nav/scripts/pedestrian_controller.py
```

行人由 Python 節點生成 SDF cylinder-like pedestrian model，並透過 Gazebo service 控制：

| Service | 用途 |
| --- | --- |
| `/gazebo/spawn_sdf_model` | 生成行人模型 |
| `/gazebo/set_model_state` | 以 10 Hz 更新行人位置 |

每個行人是 Gazebo 中真實存在的 model，因此會被 LiDAR 掃描到，也會進入 local costmap。

### 行人外觀與碰撞

每位行人包含：

- cylinder collision，半徑 0.30 m，高 1.80 m。
- 頭、身體、手、腳等 visual geometry。
- 藍色或橘色 torso，用於區分兩波人群。

### 行人運動模型

行人位置由 `_position_at()` 計算：

```python
return float(p["x"]) + ax * s, float(p["y"]) + ay * s
```

其中 `s` 是三角波或 sine wave。預設 obstacle 場景使用 linear triangle wave，讓行人做來回移動。

### 目前兩波人群配置

重要設定位於 `pedestrian_controller.py`：

```python
_ZONE_A_X = [-7.8, -8.0, -7.2, -6.4, -6.2]
_ZONE_B_X = [1.8, 3.0, 4.0, 6.4, 8.3]
_ZONE_Y = {
    "A": [-4.6, -3.7, -2.6, -4.1, -3.0],
    "B": [-1.6, -0.2, 1.1, 1.4, 2.8],
}
```

Zone A：靠近第一段路徑，分散在起點後方通道附近，且保留 AGV 起點 `(-10, -5)` 周圍 2 m keepout，避免一啟動就撞到人。

Zone B：沿著往第三目標 `(9, 5)` 的方向斜向分布，x/y 都拉開，讓 AGV 更容易展示穿行人群的效果，而不是直接把人群視為一整塊障礙繞掉。

目前固定 seed 下第二波大致初始位置：

```text
ped_B1: (1.86, -1.89)
ped_B2: (3.13,  0.19)
ped_B3: (3.76,  1.28)
ped_B4: (6.27,  1.23)
ped_B5: (8.39,  2.73)
```

### 安全檢查

`_is_safe()` 會檢查：

- 是否在場地 bounds 內。
- 是否進入靜態障礙物膨脹區。
- 是否進入起點 keepout 區。

若某個行人下一步位置不安全，controller 會維持上一個安全位置，避免行人穿進障礙物或牆壁。

---

## 自主導航流程

主要 launch：

```bash
roslaunch tjark_nav demo.launch
```

它會啟動：

1. Gazebo world + AGV。
2. map server。
3. AMCL。
4. move_base。
5. pedestrian controller。
6. navigation demo script。
7. RViz。

### navigation_demo.py

檔案：

```text
tjark_nav/scripts/navigation_demo.py
```

此腳本負責：

- 載入建圖階段儲存的 waypoints，如果存在就使用。
- 如果沒有 waypoints file，使用 launch file 中的 fallback goals。
- 發布 `/initialpose` 給 AMCL。
- 等待 AMCL 收斂。
- 依序送 goal 給 `move_base` action server。
- 對第一個黃色中繼點使用 pass-through 邏輯。

目前 obstacle fallback goals：

```yaml
[
  {x: -2.8, y: -5.9, yaw: 0.0, label: "Static obstacle bypass", pass_through: true},
  {x:  1.0, y: -1.0, yaw: 0.0, label: "Central corridor checkpoint"},
  {x:  9.0, y:  5.0, yaw: 0.0, label: "Final upper goal"}
]
```

### Pass-through waypoint

第一個黃色點不需要停下來。`navigation_demo.py` 的 `navigate_through()` 會訂閱 `/amcl_pose`，當 AGV 進入目標半徑時就取消當前 goal 並切換到下一個 goal。

預設半徑：

```python
PASS_THROUGH_RADIUS_M = 1.0
```

這讓第一個黃色點扮演「路線引導點」，而不是停車任務點。AGV 經過即可繼續，不會在行人或障礙附近停住。

---

## ROS Topic 與資料流

核心資料流：

```text
Gazebo diff-drive plugin
  ├── publishes /odom
  └── receives  /cmd_vel

Gazebo LiDAR
  └── publishes /scan

Gazebo RGB-D camera
  ├── publishes /my_camera/color/image_raw
  ├── publishes /my_camera/depth/image_raw
  └── publishes /my_camera/depth/points

map_server
  └── publishes /map

AMCL
  ├── subscribes /map, /scan, /odom
  ├── publishes /amcl_pose
  └── publishes TF map -> odom

move_base
  ├── subscribes /map, /scan, TF
  ├── runs Navfn global planner
  ├── runs DWA local planner
  └── publishes /cmd_vel

pedestrian_controller
  ├── calls /gazebo/spawn_sdf_model
  └── calls /gazebo/set_model_state

navigation_demo
  ├── publishes /initialpose
  ├── subscribes /amcl_pose
  └── sends goals to /move_base action server
```

---

## Launch files

| Launch file | 用途 |
| --- | --- |
| `tjark_nav/launch/spawn_world.launch` | 只啟動 Gazebo world 和 AGV |
| `tjark_nav/launch/mapping.launch` | 啟動建圖、RTAB-Map、marker detector、RViz |
| `tjark_nav/launch/navigation.launch` | 啟動 map server、AMCL、move_base |
| `tjark_nav/launch/demo.launch` | 完整 demo：Gazebo、導航、人群、目標腳本、RViz |
| `tjark_agv/launch/display.launch` | 只檢視 URDF |
| `tjark_agv/launch/tjark_agv.launch` | 原始 AGV Gazebo launch |

---

## 常用操作

### 啟動完整 demo

```bash
./run_demo.sh demo obstacle
```

### 啟動 cafe 場景

```bash
./run_demo.sh demo cafe
```

### 只啟動世界和車

```bash
roslaunch tjark_nav spawn_world.launch
```

### 只啟動導航 stack

```bash
roslaunch tjark_nav navigation.launch
```

### 手動啟動行人

```bash
rosrun tjark_nav pedestrian_controller.py
```

### 手動送導航目標

```bash
rosrun tjark_nav navigation_demo.py
```

### 查看 topic

```bash
rostopic list
rostopic echo /scan
rostopic echo /amcl_pose
rostopic echo /move_base/status
```

### 查看 TF

```bash
rosrun tf view_frames
rosrun tf tf_echo map base_link
rosrun tf tf_echo map odom
```

---

## 調參指南

### 想讓 AGV 更敢穿過人群

可以調整：

```text
tjark_nav/config/costmap_common.yaml
```

- 降低 `inflation_radius`，例如從 `1.00` 改成 `0.80`。
- 提高 DWA 對路徑/目標的偏好。

```text
tjark_nav/config/dwa_local_planner.yaml
```

- 增加 `path_distance_bias`。
- 增加 `goal_distance_bias`。
- 適度降低 `occdist_scale`。

注意：太激進會讓 AGV 貼人太近。

### 想讓 AGV 更保守

- 增加 `inflation_radius`。
- 增加 `occdist_scale`。
- 降低 `max_vel_x`。
- 增加 `stop_time_buffer`。

### 想調整第二波人群

修改：

```text
tjark_nav/scripts/pedestrian_controller.py
```

重點變數：

| 變數 | 說明 |
| --- | --- |
| `_ZONE_B_X` | 第二波人群的 x 基準分布 |
| `_ZONE_Y["B"]` | 第二波人群的 y 基準分布 |
| `_ZONE_AY["B"]` | 第二波人群 y 方向移動幅度 |
| `_ZONE_PHASES["B"]` | 初始相位，影響一開始的位置 |

如果希望第二波更往終點方向分布，增加後面幾個人的 x/y。  
如果希望 AGV 更容易從中穿過，增加人與人之間距離，或降低行人移動幅度。

### 想避免行人生成在 AGV 起點

`_SCENES["obstacle"]["keepouts"]` 已設定：

```python
"keepouts": [(-10.0, -5.0, 2.0)]
```

代表 `(-10, -5)` 半徑 2 m 內不允許行人進入。

---

## Troubleshooting

### AGV 不動

檢查：

```bash
rostopic echo /cmd_vel
rostopic echo /odom
```

若 `/cmd_vel` 有資料但 `/odom` 不動，可能是 Gazebo diff-drive plugin 或 robot spawn 有問題。  
若 `/cmd_vel` 沒資料，檢查 `move_base` 是否啟動、goal 是否送出。

### RViz 中看不到 map

檢查：

```bash
rostopic echo /map
rosnode list | grep map
```

確認 `map_server` 或 RTAB-Map 有發布 `/map`。

### AMCL 不收斂

檢查：

```bash
rostopic echo /scan
rosrun tf tf_echo map odom
```

常見原因：

- `/scan` 沒有資料。
- 初始姿態差太多。
- map 和 world geometry 不一致。
- laser frame TF 錯誤。

可以在 RViz 使用 `2D Pose Estimate` 手動重設初始位置。

### 行人沒有被避開

檢查 Gazebo 是否生成行人：

```bash
rosservice call /gazebo/get_model_state "model_name: ped_B1
relative_entity_name: world"
```

檢查 LiDAR 是否掃到行人：

```bash
rostopic echo /scan
```

若 Gazebo 有行人但 costmap 沒反應，檢查 `costmap_common.yaml` 的 observation source 是否指向 `/scan`。

### AGV 繞開整群人，不穿過人群

可能原因：

- `inflation_radius` 太大。
- 行人彼此距離太近。
- DWA 的 obstacle cost 權重太高。
- 全域路徑本身繞到人群外側。

可調整：

- 拉開 `_ZONE_B_X` / `_ZONE_Y["B"]`。
- 降低 `_ZONE_AY["B"]`，讓行人移動不要堵住縫隙。
- 略降 `inflation_radius`。
- 略降 `occdist_scale`。

### Gazebo model 無法載入

確認：

```bash
echo $GAZEBO_MODEL_PATH
```

必要時：

```bash
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH
```

---

## 設計取捨

### 為什麼不用 depth camera 直接做避障

深度相機適合近距離 3D 感知，但本專案導航目標是 2D 平面移動。使用 LiDAR 建立 costmap 有幾個優點：

- 2D obstacle layer 穩定且簡單。
- 行人 cylinder 一定會出現在 laser scan 中。
- AMCL 和 costmap 使用同一類 sensor，debug 更直接。
- 避免深度點雲雜訊造成 local costmap 閃爍。

深度相機保留給建圖與 marker detection。

### 為什麼用 DWA 而不是 TEB

DWA 是 ROS Navigation Stack 中穩定、經典、容易展示的 local planner。它對差速車和 2D costmap 支援完整。  
TEB 對動態障礙和時間最佳化有更強能力，但設定更複雜。本專案選 DWA 是為了讓整體 pipeline 更容易理解、部署和調參。

### 為什麼用 pass-through waypoint

中繼點的目的只是引導 AGV 走進展示路徑。如果每個黃色點都要求 `move_base` 完整收斂，AGV 會在中間停車，反而容易被行人撞上，也不符合「流暢穿行」的展示目標。

因此第一個黃色點只要接近 1 m 內就切換到下一個目標。

---

## Demo 重點

這個專案展示的是一條完整的 mobile robot navigation pipeline：

1. Gazebo 建立可重現的動態環境。
2. URDF/xacro 定義 AGV、感測器與驅動。
3. RTAB-Map 建立 2D occupancy grid。
4. RGB-D camera 自動辨識起終點標記。
5. AMCL 在已知地圖中定位。
6. Navfn 產生全域路徑。
7. costmap 使用 LiDAR 即時標記行人。
8. DWA 依照 local costmap 即時輸出避障速度。
9. 行人 controller 生成多名動態障礙。
10. navigation script 串接起點、通過點、中間點與終點。

最終效果是：AGV 從起點出發，經過黃色通過點，穿越分散的人群，避開靜態貨架與動態行人，最後抵達紅色終點。

---

## 重要檔案索引

| 檔案 | 說明 |
| --- | --- |
| `run_demo.sh` | 最方便的 setup/build/mapping/demo/test/status 入口 |
| `setup.sh` | 安裝環境與建立 workspace |
| `tjark_agv/urdf/tjark_agv.xacro` | AGV xacro 組裝入口 |
| `tjark_agv/urdf/tjark_agv.sensor.xacro` | LiDAR 與 RGB-D camera |
| `tjark_agv/urdf/tjark_agv.controller.xacro` | Gazebo diff-drive plugin |
| `tjark_nav/worlds/obstacle_avoidance.world` | 主要避障世界 |
| `tjark_nav/launch/mapping.launch` | 建圖流程 |
| `tjark_nav/launch/demo.launch` | 完整自主導航 demo |
| `tjark_nav/config/amcl.yaml` | AMCL 定位參數 |
| `tjark_nav/config/costmap_common.yaml` | footprint、inflation、obstacle source |
| `tjark_nav/config/dwa_local_planner.yaml` | DWA local planner 參數 |
| `tjark_nav/scripts/pedestrian_controller.py` | 行人生成與動態移動 |
| `tjark_nav/scripts/navigation_demo.py` | 自動發布初始姿態與導航目標 |
| `tjark_nav/scripts/marker_detector.py` | RGB-D 起終點標記辨識 |
