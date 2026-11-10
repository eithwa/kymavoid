# avoid_bringup

避障挑戰的一鍵啟動（tmux）和網頁控制台，實車和模擬共用。設計見 [docs/avoidance-redesign-plan.md](../../docs/avoidance-redesign-plan.md) 第 6、7 節。

## 啟動

```bash
# 模擬（純 Gazebo，不需要任何實車驅動）
scripts/avoid_sim.sh                      # 攝影機設定
scripts/avoid_sim.sh lidar --seed 5       # 雷射設定，指定場地 seed
scripts/avoid_sim.sh --no-gui             # 不開 Gazebo 視窗

# 實車：兩種感測器設定二選一
scripts/avoid_real.sh camera              # prosilica + black_item，不啟動雷射
scripts/avoid_real.sh lidar               # 3 顆 Hokuyo，不啟動攝影機

# 全部停掉
scripts/avoid_stop.sh
```

啟動後終端機會印出網頁位址，筆電用瀏覽器開 `http://<車子 IP>:8000/`。

腳本會先做這些檢查，不通過就停下來並說明原因：

- 需要的 ROS 套件找得到（實車另外需要 `prosilica_camera` 或 `urg_node`）；
- 會用到的 port 沒有被佔用：11311、9090、8000，實車攝影機加 8080，模擬加 11345；
- 實車攝影機設定：被 gitignore 的設定檔（`AvoidChallenge.yaml`、`FIRA.yaml`、`prosilica_driver.yaml`、`HSVcolormap.bin`）如果不存在，就從 `(default)` 範本複製一份；攝影機 `192.168.16.100` ping 不到只會警告；
- 實車雷射設定：3 個裝置都在，而且有讀寫權限。

環境一律由腳本設定：`ROS_MASTER_URI=http://localhost:11311`、`ROS_IP=127.0.0.1`。終端機殘留的 `ROS_IP` 不會被沿用，主機名稱解析不到也不影響。

## tmux 視窗

| 視窗 | 內容 |
|---|---|
| `core` | 上面一格是 `roscore`，下面一格是 `roslaunch`（Gazebo 或實車驅動、轉接層、rosbridge、node_watch） |
| `nav` | `avoid_nav`。改完程式 `catkin_make` 後，在這個視窗按 Ctrl-C，再按 ↑ Enter 就能重開，不用重開 Gazebo |
| `web` | 網頁伺服器 |
| `monitor` | 終端機版的狀態：nav_node 的狀態、各輸入的資料年齡、各 topic 的頻率 |
| `shell` | 環境已經設定好，可以直接下指令，例如 `rosservice call /avoid/command start` |

`tmux attach -t avoid_sim`（或 `avoid_real`）回到 session；`Ctrl-b d` 離開但不關掉。

## 網頁

- **頂部燈號**：rosbridge、nav_node、障礙物感測器、紅線（攝影機設定）、里程計、IMU、定位 σ，模擬時再加真值。必要輸入的資料超過 0.3 秒沒更新就變紅，這和 nav_node 拒絕開始、或行駛中停車的門檻相同。
- **節點**：依目前的設定列出應該存在的節點。crash 的節點會顯示「無回應」。
- **開始／停止**：所有必要輸入都正常才能按開始，不能按時會說明原因。停止任何時候都能按（快捷鍵是空白鍵）。
- **模擬場地**：輸入 seed 後按「重擺場地」，或按「下一個 seed」。
- **參數**：修改 nav_node 的速度、追蹤、安全、感測參數。「套用」之後，**下一次開始時生效**；「存檔」會寫回 `avoid_nav/config/tuning_<camera|lidar>.yaml`。
- **影像**：只有實車攝影機設定才有，畫面來自 mjpeg 串流。

網址參數：`?bridge=ws://主機:埠` 可以改 rosbridge 位址，`?mjpeg=http://主機:埠` 可以改影像串流位址。

## 不用 tmux

```bash
roslaunch avoid_bringup sim.launch sensor:=camera gui:=true
roslaunch avoid_bringup real.launch sensor:=lidar lidar_port_0:=/dev/ttyACM0 ...
rosrun avoid_bringup web_server.py 8000
```

## 改 port（同一台機器同時跑兩組時）

```bash
AVOID_ROS_PORT=11313 AVOID_BRIDGE_PORT=9093 AVOID_WEB_PORT=8003 AVOID_GAZEBO_PORT=11347 scripts/avoid_sim.sh
```

## 實車部分尚未驗證

開發期間沒有車可以測，`real.launch` 只確認過可以正確展開（`roslaunch --nodes` / `--dump-params`）。上車前要確認：

- `motion_adapter` 的速度比例；
- 3 顆雷射的裝置對應（後、左前、右前）和安裝半徑 `lidar_mount_r`；
- `urg_node` 的參數（型號未知）。
