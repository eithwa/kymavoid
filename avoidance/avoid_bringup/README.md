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
- **重設定位**：車子放回起點之後按，定位會歸位到起點，障礙物記憶也會清掉。模擬按「重擺場地」時會自動做。
- **速度**：大字顯示目前的指令速度，旁邊標出「限速原因」（最高速度、加速中、轉向、間隙、前方煞車距離、安全層、沒有路徑）和實際速度。「速度（這一輪）」圖畫出整輪的速度曲線，下方色帶是每個時刻的限速原因。地圖上的軌跡依速度著色（紅慢綠快）。
- **模擬場地**：輸入 seed 後按「重擺場地」，或按「下一個 seed」。
- **記錄這個 case**（模擬）：把目前的場地存成 case，寫上哪裡有問題、預期結果，之後可以重跑（見下面）。
- **參數**：修改 nav_node 的速度、追蹤、安全、感測參數。「套用」之後，**下一次開始時生效**；「存檔」會寫回 `avoid_nav/config/tuning_<camera|lidar>.yaml`。
- **影像**：只有實車攝影機設定才有，畫面來自 mjpeg 串流。

網址參數：`?bridge=ws://主機:埠` 可以改 rosbridge 位址，`?mjpeg=http://主機:埠` 可以改影像串流位址。

## 問題 case：記錄與重跑

測試時遇到有問題的場地，在網頁的「記錄這個 case」存下來。檔案在 `avoidance/avoid_sim/cases/`（預設 `recorded.yaml`），請一起 commit。障礙物座標、起點、seed 都直接存在檔案裡，格式說明在 `avoid_sim/src/avoid_sim/cases.py`。

```bash
scripts/run_cases.sh                                   # 跑 cases/ 底下所有 case（攝影機設定）
scripts/run_cases.sh lidar                             # 雷射設定錄的 case
scripts/run_cases.sh --repeat 3 --only slow-detour-seed6
```

如果已經有模擬 session 在跑，就直接使用；沒有的話，會開一個不帶 Gazebo 視窗的 session，跑完再關掉。每個 case 都會和它的預期比對，判定如下：

| 判定 | 意思 |
|---|---|
| `PASS` | 已修好（`fixed`）的 case 符合預期 |
| `FAIL` | 已修好的 case 不符合預期，表示**退步**；腳本結束碼是 1 |
| `OPEN` | 還沒修好的已知問題（`open`），仍然出現 |
| `NOW PASSES` | `open` 的 case 這次符合預期，可以考慮在檔案裡改成 `fixed` |
| `SKIP` | 用另一種感測器設定錄的 case |

模擬每次的時序會略有不同，路線選擇不穩定的 case 建議用 `--repeat` 多跑幾次。

## 不用 tmux

```bash
roslaunch avoid_bringup sim.launch sensor:=camera gui:=true
roslaunch avoid_bringup real.launch sensor:=lidar lidar_port_0:=/dev/ttyACM0 ...
rosrun avoid_bringup web_server.py 8000
```

## 改 port（同一台機器同時跑兩組時）

```bash
AVOID_ROS_PORT=11313 AVOID_BRIDGE_PORT=9093 AVOID_WEB_PORT=8003 AVOID_GAZEBO_PORT=11347 scripts/avoid_sim.sh
AVOID_ROS_PORT=11313 scripts/avoid_stop.sh          # 只停 11313 這一組
```

每個 ROS port 是一個獨立的實例：預設 port 叫 `avoid_sim`，其他 port 叫 `avoid_sim-<port>`。tmux session、行程標記、環境檔都分開，所以停其中一組，不會停到另一組。

## 實車部分尚未驗證

開發期間沒有車可以測，`real.launch` 只確認過可以正確展開（`roslaunch --nodes` / `--dump-params`）。上車前要確認：

- `motion_adapter` 的速度比例；
- 3 顆雷射的裝置對應（後、左前、右前）和安裝半徑 `lidar_mount_r`；
- `urg_node` 的參數（型號未知）。
