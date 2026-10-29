# 六關節手臂外形與裝配間隙修正

> 歷史紀錄：此三組 yaw/pitch 叉耳構型已由使用者核准的典型 6R 圓柱手臂取代；本頁保留當輪證據，修正後模型凍結於 [after.xml](after.xml)，不是現行預設。原始 JSON 中的 assets 路徑是量測當時的位置。

2026-10-29。保留六個關節的中心、軸、角度範圍與末端位置，把粗圓柱外形改成有轉動間隙的叉耳式裝配。先看 [修改前後比較](preview.html)，或回 [操作說明](../../../README.md#操作說明)。

## 改了什麼

- 保留 `yaw1/pitch1/yaw2/pitch2/yaw3/pitch3`、原 body 名稱與 actuator 對應；歷史檔名仍是 `assets/seven_cylinders.xml`。
- 每組使用固定 yaw 座、薄轉盤、叉耳橫橋與左右耳、pitch 轉軸／輪轂，再接半徑 10 mm 的細連桿。視覺與碰撞使用同一組實體，沒有關閉碰撞或加入排除清單。
- 密度明設為 1000 kg/m³，質量與慣量由 MuJoCo 依幾何計算；這是教學原型，不代表已校準的真實硬體。
- 新模型的肩部負載降低，所有模式都可維持原 `ctrlrange = [-1, 1]`，移除舊粗圓柱模型的姿態模式額外力矩範圍。preset 控制值不變，但模型動態已變，不能宣稱舊運動軌跡相同。

| 零姿態量測 | 修正前 | 修正後 |
|---|---:|---:|
| 六個活動 body 的總質量 | 12.480 kg | 0.494 kg |
| `pitch1` bias 力矩 | 19.9715 N·m | 1.1277 N·m |
| 姿態模式 `pitch1` 力矩上限 | 40 N·m 特例 | 原本的 10 N·m |

質量與 bias 原值見 [geometry_samples.json](geometry_samples.json)。力矩上限由控制範圍乘 gear 得到，並非此表用來保證所有姿態都可達。

## 這次解決的問題與邊界

原本不同 body 的粗圓柱互相重疊，使關節轉動空間與裝配關係難以辨識。父子 body 的簡化外形重疊不必然代表模擬一定錯誤：本輪原始紀錄中，舊模型零姿態有跨 body 幾何重疊，但引擎接觸數仍為零，兩者反映的檢查範圍不同。本次修正的是可明確看見、可量測的裝配穿插與轉動間隙，沒有把所有歷史重疊都當成物理錯誤。

幾何檢查另外直接量測不同 body 的 geom 距離，包含通常可能被接觸篩選略過的父子配對；距離小於 −1 µm 才記為穿透。同一 body 內的零件接合不在此穿透檢查範圍。`engine_contacts` 是 MuJoCo 的接觸結果，`penetrations` 是額外距離探測，兩者不能混為一談。

## 有限姿態取樣結果

使用 MuJoCo 3.10.0，固定設定 qpos、速度歸零後執行 forward kinematics，**沒有推進模擬，也不是連續掃掠或全域可達性驗證**。

| 範圍 | 修正前 | 修正後 |
|---|---|---|
| 必須有間隙的 27 個快照 | 27 個有跨 body 穿透 | 27 個均無穿透、無引擎接觸 |
| 全部 45 個快照 | 保存完整逐姿態紀錄 | 44 個無穿透；`pitch1 = -60°` 仍有基座／地面碰撞 |

27 個必要快照為零姿態、每個關節單獨 ±10°／±30°（24 個），以及兩個組合姿態。其餘大角度快照只作邊界紀錄，不作「全部必須無碰撞」的要求。

兩個組合姿態依 `yaw1, pitch1, yaw2, pitch2, yaw3, pitch3` 排列：

- `mixed_a = [20, 25, -15, 15, 10, 20]°`
- `mixed_b = [-25, 40, 20, 10, -15, 15]°`

`pitch1 = -60°`、其餘為零時，手臂確實會進入基座與地面；保留這項碰撞與關節角範圍，不用關閉碰撞或縮小報告範圍掩蓋。多關節任意組合仍可能碰撞，控制器也不補償接觸力。

## 建置與動態回歸

`cmd /c build.bat` 成功；`ctest --test-dir build --output-on-failure -j1` 的三項測試全通過，合計 0.36 秒。完整紀錄見 [ctest.log](ctest.log)，摘要與原始碼 hash 見 [validation.json](validation.json)。

| 測試 | 本輪通過範圍 |
|---|---|
| `seven_cylinders_model` | 模型載入、保留的關節拓撲／座標、有限數值 |
| `controller_pose` | 水平保持、兩個組合目標、持續保持、姿態捕捉、回零／重置、控制邊界與阻尼選項 |
| `arm_geometry` | 27 個指定無穿透姿態、支撐結構相接、碰撞仍啟用 |

姿態測試各項最後的最大誤差與速度，以小數六位列印皆為 `0.000000`；這表示低於該輸出精度，不宣稱數學上精確為零。為配合新模型較小的慣量，控制器在 Euler 步進與隱式阻尼啟用時加入有效慣量修正，範圍限本模型的線性 joint damping；不是任意阻尼形式的通用補償。PD 係數、30°/s 參考速率與原收斂門檻未降低。Euler 處理依據見 [MuJoCo 3.10.0 的 mj_EulerSkip](https://github.com/google-deepmind/mujoco/blob/3.10.0/src/engine/engine_forward.c#L1004)。

負向驗證也確實能抓到錯誤：移除叉耳橫橋，以及刻意加粗連桿，兩份測試副本都回傳 exit 1。分別見 [缺少橫橋紀錄](geometry_missing_bridge.log)／[加粗連桿紀錄](geometry_thick_beam.log)。

上述靜態／動態檢查都沒有開互動視窗；GUI 的鍵盤操作仍未人工實測。圖片只證明指定快照的外觀，不能替代動態收斂測試。

## 圖片來源

[比較頁](preview.html) 的六張 PNG 是 MuJoCo 原始模型的實際渲染像素，無生成式圖片、修圖或合成。使用 `llvmpipe` 軟體渲染，沒有訓練或持續開啟 GUI；每張 600 × 400。

- 全貌／關節近景：前後均為六個零角度。
- 組合姿態：前後均為 `mixed_a`。
- 每組前後使用相同相機參數。完整 model hash、角度與相機見 [before-render.json](before-render.json)／[after-render.json](after-render.json)。

## 可追溯檔案

- [before.xml](before.xml)：修改前模型快照。
- [after.xml](after.xml)：當輪修改後的舊模型快照，與原始 hash 相同。
- [geometry_samples.json](geometry_samples.json)：兩份模型 hash、mass／bias 與每個快照的距離／接觸紀錄。
- [validation.json](validation.json)：建置／CTest 結果、姿態收斂輸出、負向驗證退出碼與原始碼 hash。
- [inspect_geometry.py](inspect_geometry.py)：取樣與選擇性軟體渲染工具；從 repo root 執行 `python docs/reports/geometry-2026-10-29/inspect_geometry.py` 可重新取樣，需現有 MuJoCo Python 環境。`--render` 另需可用的軟體 OpenGL 環境；它會檢查 renderer 並拒絕未辨識為軟體的 backend。

重跑工具會覆寫這份目錄的生成紀錄；先另存此輪證據再執行；重跑工具只讀本目錄凍結的 before／after 模型。
