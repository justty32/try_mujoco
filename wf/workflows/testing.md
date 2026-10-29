# testing — 建置與驗證

← [WORKFLOWS.md](../WORKFLOWS.md)｜[INDEX.md](../INDEX.md)

## 指令

- 重新建置：`cmd /c build.bat`
- 自動模型測試：`ctest --test-dir build --output-on-failure`
- 執行互動 viewer：從 repo root 執行 `build\robot.exe`
- 檢查 dirty 範圍：`git status --short` 與相關 `git diff -- <paths>`

## 驗證原則

- `six_axis_arm_model` 以 MuJoCo 載入正式模型與核准原型，對照 9 圓柱外形、6R 軸／馬達映射、home keyframe 與工具方向（含 J6 旋轉）。
- `arm_geometry` 檢查 5 個代表姿態的非相鄰 geom 距離、啟用碰撞及質量／慣量；相鄰圓柱的接頭重疊屬教學簡化，不宣稱所有關節組合可達。`controller_pose` 驗證 home 保持、肩肘／腕追蹤、捕捉、回 home／reset、限位／限幅與受阻時有限數值。
- 其他 MJCF 修改也要以 MuJoCo loader 實際載入；必要時增加正式 headless smoke check，檢查模型尺寸、名稱、joint/site 座標與有限數值。
- viewer 是人工除錯工具。人工視覺驗證無法取代 headless 的數值測試；若本環境無法判斷外觀，將項目記入 [WAIT_USER.md](../WAIT_USER.md)。
