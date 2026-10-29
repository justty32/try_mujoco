# 六軸機械手臂：已核准外形原型 v1

**使用者已核准外形與展示姿態（2026-10-29）。** 正式模型為 [`assets/six_axis_arm.xml`](../../../../assets/six_axis_arm.xml)，沿用本原型的 9 圓柱比例、輪廓與旋轉軸，另外接入馬達與 home keyframe。正式操作與驗證範圍見 [專案 README](../../../../README.md)。

此目錄的 XML、PNG 與 metadata 保留原始外形提案證據，不回寫：metadata 的「未替換正式模型」是拍攝當時狀態，原圖本身也不證明控制收斂或完整碰撞範圍。

正式接入時，J5 手腕俯仰限位依間隙驗證收為 ±45°（原型為未驗證的 ±120°），home 35° 與外形／軸不變。建置與三項 CTest 結果另見 [正式接入紀錄](../../../reports/six-axis-integration-2026-10-29/README.md)；原型檔案維持拍攝時內容。

- [preview.png](preview.png)：9 個圓柱的實際 MuJoCo 軟體渲染，800×600。
- [model.xml](model.xml)：本次外形原型，6 軸分工為底座 yaw、肩／肘 pitch、腕 roll/pitch/roll。
- [metadata.json](metadata.json)：展示角度、相機及 llvmpipe 軟體渲染紀錄，未推進模擬。

三檔原始來源為 `/tmp/robot-6r-prototype-v1/`，2026-10-29 複製保存。展示角度 `[0, 65, -100, 0, 35, 0]°` 現已採為正式 home；它不是六個全零角度。
