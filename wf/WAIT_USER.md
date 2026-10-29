# WAIT_USER — 等待使用者的事

← [AGENTS.md](../AGENTS.md)｜[INDEX.md](INDEX.md)

只列必須由使用者親自完成或驗證的 open 項目；完成即移除。

## 待使用者項

- 新接頭外形與關節姿態 GUI：先看 [修前／修後實際渲染](../docs/reports/geometry-2026-10-29/preview.html)，再雙擊根目錄 `run.bat`，依 README「先試一個姿勢」用 F2／→ 調到 +15°、F3 調到 +10°，確認叉耳／轉軸接法符合預期、HUD 可讀且移動後保持；再確認 P、Home、暫停、Backspace 與 1／2／3 切換可用。三個 headless CTest 與小集合幾何採樣已通過，實際模型靜態渲染已核對，但尚未實際驗證 GUI 鍵盤操作；通過後 agent 移除此項，若有差異依操作步驟與畫面修正。

