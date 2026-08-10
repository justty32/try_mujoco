# feature-dev — 功能開發與修 bug

← [WORKFLOWS.md](../../WORKFLOWS.md)｜[INDEX.md](../../INDEX.md)

修改前先讀 [程式碼慣例](../common/conventions.md)，並確認 `git status`，避免覆蓋使用者既有變更。

## 流程

1. 從現行程式碼或模型重現問題，界定預期行為。
2. 做最小但完整的增量修改；介面、模型、文件保持一致。
3. 依 [testing.md](../testing.md) 重新建置並做模型／runtime 驗證。
4. 需要實機或人工視覺判斷時，先完成可自動驗證的部分，再記入 [WAIT_USER.md](../../WAIT_USER.md)。
5. 變更主要檔案職責或結構時，更新 code map 與使用文件。

