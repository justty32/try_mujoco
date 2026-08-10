# WORKFLOWS — 工作流派發器

← [AGENTS.md](../AGENTS.md)｜[專案地圖](INDEX.md)

依請求選工作流，先讀入口，再動手。

## 開發

| 觸發 | 工作流 | 入口 |
|------|--------|------|
| 開發、修改功能、修 bug | feature-dev | [workflows/feature-dev/README.md](workflows/feature-dev/README.md) |
| 建置、測試、驗證 | testing | [workflows/testing.md](workflows/testing.md) |
| 記錄或查詢踩坑 | gotchas | [workflows/common/gotchas.md](workflows/common/gotchas.md) |

碰原始碼時一併遵守 [程式碼慣例](workflows/common/conventions.md)。

## 研究與寫作

| 觸發 | 工作流 | 入口 |
|------|--------|------|
| 讀文件、比較資料、整理來源 | digest | [workflows/digest.md](workflows/digest.md) |
| 寫報告、README 或技術文件 | write | [workflows/write.md](workflows/write.md) |

產出給人閱讀的文字時一併參考 [寫作風格](workflows/common/writing.md)。都不符合時，回到 [INDEX.md](INDEX.md) 找對應區域。

## 工作流的統一形式

- 小工作流用單一 `.md` 同時作入口與內容；內容長大才拆成資料夾與 README。
- 長期知識歸到對應工作流；過時內容移入該工作流的 `archive/`。
- 跨工作流的 open 狀態放 [SESSION-LOG.md](SESSION-LOG.md)；需使用者親自處理的放 [WAIT_USER.md](WAIT_USER.md)。完成即移除。
- 結構需要拆分或分類時，才讀 [DEV-GUIDE.md](DEV-GUIDE.md)。

