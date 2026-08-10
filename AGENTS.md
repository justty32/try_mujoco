# robot — AI agent 專案備忘

robot 是以 C11 封裝 MuJoCo、用於機器手臂物理模擬與演算法實驗的專案。本檔只做最上層路由；細節收在 `wf/`。

## 先讀哪裡

- 動手修改、驗證或撰寫報告：先讀 [wf/WORKFLOWS.md](wf/WORKFLOWS.md)。
- 了解 repo 結構：先讀 [wf/INDEX.md](wf/INDEX.md)。

## Always-on 鐵律

1. 修改必須保持未要求變更的行為；改完至少執行 `cmd /c build.bat`，模型變更還要實際經 MuJoCo 載入驗證。
2. 不覆蓋或回復使用者既有的未提交變更；修改前先看 `git status` 與相關 diff。
3. 未經使用者確認，不 push、不建立與當前請求無關的新工作。
4. 工作流細節與長期知識放在 `wf/` 對應層級，不堆在本檔。

## 活狀態

- 進行中：[wf/SESSION-LOG.md](wf/SESSION-LOG.md)
- 等使用者親自驗證：[wf/WAIT_USER.md](wf/WAIT_USER.md)

