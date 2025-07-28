# CoT_pipeline

### sidc2cotype
    1. 輸入：SIDC 字串（如 "SFAPMF--------"）
    2. 驗證：檢查 SIDC 格式是否合法（sidc_validator）
    3. 解析：擷取 functionID，對應為 CoT type（sidc_parser）
    4. 建構：以 uid + CoT type + 時間戳建立 CoTInfo 物件
    5. 組裝：轉為 XML 字串（cot_builder）
    6. 驗證：確認組裝結果符合 CoT XML 規範（cot_validator）

### 流程圖(待補)

### 待補(待補)


### 貓貓(路過可以摸)
```
＿＿
　　　　　／＞　　フ
　　　　　|  　_　 _ l
　 　　　／` ミ＿꒳ノ
　　 　 /　　　 　 |
　　　 /　 ヽ　　 ﾉ
　 　 │　　|　|　|
　／￣|　　 |　|　|
　| (￣ヽ＿_ヽ_)__)
　＼二つ
```