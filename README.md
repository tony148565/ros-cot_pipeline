# CoT_pipeline

## AI 協作說明

部分本專案的內容（如說明文字、程式碼片段）由 OpenAI ChatGPT 協助生成，並經作者人工審核與整合。

## 所以ROS的部分呢

現在還沒接上，目前以sidc2cot與cot xml為主

### sidc2cotype
    1. 輸入：SIDC 字串（如 "SFAPMF--------"）
    2. 驗證：檢查 SIDC 格式是否合法（sidc_validator）
    3. 解析：擷取 functionID，對應為 CoT type（sidc_parser）
    4. 建構：以 uid + CoT type + 時間戳建立 CoTInfo 物件
    5. 組裝：轉為 XML 字串（cot_builder）
    6. 驗證：確認組裝結果符合 CoT XML 規範（cot_validator）

### 流程圖(待補)

### 代辦事項
1. wintak可解析cot event，但部分欄位無法渲染
2. type的定義"理論上"沒錯，但wintak只吃前三位
3. 補文件
4. 餵貓

### 貓貓(路過可以摸)
```
  
　　　　　／＞　　フ
　　　　　|  　_　 _l
　 　　　／` ミ＿꒳ノ
　　 　 /　　　 　 |
　　　 /　 ヽ　　 ﾉ
　 　 │　　|　|　|
　／￣|　　 |　|　|
　| (￣ヽ＿_ヽ_)__)
　＼二つ
```

### 參照
1. [FreeTAKServer文件](https://freetakteam.github.io/FreeTAKServer-User-Docs)
2. [spatialillusions github](https://github.com/spatialillusions/mil-std-2525/blob/master/tsv-tables)

### License
This project is licensed under the [MIT License](LICENSE).