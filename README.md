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

### 流程圖
#### SIDC
![SIDC](https://raw.githubusercontent.com/tony148565/ros-cot_pipeline/master/img/sidc_flow_chart.png)

#### CoT
![CoT](https://raw.githubusercontent.com/tony148565/ros-cot_pipeline/master/img/cot_flow_chart.png)

### WinTAK的匯入流程
    1. 打開WinTAK
    2. 左上角選單-> import file匯入
    3. cot event可以使用[TAK_MINIMAL_BARE.cot](example/TAK_MINIMAL_BARE.cot)
    4. 畫面上會出現"UAV001"的"a-f-A"物件

### 代辦事項
1. 補文件
2. 解釋WinTAK的解析方式? 為甚麼有沒有detail不影響匯入，保留字有哪些
3. 餵貓

### 貓貓(路過可以摸)
```
  
　　　　　／＞　　フ
　　　　　|  　_　 _l
　 　　　／` ミ＿꒳ノ
　　 　 /　　　 　 |                   
　　　 /　 ヽ　　 ﾉ                  
　 　 │　　|　|　|          
　／￣|　　 |　|　|         <o)><|      
　| (￣ヽ＿_ヽ_)__)       /￣￣￣￣\    
　＼二つ                  ￣￣￣￣￣
```

### 參照
1. [FreeTAKServer文件](https://freetakteam.github.io/FreeTAKServer-User-Docs)
2. [spatialillusions github](https://github.com/spatialillusions/mil-std-2525/blob/master/tsv-tables)

### License
This project is licensed under the [MIT License](LICENSE).