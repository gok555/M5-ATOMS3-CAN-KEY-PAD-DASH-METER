M5Atom CAN Keypad & Dash Meter (Ver 3.81)
M5Atom S3 と ATOM CAN Base を使用した、Web Bluetooth API ベースの多機能車載コントローラー & メーターアプリです。

🌟 主な機能 / Features
1. 8ボタン・カスタマイズ可能なCANキーパッド
Momentary / Toggle モード: ボタンごとに動作を選択可能。

自由なCAN ID/データ設定: ON時とOFF時の送信データをHex単位で設定。

Endian切替: ビット反転（Little/Big Endian）にも対応。

生体認証ロック: WebAuthnを利用し、指紋認証や顔認証を通さないと操作できない「安全ボタン」を設定可能。

2. 8スロット・マルチメーター
スロット1 (固定): K-Meterユニット等の排気温度監視。

スロット2-8 (汎用): 最大7つのCANメッセージを同時にリアルタイム監視。

高度な計算式: 読み取ったRAW値に対し、係数(Mul)・オフセット(Add)・小数点桁数(Dec)を指定可能。

ビジュアル・アラーム: 設定値を超えた（または下回った）場合に画面が点滅し、警告を通知。

3. 音声操作 & 音声フィードバック
Voice Control: 「〇〇をオン」「キーパッドを表示」といった音声コマンドで操作。

Voice Feedback: アラーム発生時に「排気温度、警告！」と日本語で読み上げ。

4. 設定の保存と復元
NVS (Non-Volatile Storage): IDや設定はM5Atomの内部メモリに保存され、再起動後も即座に監視を再開。

Config Backup: HTMLアプリから設定全体をJSONファイルとしてエクスポート/インポート可能。

🛠 ハードウェア / Hardware Requirements
Main Unit: M5Atom S3

Base Unit: ATOM CAN Base (Unit CAN)

Sensor (Optional): K-Meter Unit (I2C)

Vehicle: CANバス搭載車両（Subaru WRX STI VAB / Suzuki Jimny JB64等での動作を想定）

🚀 使い方 / Usage
M5Atom S3 への書き込み:

main.py を M5Atom S3 に転送します。

Webアプリの起動:

Chrome / Edge 等の Web Bluetooth 対応ブラウザで index.html を開きます。

接続:

[CONNECT START] ボタンを押し、M5AtomS3_CAN_Base を選択します。

設定:

⚙️ アイコンから、各ボタンのCAN IDやメーターの計算式を設定し、[SET] ボタンで本体へ保存します。

📡 コマンド仕様 / BLE Protocol
デバイスへの設定コマンドは以下のフォーマットで送信されます。

メーター設定: SET_RX=slotIndex,canID(dec),byteIndex,mode

ボタン設定: CFGBTN=btnIndex,onVal,offVal

ID設定: ID=hexID / KID=hexID

⚖️ 免責事項 / Disclaimer
本プロジェクトは個人のDIYプロジェクトです。CAN通信による車両操作はリスクを伴います。本ソフトウェアの使用により生じた車両の故障、事故、損害等について、作者は一切の責任を負いません。自己責任でご使用ください。

This project is a personal DIY project. Operating a vehicle via CAN bus involves risks. Use at your own risk.
