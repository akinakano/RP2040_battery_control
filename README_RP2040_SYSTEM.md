# RP2040 Battery Control System

## 概要
RP2040-zeroマイコンボードを使用したバッテリー制御システムです。SMBUSプロトコルによるバッテリー通信、1Hz JSON形式でのMPAPプロトコル送信、WS2812 LEDハートビート機能を実装しています。

## ハードウェア構成

### マイコンボード
- **RP2040-zero** (Waveshare製)
- **GPIO26**: I2C SDA (バッテリー通信用)
- **GPIO27**: I2C SCL (バッテリー通信用)
- **GPIO16**: WS2812 LED (ハートビート表示用)

### バッテリー
- **I2Cアドレス**: 0x0B
- **プロトコル**: SMBUS
- **通信レート**: 100kHz

## 主要機能

### 1. バッテリー制御 (`battery_control`)
- SMBUSプロトコルによるバッテリー通信
- 100ms間隔でバッテリー情報取得
- 以下の情報を読み取り：
  - 電流 (mA)
  - 電圧 (mV)
  - 残量 (%)
  - 温度 (0.1K)
  - 製造日
  - シリアル番号
  - 製造者アクセス情報

### 2. MPAPプロトコル (`protocol_mpap`)
- **送信レート**: 1Hz (1000ms間隔)
- **データ形式**: JSON
- **送信方式**: Serial.println() (テキスト)

#### JSON形式
```json
{
  "payload_battery": {
    "count": 123,
    "id": 1,
    "battery_current_ma": -1500,
    "battery_voltage_mv": 3700,
    "battery_capacity": 85,
    "battery_temperature": 2981,
    "battery_mfg_date": 12345,
    "battery_serial": 67890,
    "battery_mfg_access": 0,
    "battery_valid": 1,
    "system_status": 1,
    "timestamp": 456789
  }
}
```

#### JSONフィールド説明
| フィールド | 型 | 説明 |
|-----------|-----|------|
| payload_battery | object | バッテリーデータオブジェクト |
| payload_battery.count | number | 送信回数（自動増加） |
| payload_battery.id | number | マイコンID (1 or 2) |
| payload_battery.battery_current_ma | number | バッテリー電流 (mA) |
| payload_battery.battery_voltage_mv | number | バッテリー電圧 (mV) |
| payload_battery.battery_capacity | number | バッテリー残量 (%) |
| payload_battery.battery_temperature | number | バッテリー温度 (0.1K) |
| payload_battery.battery_mfg_date | number | 製造日 |
| payload_battery.battery_serial | number | シリアル番号 |
| payload_battery.battery_mfg_access | number | 製造者アクセス |
| payload_battery.battery_valid | number | バッテリーデータ有効フラグ (0/1) |
| payload_battery.system_status | number | システムステータス |
| payload_battery.timestamp | number | システム起動からの時間 (ms) |

### 3. WS2812 LEDハートビート (`heartbeat_led`)
- **ピン**: GPIO16
- **ライブラリ**: Adafruit NeoPixel
- **点滅間隔**: 1秒 (1Hz)
- **輝度**: 50/255

#### LED色分け
| 色 | 条件 | 16進値 |
|----|------|--------|
| **緑色** | バッテリー残量 > 50% | 0xFF0000 |
| **黄色** | バッテリー残量 20-50% | 0xFFFF00 |
| **赤色** | バッテリー残量 < 20% | 0x00FF00 |
| **白色** | バッテリー通信エラー | 0xFFFFFF |
| **青色** | 初期化時 | 0x0000FF |

#### 初期化パターン
起動時に青色で3回点滅して初期化完了を表示

### 4. USB通信 (`usb_communication`)
- **ボーレート**: 115200bps
- デバッグ情報出力
- コマンド受信機能

## ビルド環境

### PlatformIO設定
```ini
[env:waveshare_rp2040_zero]
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
board = waveshare_rp2040_zero
framework = arduino

build_flags = 
    -DPIN_WIRE_SDA=26
    -DPIN_WIRE_SCL=27

lib_deps = 
    adafruit/Adafruit NeoPixel@^1.10.4
```

## ファイル構成

### インクルードファイル (`include/`)
- `battery_control.h` - バッテリー制御定義
- `smbus_rp2040.h` - SMBUS通信定義
- `usb_communication.h` - USB通信定義
- `protocol_mpap.h` - MPAPプロトコル定義
- `heartbeat_led.h` - LEDハートビート定義

### ソースファイル (`src/`)
- `main.cpp` - メインループ
- `battery_control.cpp` - バッテリー制御実装
- `smbus_rp2040.cpp` - SMBUS通信実装
- `usb_communication.cpp` - USB通信実装
- `protocol_mpap.cpp` - MPAPプロトコル実装
- `heartbeat_led.cpp` - LEDハートビート実装

## 動作タイミング

| 機能 | 実行間隔 | 説明 |
|------|----------|------|
| バッテリー通信 | 100ms | SMBUS読み取り |
| MPAP送信 | 1000ms (1Hz) | JSON形式データ送信 |
| LED点滅 | 1000ms (1Hz) | ハートビート表示 |
| デバッグ出力 | 30秒 | システム稼働確認 |

## 使用方法

1. **ハードウェア接続**
   - バッテリーをI2C（GPIO26/27）に接続
   - WS2812 LEDをGPIO16に接続

2. **ビルド・書き込み**
   ```bash
   pio run
   pio run --target upload
   ```

3. **動作確認**
   - シリアルモニター（115200bps）でログ確認
   - LEDの色でバッテリー状態確認
   - APでJSON形式のMPAPデータ受信確認

## トラブルシューティング

### バッテリー通信エラー
- I2C配線確認（SDA: GPIO26, SCL: GPIO27）
- バッテリーアドレス確認（0x0B）
- プルアップ抵抗確認

### LED表示異常
- GPIO16配線確認
- WS2812電源確認
- 色定義値確認

### MPAP送信エラー
- シリアル通信確認
- JSON形式確認
- データパース確認

## 開発履歴
- 初期実装: バッテリー制御機能
- 追加機能: MPAPプロトコル送信（バイナリ形式、50Hz）
- 追加機能: WS2812 LEDハートビート
- バグ修正: LED色定義の修正
- 機能変更: MPAPプロトコルをJSON形式に変更（1Hz）
- 追加機能: マイコンID識別機能（count, id フィールド）
- 構造変更: JSONデータをpayload_batteryオブジェクト内に階層化