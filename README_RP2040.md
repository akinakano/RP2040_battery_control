# RP2040-zero Battery Control System

STM32H7のPRIMO4 MPからRP2040-zeroに移植したバッテリ制御システム

## 機能

- **バッテリ制御**: SMBus（I2C）通信によるスマートバッテリ制御
- **USB通信**: APとのUSB CDC通信によるバッテリ状態の送信
- **リアルタイム監視**: バッテリ電圧、電流、容量、温度の監視

## ハードウェア構成

- **マイコン**: RP2040-zero
- **I2C通信**: GPIO26 (SDA), GPIO27 (SCL)
- **USB通信**: USB CDC (シリアル通信)
- **バッテリ**: SMBusプロトコル対応スマートバッテリ

## 開発環境

- **PlatformIO + Visual Studio Code**
- **フレームワーク**: Arduino
- **プラットフォーム**: Raspberry Pi Pico

## ビルド・書き込み方法

1. PlatformIOをVS Codeにインストール
2. プロジェクトフォルダを開く
3. `PlatformIO: Build` でビルド
4. RP2040-zeroをBOOTSELモードで接続
5. `PlatformIO: Upload` で書き込み

## 使用方法

### シリアルコマンド

USB接続後、シリアルモニター（115200 baud）で以下のコマンドが使用可能：

- `ping` - 通信テスト
- `status` - バッテリ状態を表示（JSON形式）
- `cfet_on` - バッテリCFET有効化
- `cfet_off` - バッテリCFET無効化
- `help` - ヘルプ表示

### JSON出力例

```json
{
  "current_ma": -1250,
  "voltage_mv": 12600,
  "capacity_percent": 85,
  "temperature_celsius": 23.5,
  "manufacture_date": 21345,
  "serial_number": 1234,
  "manufacturer_access": 0,
  "data_valid": true
}
```

## ファイル構成

```
├── platformio.ini          # PlatformIO設定
├── src/
│   ├── main.cpp            # メイン処理
│   ├── battery_control.cpp # バッテリ制御
│   ├── smbus_rp2040.cpp    # SMBUS通信
│   └── usb_communication.cpp # USB通信
└── include/
    ├── battery_control.h
    ├── smbus_rp2040.h
    └── usb_communication.h
```

## 移植内容

STM32版から以下の機能を移植：

- **comm_battery.c** → **battery_control.cpp**
- **smbus.c** → **smbus_rp2040.cpp**  
- **power_control.c** → 一部機能を統合
- **USB CDC通信** → **usb_communication.cpp**

## 注意事項

- GPIO26/27がI2C専用に使用されます
- バッテリアドレス: 0x0B
- SMBUS通信速度: 100kHz
- USB Serial通信速度: 115200 baud
