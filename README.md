# gs2d: Generic Serial-bus Servo Driver library for Python

> karakuri products社製シリアルサーボドライバ kr-sac001 用の Python ライブラリ

---

## 機能

- [karakuri products社製シリアルサーボドライバ kr-sac001](https://github.com/karakuri-products) に接続されたサーボモータを Python で簡単にコントロールできるライブラリです。
- 様々なブランドのサーボモータをほぼ同じ関数で制御が可能です。
- Read系コマンドはBlockingスタイル、callbackスタイル、asyncスタイルで利用可能です。

## サポートしているサーボモータ

- 現在対応しているサーボモータ
    - FUTABA RS40x

- もうすぐ対応するサーボモータ
    - Dynamixel X

- 対応したいな、と思っているサーボモータ
    - KONDO KRS300x
    - KONDO KRS400x
    - JR programmable
    - Vstone Vs-xxx

## インストール方法

```
pip install gs2d
```

## 利用例

### フタバのサーボモータID:1をちょっとずつ動かす

```
from gs2d import SerialInterface, Futaba

# Initialize
si = SerialInterface()
futaba = Futaba(si)

# Enable torque
futaba.set_torque_enable(True, sid=1)

for i in range(11):
  angle = i * 20 - 100
  print('Angle:', angle, 'deg')
  futaba.set_target_position(angle, sid=1)
  time.sleep(0.5)

# Disable torque
futaba.set_torque_enable(False, sid=1)

# Close
futaba.close()
si.close()
```

### フタバのサーボモータID:1の電圧を取得する (Blocking版)

```
from gs2d import SerialInterface, Futaba

# 初期化
si = SerialInterface()
futaba = Futaba(si)

# 電圧取得
v = futaba.get_voltage(sid=1)
print('Voltage: %.2f(V)' % v)

# クローズ
futaba.close()
si.close()
```

### フタバのサーボモータID:1の電圧を取得する (コールバック版)

```
from gs2d import SerialInterface, Futaba

def voltage_callback(voltage):
    """電圧取得できたときに呼ばれる"""
    print('Voltage: %.2f(V)' % voltage)

    # クローズ
    futaba.close()
    si.close()

# 初期化
si = SerialInterface()
futaba = Futaba(si)

# コールバックつきで電圧取得
futaba.get_voltage(sid=1, callback=voltage_callback)
```

### フタバのサーボモータID:1の電圧を取得する (Async版)

```
import asyncio
from gs2d import SerialInterface, Futaba

async def main(loop):
    # Initialize SerialInterface & servo object
    si = SerialInterface()
    futaba = Futaba(si)

    # Get voltage
    voltage = await futaba.get_voltage_async(sid=1)
    print('Voltage: %.2f(V)' % voltage)

    # Close SerialInterface & servo object
    futaba.close()
    si.close()


# Initialize event loop
lp = asyncio.get_event_loop()
lp.run_until_complete(main(lp))
lp.close()
```



## API

### Torque Enable (get/set)

#### get_torque_enable

```
get_torque_enable(sid, callback=None)
```

- Parameters
  - sid: Servo ID
  - callback: TBD
- Response
  - None

```
get_torque_enable_async(sid, loop=None)
```

- Parameters
  - sid: Servo ID
  - loop: TBD
- Response
  - True (Torque ON)/False (Torque OFF)

#### set_torque_enable





## License
Generic Serial-bus Servo Driver library uses Apache License 2.0.
