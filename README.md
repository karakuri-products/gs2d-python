# gs2d: Generic Serial-bus Servo Driver library for Python

---

## Features

TBD

## Supported servo motors
- KONDO KRS300x
- KONDO KRS400x
- JR programmable
- FUTABA RS40x
- Dynamixel X
- Vstone Vs-xxx

## Installation

### Using pip

```
pip install gs2d
```

## How this library looks like

```
from gs2d import SerialInterface, Futaba

# Initialize
si = SerialInterface('/dev/tty.usbserial-A601X0TE')
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
