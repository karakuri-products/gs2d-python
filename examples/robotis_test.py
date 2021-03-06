#! /usr/bin/env python3
# encoding: utf-8

import sys
import traceback
import asyncio
import logging
import time

sys.path.insert(0, '..')
from gs2d import SerialInterface, RobotisP20

# ログ設定
logging.basicConfig()
logging.getLogger('gs2d').setLevel(level=logging.DEBUG)


async def main(loop):
    # 初期化
    # si = SerialInterface(baudrate=3000000)
    si = SerialInterface(baudrate=1000000)
    robotis = RobotisP20(si)

    sid = 1

    # ping = await robotis.ping_async(sid)
    # print(ping)

    robotis.set_torque_enable(True, sid=sid)

    time.sleep(0.5)

    # print('Current position:', robotis.get_current_position(sid))

    # print('Enable Torque?:', robotis.get_torque_enable(sid=sid))

    # 0.5秒ごとにサーボを動かす
    for i in range(11):
        # print('Current position:', robotis.get_current_position(sid))

        angle = i * 20 - 100
        print('Angle:', angle, 'deg')
        robotis.set_target_position(angle, sid=sid)
        # print('Target position:', robotis.get_target_position(sid))
        time.sleep(0.5)

    # クローズ
    robotis.close(force=False)
    si.close()


# Initialize event loop
lp = asyncio.get_event_loop()
lp.run_until_complete(main(lp))
lp.close()
