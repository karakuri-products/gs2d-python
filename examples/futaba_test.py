#! /usr/bin/env python3
# encoding: utf-8

import sys
import asyncio
import logging

sys.path.insert(0, '../..')
from gs2d import SerialInterface, Futaba

# ログ設定
logging.basicConfig()
logging.getLogger('gs2d').setLevel(level=logging.DEBUG)


async def main(loop):
    try:
        # 初期化
        si = SerialInterface(device='/dev/tty.usbserial-A601X0TE')
        futaba = Futaba(si)

        # futaba.reset_memory(sid=1)

        futaba.set_torque_enable(False, sid=1)
        # futaba.set_baud_rate(Futaba.BAUD_RATE_INDEX_115200, sid=1)
        # futaba.set_servo_id(1, sid=2)
        # futaba.set_limit_cw_position(150, sid=1)
        # futaba.set_limit_ccw_position(-10, sid=1)
        # futaba.write_flash_rom(sid=1)

        # futaba.set_pid_coefficient(255, sid=1)
        # futaba.set_speed(0, sid=1)

        futaba.set_torque_enable(True, sid=1)
        futaba.set_target_position(120, sid=1)
        await asyncio.sleep(3)
        # futaba.set_target_position(-120, sid=1)
        futaba.set_burst_target_positions({
            1: -120
        })

        # v = futaba.get_pid_coefficient(sid=1)
        # await asyncio.sleep(0.1)
        # data = futaba.get_limit_ccw_position(sid=1)
        # data = futaba.get_limit_cw_position(sid=1)
        # data = futaba.get_limit_temperature(sid=1)
        # data = futaba.get_servo_id(sid=1)
        # data = futaba.get_target_position(sid=2)
        # v = await futaba.get_pid_coefficient_async(sid=1, loop=loop)
        # print('######', data)

        # クローズ
        futaba.close(force=False)
        si.close()

    except Exception as e:
        print('Error', e)


# Initialize event loop
lp = asyncio.get_event_loop()
lp.run_until_complete(main(lp))
lp.close()
