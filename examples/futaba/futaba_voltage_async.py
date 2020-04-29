#! /usr/bin/env python3
# encoding: utf-8

import sys
import logging
import asyncio

sys.path.insert(0, '../..')
from gs2d import SerialInterface, Futaba

# ログ設定
logging.basicConfig()
logging.getLogger('gs2d').setLevel(level=logging.DEBUG)


async def main(loop):
    try:
        # Initialize SerialInterface & servo object
        si = SerialInterface()
        futaba = Futaba(si)

        # Get voltage
        voltage = await futaba.get_voltage_async(sid=1)
        print('Voltage: %.2f(V)' % voltage)

        # Close SerialInterface & servo object
        futaba.close()
        si.close()
    except Exception as e:
        print('Error', e)


# Initialize event loop
lp = asyncio.get_event_loop()
lp.run_until_complete(main(lp))
lp.close()
