import asyncio
import contextlib
from xsenslsl.xsens.xsens_network_client import connect


if __name__ == "__main__":
    loop = asyncio.get_event_loop()

    connection = connect(host="192.168.207.187", port="9763", loop=loop)
    connection = loop.run_until_complete(connection)

    with contextlib.suppress(KeyboardInterrupt):
        loop.run_forever()

