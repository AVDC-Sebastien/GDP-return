import asyncio
stop = False
async def looper():
    for i in range(1_000_000_000):
        print(f'Printing {i}')
        await asyncio.sleep(0.5)
        if stop:
         break

async def main():
    print('Starting')
    future = asyncio.ensure_future(looper())

    print('Waiting for a few seconds')
    await asyncio.sleep(4)

    print('Cancelling')
    stop = True

    print('Done')

asyncio.get_event_loop().run_until_complete(main())