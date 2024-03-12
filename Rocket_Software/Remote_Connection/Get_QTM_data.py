"""
Example that takes control of QTM, starts a measurement and stops after
10 seconds, then saves the measurement data to a file.
"""

import asyncio
import qtm
import numpy as np

test = asyncio.Event()
async def setup():
    """ 
    Main function that connects to the QTM server, initiates a measurement
    process, and saves the captured data to a file.
    """
    connection = await qtm.connect("138.250.154.168")
    Storage_position = []
    Storage_rotation = []

    if connection is None:
        return -1

    async with qtm.TakeControl(connection, "gdp-return"):
        state = await connection.get_state()
        if state != qtm.QRTEvent.EventConnected:
            await connection.new()
            try:
                await connection.await_event(qtm.QRTEvent.EventConnected, timeout=10)
            except asyncio.TimeoutError:
                return -1
            
        # Start the measurement
        await connection.start()
        await connection.await_event(qtm.QRTEvent.EventCaptureStarted, timeout=10)

        def on_packet(packet):
            info, bodies = packet.get_6d()
            for position, rotation in bodies:
                Storage_position = position
                Storage_rotation = rotation
            
        await connection.stream_frames(components=["6d"], on_packet=on_packet)
        
        await test.wait()
        
        # Wait for QTM to measure for 10 seconds
        # await asyncio.sleep(10)
        await connection.stream_frames_stop()
        # Stop the measurement
        await connection.stop()
        await connection.await_event(qtm.QRTEvent.EventCaptureStopped, timeout=10)
        
        # Save the measurement to a file
        await connection.save("Demo.qtm")

    connection.disconnect()

def position_offset(position,offset_x : float = 0, offset_y : float = 0, offset_z : float = 0):
    new_position = np.array([position.x + offset_x,position.y + offset_y, position.z + offset_z])
    return new_position


def check_position(pos, tolerance = 0.1, x_min = 0, x_max = 100, y_min = 0, y_max = 100, z_min = 0, z_max = 100):
    min_values = [x_min,y_min,z_min]
    max_values = [x_max,y_max,z_max]
    position = [pos.x,pos.y,pos.z]
    for i in range(len(min_values)):
        if position[i] < (1+tolerance) * min_values[i] or pos > (1-tolerance)*max_values[i]:
            return "Out of Bound"
    return    



if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(setup())