import numpy as np
import pyrealsense2 as rs
import pickle
import struct
import async_base as ab
from camera import CameraRos, CameraNotRos


class EtherSenseServer(ab.AsyncServer):
    def __init__(self, camera, address="localhost", port=8888):
        super().__init__(address=address, port=port)
        self.rsc = camera

    async def handle_read(self, reader, data=None):
        data = await reader.read(1)
        if len(data) < 1:
            return None
        command = int(data)
        if command not in ab.server_opt:
            print("Command cannot be recognized")
            return None
        return command

    async def handle_write(self, writer, data=None, readout=None):
        if readout is not None and not writer.is_closing():
            frame_data = b''
            if readout == ab.READ_RGB:
                frame_data= self.rsc.getRGBImg()
            elif readout == ab.READ_DEPTH:
                frame_data= self.rsc.getDepthImg()
            # elif readout == ab.READ_PTC:
            #     frame_data = self.rsc.getPointCloudStream()
            # elif readout == ab.READ_RGB_PTC:
            #     frame_data = self.rsc.getRGBPointCloudStream()
            else:
                raise Exception("Unknown Error")

            if len(frame_data) == 0:
                return
            writer.write(frame_data)
            await writer.drain()


# server = EtherSenseServer(address="192.168.1.8", port=18360)
# server.start_server()
