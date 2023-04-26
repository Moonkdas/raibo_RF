from time import time
import asyncio
import serial
import serial_asyncio
# import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# subprocess.Popen(["sudo","chmod","777","/dev/ttyUSB0"], stdout=subprocess.PIPE, shell=True)


class SBUSReceiver:
    class SBUSFramer(asyncio.Protocol):

        START_BYTE = 0x0f
        END_BYTE = 0x00
        SBUS_FRAME_LEN = 25

        def __init__(self):
            super().__init__()
            self._in_frame = False
            self.transport = None
            self._frame = bytearray()
            self.frames = asyncio.Queue(maxsize=1)

        def connection_made(self, transport):
            self.transport = transport

        def data_received(self, data):
            for b in data:
                if self._in_frame:
                    self._frame.append(b)
                    if len(self._frame) == SBUSReceiver.SBUSFramer.SBUS_FRAME_LEN:
                        if self._frame[-1] == SBUSReceiver.SBUSFramer.END_BYTE:
                            decoded_frame = SBUSReceiver.SBUSFrame(self._frame)
                            if decoded_frame.get_failsafe_status() == SBUSReceiver.SBUSFrame.SBUS_SIGNAL_OK:
                                self.nice_state = True
                            else:
                                self.nice_state = False
                        else:
                            self.nice_state = False

                        asyncio.run_coroutine_threadsafe(self.frames.put(decoded_frame), asyncio.get_running_loop())
                        self._in_frame = False
                else:
                    if b == SBUSReceiver.SBUSFramer.START_BYTE:
                        self._in_frame = True
                        self._frame.clear()
                        self._frame.append(b)

        def connection_lost(self, exc):
            asyncio.get_event_loop().stop()

    class SBUSFrame:
        OUT_OF_SYNC_THD = 10
        SBUS_NUM_CHANNELS = 16
        SBUS_SIGNAL_OK = 0
        SBUS_SIGNAL_LOST = 1
        SBUS_SIGNAL_FAILSAFE = 2

        def __init__(self, frame):
            self.sbusChannels = [None] * SBUSReceiver.SBUSFrame.SBUS_NUM_CHANNELS

            channel_sum = int.from_bytes(frame[1:23], byteorder="little")

            for ch in range(0, 16):
                self.sbusChannels[ch] = channel_sum & 0x7ff
                channel_sum = channel_sum >> 11

            # Failsafe
            self.failSafeStatus = SBUSReceiver.SBUSFrame.SBUS_SIGNAL_OK
            if (frame[SBUSReceiver.SBUSFramer.SBUS_FRAME_LEN - 2]) & (1 << 2):
                self.failSafeStatus = SBUSReceiver.SBUSFrame.SBUS_SIGNAL_LOST
            if (frame[SBUSReceiver.SBUSFramer.SBUS_FRAME_LEN - 2]) & (1 << 3):
                self.failSafeStatus = SBUSReceiver.SBUSFrame.SBUS_SIGNAL_FAILSAFE

        def get_rx_channels(self):
            """
            Used to retrieve the last SBUS channels values reading
            :return:  an array of 18 unsigned short elements containing 16 standard channel values + 2 digitals (ch 17 and 18)
            """

            return self.sbusChannels

        def get_rx_channel(self, num_ch):
            """
            Used to retrieve the last SBUS channel value reading for a specific channel
            :param: num_ch: the channel which to retrieve the value for
            :return:  a short value containing
            """

            return self.sbusChannels[num_ch]

        def get_failsafe_status(self):
            """
            Used to retrieve the last FAILSAFE status
            :return:  a short value containing
            """

            return self.failSafeStatus

        def __repr__(self):
            return ",".join(str(ch) for ch in self.sbusChannels)

    def __init__(self):
        self._transport = None
        self._protocol = None
        self.nice_state = True

    @staticmethod
    async def create(port='/dev/ttyUSB1'):
        receiver = SBUSReceiver()
        receiver._transport, receiver._protocol = await serial_asyncio.create_serial_connection(
            asyncio.get_running_loop(),
            SBUSReceiver.SBUSFramer,
            port,
            baudrate=100000,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_TWO,
            bytesize=serial.EIGHTBITS)
        return receiver

    async def get_frame(self):
        return await self._protocol.frames.get()


# async def main():
#     sbus = await SBUSReceiver.create("/dev/ttyUSB0")
#     print('start')
#     while True:
#         frame = await sbus.get_frame()
#         print(frame)


# if __name__ == '__main__':
#     loop = asyncio.get_event_loop()
#     loop.run_until_complete(main())
#     loop.run_forever()
#     loop.close()

class JoyPublisher(Node):

    def __init__(self):
        super().__init__('joy_publisher')
        self.publisher_ = self.create_publisher(Joy, '/joy', 5)
        self.joy_msg = [0,0,0,0,0]

async def apple(JoyPublisher):
    sbus = await SBUSReceiver.create("/dev/ttyUSB0")
    print('start')

    while True:
        msg = Joy()
        msg.axes = [0. , 0. , 0. , 0. , 0. , 0. , 0. , 0.]
        # begin = time()
        
        frame = await sbus.get_frame()
        # print(frame)
        
        if sbus.nice_state:
            frame_str=str(frame).split(',')
            
            for i in range(4):
                if int(frame_str[i])-1023 > 600:
                    JoyPublisher.joy_msg[i]=1
                elif int(frame_str[i])-1023 < -600:
                    JoyPublisher.joy_msg[i]=-1
                elif int(frame_str[i])<30 & int(frame_str[i])>-30:
                    JoyPublisher.joy_msg[i]=0
                else:
                    JoyPublisher.joy_msg[i]=round((int(frame_str[i])-1023)/600,2)
                if i in [1,3]:
                    JoyPublisher.joy_msg[i]=-1*JoyPublisher.joy_msg[i]
            if int(frame_str[11])>1500:
                JoyPublisher.joy_msg[4]=-1
            elif int(frame_str[11])<500:
                JoyPublisher.joy_msg[4]=+1

        # print(self.joy_msg[0:5])
        
        msg.axes[0] = JoyPublisher.joy_msg[1]
        msg.axes[1] = JoyPublisher.joy_msg[0]
        msg.axes[3] = JoyPublisher.joy_msg[3]
        msg.axes[4] = JoyPublisher.joy_msg[2]
        msg.axes[7] = JoyPublisher.joy_msg[4]
        
        JoyPublisher.publisher_.publish(msg)

        # end = time()
        # print('실행 시간: {0:.3f}초'.format(end - begin))

def main():

    # asyncio.run(apple())
    rclpy.init()
    joy_publisher = JoyPublisher()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(apple(joy_publisher))
    loop.run_forever()
    loop.close()
    rclpy.shutdown()





if __name__ == '__main__':
    
    main()
# #     frame = await sbus.get_frame()
# #     print(frame)
# #     rclpy.shutdown()


# if __name__ == '__main__':
#     # asyncio.run(main())
#     loop = asyncio.get_event_loop()
#     loop.run_until_complete(main())
#     loop.run_forever()
#     loop.close()
