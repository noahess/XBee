from xbee import XBee
import struct
import time


class RocketControl(XBee):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, *kwargs)
        self._next_frame = 0x01
        self.rpm_setting = 15

    @staticmethod
    def auto_configure():
        xbees = XBee.get_connected()
        return RocketControl(xbees[0][0], 115200)

    @property
    def next_frame(self) -> int:
        """
        Get the next available frame number
        :return: The frame number
        :rtype: int
        """
        return_frame = self._next_frame
        self._next_frame += 1
        if self._next_frame > 0xFF:
            self._next_frame = 0x01
        return return_frame

    def send_message_to_rocket(self, message: bytes, frame_id=0) -> None:
        """
        Sends a message to the rocket
        :param message: The message to send
        :type message: bytes
        :param frame_id: The frame id (default 0)
        :type frame_id: int
        :return: None
        """
        self.transmit_request(frame_id, 0xFFFF, 0x00, 0x00, message)

    def set_stepper_speed(self, rpm: int) -> None:
        """
        Sets the stepper motor speed in revolutions per minute
        :param rpm: The revolutions per minute to spin at
        :type rpm: int
        :return: None
        """
        self.rpm_setting = rpm                                      # Update RPM calculation
        speed_select = b'\x03' + struct.pack('>H', rpm)
        self.send_message_to_rocket(b'\x03' + speed_select)

    def do_stepper_steps(self, num_steps: int) -> None:
        """
        Turns the stepper motor num_steps number of steps
        :param num_steps: The number of steps to turn
        :type num_steps: int
        :return: None
        """
        step_command = b'\x04' + struct.pack('>I', num_steps)
        self.send_message_to_rocket(step_command)
        # Update when Teensy will become unblocked
        self.motor_unblocked = time.time() + num_steps / 200 / (self.rpm_setting / 60) + 5 * self.frame_delay

    def get_angle(self) -> int:
        """
        Query rocket for angle data
        :return: The angle
        :rtype: int
        """
        self.clear_incoming()
        this_frame = self.next_frame
        self.send_message_to_rocket(b'\x02', frame_id=this_frame)
        message = self.discard_until_frame_id(this_frame)
        return struct.unpack(">h", message[4])[0]


if __name__ == "__main__":
    rocket = RocketControl.auto_configure()

    for speed in [200, 250, 300, 320]:
        rocket.set_stepper_speed(speed)
        rocket.do_stepper_steps(200)
        time.sleep(1)

    while True:
        try:
            print(rocket.get_angle())
        except KeyboardInterrupt:
            break
