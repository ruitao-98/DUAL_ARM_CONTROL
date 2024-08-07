from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
import time




class RobotiqGripper:
    def __init__(self, port, baudrate=115200):
        self.client = ModbusSerialClient(
            method='rtu',
            port=port,
            baudrate=baudrate,
            stopbits=1,
            bytesize=8,
            parity='N',
            timeout=1
        )
        connection = self.client.connect()

    def read_gripper_status(self, unit=0x09):
        try:
            # 读取从地址 0x07D0 开始的 3 个输入寄存器
            result = self.client.read_input_registers(address=0x07D0, count=3, unit=unit)
            
            if result.isError():
                raise ModbusException("读取输入寄存器时出错")
            
            # 解包结果
            gripper_status = result.registers[0]
            fault_status = result.registers[1]
            position_status = result.registers[2]
            
            gACT = (gripper_status & 0x01)
            gGTO = (gripper_status & 0x02) >> 1
            gOBJ = (gripper_status & 0x0C) >> 2
            position = position_status & 0xFF
            finger_current = (position_status & 0xFF00) >> 8
            
            print(f"gACT: {gACT}, gGTO: {gGTO}, gOBJ: {gOBJ}, 位置: {position}, 指电流: {finger_current}")
            return {
                "gACT": gACT,
                "gGTO": gGTO,
                "gOBJ": gOBJ,
                "Position": position,
                "Finger Current": finger_current
            }
        except ModbusException as e:
            print(f"Modbus 异常: {e}")
        except Exception as e:
            print(f"异常: {e}")


    def activate_gripper(self):
        # Activate the gripper (assuming default settings)
        self.client.write_register(0x03E8, 0x0000, unit=0x0009)  # Register 0x03E8 is the activation register, unit id 0x0009
        self.client.write_register(0x03E8, 0x0100, unit=0x0009)  # Register 0x03E8 is the activation register, unit id 0x0009
        time.sleep(1)  # Wait for the gripper to activate

    # def set_gripper_position(self, position):
    #     # Set the desired position (0-255)
    #     result =

    def open_gripper(self):
        self.client.write_registers(0x03E8, [0x0900,0x0000], unit=0x0009)
        time.sleep(1)  # Wait for the gripper to open

    def close_gripper(self):
        self.client.write_registers(0x03E8, [0x0900,0x00FF], unit=0x0009)
        time.sleep(1)  # Wait for the gripper to close

    def set_gripper_position(self, position):
        padded_hex = format(position, '#06x')
        self.client.write_registers(0x03E8, [0x0900, position], unit=0x0009)
        # self.read_gripper_status()
        time.sleep(1)  # Wait for the gripper to close

    def disconnect(self):
        self.client.close()

if __name__ == "__main__":
    try:
        gripper = RobotiqGripper("/dev/ttyUSB0")  # Replace with your serial port

        gripper.activate_gripper()
        print("Gripper activated")
        gripper.set_gripper_position(160)
        # gripper.close_gripper()
        # gripper.set_gripper_position(160)
        # time.sleep(10)
        # gripper.close_gripper()
        # print("Gripper closed")
        # gripper.open_gripper()
        # # print("Gripper opened")
        # gripper.set_gripper_position(160)

    finally:
        gripper.disconnect()
        print("Disconnected from gripper")