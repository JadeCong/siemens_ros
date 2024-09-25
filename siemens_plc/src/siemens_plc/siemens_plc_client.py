from modbus.modbus_wrapper_client import ModbusWrapperClient


NUM_REGISTERS = 200
ADDRESS_READ_START = 40001
ADDRESS_WRITE_START = 40101


class SiemensPlcClient(ModbusWrapperClient):
    def __init__(self, host, port=502, rate=50, reset_registers=True, sub_topic="modbus_wrapper/output",pub_topic="modbus_wrapper/input"):
        ModbusWrapperClient.__init__(self, host, port, rate, reset_registers, sub_topic, pub_topic)
        self.setReadingRegisters(ADDRESS_READ_START, NUM_REGISTERS)
        self.setWritingRegisters(ADDRESS_WRITE_START, NUM_REGISTERS)
