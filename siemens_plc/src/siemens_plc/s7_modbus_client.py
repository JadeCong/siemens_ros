from modbus.modbus_wrapper_client import ModbusWrapperClient


class S7ModbusClient(ModbusWrapperClient):
    def __init__(self, host, port=502, rate=50, reset_registers=True):
        """
            :param host: Contains the IP adress of the modbus server
            :type host: string
            :param rate: How often the registers on the modbusserver should be read per second
            :type rate: float
            :param reset_registers: Defines if the holding registers should be reset to 0 after they have been read. Only possible if they are writeable
            :type reset_registers: bool
        """
        # print("Use the appropriate Step7 Project to enable the Modbus Server on your Siemens S1200 PLC")
        ModbusWrapperClient.__init__(self, host, port, rate, reset_registers)
        self.setReadingRegisters(0, 8)
        self.setWritingRegisters(8, 9)
        # self.startListening()
