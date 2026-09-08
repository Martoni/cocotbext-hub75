import cocotb
from cocotb_bus.monitors import BusMonitor

class Hub75Monitor(BusMonitor):
    """
    Monitors Hub75 led panel bus
    """
    _signals = [
       'R1', 'G1',
       'B1', #GND
       'R2', 'G2',
       'B2', #NC,
       'A',  'B',
       'C',  'D',
       'CLK','LATCH',
       'OEN' #GND
       ]

    def __init__(self, dut, name, clk):
        BusMonitor.__init__(self, dut, name, clk)
        self.clock = clk
        self.transactions = 0
