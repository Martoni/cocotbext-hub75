import os
from cocotb.types import LogicArray
from cocotb_bus.monitors import BusMonitor
from cocotb.triggers import RisingEdge, First


class Hub75Monitor(BusMonitor):
    """
    CocoTB monitor for HUB75 RGB LED panel bus.

    Captures pixel data shifted on the bus and assembles full frames
    based on the connected panel geometry. Frames can be saved as
    PPM (P3 ASCII) image files.

    The HUB75 bus carries two rows simultaneously:
      - RGB1: upper half pixels
      - RGB2: lower half pixels
      - ADDR:  row pair address
      - CLK:       shift clock
      - LATCH:     strobe latch (active high pulse at end of row)
      - OEN:       output enable (active low)

    Usage::

        monitor = Hub75Monitor(dut, "", dut.clk, width=64, height=32)

        # Option 1: callback on each complete frame
        monitor = Hub75Monitor(dut, "", dut.clk,
                               width=64, height=32,
                               callback=lambda frame: monitor.save_ppm("out.ppm"))

        # Option 2: wait for a frame
        frame = await monitor.get_frame()
        monitor.save_ppm("out.ppm", frame)
    """

    _signals = ['RGB1', 'RGB2', 'ADDR', 'CLK', 'LATCH', 'OEN']

    def __init__(self, dut, name, clk,
                 width=32, height=32,
                 callback=None, event=None):
        """
        Args:
            dut:            cocotb DUT handle
            name:           bus name prefix (signals resolved as {name}_{signal})
            clk:            clock signal handle
            width:          panel width in pixels (default 64)
            height:         panel height in pixels (default 32)
            callback:       optional callable invoked with each complete frame
            event:          optional cocotb Event set on each complete frame
        """
        BusMonitor.__init__(self, dut, name, clk,
                            callback=callback, event=event)

        self.width = width
        self.height = height

        self._row_pairs = height // 2
        self._addr_bits = (self._row_pairs - 1).bit_length()

        self._rgb1_signals = ['RGB1']
        self._rgb2_signals = ['RGB2']
        self._addr_signals = ['ADDR']
        self._clk_signals = ['CLK']
        self._frame_buffer = [
                                [LogicArray('000') for _ in range(self.width)]
                                for _ in range(self.height)
                             ]
        self._row_buf1 = []
        self._row_buf2 = []
        self._rows_written = []
        self._frame_count = 0

        self._check_signals()

    def _check_signals(self):
        missing = []
        for sig in self._signals:
            if not hasattr(self.bus, sig):
                missing.append(sig)
        if missing:
            raise AttributeError(
                f"Missing HUB75 signals on DUT bus: {missing}")

    async def _monitor_recv(self):
        while True:
            clk_trig = RisingEdge(self.bus.CLK)
            latch_trig = RisingEdge(self.bus.LATCH)

            trigged = await First(clk_trig, latch_trig)

            if self.in_reset:
                self._reset_buffers()
                continue

            rgb1 = self.bus.RGB1.value
            rgb2 = self.bus.RGB2.value
            oen = self.bus.OEN.value
            latch = self.bus.LATCH.value

            if (trigged is clk_trig) and oen == '0':
                self._row_buf1.append(rgb1)
                self._row_buf2.append(rgb2)

            if trigged is latch_trig:
                addr = self._read_address()
                self._write_latched_row(addr)

            if self._frame_complete():
                print(f"Frame count {self._frame_count}")
                self._frame_count += 1
                self._recv(self._copy_frame())
                self._reset_frame_complete()

    def _read_address(self):
        addr = 0
        for i in range(min(self._addr_bits, len(self._addr_signals))):
            bit = int(getattr(self.bus, self._addr_signals[i]).value)
            addr |= bit << i
        return addr

    def _write_latched_row(self, addr):
        row_top = addr
        row_bot = 0x10 + addr

        if len(self._row_buf1) >= self.width:
            for x in range(self.width):
                self._frame_buffer[row_top][x] = self._row_buf1[x]
            self._row_buf1 = []
            self._rows_written.append(row_top)
            for x in range(self.width):
                self._frame_buffer[row_bot][x] = self._row_buf2[x]
            self._row_buf2 = []
            self._rows_written.append(row_bot)

    def _frame_complete(self):
        return len(set(self._rows_written)) == self.height

    def _reset_frame_complete(self):
        self._rows_written = []

    def _reset_buffers(self):
        self._row_buf1 = []
        self._row_buf2 = []
        self._rows_written = []

    def _copy_frame(self):
        return [[pixel[:] for pixel in row] for row in self._frame_buffer]

    def display_last_ascii(self):
        display_ascii(self._frame_buffer)

    def display_ascii(self, frame):
        """
        Display  in ascii
        """
        for row in frame:
            for pix in row:
                print(f"{pix} ", end="")
            print("")

    def save_ppm(self, filepath, frame=None):
        """
        Save a frame as PPM (P3 ASCII) image.

        Args:
            filepath: output file path
            frame:    frame data (list of rows of [R, G, B] triples).
                      If None, saves the current frame buffer.
        """
        if frame is None:
            frame = self._frame_buffer

        os.makedirs(os.path.dirname(filepath) or '.', exist_ok=True)

        with open(filepath, 'w') as f:
            f.write(f"P3\n{self.width} {self.height}\n255\n")
            pixels = ''
            for row in frame:
                for pix in row:
                    r = 255 if pix[2] == '1' else 0
                    g = 255 if pix[1] == '1' else 0
                    b = 255 if pix[0] == '1' else 0
                    pixels += f'{r} {g} {b} '
                f.write(pixels + '\n')
                pixels = ''

    async def get_frame(self):
        """Wait for and return the next complete frame."""
        return await self.wait_for_recv()

    def get_pixel(self, x, y):
        """Get pixel color [R, G, B] at (x, y) from current frame buffer."""
        return self._frame_buffer[y][x]

    @property
    def frame_count(self):
        """Number of complete frames captured."""
        return self._frame_count

    def reset_frame(self):
        """Reset the frame buffer to black."""
        self._frame_buffer = [[LogicArray('000') for _ in range(self.width)]
                                          for _ in range(self.height)]
