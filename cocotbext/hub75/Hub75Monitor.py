import os
from cocotb_bus.monitors import BusMonitor
from cocotb.triggers import RisingEdge


class Hub75Monitor(BusMonitor):
    """
    CocoTB monitor for HUB75 RGB LED panel bus.

    Captures pixel data shifted on the bus and assembles full frames
    based on the connected panel geometry. Frames can be saved as
    PPM (P3 ASCII) image files.

    The HUB75 bus carries two rows simultaneously:
      - R1/G1/B1: upper half pixels
      - R2/G2/B2: lower half pixels
      - A/B/C/D:  row pair address
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

    _signals = [
        'R1', 'G1', 'B1',
        'R2', 'G2', 'B2',
        'A', 'B', 'C', 'D',
        'CLK', 'LATCH', 'OEN',
    ]

    def __init__(self, dut, name, clk,
                 width=64, height=32, rows_per_address=1,
                 callback=None, event=None):
        """
        Args:
            dut:            cocotb DUT handle
            name:           bus name prefix (signals resolved as {name}_{signal})
            clk:            clock signal handle
            width:          panel width in pixels (default 64)
            height:         panel height in pixels (default 32)
            rows_per_address: number of row pairs per address combination
                              (typically 1 for standard panels)
            callback:       optional callable invoked with each complete frame
            event:          optional cocotb Event set on each complete frame
        """
        BusMonitor.__init__(self, dut, name, clk,
                            callback=callback, event=event)

        self.width = width
        self.height = height
        self.rows_per_address = rows_per_address

        self._row_pairs = height // 2
        self._addr_bits = (self._row_pairs - 1).bit_length()

        self._rgb1_signals = ['R1', 'G1', 'B1']
        self._rgb2_signals = ['R2', 'G2', 'B2']
        self._addr_signals = ['A', 'B', 'C', 'D']

        self._frame_buffer = [[[0, 0, 0] for _ in range(width)]
                              for _ in range(height)]
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
            await RisingEdge(self.clock)

            if self.in_reset:
                self._reset_buffers()
                continue

            r1 = int(self.bus.R1.value)
            g1 = int(self.bus.G1.value)
            b1 = int(self.bus.B1.value)
            r2 = int(self.bus.R2.value)
            g2 = int(self.bus.G2.value)
            b2 = int(self.bus.B2.value)
            oen = int(self.bus.OEN.value)

            if not oen:
                self._row_buf1.append([r1, g1, b1])
                self._row_buf2.append([r2, g2, b2])

            latch = int(self.bus.LATCH.value)
            if latch:
                addr = self._read_address()
                self._write_latched_row(addr)

            if self._frame_complete():
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
        row_top = addr * self.rows_per_address
        row_bot = row_top + self._row_pairs

        if len(self._row_buf1) >= self.width:
            for x in range(self.width):
                self._frame_buffer[row_top][x] = self._row_buf1[x][:]
            self._row_buf1 = []
            self._rows_written.append(row_top)

        if len(self._row_buf2) >= self.width:
            for x in range(self.width):
                self._frame_buffer[row_bot][x] = self._row_buf2[x][:]
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
            for row in frame:
                pixels = ' '.join(f'{r * 255} {g * 255} {b * 255}'
                                  for r, g, b in row)
                f.write(pixels + '\n')

    async def get_frame(self):
        """Wait for and return the next complete frame."""
        return await self.wait_for_recv()

    def get_pixel(self, x, y):
        """Get pixel color [R, G, B] at (x, y) from current frame buffer."""
        return self._frame_buffer[y][x][:]

    @property
    def frame_count(self):
        """Number of complete frames captured."""
        return self._frame_count

    def reset_frame(self):
        """Reset the frame buffer to black."""
        self._frame_buffer = [[[0, 0, 0] for _ in range(self.width)]
                              for _ in range(self.height)]
