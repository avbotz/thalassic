"""Thin wrapper around a pyserial port in raw, non-blocking mode.

Reads never block (they return whatever is currently buffered); writes are
best-effort. Mirrors the old C++ SerialPort RAII wrapper.
"""

import serial


class SerialPort:
    """Non-blocking serial (tty) device wrapper backed by pyserial."""

    def __init__(self, device: str, baud: int):
        """Open ``device`` at ``baud``. Raises serial.SerialException on failure."""
        # timeout=0 makes read() return immediately with whatever is buffered;
        # write_timeout=0 keeps writes non-blocking (best-effort).
        self._port = serial.Serial(
            port=device,
            baudrate=baud,
            timeout=0,
            write_timeout=0,
            rtscts=False,
        )
        self._port.reset_input_buffer()
        self._port.reset_output_buffer()
        self.last_error = ""

    def write(self, data: str) -> bool:
        """Write ``data`` verbatim (ASCII). Returns False on a write error."""
        try:
            self._port.write(data.encode("ascii"))
            return True
        except (serial.SerialException, OSError) as exc:
            self.last_error = f"write(): {exc}"
            return False

    def read_available(self) -> bytes | None:
        """Return any buffered bytes (possibly empty); None on read error / disconnect."""
        try:
            waiting = self._port.in_waiting
            if waiting:
                return self._port.read(waiting)
            return b""
        except (serial.SerialException, OSError) as exc:
            self.last_error = f"read(): {exc}"
            return None

    def close(self) -> None:
        """Close the underlying port if open."""
        if self._port is not None and self._port.is_open:
            self._port.close()
