"""Serial and same-controller demonstration transports, run outside Tk's thread."""
from __future__ import annotations
import ctypes
import pathlib
import queue
import threading
import time
from .protocol import JsonLineDecoder


class Transport:
    def __init__(self, events):
        self.events = events
        self.commands = queue.Queue(maxsize=128)
        self.stopping = threading.Event()
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def send(self, command):
        if "\n" in command or "\r" in command or len(command) > 128:
            raise ValueError("invalid outgoing command")
        self.commands.put_nowait(command)

    def emit(self, event):
        # Do not block the serial reader if plotting cannot keep up.
        try:
            self.events.put_nowait(event)
        except queue.Full:
            pass

    def close(self):
        self.stopping.set()


class SerialTransport(Transport):
    HELLO_INTERVAL = 0.25
    HELLO_TIMEOUT = 3.0

    def __init__(self, events, port):
        super().__init__(events)
        self.port = port

    def run(self):
        stream = None
        try:
            import serial
            stream = serial.Serial(self.port, baudrate=115200, timeout=0.02, write_timeout=0.2)
            parser = JsonLineDecoder()
            self.emit({"type": "connected", "source": self.port})
            handshake = False
            deadline = time.monotonic() + self.HELLO_TIMEOUT
            next_hello = 0.0
            while not self.stopping.is_set():
                now = time.monotonic()
                if not handshake:
                    if now >= deadline:
                        raise TimeoutError("USB device did not answer HELLO within 3 seconds; check firmware and the native USB connector")
                    if now >= next_hello:
                        stream.write(b"HELLO\n")
                        next_hello = now + self.HELLO_INTERVAL
                for _ in range(32):
                    try:
                        command = self.commands.get_nowait()
                    except queue.Empty:
                        break
                    if not handshake and command not in ("STOP", "PING"):
                        # ARM/SET cannot be queued across an incomplete session.
                        self.emit({"type": "error", "message": "Wait for the USB HELLO handshake before sending commands"})
                        continue
                    stream.write((command + "\n").encode("ascii"))
                for event in parser.feed(stream.read(max(1, min(stream.in_waiting, 4096)))):
                    if event.get("type") == "hello" and event.get("protocol") == 1 and not handshake:
                        handshake = True
                        stream.write(b"GET\n")
                    self.emit(event)
        except Exception as error:
            self.emit({"type": "transport_error", "message": str(error)})
        finally:
            if stream is not None:
                try:
                    stream.write(b"STOP\n")
                except Exception:
                    pass
                stream.close()
            self.emit({"type": "disconnected"})


class DemoTransport(Transport):
    def __init__(self, events, library=None):
        super().__init__(events)
        self.library = pathlib.Path(library) if library else pathlib.Path(__file__).resolve().parents[1] / "build/libsimulation.so"

    def run(self):
        sim = None
        try:
            lib = ctypes.CDLL(str(self.library))
            lib.sim_create.restype = ctypes.c_void_p
            lib.sim_destroy.argtypes = [ctypes.c_void_p]
            lib.sim_command.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_void_p, ctypes.c_size_t]
            lib.sim_advance.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_size_t]
            lib.sim_sampling_us.argtypes = [ctypes.c_void_p]
            lib.sim_sampling_us.restype = ctypes.c_uint32
            sim = lib.sim_create()
            if not sim:
                raise RuntimeError("C simulation could not initialize")
            parser = JsonLineDecoder()
            output = ctypes.create_string_buffer(16384)

            def command(text):
                lib.sim_command(sim, text.encode("ascii"), output, len(output))
                for event in parser.feed(output.value):
                    self.emit(event)

            self.emit({"type": "connected", "source": "DEMO · C controllers / ideal modeled plant"})
            command("HELLO")
            command("GET")
            last = time.monotonic()
            budget = 0.0
            while not self.stopping.wait(0.01):
                while True:
                    try:
                        command(self.commands.get_nowait())
                    except queue.Empty:
                        break
                now = time.monotonic()
                budget += min(now - last, 0.1)
                last = now
                dt = lib.sim_sampling_us(sim) / 1e6
                steps = min(int(budget / dt), 256)
                if steps:
                    budget -= steps * dt
                    lib.sim_advance(sim, steps, output, len(output))
                    for event in parser.feed(output.value):
                        self.emit(event)
        except Exception as error:
            self.emit({"type": "transport_error", "message": f"{error}. Build the host simulator with make host."})
        finally:
            if sim:
                lib.sim_destroy(sim)
            self.emit({"type": "disconnected"})
