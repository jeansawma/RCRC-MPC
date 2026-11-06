"""Bounded, testable wire parsing and recording; independent of the GUI."""
from __future__ import annotations

from collections import deque
import csv
from dataclasses import dataclass, asdict
import json
import math
import re


class ProtocolError(ValueError):
    pass


class JsonLineDecoder:
    def __init__(self, maximum=8192):
        self.maximum = maximum
        self.pending = bytearray()
        self.dropping = False

    def feed(self, data: bytes):
        """Return complete JSON objects; resynchronize after oversized lines."""
        messages = []
        for byte in data:
            if byte == 10:
                if self.dropping:
                    self.dropping = False
                elif self.pending.strip():
                    try:
                        message = json.loads(self.pending.decode("utf-8"),
                                             parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
                        if not isinstance(message, dict) or not isinstance(message.get("type"), str):
                            raise ValueError("JSON object with type required")
                        messages.append(message)
                    except (UnicodeError, ValueError) as error:
                        messages.append({"type": "parse_error", "message": str(error)})
                self.pending.clear()
            elif not self.dropping:
                self.pending.append(byte)
                if len(self.pending) > self.maximum:
                    self.pending.clear()
                    self.dropping = True
                    messages.append({"type": "parse_error", "message": "oversized line discarded"})
        return messages


@dataclass(frozen=True)
class Parameter:
    name: str
    value: float
    minimum: float
    maximum: float
    unit: str = ""
    integer: bool = False

    @classmethod
    def from_message(cls, message):
        try:
            result = cls(str(message["name"]), float(message["value"]),
                         float(message["min"]), float(message["max"]),
                         str(message.get("unit", "")), bool(message.get("integer", False)))
        except (KeyError, TypeError, ValueError) as error:
            raise ProtocolError("invalid parameter definition") from error
        if not all(math.isfinite(x) for x in (result.value, result.minimum, result.maximum)):
            raise ProtocolError("nonfinite parameter")
        if result.minimum > result.maximum:
            raise ProtocolError("inverted parameter bounds")
        return result

    def command(self, text):
        try:
            value = float(text)
        except ValueError as error:
            raise ProtocolError("enter a numeric value") from error
        if not math.isfinite(value) or not self.minimum <= value <= self.maximum:
            raise ProtocolError(f"{self.name} must be within {self.minimum:g} … {self.maximum:g}")
        if self.integer and value != int(value):
            raise ProtocolError(f"{self.name} requires a whole number")
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", self.name):
            raise ProtocolError("invalid parameter name")
        return f"SET {self.name} {value:.9g}"


@dataclass(frozen=True)
class Sample:
    t_us: int
    v1: float
    v2: float
    vbus: float
    reference: float
    duty: float
    mode: int
    armed: int
    fault: int
    qp_residual: float = 0.0
    qp_iterations: int = 0

    @classmethod
    def from_message(cls, message):
        try:
            result = cls(int(message["t_us"]), *(float(message[key]) for key in
                         ("v1", "v2", "vbus", "reference", "duty")),
                         *(int(message[key]) for key in ("mode", "armed", "fault")),
                         float(message.get("qp_residual", 0)), int(message.get("qp_iterations", 0)))
        except (KeyError, ValueError, TypeError, OverflowError) as error:
            raise ProtocolError("invalid sample fields") from error
        if not all(math.isfinite(getattr(result, key)) for key in
                   ("v1", "v2", "vbus", "reference", "duty", "qp_residual")):
            raise ProtocolError("nonfinite telemetry")
        if result.t_us < 0 or result.mode not in (0, 1, 2) or result.armed not in (0, 1):
            raise ProtocolError("invalid sample state")
        return result


class History:
    """A bounded recording window; export contains exactly the retained samples."""
    def __init__(self, maximum=60000):
        self.samples = deque(maxlen=maximum)
        self.last_raw = None
        self.wrap = 0
        self.display_times = deque(maxlen=maximum)

    def append(self, sample):
        # MCU micros wraps at 2^32. A smaller backwards jump is a reset/session.
        if self.last_raw is not None and sample.t_us < self.last_raw:
            if self.last_raw - sample.t_us > 2**31:
                self.wrap += 2**32
            else:
                self.clear()
        self.last_raw = sample.t_us
        self.samples.append(sample)
        self.display_times.append((self.wrap + sample.t_us) / 1e6)

    def clear(self):
        self.samples.clear()
        self.display_times.clear()
        self.last_raw = None
        self.wrap = 0

    def export(self, file_object):
        writer = csv.DictWriter(file_object, fieldnames=tuple(Sample.__dataclass_fields__))
        writer.writeheader()
        for sample in self.samples:
            writer.writerow(asdict(sample))
