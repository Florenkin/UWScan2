from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import TYPE_CHECKING, Any, Dict, Tuple

if TYPE_CHECKING:
    from .dll import UWScan3DDll


_PIXEL_FORMAT_TO_CODE = {
    "Mono8": 0,
    "Mono10": 1,
    "Mono12": 2,
    "Mono16": 3,
}

_CODE_TO_PIXEL_FORMAT = {value: key for key, value in _PIXEL_FORMAT_TO_CODE.items()}


@dataclass(slots=True)
class BaslerCameraConfig:
    exposure_time: float = 20000.0
    gain: float = 0.0
    frame_rate: float = 10.0
    pixel_format: str = "Mono8"
    reverse_x: bool = False
    reverse_y: bool = False
    trigger_selector: str = "FrameStart"
    trigger_mode: bool = False
    trigger_source: str = "Line0"

    def clamp(
        self,
        exposure_range: Tuple[float, float] | None = None,
        gain_range: Tuple[float, float] | None = None,
    ) -> "BaslerCameraConfig":
        if exposure_range is not None:
            self.exposure_time = min(max(self.exposure_time, exposure_range[0]), exposure_range[1])
        if gain_range is not None:
            self.gain = min(max(self.gain, gain_range[0]), gain_range[1])
        return self

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "BaslerCameraConfig":
        return cls(**{key: value for key, value in data.items() if key in cls.__dataclass_fields__})

    def to_dll_dict(self) -> Dict[str, Any]:
        return {
            "exposure_time": self.exposure_time,
            "gain": self.gain,
            "frame_rate": self.frame_rate,
            "pixel_format": _PIXEL_FORMAT_TO_CODE.get(self.pixel_format, 0),
            "reverse_x": int(self.reverse_x),
            "reverse_y": int(self.reverse_y),
            "trigger_selector": 0,
            "trigger_mode": int(self.trigger_mode),
            "trigger_source": 0,
        }

    @classmethod
    def from_dll_dict(cls, data: Dict[str, Any]) -> "BaslerCameraConfig":
        return cls(
            exposure_time=float(data.get("exposure_time", 20000.0)),
            gain=float(data.get("gain", 0.0)),
            frame_rate=float(data.get("frame_rate", 10.0)),
            pixel_format=_CODE_TO_PIXEL_FORMAT.get(int(data.get("pixel_format", 0)), "Mono8"),
            reverse_x=bool(data.get("reverse_x", 0)),
            reverse_y=bool(data.get("reverse_y", 0)),
            trigger_mode=bool(data.get("trigger_mode", 0)),
        )

    def clamp_with_dll(
        self,
        dll: "UWScan3DDll",
        exposure_range: Tuple[float, float] | None = None,
        gain_range: Tuple[float, float] | None = None,
    ) -> "BaslerCameraConfig":
        clamped = dll.clamp_basler_config(self.to_dll_dict(), exposure_range=exposure_range, gain_range=gain_range)
        updated = BaslerCameraConfig.from_dll_dict(clamped)
        self.exposure_time = updated.exposure_time
        self.gain = updated.gain
        self.frame_rate = updated.frame_rate
        self.pixel_format = updated.pixel_format
        self.reverse_x = updated.reverse_x
        self.reverse_y = updated.reverse_y
        self.trigger_mode = updated.trigger_mode
        return self
