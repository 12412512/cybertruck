"""雷达工作台请求契约；设备所有权与写入许可仍归唯一核心。"""
from pydantic import BaseModel, ConfigDict, Field


class ScanRequest(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)
    request_id: str = Field(min_length=1, max_length=64)
    zone: int = Field(ge=1, le=2)
    duration_s: float = Field(default=10, ge=2, le=10)


class CalibrationPrepare(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    request_id: str = Field(min_length=1, max_length=64)
    zone: int = Field(ge=1, le=2)
    placement_id: str = Field(min_length=1, max_length=64)
    manual_fixed_deployed: bool


class CalibrationCommit(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    request_id: str = Field(min_length=1, max_length=64)
    calibration_id: str = Field(pattern=r'^[a-f0-9]{32}$')
    backup_version: str = Field(pattern=r'^[a-f0-9]{64}$')
    placement_id: str = Field(min_length=1, max_length=64)
    confirm_save: bool
    allow_can_latched: bool = False
