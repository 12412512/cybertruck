"""工作台公共接口：米、秒、弧度；输入草稿不能直接成为设备命令。"""

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator


class Input(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False, strict=True)


class Command(Input):
    op: Literal['claim', 'renew', 'release', 'stop', 'manual_stop', 'manual', 'move', 'reset',
                'plan', 'run', 'save', 'load', 'defaults', 'mechanism']
    client: str = Field(min_length=1, max_length=64, pattern=r'^[a-zA-Z0-9_-]+$')
    token: str = Field(default='', max_length=64)
    request_id: str = Field(min_length=1, max_length=64, pattern=r'^[a-zA-Z0-9_-]+$')
    sequence: int = Field(ge=0, le=2147483647)
    args: dict = Field(default_factory=dict)


class Manual(Input):
    vx_m_s: float = Field(ge=-.15, le=.15)
    vy_m_s: float = Field(ge=-.15, le=.15)
    wz_rad_s: float = Field(ge=-.5, le=.5)


class Move(Input):
    x_m: float = Field(default=0., ge=-.5, le=.5)
    y_m: float = Field(default=0., ge=-.5, le=.5)
    yaw_rad: float = Field(default=0., ge=-3.142, le=3.142)
    speed_m_s: float = Field(default=.05, ge=.01, le=.15)


class Obstacle(Input):
    id: str = Field(max_length=48, min_length=1, pattern=r'^[a-zA-Z0-9_-]+$')
    x_m: float = Field(ge=0, le=2.4)
    y_m: float = Field(ge=0, le=2.4)
    diameter_m: float = Field(default=.05, ge=.01, le=.3)


class Docking(Input):
    raw_m: float = Field(default=.05, ge=0, le=.25)
    rough_m: float = Field(default=.05, ge=0, le=.25)
    staging_m: float = Field(default=.05, ge=0, le=.25)


class PlanInput(Input):
    source: Literal['batch', 'tour', 'raw', 'custom'] = 'batch'
    start: Literal['start_1', 'start_2'] = 'start_1'
    targets: list[str] = Field(default_factory=lambda: ['qr', 'raw', 'rough', 'staging', 'start_1'],
                               min_length=1, max_length=9)
    blocked: list[str] = Field(default_factory=list, max_length=24)
    speed_m_s: float = Field(default=.30, ge=.03, le=1.)
    max_rpm: float = Field(default=100., ge=20., le=350.)
    docking: Docking = Field(default_factory=Docking)
    obstacles: list[Obstacle] = Field(default_factory=list, max_length=20)

    @model_validator(mode='after')
    def unique_obstacles(self):
        if len({item.id for item in self.obstacles}) != len(self.obstacles):
            raise ValueError('障碍物ID不能重复')
        return self


class Run(Input):
    plan_id: str = Field(pattern=r'^[a-f0-9]{32}$')


class Reset(Input):
    start: Literal['start_1', 'start_2'] = 'start_1'


class Empty(Input):
    pass


class MechGoal(Input):
    """预留机械SI接口；启用前还必须接入实测限位、零点和互锁。"""
    action: Literal['jog', 'move', 'teach', 'zero', 'stop']
    axis: Literal['lift', 'reach', 'arm', 'tray', 'gripper']
    target_m: float | None = None
    target_rad: float | None = None
    velocity_m_s: float | None = None
    velocity_rad_s: float | None = None
    zero_version: int | None = Field(default=None, ge=1)
    pose_id: str | None = Field(default=None, max_length=64)

    @model_validator(mode='after')
    def check_units(self):
        linear = self.axis in ('lift', 'reach')
        if (linear and (self.target_rad is not None or self.velocity_rad_s is not None)
                or not linear and (self.target_m is not None or self.velocity_m_s is not None)):
            raise ValueError('平移轴使用米，旋转执行器使用弧度')
        return self


ARGUMENTS = dict(claim=Empty, renew=Empty, release=Empty, stop=Empty, manual_stop=Empty,
                 manual=Manual, move=Move, reset=Reset, plan=PlanInput, run=Run, save=Run,
                 load=Run, defaults=Empty, mechanism=MechGoal)

# 后续真实机构适配器必须提供这些字段；未接入时禁止编造轴位置和零点。
MECHANISM = dict(available=False, backend='not_connected', zero_valid=False,
                 axes=[dict(id=name, position_m=None, command_m=None, limits_m=None)
                       for name in ('lift', 'reach')],
                 actuators=['arm', 'tray', 'gripper'],
                 actions=['jog', 'move', 'teach', 'zero', 'stop'],
                 reason='机械控制尚未接入；后续共用控制权、动作结果和互锁接口')
