"""工作台公共接口：米、秒、弧度；输入草稿不能直接成为设备命令。"""

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator


class Input(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False, strict=True)


class Command(Input):
    op: Literal['claim', 'renew', 'release', 'stop', 'manual_stop', 'manual', 'move', 'reset',
                'plan', 'run', 'save', 'load', 'defaults', 'mechanism', 'transfer']
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
    """机构离线接口；草稿、明确执行与实物验收分别表示。"""
    action: Literal['jog', 'move', 'teach', 'preview', 'pose', 'next', 'zero', 'anchor', 'invalidate', 'reset_model', 'stop']
    axis: Literal['lift', 'reach', 'arm', 'tray', 'gripper'] = 'lift'
    target_m: float | None = None
    target_rad: float | None = None
    velocity_m_s: float | None = None
    velocity_rad_s: float | None = None
    zero_version: int | None = Field(default=None, ge=1)
    pose_id: str | None = Field(default=None, min_length=1, max_length=48, pattern=r'^[a-zA-Z0-9_-]+$')
    name: str = Field(default='', max_length=64)
    revision: int = Field(default=0, ge=0)
    confirmed: bool = False
    release_confirmed: bool = False
    single_step: bool = False
    action_id: int | None = Field(default=None, ge=1)
    step_index: int | None = Field(default=None, ge=0, le=8)
    targets: dict[str, float] = Field(default_factory=dict, max_length=5)

    @model_validator(mode='after')
    def check_units(self):
        if self.action == 'next' and (self.action_id is None or self.step_index is None):
            raise ValueError('单步继续必须绑定当前动作和步骤')
        if self.single_step and self.action != 'pose':
            raise ValueError('逐步确认仅用于姿态执行')
        linear = self.axis in ('lift', 'reach')
        if (linear and (self.target_rad is not None or self.velocity_rad_s is not None)
                or not linear and (self.target_m is not None or self.velocity_m_s is not None)):
            raise ValueError('平移轴使用米，旋转执行器使用弧度')
        return self


class TransferGoal(Input):
    action: Literal['preview', 'run', 'confirm', 'stop', 'reset']
    slot: int = Field(default=0, ge=0, le=2)
    color: Literal['red','green','blue','yellow','cyan','black'] = 'red'
    raw_lift_m: float = Field(default=.049, ge=0., le=.049)
    raw_reach_m: float = Field(default=.05, ge=0., le=.05)
    slot_lift_m: float = Field(default=.043, ge=0., le=.049)
    place_lift_m: float = Field(default=.049, ge=0., le=.049)
    place_reach_m: float = Field(default=.05, ge=0., le=.05)
    auto_model: bool = False
    pace: int = Field(default=1, ge=1, le=20)
    confirmed: bool = False
    action_id: int | None = Field(default=None, ge=1)
    step_index: int | None = Field(default=None, ge=0, le=64)
    verdict: Literal['PICKED','STORED','RETRIEVED','PLACED','UNKNOWN'] = 'UNKNOWN'


ARGUMENTS = dict(claim=Empty, renew=Empty, release=Empty, stop=Empty, manual_stop=Empty,
                 manual=Manual, move=Move, reset=Reset, plan=PlanInput, run=Run, save=Run,
                 load=Run, defaults=Empty, mechanism=MechGoal, transfer=TransferGoal)
