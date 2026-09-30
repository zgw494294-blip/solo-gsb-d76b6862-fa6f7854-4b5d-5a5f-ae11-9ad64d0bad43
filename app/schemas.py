"""FastAPI 的请求/响应数据模型（含录入校验）。"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field, field_validator, model_validator


class ChannelInput(BaseModel):
    id: str = Field(..., description="通道 ID，全局唯一")
    r: int = Field(..., ge=0, le=1000, description="满档线性 R")
    g: int = Field(..., ge=0, le=1000, description="满档线性 G")
    b: int = Field(..., ge=0, le=1000, description="满档线性 B")
    power_mw: int = Field(..., ge=1, le=10000, description="满档功率 (mW)")

    @field_validator("id")
    @classmethod
    def _strip_id(cls, v: str) -> str:
        v = v.strip()
        if not v:
            raise ValueError("通道 ID 不能为空")
        return v

    @model_validator(mode="after")
    def _not_all_zero(self) -> "ChannelInput":
        if self.r == 0 and self.g == 0 and self.b == 0:
            raise ValueError("满档 RGB 不能全为零")
        return self


class SolveRequest(BaseModel):
    channels: list[ChannelInput] = Field(..., description="3～6 路 LED，ID 唯一")
    target_r: int = Field(..., ge=0, le=1000)
    target_g: int = Field(..., ge=0, le=1000)
    target_b: int = Field(..., ge=0, le=1000)
    tolerance: int = Field(..., ge=0, le=20, description="共用整数容差")

    @model_validator(mode="after")
    def _check_channels(self) -> "SolveRequest":
        if not 3 <= len(self.channels) <= 6:
            raise ValueError("LED 通道数必须为 3～6 路")
        ids = [c.id for c in self.channels]
        if len(set(ids)) != len(ids):
            dup = sorted({x for x in ids if ids.count(x) > 1})
            raise ValueError(f"通道 ID 必须唯一，重复 ID：{', '.join(dup)}")
        return self


class ChannelSolution(BaseModel):
    id: str
    drive: int = Field(..., ge=0, le=255)
    r_full: int
    g_full: int
    b_full: int
    power_full_mw: int


class SolveResponse(BaseModel):
    feasible: bool
    target: list[int]
    tolerance: int
    # 字典序 tie-break 实际使用的 ID 顺序
    sorted_ids: list[str] = []
    channels: list[ChannelSolution] = []
    actual_rgb: list[float] = []
    errors: list[float] = []
    in_range: list[bool] = []
    total_power_mw: float | None = None
    power_num: int | None = Field(
        default=None, description="总功率的整数分子，实际功率 = 该值 / 255"
    )
    message: str = ""


# ---------------- 故障容错配方（正常 + 逐路熄灭备用） ----------------


class RobustSolveRequest(SolveRequest):
    max_adjust: int = Field(
        ..., ge=1, le=255,
        description="某路熄灭后，其余各路备用驱动级相对正常值允许的最大级差 1～255",
    )


class RobustVariantSolution(BaseModel):
    """一个场景（正常或某路熄灭）的配方。"""

    # 备用场景：被熄灭的通道 ID；正常场景为 None
    failed_id: str | None = None
    drives: list[int] = Field(..., description="各路驱动级，与 channel_ids 同序")
    # 备用场景相对正常方案的驱动级之差；正常场景全为 0
    deltas: list[int] = []
    actual_rgb: list[float]
    errors: list[float] = Field(..., description="实际 − 目标")
    in_range: list[bool]
    power_mw: float
    power_num: int = Field(
        ..., description="该场景功率的整数分子，实际功率 = 该值 / 255"
    )


class RobustSolveResponse(BaseModel):
    # optimal：正常方案 + 全部 n 路备用配方均返回；
    # infeasible：无完整容错方案；incomplete：搜索未证明最优（均不返回任何配方）
    status: Literal["optimal", "infeasible", "incomplete"]
    channel_ids: list[str] = Field(
        default=[], description="通道 ID 顺序（与各 drives/输入 channels 同序）"
    )
    target: list[int] = []
    tolerance: int = 0
    max_adjust: int = 0
    # 字典序 tie-break 实际使用的 ID 顺序
    sorted_ids: list[str] = []
    normal: RobustVariantSolution | None = None
    # 逐路熄灭的备用配方，顺序与 channel_ids 一一对应
    backups: list[RobustVariantSolution] = []
    message: str = ""


# ---------------- 满档读数标定 ----------------


class CalibChannelRef(BaseModel):
    """标定只需要通道 ID；满档 RGB 是被反求的对象。"""

    id: str = Field(..., description="通道 ID，与样本驱动级按顺序一一对应")

    @field_validator("id")
    @classmethod
    def _strip_id(cls, v: str) -> str:
        v = v.strip()
        if not v:
            raise ValueError("通道 ID 不能为空")
        return v


class CalibSampleInput(BaseModel):
    drives: list[int] = Field(..., description="各路驱动级，顺序与 channels 一致")
    measured: list[int] = Field(..., description="实测线性 RGB（四舍五入后的整数）")

    @field_validator("drives")
    @classmethod
    def _check_drives(cls, v: list[int]) -> list[int]:
        if any(not isinstance(d, int) or not 0 <= d <= 255 for d in v):
            raise ValueError("每个驱动级必须是 0～255 的整数")
        return v

    @field_validator("measured")
    @classmethod
    def _check_measured(cls, v: list[int]) -> list[int]:
        if len(v) != 3:
            raise ValueError("实测 RGB 必须恰好包含 R、G、B 三个整数")
        if any(not isinstance(m, int) or not 0 <= m <= 6000 for m in v):
            raise ValueError("每个实测分量必须是 0～6000 的整数")
        return v


class CalibrateRequest(BaseModel):
    channels: list[CalibChannelRef] = Field(..., description="3～6 路通道，ID 唯一")
    samples: list[CalibSampleInput] = Field(..., description="3～12 组实测样本")

    @model_validator(mode="after")
    def _check(self) -> "CalibrateRequest":
        if not 3 <= len(self.channels) <= 6:
            raise ValueError("LED 通道数必须为 3～6 路")
        ids = [c.id for c in self.channels]
        if len(set(ids)) != len(ids):
            dup = sorted({x for x in ids if ids.count(x) > 1})
            raise ValueError(f"通道 ID 必须唯一，重复 ID：{', '.join(dup)}")
        if not 3 <= len(self.samples) <= 12:
            raise ValueError("标定样本数必须为 3～12 组")
        n = len(self.channels)
        bad = [
            str(i + 1)
            for i, s in enumerate(self.samples)
            if len(s.drives) != n
        ]
        if bad:
            raise ValueError(
                f"以下样本的驱动级数量与通道数（{n} 路）不一致：第 {', '.join(bad)} 组"
            )
        return self


class CalibChannelResult(BaseModel):
    id: str
    r: int
    g: int
    b: int


class CalibrateResponse(BaseModel):
    # unique：唯一（channels 有值）；contradictory：样本矛盾；
    # non_unique：存在多组合法参数；unknown：搜索未完成（不得据此下结论）
    status: Literal["unique", "contradictory", "non_unique", "unknown"]
    channels: list[CalibChannelResult] = []
    message: str = ""


# ---------------- 调光路径规划 ----------------


class PathSolveRequest(BaseModel):
    channels: list[ChannelInput] = Field(..., description="3～6 路 LED，ID 唯一")
    start_drives: dict[str, int] = Field(
        ..., description="按通道 ID 的起始驱动级，每个 0～255，须与通道一一对应"
    )
    target_r: int = Field(..., ge=0, le=1000, description="终点线性 R")
    target_g: int = Field(..., ge=0, le=1000, description="终点线性 G")
    target_b: int = Field(..., ge=0, le=1000, description="终点线性 B")
    steps: int = Field(..., ge=2, le=5, description="等时步骤数 2～5")
    max_step_delta: int = Field(
        ..., ge=1, le=255, description="每步每路驱动变化上限 1～255"
    )
    tolerance: int = Field(..., ge=0, le=20, description="共用整数容差")

    @field_validator("start_drives")
    @classmethod
    def _check_start_drives(cls, v: dict[str, int]) -> dict[str, int]:
        for cid, d in v.items():
            if not isinstance(d, int) or not 0 <= d <= 255:
                raise ValueError(f"通道 “{cid}” 的起始驱动级必须是 0～255 的整数")
        return v

    @model_validator(mode="after")
    def _check_channels(self) -> "PathSolveRequest":
        if not 3 <= len(self.channels) <= 6:
            raise ValueError("LED 通道数必须为 3～6 路")
        ids = [c.id for c in self.channels]
        if len(set(ids)) != len(ids):
            dup = sorted({x for x in ids if ids.count(x) > 1})
            raise ValueError(f"通道 ID 必须唯一，重复 ID：{', '.join(dup)}")
        missing = [cid for cid in ids if cid not in self.start_drives]
        if missing:
            raise ValueError(
                f"缺少以下通道的起始驱动级：{', '.join(missing)}"
            )
        extra = sorted(k for k in self.start_drives if k not in set(ids))
        if extra:
            raise ValueError(
                f"起始驱动级包含未知通道 ID：{', '.join(extra)}"
            )
        return self


class PathStepSolution(BaseModel):
    step: int = Field(..., description="步骤序号（1 起始）")
    drives: list[int] = Field(..., description="各路驱动级，与 channel_ids 同序")
    actual_rgb: list[float]
    ref_rgb: list[float] = Field(..., description="未取整的插值参考色")
    errors: list[float] = Field(..., description="实际 − 参考")
    in_range: list[bool]
    power_mw: float
    power_num: int = Field(
        ..., description="该步功率的整数分子，实际功率 = 该值 / 255"
    )


class PathSolveResponse(BaseModel):
    # optimal：已证明总功率最小的完整路径（steps 有值）；
    # infeasible：无完整路径；incomplete：搜索未证明最优（均不返回任何步骤）
    status: Literal["optimal", "infeasible", "incomplete"]
    channel_ids: list[str] = []
    start_drives: list[int] = Field(
        default=[], description="起点（第 0 步）驱动级，与 channel_ids 同序"
    )
    start_actual_rgb: list[float] = Field(
        default=[], description="起点实际色（未裁剪公式，不取整）"
    )
    target: list[int] = []
    tolerance: int = 0
    max_step_delta: int = 0
    step_count: int = 0
    steps: list[PathStepSolution] = []
    total_power_mw: float | None = None
    total_power_num: int | None = Field(
        default=None, description="各步功率之和的整数分子，实际功率 = 该值 / 255"
    )
    message: str = ""
