"""LED 混光配方台 —— FastAPI 入口。"""

from __future__ import annotations

from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from .schemas import (
    CalibrateRequest,
    CalibrateResponse,
    CalibChannelResult,
    ChannelSolution,
    PathSolveRequest,
    PathSolveResponse,
    PathStepSolution,
    RobustSolveRequest,
    RobustSolveResponse,
    RobustVariantSolution,
    SolveRequest,
    SolveResponse,
)
from .calibration import CalibSample, calibrate
from .path_solver import solve_path
from .robust_solver import solve_robust
from .solver import Channel, solve, SolverError

# static/ 与 app/ 同级；用绝对路径，保证从任意工作目录启动都能找到页面
BASE_DIR = Path(__file__).resolve().parent.parent
STATIC_DIR = BASE_DIR / "static"

app = FastAPI(
    title="LED 混光配方台 API",
    description="基于 OR-Tools CP-SAT 的多通道 LED 混光最小功率配方求解",
    version="1.0.0",
)


@app.get("/api/health")
def health() -> dict[str, str]:
    return {"status": "ok"}


@app.post("/api/solve", response_model=SolveResponse)
def api_solve(req: SolveRequest) -> SolveResponse:
    channels = [
        Channel(id=c.id, rgb=(c.r, c.g, c.b), power_mw=c.power_mw)
        for c in req.channels
    ]
    target = (req.target_r, req.target_g, req.target_b)
    try:
        sol = solve(channels, target, req.tolerance)
    except SolverError as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc

    if sol is None:
        # 明确判定不可行：不返回任何近似配方
        return SolveResponse(
            feasible=False,
            target=[target[0], target[1], target[2]],
            tolerance=req.tolerance,
            message=(
                "无可行配方：在给定容差下，3～6 路 LED 的任意驱动级组合都无法"
                "使三色分量同时落入要求区间（且不裁剪）。可尝试放宽容差、"
                "调整目标色或增加/更换通道。"
            ),
        )

    lo = [t - req.tolerance for t in target]
    hi = [t + req.tolerance for t in target]
    return SolveResponse(
        feasible=True,
        target=list(target),
        tolerance=req.tolerance,
        sorted_ids=sol.sorted_ids,
        channels=[
            ChannelSolution(
                id=req.channels[i].id,
                drive=sol.drives[i],
                r_full=req.channels[i].r,
                g_full=req.channels[i].g,
                b_full=req.channels[i].b,
                power_full_mw=req.channels[i].power_mw,
            )
            for i in range(len(req.channels))
        ],
        actual_rgb=sol.actual_rgb,
        errors=sol.errors,
        in_range=[
            lo[c] <= sol.actual_rgb[c] <= hi[c]
            and 0 <= sol.actual_rgb[c] <= 1000
            for c in range(3)
        ],
        total_power_mw=sol.power_mw,
        power_num=sol.power_num,
        message="求解成功：总功率已最小，相同功率下驱动级序列按 ID 字典序最小。",
    )


@app.post("/api/robust/solve", response_model=RobustSolveResponse)
def api_solve_robust(req: RobustSolveRequest) -> RobustSolveResponse:
    """规划可承受任意单路 LED 熄灭的配方（正常总功率最小 + 逐路备用）。

    三态结论：optimal 返回正常方案与全部备用配方；infeasible / incomplete
    不返回正常方案，也不返回任何部分备用配方。
    """
    channels = [
        Channel(id=c.id, rgb=(c.r, c.g, c.b), power_mw=c.power_mw)
        for c in req.channels
    ]
    target = (req.target_r, req.target_g, req.target_b)
    try:
        res = solve_robust(channels, target, req.tolerance, req.max_adjust)
    except SolverError as exc:  # 预留：求解器内部异常统一按 500
        raise HTTPException(status_code=500, detail=str(exc)) from exc

    if res.status != "optimal":
        # 无完整容错方案 / 搜索未完成：不返回任何正常或备用驱动级
        return RobustSolveResponse(
            status=res.status,
            channel_ids=[c.id for c in req.channels],
            target=list(target),
            tolerance=req.tolerance,
            max_adjust=req.max_adjust,
            message=res.message,
        )

    def variant(v) -> RobustVariantSolution:
        return RobustVariantSolution(
            failed_id=None,
            drives=v.drives,
            deltas=v.deltas,
            actual_rgb=v.actual_rgb,
            errors=v.errors,
            in_range=v.in_range,
            power_mw=v.power_mw,
            power_num=v.power_num,
        )

    backups = []
    for i, v in enumerate(res.backups):
        bv = variant(v)
        bv.failed_id = req.channels[i].id
        backups.append(bv)

    return RobustSolveResponse(
        status="optimal",
        channel_ids=[c.id for c in req.channels],
        target=list(target),
        tolerance=req.tolerance,
        max_adjust=req.max_adjust,
        sorted_ids=res.sorted_ids,
        normal=variant(res.normal),
        backups=backups,
        message=res.message,
    )


@app.post("/api/calibrate", response_model=CalibrateResponse)
def api_calibrate(req: CalibrateRequest) -> CalibrateResponse:
    """由 3～12 组实测样本反求各路 0～1000 满档 RGB（整数约束，唯一性可证明）。"""
    ids = [c.id for c in req.channels]
    samples = [
        CalibSample(
            drives=tuple(s.drives),
            measured=(s.measured[0], s.measured[1], s.measured[2]),
        )
        for s in req.samples
    ]
    result = calibrate(ids, samples)
    return CalibrateResponse(
        status=result.status,
        channels=[
            CalibChannelResult(id=cid, r=r, g=g, b=b)
            for (cid, r, g, b) in result.channels
        ],
        message=result.message,
    )


@app.post("/api/path/solve", response_model=PathSolveResponse)
def api_solve_path(req: PathSolveRequest) -> PathSolveResponse:
    """规划从起始驱动级到目标色的等时调光路径（各步功率之和最小）。

    三态结论：optimal 返回完整路径；infeasible / incomplete 不返回任何步骤。
    """
    channels = [
        Channel(id=c.id, rgb=(c.r, c.g, c.b), power_mw=c.power_mw)
        for c in req.channels
    ]
    start = [req.start_drives[c.id] for c in req.channels]
    target = (req.target_r, req.target_g, req.target_b)
    res = solve_path(
        channels, start, target, req.steps, req.max_step_delta, req.tolerance
    )
    return PathSolveResponse(
        status=res.status,
        channel_ids=[c.id for c in req.channels],
        start_drives=res.start_drives,
        start_actual_rgb=res.start_actual_rgb,
        target=list(target),
        tolerance=req.tolerance,
        max_step_delta=req.max_step_delta,
        step_count=req.steps,
        steps=[
            PathStepSolution(
                step=k + 1,
                drives=st.drives,
                actual_rgb=st.actual_rgb,
                ref_rgb=st.ref_rgb,
                errors=st.errors,
                in_range=st.in_range,
                power_mw=st.power_mw,
                power_num=st.power_num,
            )
            for k, st in enumerate(res.steps)
        ],
        total_power_mw=res.total_power_mw,
        total_power_num=res.total_power_num,
        message=res.message,
    )


# 静态页面（放最后注册，避免吞掉 /api 路由）
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


@app.get("/")
def index() -> FileResponse:
    return FileResponse(STATIC_DIR / "index.html")
