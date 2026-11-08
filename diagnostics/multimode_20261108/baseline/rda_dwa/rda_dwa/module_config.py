"""
module_config.py — 仅主算法 V15_latesc 的参数构建。

本工程已移除全部对比基线（APF / TEB / Classic-DWA / MPC / RRT* / LT-DWA /
A*-DWA 等）与所有消融/候选预设（v1~v21、p1~p5、g1~g12、P0 等）。
局部路径规划只保留主算法 V15_latesc（等价别名 mtdwa）。

V15_latesc 的配置 = 全部模块（M1~M6, C1~C4）开启，叠加网格调优覆盖：
  headway_time=0.25, acc/dec_ang_max=2.0,
  enable_c12_headon_evade=True, c12_rear_ang=0.6,
  enable_c11_aabb_filter=True, c12_escape_perp=True。

构建顺序与原工程 module_config('v15_latesc') 逐行一致：
  params() 默认 → 全部模块开关先置 False → 再开 V15 需要的模块 → 应用覆盖。
这样保证同等输入下 Params 属性与原工程逐位一致。
"""
from __future__ import annotations

from .params import Params, params


# V15_latesc 开启的模块（原 _ALL_MODULES）
_V15_MODULES = ['M1', 'M2', 'M3', 'M4', 'M5', 'M6', 'C1', 'C2', 'C3', 'C4']

# 模块编号 → 要打开的 (属性, 值)（与原 _MODULE_TOGGLES 一致）
_MODULE_TOGGLES = {
    'M1': [('enable_m1_dynamic_window', True)],
    'M2': [('enable_m2_multistep', True)],
    'M3': [('enable_m3_gaussian_risk', True)],
    'M4': [('enable_m4_normalize', True)],
    'M5': [('enable_m5_filter', True)],
    'M6': [('enable_m6_fallback', True)],
    'C1': [('enable_c1_trunc_norm', True), ('normalize_trunc_D', 0.5)],
    'C2': [('enable_m7_imm_ukf', True)],
    'C3': [('enable_c3_lookahead', True), ('enable_c3_trap_escape', True)],
    'C4': [('enable_c4_apf_omega', True)],
}

# V15_latesc 的网格调优覆盖（原 _OVERRIDES['v15_latesc']）
_V15_OVERRIDES = {
    'headway_time': 0.25,
    'acc_ang_max': 2.0,
    'dec_ang_max': 2.0,
    'enable_c12_headon_evade': True,
    'c12_rear_ang': 0.6,
    'enable_c11_aabb_filter': True,
    'c12_escape_perp': True,
}


def _disable_all_modules(pp: Params) -> None:
    """先把所有模块/改进开关置 False（与原 module_config 开头一致）。"""
    pp.enable_m1_dynamic_window = False
    pp.enable_m2_multistep = False
    pp.enable_m3_gaussian_risk = False
    pp.enable_m4_normalize = False
    pp.enable_m5_filter = False
    pp.enable_m6_fallback = False
    pp.enable_c1_trunc_norm = False
    pp.enable_m7_imm_ukf = False
    pp.enable_c3_lookahead = False
    pp.enable_c3_trap_escape = False
    pp.enable_c4_apf_omega = False
    pp.normalize_trunc_D = 0.0   # C1 关闭时恢复原始截断
    pp.enable_c5_wait_aware_escape = False
    pp.enable_c6_escape_recheck = False
    pp.enable_c7_soft_relax = False
    pp.enable_c8_global_guide = False
    pp.enable_c9_uncertainty_inflate = False
    pp.enable_c11_aabb_filter = False
    pp.enable_c12_headon_evade = False
    pp.enable_s9_lateral = False
    pp.enable_s13_gap = False
    pp.enable_rss_speed_limit = False
    pp.enable_overtake_sm = False
    pp.enable_brake_wait = False
    pp.enable_maneuver_arc = False
    pp.enable_goal_ttc = False
    pp.enable_unc_vmax = False
    pp.enable_headon_cost = False
    pp.enable_imm_cost = False
    pp.enable_p0_1_hp_gate = False
    pp.enable_p0_2_capsule = False
    pp.enable_p0_2_early_term = False


def build_v15_latesc_params() -> Params:
    """构建 V15_latesc 的 Params（与原 module_config('v15_latesc') 逐位一致）。"""
    pp = params()
    _disable_all_modules(pp)
    for mod in _V15_MODULES:
        for attr, val in _MODULE_TOGGLES[mod]:
            setattr(pp, attr, val)
    for attr, val in _V15_OVERRIDES.items():
        setattr(pp, attr, val)
    pp.max_sim_time = 60.0
    return pp


def module_config(name='v15_latesc') -> Params:
    """兼容入口：本工程只接受主算法 V15_latesc（mtdwa 为其别名）。"""
    key = str(name).strip().lower()
    if key in ('v15_latesc', 'mtdwa', 'all', ''):
        return build_v15_latesc_params()
    if key == 'purehahn':
        # 单测参考点：全部模块关闭的朴素 DWA（非对外算法）。
        pp = params()
        _disable_all_modules(pp)
        pp.max_sim_time = 60.0
        return pp
    raise ValueError(
        f'本工程只保留主算法 V15_latesc，收到不支持的预设: {name!r}。'
        f'对比基线与消融预设已全部移除。')


# 仅服务 tests/ 下单测隔离组件用的消融参数（非对外规划器，不影响 V15 生产路径）。
# 与原工程一致：从 params() 默认（M1~M6 开）起，只叠加指定 C 模块开关。
_ABLATION_TOGGLES = {
    'BASE': [],
    'C1': ['C1'],
    'ALL': ['C1', 'C2', 'C3', 'C4'],
}


def ablation_params(name: str) -> Params:
    """单测辅助：在 params() 默认上叠加指定 C 模块开关（仅测试用）。"""
    import warnings
    warnings.warn(
        'ablation_params 仅用于 tests/ 组件隔离测试，生产路径请用 '
        'module_config("v15_latesc")。', stacklevel=2)
    pp = params()
    for mod in _ABLATION_TOGGLES.get(str(name).strip().upper(), []):
        for attr, val in _MODULE_TOGGLES[mod]:
            setattr(pp, attr, val)
    pp.max_sim_time = 60.0
    return pp
