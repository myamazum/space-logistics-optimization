"""
Data classes for user-defined input wrapper
Children data classes are wrapped in a parent data class in input_data_class
"""

# src/network_builder/network_builder_v2.py
from __future__ import annotations

import math
from dataclasses import dataclass, field
from collections import defaultdict
from typing import Dict, List, Tuple, Optional, Iterable, Literal

import numpy as np

# 親クラス
from input_data_class import NodeDetails, InputData
from network_builder.network_builder_class import NetworkBuilder

G0 = 9.80665  # m/s^2, standard gravity

# 既存:
# @dataclass
# class NodeDetails:
#     node_names: List[str]
#     is_path_graph: bool = True
#     holdover_nodes: List[str] = field(default_factory=list)
#     outbound_path: List[str] = field(default_factory=list)

# ----------------- V2 用の補助データ構造 -----------------
Role = Literal["outbound", "inbound", "holdover", "any"]
TWType = Literal["mission_relative", "absolute"]
ArcKind = Literal["transport", "holdover", "process"]

@dataclass
class NodeSpec:
    """A single node with optional attributes."""
    name: str
    kind: Optional[str] = None       # e.g., "planet", "orbit", "surface"
    holdover: bool = False           # whether inventory can be stored here
    attrs: Dict[str, Any] = field(default_factory=dict)  # extensible bag

@dataclass
class TimeWindow:
    """A time window for using an arc."""
    role: Role = "any"
    type: TWType = "mission_relative"
    # starts/ends: if mission_relative, allow tokens like "mis_start", "mis_end",
    # or integer offsets (e.g., +3, -1). If absolute, use integers (days).
    starts: List[Any] = field(default_factory=list)  # list[int|str]
    ends: Optional[List[Any]] = None                 # list[int|str] | None

@dataclass
class ArcSpec:
    """A directed arc with optional flight characteristics and windows."""
    dep: str
    arr: str
    kind: ArcKind = "transport"      # transport: dep!=arr, holdover/process: dep==arr
    tof_days: Optional[float] = None # time of flight (transport only)
    delta_v_ms: Optional[float] = None  # Δv [m/s] (transport only; Earth↔LEO=0 is OK)
    windows: List[TimeWindow] = field(default_factory=list)
    max_parallel: Optional[int] = None     # optional concurrency cap
    notes: Optional[str] = None
    attrs: Dict[str, Any] = field(default_factory=dict)

@dataclass
class ArcIdx:
    """Internal arc record (arc-centric Phase A)."""
    id: int
    dep: int                 # node id (index into node_names)
    arr: int                 # node id
    kind: str                # "transport" | "holdover" | "process"
    tof_days: float          # time of flight (days); 0 for non-transport
    dv_ms: float             # delta-v [m/s]; 0 for non-transport
    alpha: float             # propellant mass fraction for this arc; 0 for non-transport
    max_parallel: Optional[int] = None
    notes: Optional[str] = None

@dataclass
class NodeDetailsV2(NodeDetails):
    """
    Generalized node/arc interface (JSR general form) while keeping backward
    compatibility with NodeDetails (path graph). When you pass this to the
    NetworkBuilder, you can either use it directly (preferred) or adapt it
    back to a path-graph view via to_parent() if needed.
    """
    nodes: List[NodeSpec] = field(default_factory=list)
    arcs: List[ArcSpec] = field(default_factory=list)

    # ---------- Validation ----------
    def validate(self) -> None:
        """Raise ValueError if the structure is inconsistent."""
        node_set = {n.name for n in self.nodes} if self.nodes else set(self.node_names)

        # 1) Node existence
        for a in self.arcs:
            if a.dep not in node_set or a.arr not in node_set:
                raise ValueError(f"Arc ({a.dep}->{a.arr}) references undefined node.")

        # 2) Kind-specific checks
        for a in self.arcs:
            if a.kind == "transport":
                if a.dep == a.arr:
                    raise ValueError(f"Transport arc must have dep!=arr: {a.dep}->{a.arr}")
                if a.tof_days is None or a.tof_days <= 0:
                    raise ValueError(f"Transport arc must specify positive tof_days: {a.dep}->{a.arr}")
                if a.delta_v_ms is None or a.delta_v_ms < 0:
                    raise ValueError(f"Transport arc must specify non-negative delta_v_ms: {a.dep}->{a.arr}")
            elif a.kind in ("holdover", "process"):
                if a.dep != a.arr:
                    raise ValueError(f"{a.kind} arc must be a self-loop: {a.dep}->{a.arr}")

        # 3) Holdover permission
        holdover_allowed = {n.name for n in self.nodes if n.holdover}
        for a in self.arcs:
            if a.kind == "holdover" and a.dep not in holdover_allowed:
                raise ValueError(f"Holdover not allowed on node '{a.dep}' (set NodeSpec.holdover=True).")

        # 4) TimeWindow sanity
        for a in self.arcs:
            for w in a.windows:
                if w.ends is not None and len(w.starts) != len(w.ends):
                    raise ValueError(f"TimeWindow starts/ends length mismatch on {a.dep}->{a.arr}.")
                if w.type not in ("mission_relative", "absolute"):
                    raise ValueError(f"Unsupported TimeWindow.type: {w.type}")

    # ---------- Allowed-time expansion (for NetworkBuilder) ----------
    def expand_allowed_times(
        self,
        mission_starts: Iterable[int],
        mission_ends: Iterable[int],
        time_grid: Iterable[int],
        resolve_token: Optional[Dict[str, int]] = None,
    ) -> Dict[Tuple[str, str], List[int]]:
        """
        Convert TimeWindow entries to concrete time indices on the given grid.
        Returns a dict: (dep,arr) -> sorted unique list of allowed times t.
        """
        time_set = set(time_grid)
        allowed: Dict[Tuple[str, str], set[int]] = {}

        # Default token map for mission_relative
        token_map_default: Dict[str, List[int]] = {
            "mis_start": list(mission_starts),
            "mis_end": list(mission_ends),
        }
        # Allow external resolution (e.g., special events)
        if resolve_token:
            for k, v in resolve_token.items():
                token_map_default[k] = [v]

        def _resolve_list(items: List[Any]) -> List[int]:
            out: List[int] = []
            for it in items:
                if isinstance(it, int):
                    out.append(it)
                elif isinstance(it, str):
                    if it.startswith(("+", "-")) and it[1:].isdigit():
                        out.append(int(it))  # relative offset; apply later
                    else:
                        # token like "mis_start" or "mis_end"
                        out.extend(token_map_default.get(it, []))
                else:
                    raise ValueError(f"Unsupported start/end token: {it}")
            return out

        for a in self.arcs:
            key = (a.dep, a.arr)
            S = allowed.setdefault(key, set())

            # If no window is provided: for holdover -> all time; for transport -> inherit mission start/end by role
            if not a.windows:
                if a.kind == "holdover":
                    S.update(time_set)
                elif a.kind == "transport":
                    # Fallback: outbound at mission starts, inbound at mission ends, else any
                    # This preserves current path-graph default behavior.
                    S.update(token_map_default["mis_start"])
                    S.update(token_map_default["mis_end"])
                continue

            for w in a.windows:
                if w.type == "absolute":
                    starts = [int(x) for x in w.starts]
                    ends = [int(x) for x in (w.ends or [])]
                    if not ends:
                        # instantaneous windows
                        for s in starts:
                            if s in time_set: S.add(s)
                    else:
                        for s, e in zip(starts, ends):
                            for t in range(s, e + 1):
                                if t in time_set: S.add(t)

                elif w.type == "mission_relative":
                    starts_raw = _resolve_list(w.starts)
                    ends_raw = _resolve_list(w.ends or [])
                    # If relative offsets (like "+3") present, apply to each mission token
                    def _apply_offset(vals: List[int]) -> List[int]:
                        # expand "+k"/"-k" to shift all mission tokens
                        expanded: List[int] = []
                        offs = [int(v) for v in vals if isinstance(v, int)]
                        tokens = [v for v in vals if not isinstance(v, int)]  # already resolved tokens
                        expanded.extend(offs)  # literal ints (rare)
                        expanded.extend(tokens)
                        return expanded

                    if not ends_raw:
                        for s in starts_raw:
                            if s in time_set: S.add(s)
                    else:
                        for s, e in zip(starts_raw, ends_raw):
                            lo, hi = (int(s), int(e))
                            for t in range(lo, hi + 1):
                                if t in time_set: S.add(t)
                else:
                    raise ValueError(f"Unsupported window type {w.type}")

        # return as sorted lists
        return {k: sorted(v) for k, v in allowed.items()}

    # ---------- 後方互換アダプタ ----------

    @classmethod
    def from_parent(
        cls,
        old: NodeDetails,
        tof_days_fn: Optional[callable] = None,
        delta_v_ms_fn: Optional[callable] = None,
    ) -> "NodeDetailsV2":
        """
        Adapt legacy NodeDetails (path graph) to NodeDetailsV2.
        You can pass custom TOF/Δv providers; otherwise simple defaults are used.
        """
        nodes = [NodeSpec(n, holdover=(n in old.holdover_nodes)) for n in old.node_names]

        # Defaults (can be overridden)
        def _tof(i: str, j: str) -> float:
            if tof_days_fn: return float(tof_days_fn(i, j))
            # simple known pairs; otherwise 1 day
            if {i, j} == {"Earth", "LEO"}: return 1.0
            if {i, j} == {"LEO", "LLO"}:   return 3.0
            if {i, j} == {"LLO", "LS"}:    return 1.0
            return 1.0

        def _dv(i: str, j: str) -> float:
            if delta_v_ms_fn: return float(delta_v_ms_fn(i, j))
            if {i, j} == {"Earth", "LEO"}: return 0.0
            if {i, j} == {"LEO", "LLO"}:   return 4040.0
            if {i, j} == {"LLO", "LS"}:    return 1870.0
            return 0.0

        arcs: List[ArcSpec] = []
        if old.is_path_graph:
            # build bidirectional transport between consecutive nodes in outbound_path
            path = old.outbound_path or old.node_names
            for u, v in zip(path, path[1:]):
                arcs.append(ArcSpec(u, v, "transport", tof_days=_tof(u, v), delta_v_ms=_dv(u, v),
                                    windows=[TimeWindow(role="outbound", starts=["mis_start"])]))
                arcs.append(ArcSpec(v, u, "transport", tof_days=_tof(v, u), delta_v_ms=_dv(v, u),
                                    windows=[TimeWindow(role="inbound", starts=["mis_end"])]))
            for n in old.holdover_nodes:
                arcs.append(ArcSpec(n, n, "holdover"))
        else:
            # non-path legacy is not fully supported; users should specify arcs explicitly
            raise NotImplementedError("Non-path legacy NodeDetails: please specify arcs explicitly in V2.")

        v2 = cls(
            node_names=old.node_names,
            is_path_graph=old.is_path_graph,
            holdover_nodes=old.holdover_nodes,
            outbound_path=old.outbound_path,
            nodes=nodes,
            arcs=arcs,
        )
        v2.validate()
        return v2

    def to_parent(self) -> NodeDetails:
        """
        Try to collapse V2 to a path-graph NodeDetails if it is equivalent
        (single simple chain, symmetric transport, standard windows).
        """
        # Attempt to detect a simple chain Earth->...->LS
        # Fallback: return a shallow copy of the legacy fields
        return NodeDetails(
            node_names=self.node_names,
            is_path_graph=True,
            holdover_nodes=[n.name for n in self.nodes if n.holdover],
            outbound_path=self.outbound_path if self.outbound_path else self.node_names
        )

class NetworkBuilderV2(NetworkBuilder):
    """
    Phase A: arc-centric internals while preserving legacy (i,j,t) views.

    - Reads NodeDetailsV2 (or adapts from legacy NodeDetails).
    - Builds arc list, allowed time windows, alpha (propellant fraction), TOF.
    - Backfills parent-visible arrays:
        fin_ini_mass_frac[i,j,t], real_arc_time[i,j], allowed_time_window[(i,j)], delta_t[i,j,t]
    - Keeps OptModelBuilder unchanged.
    - Phase A guard: at most ONE transport arc per (dep,arr) pair.
    """

    # ---------------- public ctor ----------------

    def __init__(self, input_data: InputData) -> None:
        # 親の初期化を呼んでおく（従来フィールドが必要な箇所があるため）
        super().__init__(input_data)

        # ---- Phase A: arc-centric structures (new) ----
        self.arc_list: List[ArcIdx] = []
        self.arcs_by_dep: Dict[int, List[int]] = defaultdict(list)
        self.arcs_by_arr: Dict[int, List[int]] = defaultdict(list)
        self.allowed_times_by_arc: Dict[int, List[int]] = {}

        # Isp (s)
        self._sc_isp = float(self.input.sc.isp)

        # 1) 時間軸が親で未確定なら、ここで確定（保険）
        self._initialize_time_axis_if_needed()

        # 2) NodeDetailsV2（なければ from_parent）からアークを構築
        node_cfg_v2 = self._get_node_cfg_v2()

        self._build_arcs_and_windows_from_node_details(node_cfg_v2)

        # 3) 親が期待する (i,j,t) ビューを上書き生成（互換）
        self._backfill_legacy_views(node_cfg_v2)

    # ------------- helpers (Phase A) -------------

    def _initialize_time_axis_if_needed(self) -> None:
        """
        親の __init__ が time_steps / mis_start_dates / mis_end_dates を
        既に作っていれば何もしない。なければ最小構成を作る。
        """
        if getattr(self, "time_steps", None) and getattr(self, "mis_start_dates", None):
            return

        mis = self.input.mission
        self.time_interval = int(mis.time_interval)
        self.n_mis = int(mis.n_mis)

        self.mis_start_dates = [m * self.time_interval for m in range(self.n_mis)]
        self.mis_end_dates = [s + 1 for s in self.mis_start_dates]

        # e.g., [0, 1, 365, 366, ...] （各ミッション：start/endの2スロット）
        self.time_steps = []
        for s, e in zip(self.mis_start_dates, self.mis_end_dates):
            self.time_steps += [s, e]

    def _get_node_cfg_v2(self) -> NodeDetailsV2:
        """legacy NodeDetails を渡された場合は NodeDetailsV2 にアダプト。"""
        node_cfg = self.input.node
        if isinstance(node_cfg, NodeDetails) and not isinstance(node_cfg, NodeDetailsV2):
            return NodeDetailsV2.from_parent(node_cfg)
        assert isinstance(node_cfg, NodeDetailsV2), "input_data.node must be NodeDetails or NodeDetailsV2"
        return node_cfg

    def _build_arcs_and_windows_from_node_details(self, node_cfg_v2: NodeDetailsV2) -> None:
        """NodeDetailsV2 から arc_list / allowed_times_by_arc を組み立て。"""
        name2id = {name: i for i, name in enumerate(node_cfg_v2.node_names)}

        # 1) ArcSpec → ArcIdx
        self.arc_list.clear()
        self.arcs_by_dep.clear()
        self.arcs_by_arr.clear()

        for aid, a in enumerate(node_cfg_v2.arcs):
            dep = name2id[a.dep]
            arr = name2id[a.arr]
            tof = float(a.tof_days or 0.0)
            dv = float(a.delta_v_ms or 0.0)
            if a.kind == "transport":
                alpha = 1.0 - math.exp(-dv / (self._sc_isp * G0))
            else:
                alpha = 0.0
            arc = ArcIdx(
                id=aid,
                dep=dep,
                arr=arr,
                kind=a.kind,
                tof_days=tof,
                dv_ms=dv,
                alpha=alpha,
                max_parallel=a.max_parallel,
                notes=a.notes,
            )
            self.arc_list.append(arc)
            self.arcs_by_dep[dep].append(aid)
            self.arcs_by_arr[arr].append(aid)

        # 2) 時刻窓（(dep,arr)->[t]）を展開し、arc_id へ割当
        allowed_map = node_cfg_v2.expand_allowed_times(
            mission_starts=self.mis_start_dates,
            mission_ends=self.mis_end_dates,
            time_grid=self.time_steps,
        )

        self.allowed_times_by_arc.clear()
        for arc in self.arc_list:
            key = (node_cfg_v2.node_names[arc.dep], node_cfg_v2.node_names[arc.arr])
            self.allowed_times_by_arc[arc.id] = allowed_map.get(key, [])

        # 3) Phase A ガード：同一 (dep,arr) の輸送アークが複数ならエラー
        seen: Dict[Tuple[int, int], int] = {}
        for arc in self.arc_list:
            if arc.kind == "transport":
                key = (arc.dep, arc.arr)
                if key in seen:
                    raise ValueError(
                        f"[Phase A] Multiple transport arcs for pair "
                        f"{node_cfg_v2.node_names[arc.dep]}->{node_cfg_v2.node_names[arc.arr]}.\n"
                        f"Use a single arc per pair (Phase B+ で対応)."
                    )
                seen[key] = arc.id

    def _backfill_legacy_views(self, node_cfg_v2: NodeDetailsV2) -> None:
        """
        親クラス／OptModelBuilder が参照する (i,j,t) 配列を
        arc_list から上書き構築して、動作を互換に保つ。
        """
        N = len(node_cfg_v2.node_names)
        T = len(self.time_steps)

        # 既存が期待する属性をすべて上書き作成
        self.fin_ini_mass_frac = np.zeros((N, N, T), dtype=float)   # α[i,j,t]
        self.real_arc_time = np.zeros((N, N), dtype=float)          # TOF[i,j]
        self.allowed_time_window: Dict[Tuple[int, int], List[int]] = {}
        self.delta_t = np.zeros((N, N, T), dtype=int)               # holdover 用

        # 1) 輸送アークの α・TOF・許可時刻を (i,j) に転写
        for arc in self.arc_list:
            i, j = arc.dep, arc.arr
            if arc.kind == "transport":
                self.fin_ini_mass_frac[i, j, :] = arc.alpha           # 時間一定なら全スライス同値
                self.real_arc_time[i, j] = arc.tof_days
                self.allowed_time_window[(i, j)] = list(self.allowed_times_by_arc.get(arc.id, []))

        # 2) ホールドオーバ自己ループの Δt（従来規約：start→interval-1, end→1）
        #    従来 NodeDetails.holdover_nodes がなければ NodeSpec.holdover から導出
        legacy_holdovers = set(getattr(self.input.node, "holdover_nodes", []) or [])
        if not legacy_holdovers and getattr(node_cfg_v2, "nodes", None):
            legacy_holdovers = {ns.name for ns in node_cfg_v2.nodes if getattr(ns, "holdover", False)}

        for n, name in enumerate(node_cfg_v2.node_names):
            if name in legacy_holdovers:
                for tidx, t in enumerate(self.time_steps):
                    if t in self.mis_start_dates:
                        self.delta_t[n, n, tidx] = self.time_interval - 1
                    elif t in self.mis_end_dates:
                        self.delta_t[n, n, tidx] = 1
                    else:
                        self.delta_t[n, n, tidx] = 1

        # 3) 互換ユーティリティ：feasible な (i,j) ペアの集合
        self._feasible_pair = {(arc.dep, arc.arr) for arc in self.arc_list}

    # 親の is_feasible_arc を、V2の feasible pair に置き換え（OptModelBuilder 互換のため）
    def is_feasible_arc(self, dep_node_id: int, arr_node_id: int) -> bool:
        return (dep_node_id, arr_node_id) in getattr(self, "_feasible_pair", set())