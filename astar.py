import heapq
import itertools
import math
import time
from collections import defaultdict
from dataclasses import dataclass, field


def haversine_km(a, b):
    R = 6371.0088
    lat1, lon1 = map(math.radians, a)
    lat2, lon2 = map(math.radians, b)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    x = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * R * math.asin(math.sqrt(x))


def heuristic(halte, goal_halte, coords, vmax_kmh):
    return haversine_km(coords[halte], coords[goal_halte]) / vmax_kmh * 60.0


def build_graph(segments, transfer_time):
    graph = defaultdict(list)
    lines_at = defaultdict(set)

    for a, b, line, t in segments:
        if t < 0:
            raise ValueError(f"Waktu negatif di segmen {a}-{b} ({line})")
        graph[(a, line)].append(((b, line), t))
        graph[(b, line)].append(((a, line), t))
        lines_at[a].add(line)
        lines_at[b].add(line)

    for halte, lines in lines_at.items():
        for l1 in sorted(lines):
            for l2 in sorted(lines):
                if l1 != l2:
                    graph[(halte, l1)].append(((halte, l2), transfer_time))

    return graph, lines_at


def check_admissible(segments, coords, vmax_kmh, tol=1e-9):
    problems = []
    for a, b, line, t in segments:
        min_time = haversine_km(coords[a], coords[b]) / vmax_kmh * 60.0
        if t + tol < min_time:
            problems.append((a, b, line, t, round(min_time, 2)))
    if problems:
        msg = "\n".join(
            f"  {a}-{b} ({ln}): waktu data={t} menit, minimal harusnya {m} menit"
            for a, b, ln, t, m in problems
        )
        raise ValueError("Nilai heuristik tidak valid (melebihi waktu asli) untuk data ini:\n" + msg)


@dataclass
class Node:
    state: tuple
    parent: "Node | None" = None
    g: float = 0.0


@dataclass
class Result:
    found: bool
    path: list = field(default_factory=list)
    total_time: float = math.inf
    transfers: int = 0
    expanded: int = 0
    generated: int = 0
    max_fringe: int = 0
    runtime_ms: float = 0.0


def astar_search(graph, lines_at, coords, start_halte, goal_halte, vmax_kmh=60.0):
    for h_ in (start_halte, goal_halte):
        if h_ not in lines_at:
            raise ValueError(f"Halte '{h_}' tidak ada pada graph")

    t0 = time.perf_counter()
    counter = itertools.count()
    fringe = []
    closed = set()
    expanded = generated = max_fringe = 0

    def push(node):
        nonlocal generated, max_fringe
        h = heuristic(node.state[0], goal_halte, coords, vmax_kmh)
        f = node.g + h  # <-- satu-satunya beda dari GBFS (GBFS: f = h)
        heapq.heappush(fringe, (f, next(counter), node))
        generated += 1
        max_fringe = max(max_fringe, len(fringe))

    for line in sorted(lines_at[start_halte]):
        push(Node((start_halte, line)))

    while fringe:
        _, _, node = heapq.heappop(fringe)

        if node.state[0] == goal_halte:
            return _make_result(node, expanded, generated, max_fringe, t0)

        if node.state in closed:
            continue
        closed.add(node.state)
        expanded += 1

        for nxt_state, cost in graph[node.state]:
            if nxt_state not in closed:
                push(Node(nxt_state, node, node.g + cost))

    return Result(False, expanded=expanded, generated=generated,
                  max_fringe=max_fringe, runtime_ms=(time.perf_counter() - t0) * 1000)


def _make_result(goal_node, expanded, generated, max_fringe, t0):
    path = []
    n = goal_node
    while n is not None:
        path.append(n.state)
        n = n.parent
    path.reverse()

    transfers = sum(1 for a, b in zip(path, path[1:]) if a[1] != b[1])
    return Result(True, path, goal_node.g, transfers, expanded, generated, max_fringe,
                  (time.perf_counter() - t0) * 1000)


def print_result(res, start, goal):
    print(f"=== A*: {start} -> {goal} ===")
    if not res.found:
        print("Rute tidak ditemukan")
    else:
        print("Rute        :", " -> ".join(f"{h}[{l}]" for h, l in res.path))
        print(f"Total waktu : {res.total_time:.1f} menit")
        print(f"Pindah moda : {res.transfers}x")
    print(f"Expanded    : {res.expanded} | Generated: {res.generated} | Max fringe: {res.max_fringe}")
    print(f"Runtime     : {res.runtime_ms:.3f} ms\n")


COORDS = {
    "Tugu Pahlawan":           (-7.2459, 112.7378),
    "Tunjungan Plaza":         (-7.2625, 112.7399),
    "Stasiun Gubeng":          (-7.2651, 112.7523),
    "Monumen Kapal Selam":     (-7.2652, 112.7508),
    "Galaxy Mall":             (-7.2753, 112.7803),
    "ITS":                     (-7.2819, 112.7949),
    "Taman Bungkul":           (-7.2912, 112.7397),
    "Kebun Binatang Surabaya": (-7.2958, 112.7366),
    "Royal Plaza":             (-7.3125, 112.7342),
    "Universitas Airlangga":   (-7.2699, 112.7580),
    "Ciputra World":           (-7.2917, 112.7185),
}

SEGMENTS = [
    ("Royal Plaza",             "Kebun Binatang Surabaya", "Suroboyo Bus R1",    6),
    ("Kebun Binatang Surabaya", "Taman Bungkul",           "Suroboyo Bus R1",    4),
    ("Taman Bungkul",           "Tunjungan Plaza",         "Suroboyo Bus R1",   12),
    ("Tunjungan Plaza",         "Tugu Pahlawan",           "Suroboyo Bus R1",    8),
    ("Ciputra World",           "Universitas Airlangga",   "Trans Semanggi K2L", 22),
    ("Universitas Airlangga",   "Galaxy Mall",             "Trans Semanggi K2L", 11),
    ("Galaxy Mall",             "ITS",                     "Trans Semanggi K2L",  8),
    ("Stasiun Gubeng",          "Monumen Kapal Selam",     "Jalan Kaki",          3),
    ("Stasiun Gubeng",          "Universitas Airlangga",   "Jalan Kaki",         15),
    ("Stasiun Gubeng",          "Tunjungan Plaza",         "Jalan Kaki",         20),
    ("Stasiun Gubeng",          "Taman Bungkul",           "Angkot",            18),
    ("Monumen Kapal Selam",     "Tunjungan Plaza",         "Angkot",             8),
    ("Taman Bungkul",           "Royal Plaza",             "Angkot",            12),
    ("Taman Bungkul",           "Ciputra World",           "Angkot",            10),
]

TRANSFER_TIME = 5.0
VMAX_KMH = 60.0


def pilih_halte(message, names):
    lookup = {n.lower(): n for n in names}
    while True:
        try:
            teks = input(message).strip()
        except EOFError:
            return None
        if teks.lower() in ("stop", "quit", "keluar"):
            return None
        if teks.isdigit() and 1 <= int(teks) <= len(names):
            return names[int(teks) - 1]
        if teks.lower() in lookup:
            return lookup[teks.lower()]
        print("  Input tidak dikenali. Ketik nomor/nama halte, atau 'stop' untuk keluar.")


def main():
    check_admissible(SEGMENTS, COORDS, VMAX_KMH)
    graph, lines_at = build_graph(SEGMENTS, TRANSFER_TIME)
    names = sorted(COORDS)

    print("=== A* Search - Rute Transportasi Umum Surabaya ===")
    print("Daftar halte:")
    for i, n in enumerate(names, 1):
        print(f"  {i}. {n}")
    print("(ketik nomor atau nama halte; 'stop' untuk keluar)\n")

    while True:
        start = pilih_halte("Halte asal   : ", names)
        if start is None:
            break
        goal = pilih_halte("Halte tujuan : ", names)
        if goal is None:
            break
        print()
        res = astar_search(graph, lines_at, COORDS, start, goal, VMAX_KMH)
        print_result(res, start, goal)


if __name__ == "__main__":
    main()