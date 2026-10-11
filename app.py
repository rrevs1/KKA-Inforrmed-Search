import streamlit as st
import folium
from streamlit_folium import st_folium

from greedybfs import (COORDS, SEGMENTS, TRANSFER_TIME, VMAX_KMH, build_graph, check_admissible,greedy_best_first_search)

try:
    from astar import astar_search
except ImportError:
    astar_search = None


PALET = ["#2e8b57", "#e67e22", "#8e44ad", "#c0392b", "#16a085", "#7f8c8d"]
DAFTAR_JALUR = sorted({seg[2] for seg in SEGMENTS})
WARNA_JALUR = {j: PALET[i % len(PALET)] for i, j in enumerate(DAFTAR_JALUR)}

GAYA_RUTE = {
    "Greedy": ("#d62728", 10, 0.55),
    "A*": ("#1f77b4", 4, 1.0),
}


def titik_rute(res):
    halte = []
    for nama, _ in res.path:
        if not halte or halte[-1] != nama:
            halte.append(nama)
    return halte


def teks_rute(res):
    return " → ".join(f"{h} [{l}]" for h, l in res.path)


def buat_peta(hasil):
    lat_tengah = sum(c[0] for c in COORDS.values()) / len(COORDS)
    lon_tengah = sum(c[1] for c in COORDS.values()) / len(COORDS)
    peta = folium.Map(location=[lat_tengah, lon_tengah], zoom_start=12)

    for a, b, jalur, t in SEGMENTS:
        folium.PolyLine([COORDS[a], COORDS[b]], color=WARNA_JALUR[jalur],
                        weight=3, opacity=0.5,
                        tooltip=f"{jalur} ({t} menit)").add_to(peta)

    for nama, (warna, tebal, opa) in GAYA_RUTE.items():
        res = hasil.get(nama)
        if res is None or not res.found:
            continue
        grup = folium.FeatureGroup(name=f"Rute {nama}")
        folium.PolyLine([COORDS[h] for h in titik_rute(res)], color=warna,
                        weight=tebal, opacity=opa,
                        tooltip=f"{nama}: {res.total_time:.0f} menit").add_to(grup)
        grup.add_to(peta)

    for nama, koor in COORDS.items():
        folium.CircleMarker(koor, radius=6, color="black", weight=1,
                            fill=True, fill_color="white", fill_opacity=1,
                            tooltip=nama).add_to(peta)

    folium.Marker(COORDS[hasil["start"]], icon=folium.Icon(color="green"), tooltip=f"Asal: {hasil['start']}").add_to(peta)
    folium.Marker(COORDS[hasil["goal"]], icon=folium.Icon(color="red"), tooltip=f"Tujuan: {hasil['goal']}").add_to(peta)

    folium.LayerControl().add_to(peta)
    return peta


def buat_tabel(hasil):
    baris = []
    for nama in ("Greedy", "A*"):
        res = hasil.get(nama)
        if res is None:
            continue
        if not res.found:
            baris.append({"Algoritma": nama, "Total waktu (menit)": "rute tidak ditemukan"})
            continue
        baris.append({
            "Algoritma": nama,
            "Total waktu (menit)": round(res.total_time, 1),
            "Pindah jalur": res.transfers,
            "Node expanded": res.expanded,
            "Node generated": res.generated,
            "Max fringe": res.max_fringe,
            "Runtime (ms)": round(res.runtime_ms, 3),
        })
    return baris


def jalankan_pencarian(start, goal, transfer):
    graph, lines_at = build_graph(SEGMENTS, transfer)
    hasil = {"start": start, "goal": goal}
    hasil["Greedy"] = greedy_best_first_search(graph, lines_at, COORDS, start, goal, VMAX_KMH)
    if astar_search is not None:
        hasil["A*"] = astar_search(graph, lines_at, COORDS, start, goal, VMAX_KMH)
    return hasil


def tampilkan_hasil(hasil):
    if astar_search is None:
        st.warning("File astar.py belum ada, jadi baru rute Greedy yang ditampilkan.")

    kolom_peta, kolom_info = st.columns([3, 2])

    with kolom_peta:
        legend = " &nbsp; ".join(
            f"<span style='color:{w}; font-weight:bold'>━ {n}</span>"
            for n, w in WARNA_JALUR.items()
        )
        st.markdown(legend, unsafe_allow_html=True)
        st_folium(buat_peta(hasil), height=600, use_container_width=True, returned_objects=[])

    with kolom_info:
        st.subheader("Perbandingan")
        st.dataframe(buat_tabel(hasil), hide_index=True)

        g, a = hasil.get("Greedy"), hasil.get("A*")
        if g is not None and a is not None and g.found and a.found:
            selisih = g.total_time - a.total_time
            if selisih > 1e-9:
                st.success(f"A* lebih cepat {selisih:.0f} menit dibanding Greedy.")
            else:
                st.info("Greedy dan A* menghasilkan waktu yang sama.")

        for nama in ("Greedy", "A*"):
            res = hasil.get(nama)
            if res is not None and res.found:
                st.markdown(f"**Rute {nama}**")
                st.write(teks_rute(res))


def main():
    st.set_page_config(page_title="Rute Transportasi Umum Surabaya", layout="wide")
    st.title("Rute Transportasi Umum Surabaya")
    st.caption("Perbandingan Greedy Best-First Search dan A*")

    check_admissible(SEGMENTS, COORDS, VMAX_KMH)
    names = sorted(COORDS)

    with st.sidebar:
        st.header("Input")
        start = st.selectbox("Halte asal", names, index=0)
        goal = st.selectbox("Halte tujuan", names, index=len(names) - 1)
        transfer = st.slider("Waktu pindah jalur (menit)", 0.0, 20.0, float(TRANSFER_TIME), 0.5)
        cari = st.button("Cari rute", type="primary")

    if cari:
        if start == goal:
            st.warning("Halte asal dan tujuan sama, pilih yang berbeda.")
            st.session_state.pop("hasil", None)
        else:
            st.session_state["hasil"] = jalankan_pencarian(start, goal, transfer)

    hasil = st.session_state.get("hasil")
    if hasil is None:
        st.info("Pilih halte asal dan tujuan di sebelah kiri, lalu klik Cari rute.")
    else:
        tampilkan_hasil(hasil)


if __name__ == "__main__":
    main()
