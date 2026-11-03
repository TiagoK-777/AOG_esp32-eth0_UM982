#!/usr/bin/env python3
"""Reproduz as figuras do ensaio de 30–31/10/2026 pelo JSON Ploty original.

SPDX-License-Identifier: GPL-3.0-only
Uso: python tools/plot_stability.py sessao.json --output docs/performance/2026-10-30
Dependência externa: matplotlib (inclui NumPy).
"""

import argparse
import csv
import hashlib
import json
from collections import Counter
from functools import cache
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MaxNLocator


ERROR_KEYS = [
    "gps.nmeaErr", "gps.crcErr", "gps.fifoOvf", "gps.rxFull",
    "gps.frameErr", "gps.parityErr", "gps.buildFail", "gps.txFail",
    "net.txDrops", "net.rtkDrops", "net.sendFail", "net.invalid",
]
TASKS = {"control": "Controle", "navigation": "Navegação",
         "network": "Rede", "safety": "Supervisor"}
COLORS = ["#0072B2", "#D55E00", "#009E73", "#8753A3"]
SOURCE_SHA256 = "fbf411cede67958eed348d72cef6e16044c1f085f05724574834b73310bfb293"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    payload = args.source.read_bytes()
    # As legendas incluem condições e resultados deste ensaio específico.
    if hashlib.sha256(payload).hexdigest() != SOURCE_SHA256:
        raise ValueError("Este script requer o JSON original do ensaio de 30–31/10/2026. "
                         "Para outro ensaio, revisar também as legendas das figuras.")
    data = json.loads(payload)
    samples = data["samples"]
    if len(samples) < 2:
        raise ValueError("São necessários pelo menos dois relatórios.")
    if len({s["boot"] for s in samples}) != 1:
        raise ValueError("Separe os boots antes de gerar o relatório.")
    if any(not s.get("complete") or s.get("estimated") for s in samples):
        raise ValueError("A sessão contém relatórios incompletos ou tempo estimado.")

    @cache
    def values(key):
        return np.array([s["values"][key] for s in samples], dtype=float)

    uptime = np.array([s["uptimeMs"] for s in samples], dtype=np.int64)
    dt = np.diff(uptime)
    if np.any(dt <= 0):
        raise ValueError("Uptime não estritamente crescente; verificar reset/wrap.")
    windows = np.array([s["windowMs"] for s in samples], dtype=np.int64)
    if not np.array_equal(dt, windows[1:]):
        raise ValueError("Há lacunas entre relatórios; não agregar como captura contínua.")
    for key in ERROR_KEYS + ["gps.gga", "gps.panda", "gps.queued", "gps.tx", "gps.hpr"]:
        if np.any(np.diff(values(key)) < 0):
            raise ValueError(f"Contador não monotônico: {key}")

    elapsed = uptime - uptime[0]
    hours = elapsed / 3_600_000
    # Exclui o primeiro relatório das diferenças: sua janela precede a captura.
    blocks = [np.arange(i, min(i + 12, len(samples)))
              for i in range(1, len(samples), 12)]
    block_h = np.array([(elapsed[b[0] - 1] + elapsed[b[-1]]) / 7_200_000
                        for b in blocks])
    block_ms = np.array([int(dt[b - 1].sum()) for b in blocks])

    def reduce(key, operation="max"):
        v = values(key)
        return np.array([getattr(np, operation)(v[b]) for b in blocks])

    def rate(key):
        delta = np.diff(values(key))
        return np.array([delta[b - 1].sum() * 1000 / ms
                         for b, ms in zip(blocks, block_ms)])

    def extrema(key):
        v = values(key)
        return {"first": float(v[0]), "last": float(v[-1]),
                "min": float(v.min()), "max": float(v.max())}

    summary = {
        "source": {"file": args.source.name, "sha256": hashlib.sha256(payload).hexdigest(),
                   "bytes": len(payload), "schema": data["schema"]},
        "reports": len(samples), "complete_reports": len(samples), "boot_segments": 1,
        "received_at_utc": {"first": samples[0]["receivedAt"],
                            "last": samples[-1]["receivedAt"]},
        "uptime_ms": {"first": int(uptime[0]), "last": int(uptime[-1])},
        "duration_ms": int(elapsed[-1]),
        "diagnostic_window_ms": {"min": int(windows.min()), "max": int(windows.max())},
        "aggregation": {"windows_per_block": 12, "blocks": len(blocks),
                        "last_block_windows": len(blocks[-1]),
                        "first_window_excluded_from_deltas": True},
        "gps": {}, "errors": {}, "tasks": {},
        "metrics": {}, "steer_sample_values": {},
    }
    for key in ["gga", "panda", "queued", "tx", "hpr"]:
        v = values("gps." + key)
        summary["gps"][key] = {
            "first": int(v[0]), "last": int(v[-1]), "delta": int(v[-1] - v[0]),
            "mean_hz": float((v[-1] - v[0]) * 1000 / elapsed[-1]),
        }
        if key in ["gga", "panda", "tx"]:
            summary["gps"][key]["reported_hz"] = extrema("gps." + key + "Hz")
            summary["gps"][key]["messages_per_window"] = dict(
                sorted(Counter(int(x) for x in np.diff(v)).items()))
    hpr_changes = np.flatnonzero(np.diff(values("gps.hpr")) > 0) + 1
    summary["gps"]["hpr"]["last_change_elapsed_ms"] = (
        int(elapsed[hpr_changes[-1]]) if len(hpr_changes) else None)
    for key in ERROR_KEYS:
        v = values(key)
        summary["errors"][key] = {"first": int(v[0]), "last": int(v[-1]),
                                  "delta": int(v[-1] - v[0])}
    for task in TASKS:
        summary["tasks"][task] = {
            field: extrema(f"task.{task}.{field}")
            for field in ["hz", "peakUs", "gapUs", "lateMs", "stack"]}
        for field in ["late", "over"]:
            # Todos os relatórios contêm acumuladores por janela, inclusive o primeiro.
            summary["tasks"][task][field + "_sum_all_reports"] = int(
                values(f"task.{task}.{field}").sum())
        nominal_us = 20_000 if task == "safety" else 1000
        summary["tasks"][task]["nominal_period_us"] = nominal_us
        summary["tasks"][task]["max_positive_interval_jitter_us"] = max(
            0, int(values(f"task.{task}.gapUs").max()) - nominal_us)
    for key in ["heap", "cycle.peak", "cycle.gapMax", "cycle.lateMax", "ads.peak",
                "control.peak", "gps.gapGga", "gps.gapPanda", "gps.gapTx",
                "motor.drive.totalMax", "motor.drive.waitMax"]:
        summary["metrics"][key] = extrema(key)
    for key in ["cycle.late", "cycle.over25ms"]:
        summary["metrics"][key] = {**extrema(key),
                                    "sum_all_reports": int(values(key).sum())}
    for key in ["available", "engaged", "mode", "guidance", "speed", "pwm"]:
        summary["steer_sample_values"][key] = dict(Counter(
            str(s["values"]["steer." + key]) for s in samples))

    out = args.output
    out.mkdir(parents=True, exist_ok=True)
    (out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    with (out / "minute-summary.csv").open("w", newline="") as f:
        fields = ["elapsed_start_s", "elapsed_end_s", "window_ms", "reports",
                  "gga_hz", "panda_hz", "udp_accepted_hz", "gga_reported_hz_min",
                  "gga_reported_hz_max", "heap_min_bytes", "heap_max_bytes",
                  "cycle_peak_max_us", "cycle_gap_max_us", "ads_peak_max_us",
                  "hpr_delta", "invalid_delta", "cycle_late_sum", "cycle_over25ms_sum"]
        writer = csv.writer(f)
        writer.writerow(fields)
        rates = [rate("gps." + key) for key in ["gga", "panda", "tx"]]
        for i, b in enumerate(blocks):
            first, last = b[0] - 1, b[-1]
            writer.writerow([
                elapsed[first] / 1000, elapsed[last] / 1000, block_ms[i], len(b),
                *(round(r[i], 9) for r in rates),
                values("gps.ggaHz")[b].min(), values("gps.ggaHz")[b].max(),
                int(values("heap")[b].min()), int(values("heap")[b].max()),
                *(int(values(k)[b].max()) for k in ["cycle.peak", "cycle.gapMax", "ads.peak"]),
                int(values("gps.hpr")[last] - values("gps.hpr")[first]),
                int(values("net.invalid")[last] - values("net.invalid")[first]),
                int(values("cycle.late")[b].sum()), int(values("cycle.over25ms")[b].sum()),
            ])

    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 11, "axes.titlesize": 13,
        "axes.titleweight": "bold", "axes.labelcolor": "#334155",
        "axes.edgecolor": "#CBD5E1", "axes.spines.top": False,
        "axes.spines.right": False, "grid.color": "#E2E8F0", "grid.alpha": 0.9,
        "figure.facecolor": "white", "savefig.facecolor": "white",
        "path.simplify": False,
    })
    duration = int(elapsed[-1] // 1000)
    stamp = f"{duration // 3600:02d}h{duration % 3600 // 60:02d}min{duration % 60:02d}s · {len(samples):,} relatórios".replace(",", ".")

    def figure(title, subtitle, rows=2, height=8):
        fig, axes = plt.subplots(rows, 1, figsize=(12, height), squeeze=False)
        fig.subplots_adjust(top=0.82, bottom=0.13, left=0.09, right=0.97, hspace=0.58)
        fig.text(0.09, 0.96, title, fontsize=18, fontweight="bold", color="#0F172A", va="top")
        fig.text(0.09, 0.905, subtitle, fontsize=11, color="#475569", va="top")
        fig.text(0.09, 0.035, f"Ensaio sintético em bancada · 30–31/10/2026 · {stamp}",
                 color="#475569", fontsize=10)
        for ax in axes[:, 0]:
            ax.grid(True, axis="both")
            ax.set_axisbelow(True)
        return fig, axes[:, 0]

    def timeline(ax):
        ax.set_xlim(0, float(hours[-1]))
        ax.set_xticks(np.arange(0, 25, 4))
        ax.set_xlabel("Tempo desde o primeiro relatório (h)")

    def save(fig, filename):
        fig.savefig(out / filename, dpi=160)
        plt.close(fig)

    fig, ax = figure("GPS: taxa sustentada e intervalos entre eventos",
                      "GPS simulado a 10 Hz · média agregada por contagem / tempo · UDP = envio aceito pela pilha")
    ax[0].fill_between(block_h, reduce("gps.ggaHz", "min"), reduce("gps.ggaHz"),
                       color=COLORS[0], alpha=0.12, label="Faixa das taxas por janela (~5 s)")
    for key, label, color, style in zip(["gga", "panda", "tx"],
            ["GGA aceita", "PANDA/PAOGI produzida", "UDP aceito"], COLORS, ["-", "--", ":"]):
        ax[0].plot(block_h, rate("gps." + key), label=label + " · ~60 s",
                   color=color, linestyle=style, linewidth=1.5)
    ax[0].axhline(10, color="#64748B", linestyle="--", linewidth=0.8)
    ax[0].set_ylim(9.97, 10.22)
    ax[0].set_ylabel("Taxa (Hz)")
    ax[0].set_title("Média do período: 10,00013 Hz nos três estágios", loc="left")
    ax[0].legend(loc="upper center", ncol=2, fontsize=9)
    for key, label, color in zip(["gps.gapGga", "gps.gapPanda", "gps.gapTx"],
            ["GGA · máx. 101,607 ms", "PANDA · máx. 101,642 ms", "UDP · máx. 103,553 ms"], COLORS):
        ax[1].step(hours, values(key) / 1000, where="post", label=label, color=color, linewidth=1.3)
    ax[1].set_title("Maiores intervalos acumulados desde o boot", loc="left")
    ax[1].set_ylabel("Intervalo máximo (ms)")
    ax[1].set_ylim(99.5, 104.5)
    ax[1].legend(loc="lower right", fontsize=9)
    for a in ax:
        timeline(a)
    save(fig, "01-gps.png")

    fig, ax = figure("Por que aparecem espinhos na taxa de 10 Hz?",
                      "Contagem inteira em janelas próximas de 5 s · dados originais preservados")
    select = (elapsed > 0) & (elapsed <= 600_000)
    ax[0].plot(elapsed[select] / 60_000, values("gps.ggaHz")[select],
               color=COLORS[0], marker=".", markersize=3, linewidth=1)
    ax[0].axhline(10, color="#64748B", linestyle="--", linewidth=1)
    ax[0].set_title("Detalhe dos primeiros 10 minutos · GGA", loc="left")
    ax[0].set_xlabel("Tempo desde o primeiro relatório (min)")
    ax[0].set_ylabel("Taxa reportada (Hz)")
    ax[0].set_ylim(9.96, 10.23)
    count = Counter(np.diff(values("gps.gga")).astype(int))
    bars = ax[1].bar([str(k) for k in sorted(count)], [count[k] for k in sorted(count)],
                     color=[COLORS[0], COLORS[1]], width=0.45)
    ax[1].bar_label(bars, labels=[f"{count[k]:,} janelas".replace(",", ".") for k in sorted(count)], padding=4)
    ax[1].set_title("Todas as 17.292 diferenças entre relatórios", loc="left")
    ax[1].set_xlabel("Mensagens GGA por janela")
    ax[1].set_ylabel("Quantidade de janelas")
    ax[1].set_ylim(0, max(count.values()) * 1.3)
    ax[1].text(0.96, 0.65, "50 / 5,004 s ≈ 9,99 Hz\n51 / 5,004 s ≈ 10,19 Hz\n50 / 5,008 s ≈ 9,98 Hz",
               transform=ax[1].transAxes, ha="right", fontsize=12,
               bbox={"facecolor": "#F1F5F9", "edgecolor": "none", "pad": 10})
    save(fig, "02-janelas-hz.png")

    fig, ax = figure("Controle: execução e espaçamento dos ciclos",
                      "Cada ponto preserva o maior valor de até 12 relatórios (~60 s) · tempos de relógio")
    for key, label, color in [("cycle.peak", "Ciclo de controle · máx. 2,424 ms", COLORS[0]),
                              ("ads.peak", "Bloco ADS1115 · máx. 2,013 ms", COLORS[1])]:
        ax[0].plot(block_h, reduce(key) / 1000, label=label, color=color, linewidth=1)
    ax[0].set_ylim(0, 3)
    ax[0].set_ylabel("Duração máxima (ms)")
    ax[0].set_title("Zero ciclos com duração superior a 25 ms nos relatórios", loc="left")
    ax[0].legend(loc="lower right", fontsize=9)
    ax[1].plot(block_h, reduce("cycle.gapMax") / 1000, color=COLORS[0], linewidth=1,
               label="Intervalo entre inícios · máx. 26,334 ms")
    ax[1].axhline(25, color=COLORS[1], linestyle="--", label="Gate nominal: 25 ms")
    ax[1].set_ylim(24.7, 26.7)
    ax[1].set_ylabel("Intervalo máximo (ms)")
    ax[1].set_title("Espaçamento inclui atrasos de agendamento", loc="left")
    ax[1].legend(loc="lower right", fontsize=9)
    for a in ax:
        timeline(a)
    save(fig, "03-controle.png")

    fig, ax = figure("Tasks: desempenho e jitter de intervalo",
                      "Máximos em blocos de ~60 s · controle, navegação e rede: 1 ms nominal; supervisor: 20 ms")
    for task, label, color in zip(TASKS, TASKS.values(), COLORS):
        ax[0].plot(block_h, reduce(f"task.{task}.peakUs") / 1000, label=label, color=color, linewidth=1)
        nominal_ms = 20 if task == "safety" else 1
        ax[1].plot(block_h, np.maximum(0, reduce(f"task.{task}.gapUs") / 1000 - nominal_ms),
                   label=label, color=color, linewidth=1)
    ax[0].axhline(1, color="#64748B", linestyle="--", linewidth=0.8)
    ax[0].set_ylabel("Duração máxima (ms)")
    ax[0].set_ylim(0, 4.7)
    ax[0].set_title("Tempos incluem esperas e preempções dentro do handler", loc="left")
    ax[0].legend(ncol=4, fontsize=9, loc="upper right")
    ax[1].set_ylabel("Excesso de intervalo (ms)")
    ax[1].set_ylim(0, 3.6)
    ax[1].set_title("Jitter positivo = máximo intervalo entre inícios − período nominal", loc="left")
    for a in ax:
        timeline(a)
    save(fig, "04-tasks.png")

    fig, ax = figure("Memória e contadores de erro",
                      "Heap: faixa mínima–máxima por bloco de ~60 s · contadores de erro acumulados")
    ax[0].fill_between(block_h, reduce("heap", "min") / 1024, reduce("heap") / 1024,
                       color=COLORS[0], alpha=0.25, label="Faixa do heap observado")
    ax[0].plot(block_h, reduce("heap", "min") / 1024, color=COLORS[0], linewidth=0.8)
    ax[0].set_ylim(196.5, 200)
    ax[0].set_ylabel("Heap livre (KiB)")
    ax[0].set_title("Inicial: 204.148 B · final: 204.160 B · mínimo: 202.028 B", loc="left")
    ax[0].legend(loc="lower right", fontsize=9)
    ax[1].step(hours, values("net.invalid"), where="post", color=COLORS[1],
               label="Pacotes inválidos de rede: 164")
    ax[1].plot(hours, values("gps.nmeaErr"), color=COLORS[0], linestyle="--",
               label="Erros GPS, descartes TX/RTK e falhas de envio: 0")
    ax[1].set_ylabel("Contagem acumulada")
    ax[1].set_ylim(-8, 185)
    ax[1].set_title("Pacotes inválidos são uma métrica separada dos erros de GPS", loc="left")
    ax[1].yaxis.set_major_locator(MaxNLocator(integer=True))
    ax[1].legend(loc="upper left", fontsize=9)
    for a in ax:
        timeline(a)
    save(fig, "05-memoria-erros.png")
    print(json.dumps({"reports": len(samples), "duration_ms": int(elapsed[-1]),
                      "source_sha256": summary["source"]["sha256"],
                      "output": str(out)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
