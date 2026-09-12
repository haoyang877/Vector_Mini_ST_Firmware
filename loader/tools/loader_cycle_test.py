"""Run repeated APP-to-Loader handoff and firmware-update endurance cycles."""

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import statistics
import time

from loader_updater import DEFAULT_DLL, LoaderBus, LoaderClient, update_firmware

CSV_FIELDS = ("cycle", "t_start_iso", "loader_ms", "update_ms", "app_ms", "ok")


def _write_csv_row(path: Path, row: dict[str, str | int | float]) -> None:
    write_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        if write_header:
            writer.writeheader()
        writer.writerow(row)


def main() -> int:  # noqa: BROAD_EXCEPT_OK
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--node", type=int, required=True)
    parser.add_argument("--count", type=int, default=1000)
    parser.add_argument("--logdir", type=Path,
                        default=Path("outputs") / f"loader_cycles_{datetime.now():%Y%m%d_%H%M%S}")
    parser.add_argument("--dll", type=Path, default=DEFAULT_DLL)
    parser.add_argument("--start-cycle", type=int, default=1)
    args = parser.parse_args()
    if args.count <= 0 or args.start_cycle <= 0:
        parser.error("--count and --start-cycle must be positive")

    args.logdir.mkdir(parents=True, exist_ok=True)
    csv_path = args.logdir / "cycles.csv"
    cycle_times: list[float] = []
    success_count = 0
    failed_count = 0
    run_started = time.perf_counter()
    bus: LoaderBus | None = None
    close_error: str | None = None
    setup_started = time.perf_counter()
    setup_started_iso = datetime.now(timezone.utc).isoformat()

    try:
        image = args.image.read_bytes()
        bus = LoaderBus(args.dll)
        client = LoaderClient(bus, node=args.node)
        for cycle in range(args.start_cycle, args.start_cycle + args.count):
            cycle_started = time.perf_counter()
            started_iso = datetime.now(timezone.utc).isoformat()
            loader_ms = 0.0
            update_ms = 0.0
            app_ms = 0.0
            phase = "loader"
            try:
                loader_started = time.perf_counter()
                client.enter_boot()
                client.wait_loader_alive(timeout=5.0)
                loader_ms = (time.perf_counter() - loader_started) * 1000

                phase = "update"
                update_started = time.perf_counter()
                update_firmware(client, image, version=cycle)
                update_ms = (time.perf_counter() - update_started) * 1000

                phase = "app"
                app_started = time.perf_counter()
                client.wait_app_alive(timeout=5.0)
                app_ms = (time.perf_counter() - app_started) * 1000
                success_count += 1
                ok = 1
            except Exception as exc:  # noqa: BROAD_EXCEPT_OK
                if phase == "loader":
                    loader_ms = (time.perf_counter() - loader_started) * 1000
                elif phase == "update":
                    update_ms = (time.perf_counter() - update_started) * 1000
                else:
                    app_ms = (time.perf_counter() - app_started) * 1000
                failed_count += 1
                ok = 0
                failure = {
                    "cycle": cycle,
                    "t_start_iso": started_iso,
                    "error": repr(exc),
                    "loader_ms": loader_ms,
                    "update_ms": update_ms,
                    "app_ms": app_ms,
                    "communication_errors": client.errors,
                }
                (args.logdir / f"failure_{cycle}.json").write_text(
                    json.dumps(failure, indent=2), encoding="utf-8")

            cycle_ms = (time.perf_counter() - cycle_started) * 1000
            cycle_times.append(cycle_ms)
            _write_csv_row(csv_path, {
                "cycle": cycle,
                "t_start_iso": started_iso,
                "loader_ms": round(loader_ms, 3),
                "update_ms": round(update_ms, 3),
                "app_ms": round(app_ms, 3),
                "ok": ok,
            })
            if len(cycle_times) % 5 == 0:
                rolling = statistics.fmean(cycle_times[-5:])
                print(f"cycle {cycle}: {success_count} succeeded, rolling average {rolling:.1f} ms")
            if ok == 0:
                break
    except Exception as exc:  # noqa: BROAD_EXCEPT_OK
        failed_count += 1
        cycle_ms = (time.perf_counter() - setup_started) * 1000
        cycle_times.append(cycle_ms)
        failure = {"cycle": args.start_cycle, "t_start_iso": setup_started_iso, "error": repr(exc)}
        (args.logdir / f"failure_{args.start_cycle}.json").write_text(
            json.dumps(failure, indent=2), encoding="utf-8")
        _write_csv_row(csv_path, {"cycle": args.start_cycle, "t_start_iso": setup_started_iso,
                                  "loader_ms": 0.0, "update_ms": 0.0, "app_ms": 0.0, "ok": 0})
    finally:
        if bus is not None:
            try:
                bus.close()
            except OSError as exc:
                close_error = repr(exc)

    elapsed = time.perf_counter() - run_started
    summary = {
        "total": len(cycle_times),
        "ok": success_count,
        "failed": failed_count,
        "avg_cycle_ms": statistics.fmean(cycle_times) if cycle_times else 0.0,
        "median_cycle_ms": statistics.median(cycle_times) if cycle_times else 0.0,
        "elapsed_s": elapsed,
    }
    if close_error is not None:
        summary["close_error"] = close_error
    (args.logdir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if failed_count == 0 and close_error is None else 1


if __name__ == "__main__":
    raise SystemExit(main())
