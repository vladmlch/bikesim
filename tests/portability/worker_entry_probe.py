"""Guarded worker/BLAS lifecycle probe; run explicitly in the selected uv environment.

This is a test helper, not an application entry point. Its alarm bounds both
startup and collect(), and the parent test imposes a second process timeout.
"""
import argparse
import json
import multiprocessing as mp
from pathlib import Path
import platform
import signal
import sys
import threading


ROOT = Path(__file__).resolve().parents[2]


def _timeout(signum, frame):
    raise TimeoutError('worker entry probe exceeded 45 seconds')


def _exercise(start_method, interrupt):
    import numpy as np
    from bike_sim.sim.ride.async_close import AsyncCloseRunner

    # Reuse the synthetic public-data fixture without creating a MuJoCo model.
    # No pytest tests execute when their module is imported.
    sys.path.insert(0, str(ROOT / 'tests'))
    from test_async_close import _raw, _synthetic_ctx

    ready, stop = threading.Event(), threading.Event()
    thread_errors = []

    def threaded_blas():
        try:
            matrix = np.full((192, 192), .125)
            while not stop.is_set():
                product = matrix @ matrix
                if not np.isfinite(product).all():
                    raise AssertionError('non-finite BLAS control')
                ready.set()  # only after a completed BLAS call
        except BaseException as error:
            thread_errors.append(error)
            ready.set()

    thread = threading.Thread(target=threaded_blas, daemon=True)
    thread.start()
    interrupted = False
    try:
        if not ready.wait(timeout=10.) or thread_errors:
            raise RuntimeError(f'parent BLAS warmup failed: {thread_errors}')
        for repetition in range(3):
            runner = AsyncCloseRunner(start_method=start_method)
            try:
                runner.send_context(_synthetic_ctx(ROOT))
                runner.submit([_raw(i) for i in range(10)])
                if interrupt:
                    raise KeyboardInterrupt
                if repetition == 1:
                    # Close without consuming large results; queue feeders
                    # must not hold interpreter shutdown after termination.
                    for start in range(10, 210, 10):
                        runner.submit([_raw(i) for i in range(start, start + 10)])
                else:
                    result, = runner.collect(block=True)
                    assert [sample[0] for sample in result['samples']] == list(range(10))
                    assert abs(result['fields']['loss_j'] - 1.) < 1e-12
                    runner.send_context(_synthetic_ctx(ROOT))
                    runner.submit([_raw(i) for i in range(10)])
                    reset_result, = runner.collect(block=True)
                    assert reset_result['fields']['loss_j'] == result['fields']['loss_j']
            finally:
                runner.close()
                runner.close()
                assert not runner._proc.is_alive()
    except KeyboardInterrupt:
        interrupted = True
    finally:
        stop.set()
        thread.join(timeout=3.)
        if thread.is_alive():
            raise RuntimeError('parent BLAS thread did not stop')
    if thread_errors:
        raise RuntimeError(f'parent BLAS thread failed: {thread_errors}')
    live = [child.pid for child in mp.active_children() if child.is_alive()]
    assert not live, f'orphan workers: {live}'
    print(json.dumps({'start_method': start_method, 'live_children': live,
                      'blas_thread_started': ready.is_set(), 'interrupted': interrupted}))
    return 130 if interrupted else 0


def main(argv=None):
    from bike_sim.sim.ride.async_close import _worker_start_method
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start-method', choices=mp.get_all_start_methods(),
                        default=_worker_start_method(platform.system()))
    parser.add_argument('--interrupt', action='store_true')
    args = parser.parse_args(argv)
    previous_handler = signal.signal(signal.SIGALRM, _timeout)
    signal.setitimer(signal.ITIMER_REAL, 45.)
    try:
        return _exercise(args.start_method, args.interrupt)
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0.)
        signal.signal(signal.SIGALRM, previous_handler)


if __name__ == '__main__':
    raise SystemExit(main())
