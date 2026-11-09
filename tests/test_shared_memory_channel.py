import multiprocessing

import numpy as np
from gym_quadruped.utils.quadruped_utils import LegsAttr

from quadruped_pympc.helpers.shared_memory_channel import LatestValueChannel


def _assert_equal(a, b):
    if isinstance(a, LegsAttr):
        assert isinstance(b, LegsAttr)
        for leg in ("FL", "FR", "RL", "RR"):
            _assert_equal(a[leg], b[leg])
    elif isinstance(a, dict):
        assert list(a.keys()) == list(b.keys())
        for key in a:
            _assert_equal(a[key], b[key])
    elif isinstance(a, (list, tuple)):
        assert type(a) is type(b) and len(a) == len(b)
        for x, y in zip(a, b):
            _assert_equal(x, y)
    elif isinstance(a, np.ndarray):
        assert a.shape == b.shape and a.dtype == b.dtype
        np.testing.assert_array_equal(a, b)
    else:
        assert type(a) is type(b) and a == b


def test_roundtrip_and_schema_change():
    channel = LatestValueChannel()
    try:
        assert channel.read() is None

        msg = (
            {"position": np.arange(3.0), "constraint": None},
            LegsAttr(*[np.full(3, i, dtype=np.float64) for i in range(4)]),
            np.eye(4, 12, dtype=np.int64),
            1,
            0.5,
            True,
            None,
            [np.zeros((1, 3)), 2.0, np.float64(0.25), np.int32(7), np.bool_(True)],
        )
        channel.write(msg)
        _assert_equal(msg, channel.read())
        assert channel.read() is None  # nothing new

        # The schema changes (None -> array, None -> LegsAttr)
        msg2 = ({"position": np.arange(3.0), "constraint": [np.ones(3), np.ones(3)]},) + msg[1:6] + (
            LegsAttr(*[np.ones(3)] * 4),
            msg[7],
        )
        channel.write(msg2)
        _assert_equal(msg2, channel.read())

        # Only the latest value is returned
        channel.write(msg)
        channel.write(msg2)
        _assert_equal(msg2, channel.read())
    finally:
        channel.close()
        channel.unlink()


def _writer(channel, n_messages):
    for k in range(1, n_messages + 1):
        size = 1000 + (k % 7) * 500  # the schema changes every message
        channel.write((k, np.full(size, float(k))))


def test_no_torn_reads_across_processes():
    channel = LatestValueChannel()
    n_messages = 20000
    try:
        writer = multiprocessing.get_context("fork").Process(target=_writer, args=(channel, n_messages))
        writer.start()
        last_k, n_read = 0, 0
        while writer.is_alive() or last_k < n_messages:
            msg = channel.read()
            if msg is None:
                if not writer.is_alive() and channel.read() is None:
                    break
                continue
            k, data = msg
            assert k > last_k
            assert data.size == 1000 + (k % 7) * 500
            assert np.all(data == k), "torn read"
            last_k, n_read = k, n_read + 1
        writer.join()
        assert last_k == n_messages
        assert n_read > 100
    finally:
        channel.close()
        channel.unlink()
