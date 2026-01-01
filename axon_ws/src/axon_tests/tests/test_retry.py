from axon_utils.retry import Backoff


def test_backoff_sequence_caps() -> None:
    backoff = Backoff(initial_s=0.5, max_s=2.0, multiplier=2.0)
    seq = backoff.sequence()
    assert next(seq) == 0.5
    assert next(seq) == 1.0
    assert next(seq) == 2.0
    assert next(seq) == 2.0
