from act_lab.adapters.mediapipe.one_euro import OneEuroFilter


def test_filter_reduces_stationary_jitter() -> None:
    input_filter = OneEuroFilter(1.0, 0.0, 1.0)
    raw = [0.0, 0.02, -0.02, 0.02, -0.02]
    filtered = [
        input_filter.filter(value, index * 20_000_000)
        for index, value in enumerate(raw)
    ]

    assert max(filtered[1:]) - min(filtered[1:]) < max(raw) - min(raw)


def test_speed_adaptation_reduces_fast_motion_lag() -> None:
    fixed = OneEuroFilter(1.0, 0.0, 1.0)
    adaptive = OneEuroFilter(1.0, 0.3, 1.0)
    for timestamp_ns in (0, 20_000_000, 40_000_000):
        fixed.filter(0.0, timestamp_ns)
        adaptive.filter(0.0, timestamp_ns)

    fixed_result = fixed.filter(1.0, 60_000_000)
    adaptive_result = adaptive.filter(1.0, 60_000_000)

    assert adaptive_result > fixed_result


def test_reset_forgets_previous_sample() -> None:
    input_filter = OneEuroFilter(1.0, 0.3, 1.0)
    input_filter.filter(1.0, 0)
    input_filter.reset()

    assert input_filter.filter(0.0, 20_000_000) == 0.0
