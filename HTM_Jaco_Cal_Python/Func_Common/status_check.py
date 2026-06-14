from __future__ import annotations


def status_check(data_size, current_iter_num):
    """Direct Python port of MATLAB Status_check.m."""
    interval = round(data_size / 100)
    percent = round(100 * (current_iter_num / data_size))

    if interval == 0:
        return [percent, 0]

    if current_iter_num % interval == 0:
        return [percent, 1]
    return [percent, 0]
