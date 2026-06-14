from __future__ import annotations

import sympy as sp


def _simplify_trig(value):
    if value in (0, sp.pi, -sp.pi, sp.pi / 2, -sp.pi / 2):
        cos_value = sp.cos(value)
        sin_value = sp.sin(value)
        return (
            cos_value if abs(float(cos_value)) >= 0.0001 else sp.Integer(0),
            sin_value if abs(float(sin_value)) >= 0.0001 else sp.Integer(0),
        )

    return sp.simplify(sp.cos(value)), sp.simplify(sp.sin(value))


def forward_kinemaics(alpha, a, d, theta):
    """Craig DH homogeneous transform.

    This is a direct Python/SymPy port of MATLAB Forward_kinemaics.m.
    The original filename keeps the typo "kinemaics"; this module keeps it too
    for traceability with the MATLAB source.
    """
    sim_cos_al, sim_sin_al = _simplify_trig(alpha)
    sim_cos_the, sim_sin_the = _simplify_trig(theta)

    return sp.Matrix(
        [
            [sim_cos_the, -sim_sin_the, 0, a],
            [
                sim_sin_the * sim_cos_al,
                sim_cos_the * sim_cos_al,
                -sim_sin_al,
                -d * sim_sin_al,
            ],
            [
                sim_sin_the * sim_sin_al,
                sim_cos_the * sim_sin_al,
                sim_cos_al,
                d * sim_cos_al,
            ],
            [0, 0, 0, 1],
        ]
    )
