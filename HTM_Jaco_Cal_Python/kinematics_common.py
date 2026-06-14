from __future__ import annotations

from pathlib import Path

import sympy as sp

from Func_Common import forward_kinemaics


def _maybe_simplify(value, simplify_expressions):
    if simplify_expressions:
        return sp.simplify(value)
    return value


def generate_kinematics(
    dh_al,
    dh_a,
    dh_d,
    dh_th,
    verbose=True,
    simplify_expressions=False,
):
    tp = [
        forward_kinemaics(alpha, a, d, theta)
        for alpha, a, d, theta in zip(dh_al, dh_a, dh_d, dh_th)
    ]

    to = [tp[0]]
    for i in range(1, len(tp)):
        to.append(_maybe_simplify(to[i - 1] * tp[i], simplify_expressions))
        if verbose:
            print(f"FK {i + 1} th iter finished ")

    if verbose:
        print("Forward kinematics step done")
        sp.printing.pprint(to[-1])

    origins = [transform[:3, 3] for transform in to]
    axis_z = [transform[:3, 2] for transform in to]

    jv_columns = [
        _maybe_simplify(axis.cross(origins[-1] - origin), simplify_expressions)
        for axis, origin in zip(axis_z, origins)
    ]
    jw_columns = axis_z

    jv = sp.Matrix.hstack(*jv_columns)
    jw = sp.Matrix.hstack(*jw_columns)
    jacobian = sp.Matrix.vstack(jv, jw)

    if verbose:
        print("Jacobian step done")
        sp.printing.pprint(jacobian)

    return tp, to, jacobian


def verify_kinematics(final_transform, jacobian, dh_a, dh_d, dh_th, ver_a, ver_d, ver_th, ver_tq):
    substitutions = {}
    for a_sym, d_sym, th_sym, a_val, d_val, th_val in zip(
        dh_a, dh_d, dh_th, ver_a, ver_d, ver_th
    ):
        substitutions[a_sym] = a_val
        substitutions[d_sym] = d_val
        substitutions[th_sym] = th_val

    verified_transform = sp.N(final_transform.subs(substitutions))
    verified_jacobian = sp.N(jacobian.subs(substitutions))
    total_force = verified_jacobian * sp.Matrix(ver_tq)

    return verified_transform, verified_jacobian, total_force


def print_verification(transform, total_force):
    position = transform[:3, 3]
    print(
        "X(mm): {:.1f}, Y(mm): {:.1f}, Z(mm): {:.1f} ".format(
            float(position[0]), float(position[1]), float(position[2])
        )
    )


def _format_cpp_expr(expr):
    return sp.ccode(expr).replace("M_PI", "pi")


def _format_cpp_matrix_assignments(matrix, matrix_name):
    lines = []
    for row in range(matrix.rows):
        for col in range(matrix.cols):
            lines.append(
                f"{matrix_name}[{row}][{col}] = {_format_cpp_expr(matrix[row, col])};"
            )
    return "\n".join(lines)


def save_cpp_assignment_txt(
    output_path,
    final_transform,
    jacobian,
    robot_name,
    simplify_expressions=False,
):
    """Save HTM/Jacobian as C++ YMatrix assignment statements."""
    output_path = Path(output_path)
    htm = sp.simplify(final_transform) if simplify_expressions else final_transform
    jac = sp.simplify(jacobian) if simplify_expressions else jacobian

    text = "\n".join(
        [
            f"// {robot_name} generated HTM and Jacobian",
            "// Paste the T block inside forwardKinematics() after `YMatrix T(4, 4);`.",
            "// Paste the J block inside calculateJacobian() after `YMatrix J(6, numOfAxis);`.",
            "",
            "// ===== HTM / Forward Kinematics =====",
            "YMatrix T(4, 4);",
            _format_cpp_matrix_assignments(htm, "T"),
            "",
            "// ===== Jacobian =====",
            f"YMatrix J({jac.rows}, numOfAxis);",
            _format_cpp_matrix_assignments(jac, "J"),
            "",
        ]
    )

    output_path.write_text(text, encoding="utf-8")
    return output_path
    print(
        "Fx(N): {:.2f}, Fy(N): {:.2f}, Fz(N): {:.2f} ".format(
            float(total_force[0] / 1000),
            float(total_force[1] / 1000),
            float(total_force[2] / 1000),
        )
    )
