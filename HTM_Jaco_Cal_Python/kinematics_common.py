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


def _format_numeric_list(values, precision=6):
    return "[" + ", ".join(f"{float(sp.N(value)):.{precision}g}" for value in values) + "]"


def print_verification(robot_name, ver_a, ver_d, ver_th, ver_tq, transform, total_force):
    position = transform[:3, 3]
    force = [total_force[i] / 1000 for i in range(3)]

    print(f"=== {robot_name} verification ===")
    print("Input:")
    print(f"  a(mm)      = {_format_numeric_list(ver_a)}")
    print(f"  d(mm)      = {_format_numeric_list(ver_d)}")
    print(f"  theta(rad) = {_format_numeric_list(ver_th)}")
    print(f"  torque(Nm) = {_format_numeric_list(ver_tq)}")
    print("Result:")
    print(f"  position(mm) = {_format_numeric_list(position, precision=4)}")
    print(f"  force(N)     = {_format_numeric_list(force, precision=4)}")


def _format_cpp_expr(expr):
    code = sp.ccode(expr).replace("M_PI", "pi")
    for index in range(10, 0, -1):
        code = code.replace(f"sin(th{index})", f"s{index}")
        code = code.replace(f"cos(th{index})", f"c{index}")
    return code


def _format_cpp_matrix_assignments(matrix, matrix_name):
    lines = []
    for row in range(matrix.rows):
        for col in range(matrix.cols):
            lines.append(
                f"{matrix_name}[{row}][{col}] = {_format_cpp_expr(matrix[row, col])};"
            )
    return "\n".join(lines)


def _format_joint_declarations(num_axes):
    names = [f"th{i}" for i in range(1, num_axes + 1)]
    assignments = [f"{name} = q[{i}]" for i, name in enumerate(names)]
    lines = []
    for start in range(0, len(assignments), 4):
        lines.append("double " + ", ".join(assignments[start : start + 4]) + ";")
    return "\n".join(lines)


def _format_trig_declarations(num_axes):
    lines = ["// Pre-calculate trigonometric functions"]
    for index in range(1, num_axes + 1):
        lines.append(f"double s{index} = sin(th{index}), c{index} = cos(th{index});")
    return "\n".join(lines)


def _format_cpp_function_block(matrix, matrix_name, return_line, num_axes):
    return "\n".join(
        [
            _format_joint_declarations(num_axes),
            "",
            _format_trig_declarations(num_axes),
            "",
            f"YMatrix {matrix_name}({matrix.rows}, {matrix.cols if matrix_name == 'T' else 'numOfAxis'});",
            _format_cpp_matrix_assignments(matrix, matrix_name),
            "",
            return_line,
        ]
    )


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
    num_axes = jac.cols

    text = "\n".join(
        [
            f"// {robot_name} generated HTM and Jacobian",
            "// Copy/paste-ready for Y2Kinematics robot classes.",
            "// Replace the body after the q.size() check in each function.",
            "// The blocks include q -> th declarations, s/c pre-calculation, YMatrix allocation, assignments, and return.",
            "",
            "// ===== forwardKinematics() paste block =====",
            _format_cpp_function_block(htm, "T", "return T * EE2TCP;", num_axes),
            "",
            "// ===== calculateJacobian() paste block =====",
            _format_cpp_function_block(jac, "J", "return J;", num_axes),
            "",
        ]
    )

    output_path.write_text(text, encoding="utf-8")
    return output_path
