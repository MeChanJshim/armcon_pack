from __future__ import annotations

from pathlib import Path

import sympy as sp

from kinematics_common import (
    generate_kinematics,
    print_verification,
    save_cpp_assignment_txt,
    verify_kinematics,
)


def main(verbose=True, simplify_expressions=False, export_txt=True):
    th1, th2, th3, th4, th5, th6, th7, th8, th9, th10 = sp.symbols(
        "th1 th2 th3 th4 th5 th6 th7 th8 th9 th10"
    )
    a1, a2, a3, a4, a5, a6, a7, a8, a9, a10 = sp.symbols(
        "a1 a2 a3 a4 a5 a6 a7 a8 a9 a10"
    )
    d1, d2, d3, d4, d5, d6, d7, d8, d9, d10 = sp.symbols(
        "d1 d2 d3 d4 d5 d6 d7 d8 d9 d10"
    )

    dh_al = [0, -sp.pi / 2, sp.pi / 2, sp.pi / 2, -sp.pi / 2, -sp.pi / 2, sp.pi / 2]
    dh_a = [0, 0, 0, 0, 0, 0, 0]
    dh_d = [d1, 0, d3, 0, d5, 0, d7]
    dh_th = [th1, th2, th3, th4, th5, th6, th7]

    ver_al = [0, -sp.pi / 2, sp.pi / 2, sp.pi / 2, -sp.pi / 2, -sp.pi / 2, sp.pi / 2]
    ver_a = [0, 0, 0, 0, 0, 0, 0]
    ver_d = [340, 0, 400, 0, 400, 0, 126]
    ver_th = [0, 0, 0, 0, 0, 0, 0]
    ver_tq = [0, 1, 0, 0, 0, 0, 0]

    tp, to, jacobian = generate_kinematics(
        dh_al,
        dh_a,
        dh_d,
        dh_th,
        verbose=verbose,
        simplify_expressions=simplify_expressions,
    )
    verified_transform, verified_jacobian, total_force = verify_kinematics(
        to[-1], jacobian, dh_a, dh_d, dh_th, ver_a, ver_d, ver_th, ver_tq
    )

    if verbose:
        print_verification(
            "KUKA iiwa 7",
            ver_a,
            ver_d,
            ver_th,
            ver_tq,
            verified_transform,
            total_force,
        )

    output_txt = None
    if export_txt:
        output_txt = save_cpp_assignment_txt(
            Path(__file__).with_name("Kinematics_KUKAiiwa7_HTM_Jacobian_cpp.txt"),
            to[-1],
            jacobian,
            "KUKA iiwa 7",
            simplify_expressions=simplify_expressions,
        )
        if verbose:
            print(f"Saved C++ assignment text: {output_txt}")

    return {
        "Tp": tp,
        "To": to,
        "J": jacobian,
        "verified_transform": verified_transform,
        "verified_jacobian": verified_jacobian,
        "total_force": total_force,
        "VER_al": ver_al,
        "output_txt": output_txt,
    }


if __name__ == "__main__":
    main()
