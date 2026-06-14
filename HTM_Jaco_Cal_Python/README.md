# HTM Jaco Cal Python

This folder contains a Python/SymPy port of the original MATLAB
HTM/Jacobian generation scripts. It generates symbolic forward kinematics
HTM and Jacobian expressions, then saves C++ assignment statements that can be
pasted into `Y2Kinematics/src`.

## Layout

```text
HTM_Jaco_Cal_Python/
  Kinematics_UR10.py       UR10 DH setup and code generation
  Kinematics_KUKAiiwa7.py  KUKA iiwa 7 DH setup and code generation
  Kinematics.py            Default KUKA iiwa 7 example
  kinematics_common.py     Shared symbolic generation, verification, export
  Func_Common/
    forward_kinemaics.py   Craig DH transform helper
    status_check.py        MATLAB Status_check port
  DH_params/               Original DH reference documents
```

## Run

Install the Python dependency first:

```bash
cd /home/jay/armcon_ws/src/armcon_pack
python3 -m pip install -r requirements.txt
```

Then run the generator scripts:

```bash
cd /home/jay/armcon_ws/src/armcon_pack/HTM_Jaco_Cal_Python
python3 Kinematics_UR10.py
python3 Kinematics_KUKAiiwa7.py
python3 Kinematics.py
```

Each script prints the verification input values and the resulting position/
force summary. The symbolic HTM and Jacobian matrices are not printed to the
terminal; they are saved only in the generated `.txt` file.

## Generated Output

```text
Kinematics_UR10_HTM_Jacobian_cpp.txt
Kinematics_KUKAiiwa7_HTM_Jacobian_cpp.txt
Kinematics_HTM_Jacobian_cpp.txt
```

The generated text contains two complete paste blocks:

```cpp
// forwardKinematics() paste block
double th1 = q[0], th2 = q[1], ...
double s1 = sin(th1), c1 = cos(th1);
YMatrix T(4, 4);
T[0][0] = ...;
return T * EE2TCP;

// calculateJacobian() paste block
double th1 = q[0], th2 = q[1], ...
double s1 = sin(th1), c1 = cos(th1);
YMatrix J(6, numOfAxis);
J[0][0] = ...;
return J;
```

In the target robot class, keep the existing `q.size()` check, then replace
the rest of the function body with the matching generated block.

Do not paste the generated block below the existing `double th...`,
`double s...`, or `YMatrix T/J` lines, because the generated block already
contains those declarations.

Target files are typically:

```text
../Y2Kinematics/src/KinematicsUR10.cpp
../Y2Kinematics/src/KinematicsKUKAiiwa.cpp
```

## Notes

The default export keeps expressions structurally close to the generated
symbolic result while avoiding expensive global simplification. To force
additional SymPy simplification from Python code, call:

```python
main(simplify_expressions=True)
```

This can produce shorter expressions, but it may take much longer for large
Jacobians.
