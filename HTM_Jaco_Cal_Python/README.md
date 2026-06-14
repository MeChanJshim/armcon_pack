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

Each script prints the verification pose/force values and writes a `.txt`
file in this folder.

## Generated Output

```text
Kinematics_UR10_HTM_Jacobian_cpp.txt
Kinematics_KUKAiiwa7_HTM_Jacobian_cpp.txt
Kinematics_HTM_Jacobian_cpp.txt
```

The generated text uses this format:

```cpp
YMatrix T(4, 4);
T[0][0] = ...;
T[0][1] = ...;

YMatrix J(6, numOfAxis);
J[0][0] = ...;
J[0][1] = ...;
```

Paste the `T[...]` block into the robot class `forwardKinematics()` function
after `YMatrix T(4, 4);`.

Paste the `J[...]` block into the robot class `calculateJacobian()` function
after `YMatrix J(6, numOfAxis);`.

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
