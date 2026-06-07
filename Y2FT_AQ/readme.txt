Python G_Comp_Matrix.py로 추정한 값을 보고

TOOL_MASS 에 질량 [kg]

TOOL_COG 에 FT frame 기준 CoM [m]
를 채워주면 된다.

static double TOOL_MASS = 1.234;

YMatrix TOOL_COG =
{
    { 0.015 },  // r_x
    { 0.002 },  // r_y
    { 0.120 }   // r_z
};


GRAVITY_COMPENSATION_MODE 를 1로 두면:

센서 frame 에서 중력 성분을 빼고

그 후 Base frame 으로 변환된 값이 /ur10skku/ftdata로 나간다.

free-space에서 툴을 이리저리 돌려 보면서

보정 후 force/torque 평균이 0 근처로 오는지

잔차 수준이 몇 N / Nm 정도인지
를 확인하면 중력 보상 품질을 체크할 수 있어.