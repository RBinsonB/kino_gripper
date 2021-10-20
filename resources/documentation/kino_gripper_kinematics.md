# Gripper Kinematics

## Forward Kinematics
The gripper dimensions (right finger) are simplified and detailed below.

<img src="/resources/documentation/kino_gripper_schematics.png" align="center" width="500"/>

The following equations can be deducted:
* Ax = u + a×cos(θ)
* c² = (Ax-Bx)² + (Ay-By)²
* f² = (u-Bx)² + (v-By)²
* By = v + q + b×sin(ψ)
* By = u + p + b×cos(ψ)
* g = (p²+q²)½
* c² = e² + h²
* Cx = u - r
* Cy = v + s
* cos(γ+σ) = (g²+f²-b²)/2gf
* Dx = u + (f-e)×cos(γ)
* Dy = v + (f-e)×sin(γ)
* Ay = Dy + h×sin((π/2)-γ)
	 = Dy + h×cos(-γ)
* γ = acos(g²+f²-b²/2gf) - σ
* e = (a²+f²+c²)/2gf

from those:
* σ = asin(q/g) = acos(p/g)

* cos(π-σ-ψ) = (g² + b² - f²) / 2gb 
	<=> f² = -2gb×cos(π-σ-ψ) + g² + b²

* h = (c² - e²)½

* c² - f² = Ax² - 2AxBx + Ay² - 2AyBy - u² + 2uBx - v² + 2vBy
	<=> Ax² - (2Bx)×Ax + (Ay² - 2AyBy - u² + 2uBx - v² + 2vBy - c² + f²) = 0 ; and defining z = (Ay² - 2AyBy - u² + 2uBx - v² + 2vBy - c² + f²)
	<=> Ax = (2Bx ± (4Bx² - 4z)½) / 2 = Bx ± (Bx²-z)½
	<=> since Ax < Bx and (...)½ > 0 => Ax = Bx - (Bx²-z)½

* Cx = Ax - r
* Opening distance = Cx × 2

The distance can therefor be obtained from the servo angle ψ.

## Inverse Kinatics
The inverse kinematics solution is obtain using a lookup table. The table is populated using the forward kinematics for every possible servo angle command.