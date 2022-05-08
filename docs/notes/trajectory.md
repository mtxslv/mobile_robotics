# Theory

The trajectory class (_CubicPolynomials_) is responsible for creating a smooth, cubic polynomial curve between two given points. This curve is contained in the square defined by the given points, such that no maximum is found in any of the axis. The theory behind this was introduced by Diogo Pedrosa in his [master's thesis](https://repositorio.ufrn.br/handle/123456789/15417). 

According to [the material provided by Pablo on the topic](https://arquivos.info.ufrn.br/arquivos/2020142071eb967875331f9600e519a13/Gerao_de_trajetria.pdf), the curve parameters can be computed using simple formulas, and they vary depending on critical situations. That is, the formulas change if (any of) the provided points are near a singularity (90°).

Thus, 4 situations can emerge:
* Both points are near singularities
* Only the initial point is near the singularity
* Only the final point is near the singularity
* No point is near singularities

Each one of these situations has its own calculations, and they are embodied in the class' methods. Whatever the case, in general they rely on the same set of values:
* delta_x and delta_y, derived from the initial and final positions
* alpha_i and alpha_f, defined as the tangent of the initial and final orientations

Once all the computations are done, the user will have two equations, defining the _x_ and _y_ coordinates of the curve between the provided points:
* `x(lmb) = a_0 + a_1*lmb + a_2*lmb² + a_3*lmb³`
* `y(lmb) = b_0 + b_1*lmb + b_2*lmb² + b_3*lmb³`

# Class testing

The class has some (not all of them) abilities tested already. They can be found on `/tests/code_tests/trajectory_class_test.ipynb`

First, the choice of which set of equations should be used during computations of the curve (i.e., based on the critical situations) was evaluated. The class chose correctly the equations.

Secondly, a use case was assessed based on the third question of [the Exercises' List provided by Pablo](https://arquivos.info.ufrn.br/arquivos/2021127061d6711010583191fdeb887534/Experccio_sobre_modelos_cinemticos_de_robs_mveis.pdf). The instancing and curve points generation was tested, and performed as expected.

**It is important to note that, even though critical situations were tested, they had no written example to be compared with so their quality could be determined precisely.**

**The situation with only the first point near a singularity was tested. There, a curious thing occurred. Even though the final point had an angle of _pi_, the system created a curve with final angle equal to  0.**

# 8th May Updates 

~I noticed some errors on the theta angle. If the angles are supposed to lay on the left circle's quadrants, it will be computed wrongly. That's because the math.atan() function limits the result between pi/2 and -pi/2. This happens when the end of the curve is at the left. Thus, I suggest summing the angle result from atan with 180º when the end of the curve is at the left side of the starting point.~ **FIXED**

~Another error happens when the singularities are upside down (-pi/2). The algorithm works only with positive angles. What to do? I messaged Pablo to find out.~ **FIXED**