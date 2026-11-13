# How the partial-derivative Jacobian works

This explains the `partial` branch of `MiniArmIK.jacobian` and its helper
`MiniArmIK.mdh`. The code is commented at each step. It uses the same DH model
as FK, rather than a separately derived geometric-axis formula.

## 1. Start with the meaning of one column

For this robot, q=[base;shoulder;elbow;wrist] in radians and p=[x;y;z] in metres.
The position Jacobian is:

```math
J=\begin{bmatrix}
\partial x/\partial q_1&\partial x/\partial q_2&\partial x/\partial q_3&\partial x/\partial q_4\\
\partial y/\partial q_1&\partial y/\partial q_2&\partial y/\partial q_3&\partial y/\partial q_4\\
\partial z/\partial q_1&\partial z/\partial q_2&\partial z/\partial q_3&\partial z/\partial q_4
\end{bmatrix}.
```

**Column j tells how the TCP moves if only joint j rotates a little, with all
the other joint angles held fixed.** Its units are metres per radian. A column
does not give a new joint angle or an actual finite displacement by itself.

For a single planar link, x=L*cos(q), y=L*sin(q), so differentiating gives
dx/dq=-L*sin(q), dy/dq=L*cos(q). Our robot uses exactly this sine/cosine
differentiation, organized through its transformation-matrix product.

## 2. Differentiate the same FK transformation

FK calculates:

```math
T=T_b A_1(q_1)A_2(q_2)A_3(q_3)A_4(q_4)T_t,
\qquad p=T_{1:3,4}.
```

Each A_i describes one independent revolute joint. The DH lengths, twists,
displacements and fixed offsets are constants. Only q_i varies in A_i. The
base and tool transforms are also constant with respect to q.

For example, differentiating with respect to q2 gives:

```math
\frac{\partial T}{\partial q_2}
=T_b A_1\underbrace{\frac{\partial A_2}{\partial q_2}}_{D_2}A_3A_4T_t.
```

The order of matrix multiplication stays the same. Replace only A2 by D2.

**Why is there no long sum from the product rule?** For two factors:

```math
\frac{\partial(A_1A_2)}{\partial q_2}
=\frac{\partial A_1}{\partial q_2}A_2
+A_1\frac{\partial A_2}{\partial q_2}
=0+A_1D_2.
```

A1 does not depend on q2, so its derivative term vanishes. The same reasoning
leaves only one product for each partial derivative of the full chain.
This relies on independent joint variables; coupled joints would require the
appropriate additional chain-rule terms.

## 3. Where D comes from

In `MiniArmIK.mdh`, let theta=thetaOffset+q, c=cos(theta), s=sin(theta),
ca=cos(alpha), sa=sin(alpha). The ordinary modified-DH matrix is:

```matlab
A = [c     -s      0     a;
     s*ca   c*ca  -sa   -d*sa;
     s*sa   c*sa   ca    d*ca;
     0      0      0     1];
```

Differentiate **each entry with respect to q in radians**. Since the fixed
thetaOffset is constant, d(thetaOffset+q)/dq=1. Thus d(c)/dq=-s and d(s)/dq=c.
The other constant entries become zero:

```matlab
D = [-s     -c      0  0;
      c*ca  -s*ca   0  0;
      c*sa  -s*sa   0  0;
      0      0      0  0];
```

This formula was differentiated once when writing the program. At runtime,
MATLAB substitutes the current q and evaluates the trigonometric values. It
does not invoke symbolic differentiation or repeatedly perturb q in this branch.
The derivative is exact for the implemented model, up to floating-point error.
It does not establish that the model matches the physical robot.

D is **not a pose transformation**: its bottom row is zero, not [0 0 0 1].
It is a matrix of derivatives used inside the product for partial T/partial q.

## 4. How the two loops build J

The outer loop chooses the column/joint variable j. The inner loop visits
the factors A1 through A4 in order:

```matlab
for j=1:n
    dT=m.base;
    for i=1:n
        [A,D]=MiniArmIK.mdh(m.mdh(i,:),q(i));
        if i==j
            dT=dT*D; % differentiate THIS factor
        else
            dT=dT*A; % retain the other factors unchanged
        end
    end
    dT=dT*m.tool;
    J(:,j)=dT(1:3,4);
end
```

| Outer j | Joint | Product constructed by the inner loop and tool multiplication |
|---:|---|---|
| 1 | Base | base * D1 * A2 * A3 * A4 * tool |
| 2 | Shoulder | base * A1 * D2 * A3 * A4 * tool |
| 3 | Elbow | base * A1 * A2 * D3 * A4 * tool |
| 4 | Wrist | base * A1 * A2 * A3 * D4 * tool |

For j=2, the accumulator is built as base, base*A1, base*A1*D2,
base*A1*D2*A3, base*A1*D2*A3*A4, then the final product with tool. The variable
name `dT` anticipates the completed derivative; its initialization does not
mean that m.base is being differentiated.

The final line works because p is T(1:3,4), so:

```math
J_{:,j}=\frac{\partial p}{\partial q_j}
=\left(\frac{\partial T}{\partial q_j}\right)_{1:3,4}.
```

We extract only the position derivative. A full spatial Jacobian would also
need angular-velocity rows; differentiating all nine rotation entries is not
itself a three-component angular-velocity vector.

## 5. Why the fixed tool transform must remain

The tool has no variable, so its derivative is zero. That does **not** mean
dropping the tool factor from the nonzero derivative product. The product rule
is d(A*tool)/dq=(dA/dq)*tool, because only A varies.

For a fixed tool translation r, TCP position has the form p=t+R*r. Rotating the
wrist changes R, so dp/dq includes (dR/dq)*r. Even when D4's fourth column is
zero, the fourth column of the completed product D4*tool can be nonzero.
This is how the Jacobian includes the 85 mm distance from wrist to TCP.
Without the offset, wrist rotation about the wrist origin would not move that
origin. With the offset, wrist rotation moves the TCP along an arc.

## 6. Worked numerical example with your miniArm

Use the current default pose, expressed here in degrees for readability:

```matlab
m=miniarm_model();
q=deg2rad([90;60;-60;-60]);
J=MiniArmIK.jacobian(m,q,'partial')
```

The Jacobian, approximately in metres/radian, is:

```math
J\approx\begin{bmatrix}
-0.132250&0&0&0\\
0&0.024682&0.073612&0.073612\\
0&0.132250&0.104000&0.042500
\end{bmatrix}.
```

Very small numerical entries near 1e-18 are shown as zero. For example, column
4 predicts that a small positive wrist rotation increases y and z, with almost
no x change at this forward-facing pose.

```matlab
dq=zeros(4,1);
dq(4)=deg2rad(1); % rotate only the wrist by one degree
predicted=J*dq;
actual=MiniArmIK.fk(m,q+dq)-MiniArmIK.fk(m,q);
disp(1000*[predicted actual]) % compare displacement columns, in mm
```

| Displacement | Jacobian prediction, mm | Actual FK change, mm |
|---|---:|---:|
| x | approximately 0 | approximately 0 |
| y | 1.284775 | 1.278236 |
| z | 0.741765 | 0.752939 |

The exact derivative is a **local slope**. Its linear prediction for a finite
one-degree change is approximate because FK is nonlinear. Try 0.1 degree and
compare again. These numerical values were checked with the NumPy reference;
the MATLAB commands are provided for execution in your own session.

For several joints moving at once, `J*dq` adds their column contributions:

```math
\Delta p\approx J_{:,1}\Delta q_1+J_{:,2}\Delta q_2
+J_{:,3}\Delta q_3+J_{:,4}\Delta q_4.
```

## 7. Compare with perturbing one joint

```matlab
Ja=MiniArmIK.jacobian(m,q,'partial');
Jn=MiniArmIK.jacobian(m,q,'finite',1e-5);
norm(Ja-Jn,'fro')
```

The finite branch changes one q_j by +h and -h, evaluates FK twice, and divides
the position difference by 2h. It approximates the same column that the partial
branch obtains from the derivative product. Both use the same current q, frame
and TCP. Their agreement checks mathematical consistency; physical measurements
are still required to validate geometry and calibration.

The [main code guide](README.md#execution-flow-when-you-type-demo_ik) traces how
the solver uses this J, calculates dq, tests candidates and accepts the next q.
