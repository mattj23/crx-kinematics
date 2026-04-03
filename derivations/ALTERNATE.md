# Alternate Method

![Diagram of kinematic parameters](../images/parameters.png)

## Derivation of an Alternate Solution

### Conditions on a Solution

For any solution to be valid, it must satisfy all the following conditions:

1. The distance from $O_3$ to $O_1$ must be equal to $z_1$.
2. The distance from $O_3$ to $O_4$ must be equal to $x_1$.
3. The distance from $O_5$ to $O_4$ must be equal to $y_1$.
4. The cross-product of the vectors $\overrightarrow{O_1O_3}$ and $\overrightarrow{O_1O_4}$ have a z component of zero.
5. The vectors $\overrightarrow{O_4O_5}$ and $\overrightarrow{O_4O_3}$ are perpendicular.
    - This also means that the distance from $O_5$ to $O_3$ must be equal to $\sqrt{x_1^2 + y_1^2}$ 

### Geometric Interpretation of Conditions

From these conditions, we can derive the following geometric interpretations:

1. $O_4$ must exist on a circle centered around $O_5$, with radius $y_1$, normal to the vector $\overrightarrow{O_5O_6}$.
2. For any candidate $O_{4c}$, the point $O_{3c}$ must: 
    - Exist on a circle centered around $O_{4c}$, with radius $x_1$, normal to the vector $\overrightarrow{O_{4c}O_5}$.
    - Exist on the surface of a sphere centered around $O_1$, with radius $z_1$.
3. For any candidate $O_{4c}$, there can be no more than 2 that satisfy the above conditions.

Additionally, the rotation of a circle $x^2 + y^2 = r^2$, $z=c$ around the x-axis by all possible angles $\theta$ forms 
a partial sphere (the sphere is missing all data where $x < -r$ and $x > r$) with radius $\sqrt{r^2 + c^2}$, centered at the origin.

This means that we can consolidate the geometric assumptions:
1. $O_4$ must exist on a circle centered around $O_5$, with radius $y_1$, normal to the vector $\overrightarrow{O_5O_6}$.
2. $O_3$ must exist at the intersection of a sphere at $O_1$ with radius $z_1$, and a sphere at $O_5$ with radius $\sqrt{x_1^2 + y_1^2}$. The intersection of two spheres is a 2D circle.  The circle can be further clipped at the planes at $d=\pm x_1$ with normal vector $\overrightarrow{O_5O_6}$.
3. The relationship between the $O_3$ and $O_4$ points is such that the cross-product of the vectors $\overrightarrow{O_1O_3}$ and $\overrightarrow{O_1O_4}$ have a z component of zero, and the distance between $O_3$ and $O_4$ is $x_1$.

# Scratch thoughts

- If we reduce the core problem to a 3d circle at a height z above the origin, with a circle in the x-y plane with a
  radius of the sphere intersection, and the circle is always tilted in the same direction, what are the possible 
  configurations of the problem?
    - circle flat
    - circle vertical
    - circle tilted
- Under this coordinate system, if the large circle is ever cut, it will always be symmetrical about x and in the 
- direction where the circle is tilted down
- From this can we create a relationship between $\theta$ for the $O_4$ candidate and $\theta$ for the $O_3$ candidate?
  We already can probably tell directions on any point since we can compute the partial derivatives, I think?
- Can we use that to see where we would be spanning the zero cross-product value?