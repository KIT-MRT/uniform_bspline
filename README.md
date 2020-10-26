<!---
This file is auto-generated. To not edit this file. Use the file doc/README_doxygen.md instead.
Then regenerate this file by running the script 'generate_readme_md'
--->
# Uniform B-spline

An implementation of a uniform B-spline from $` \mathbb{R}^n \rightarrow \mathbb{R}^m `$. Uniform means, that the knot vector of the B-spline is equally distributed. Using such a knot vector makes the computation much more efficient, as the basis can be precomputed.

## Usage

### 1D -> 1D spline

Let's start with a B-spline $` f: \mathbb{R} \rightarrow \mathbb{R} `$.

```cpp
    ubs::UniformBSpline<double, 3, double, double, std::vector<double>> spline;
```

The first template argument is the value type, the second one is the degree of the spline, the third one is the input type and the last one the output type. This means, that we have defined a uniform B-spline of degree 3 which takes a double as input and a double as output. The last template parameter is the container of control points.

As this is a pretty common spline, there is a template alias for it:

```cpp
    ubs::UniformBSpline11d<3> spline;
```

As we have used the default constructor for our spline, the spline is initialized with the minimum number of control points all set to zero. In addition the spline is in the range between zero and one. Now lets set the control points.

```cpp
    std::vector<double> controlPoints{0.0, 1.0, 2.0, 3.0, 4.0};
    spline.setControlPoints(controlPoints);
```

Now we have defined a spline which is a straight line with a slope of two. One can change the lower bound and the upper bound of the spline like this:

```cpp
    spline.setBounds(-2.0, 5.0);
```

Now the range of the spline is between -2 and 5.

The spline can be evaluated by calling evaluate as follows:

```cpp
    double val0 = spline.evaluate(0.0);
    double val1 = spline.evaluate(1.0);
```
 
This evaluates the spline at the positions zero and one.

The derivative can be calculated using the derivative function:

```cpp
    double deriv1 = spline.derivative(0.0, 1);
    double deriv2 = spline.derivative(1.0, 2);
```

The first statement evaluates the first derivative at zero, the second the second derivative at one.

Also the smoothness of the spline can be evaluated. For the used definition of the smoothness see UniformBSpline::smoothness.

```cpp
    double smoothness = spline.smoothness<1>();
```

This computes the smoothness using the first derivative.

### 1D -> 3D Eigen spline
Now lets look at a spline $` f: \mathbb{R} \rightarrow \mathbb{R}^3 `$.

```cpp
    ubs::UniformBSpline<double, 3, double, Eigen::Vector3d, EigenAlignedVec<Eigen::Vector3d>> spline;
```

The definition is very similar to the one dimensional case but the output is now a Eigen::Vector3d.

As we still have a one dimensional input, spline evaluation, derivatives and smoothness works similar, the only change is its output type:

```cpp
    Eigen::Vector3d val0 = spline.evaluate(0.0);
    Eigen::Vector3d val1 = spline.evaluate(1.0);
```

```cpp
    Eigen::Vector3d deriv1 = spline.derivative(0.0, 1);
    Eigen::Vector3d deriv2 = spline.derivative(1.0, 2);
```

```cpp
    Eigen::Vector3d smoothness = spline.smoothness<1>();
```

The smoothness is now divided into each dimension. To get the smoothness of the whole spline, those values needs to be summed together.

### 2D -> 1D Eigen spline
Now lets look at a spline $` f: \mathbb{R}^2 \rightarrow \mathbb{R} `$.

```cpp
    ubs::UniformBSpline<double, 3, Eigen::Vector2d, double, Eigen::MatrixXd> spline;
```

The input type is now Eigen::Vector2d and the output type is double. As the control points must be on a two dimensional grid, we use an Eigen::Matrix here.

To evaluate the spline two parameters are needed:

```cpp
    double val = spline.evaluate(Eigen::Vector2d{0.0, 0.0});
```

This would evaluate the spline at (0, 0). This can be shortened by using brace initialization of Eigen::Vector2d:

```cpp
    val = spline.evaluate({0.0, 0.0});
```

For the derivatives we now have partial derivatives. One can specify the derivative for each dimension separately.

This calculates the expression $` \frac{\partial f(x, y)}{\partial x} `$.

```cpp
    double deriv10 = spline.derivative({0.0, 0.0}, {1, 0});
```

This calculates the expression $` \frac{\partial^2 f(x, y)}{\partial x \partial y} `$.

```cpp
    double deriv11 = spline.derivative({0.0, 0.0}, {1, 1});
```

### 2D -> 3D Eigen spline
Not lets look at a spline $` f: \mathbb{R}^2 \rightarrow \mathbb{R}^3 `$.

```cpp
    ubs::UniformBSpline<double, 3, Eigen::Vector2d, Eigen::Vector3d, ubs::EigenAlignedMultiArray<Eigen::Vector3d, 2>>
        spline;
```

So we have used an Eigen::Vector2d as input, an Eigen::Vector3d as output and a boost::multi_array as control points container. As the definition of such splines are getting pretty lengthy, the alias declaration "EigenUniformBSpline" exists:

```cpp
    ubs::EigenUniformBSpline<double, 3, 2, 3> splineS;
```

To evaluate a spline, one need a two dimensional input and receives a three dimensional output:

```cpp
    Eigen::Vector3d val = spline.evaluate({0.0, 0.2});
```

Derivatives and smoothness is similar to the other cases.