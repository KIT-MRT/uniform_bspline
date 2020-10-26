# Uniform B-spline

An implementation of a uniform B-spline from @f$ \mathbb{R}^n \rightarrow \mathbb{R}^m @f$. Uniform means, that the knot vector of the B-spline is equally distributed. Using such a knot vector makes the computation much more efficient, as the basis can be precomputed.

## Usage

### 1D -> 1D spline

Let's start with a B-spline @f$ f: \mathbb{R} \rightarrow \mathbb{R} @f$.

\snippet examples.cpp Spline1d1d_Definition

The first template argument is the value type, the second one is the degree of the spline, the third one is the input type and the last one the output type. This means, that we have defined a uniform B-spline of degree 3 which takes a double as input and a double as output. The last template parameter is the container of control points.

As this is a pretty common spline, there is a template alias for it:

\snippet examples.cpp Spline1d1d_DefinitionShort

As we have used the default constructor for our spline, the spline is initialized with the minimum number of control points all set to zero. In addition the spline is in the range between zero and one. Now lets set the control points.

\snippet examples.cpp Spline1d1d_ControlPoints

Now we have defined a spline which is a straight line with a slope of two. One can change the lower bound and the upper bound of the spline like this:

\snippet examples.cpp Spline1d1d_Bounds

Now the range of the spline is between -2 and 5.

The spline can be evaluated by calling evaluate as follows:

\snippet examples.cpp Spline1d1d_Eval
 
This evaluates the spline at the positions zero and one.

The derivative can be calculated using the derivative function:

\snippet examples.cpp Spline1d1d_Derivative

The first statement evaluates the first derivative at zero, the second the second derivative at one.

Also the smoothness of the spline can be evaluated. For the used definition of the smoothness see UniformBSpline::smoothness.

\snippet examples.cpp Spline1d1d_Smoothness

This computes the smoothness using the first derivative.

### 1D -> 3D Eigen spline
Now lets look at a spline @f$ f: \mathbb{R} \rightarrow \mathbb{R}^3 @f$.

\snippet examples.cpp Spline1d3d_Definition

The definition is very similar to the one dimensional case but the output is now a Eigen::Vector3d.

As we still have a one dimensional input, spline evaluation, derivatives and smoothness works similar, the only change is its output type:

\snippet examples.cpp Spline1d3d_Eval

\snippet examples.cpp Spline1d3d_Derivative

\snippet examples.cpp Spline1d3d_Smoothness

The smoothness is now divided into each dimension. To get the smoothness of the whole spline, those values needs to be summed together.

### 2D -> 1D Eigen spline
Now lets look at a spline @f$ f: \mathbb{R}^2 \rightarrow \mathbb{R} @f$.

\snippet examples.cpp Spline2d1d_Definition

The input type is now Eigen::Vector2d and the output type is double. As the control points must be on a two dimensional grid, we use an Eigen::Matrix here.

To evaluate the spline two parameters are needed:

\snippet examples.cpp Spline2d1d_Evaluate_Long

This would evaluate the spline at (0, 0). This can be shortened by using brace initialization of Eigen::Vector2d:

\snippet examples.cpp Spline2d1d_Evaluate_Short

For the derivatives we now have partial derivatives. One can specify the derivative for each dimension separately.

This calculates the expression @f$ \frac{\partial f(x, y)}{\partial x} @f$.

\snippet examples.cpp Spline2d1d_Derivative_10

This calculates the expression @f$ \frac{\partial^2 f(x, y)}{\partial x \partial y} @f$.

\snippet examples.cpp Spline2d1d_Derivative_11

### 2D -> 3D Eigen spline
Not lets look at a spline @f$ f: \mathbb{R}^2 \rightarrow \mathbb{R}^3 @f$.

\snippet examples.cpp Spline2d3d_Definition

So we have used an Eigen::Vector2d as input, an Eigen::Vector3d as output and a boost::multi_array as control points container. As the definition of such splines are getting pretty lengthy, the alias declaration "EigenUniformBSpline" exists:

\snippet examples.cpp Spline2d3d_Definition_Short

To evaluate a spline, one need a two dimensional input and receives a three dimensional output:

\snippet examples.cpp Spline2d3d_Evaluate

Derivatives and smoothness is similar to the other cases.
