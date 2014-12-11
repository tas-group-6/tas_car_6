#ifdef _MathHelper_H_
#define _MathHelper_H_

#include <math.h>


#include "DataType.h"


#define RAND_DOUBLE ((double) rand() / (double) RAND_MAX )


class MathHelper
{
public:
  /**
 * @brief qnorm calculates the euclidean norm of p
 * @param p a reference to a QPointF
 * @return the norm of p
 */
  qreal qnorm( const QPointF& p)
  {
    return sqrt(  p.x() * p.x() + p.y() * p.y() );
  }

  /**
 * @brief angle_to_dir converts the given angle into a direction
 * @param angle in radians
 * @return the direction vector
 */
  QPointF angle_to_dir( const qreal& rad)
  {
    QPointF dir;

    //TODO: Is this definition correct?
    dir.setX( cos( rad ) );
    dir.setY( sin( rad ) );

    return dir;
  }

  /**
 * @brief gauss_curve
 * @param x
 * @param mean
 * @param sigma
 * @return 1 / ( sqrt(2Pi) * sigma) * exp( - 0.5 ( x - mean )**2 / sigma**2 )
 */
  qreal gauss_curve( qreal x, qreal mean, qreal sigma)
  {
    return exp(-0.5 * (x - mean) * (x - mean ) / ( sigma*sigma) ) / ( sigma * sqrt(2.0 * PI));
  }


  /**
 * @brief Transform_Rotation rotates the vector p around the z axis by s
 * @param p The vector to rotate
 * @param s The AMmount of rotation in radians
 * @return The rotated vector
 */
  QPointF Transform_Rotation( const QPointF& p, const qreal s)
  {
    QPointF result;

    // |x'|   | cos -sin |   |x|
    // |y'| = | sin  cos | * |y|
    result.setX( cos(s) * p.x() - sin(s) * p.y() );
    result.setY( sin(s) * p.x() + cos(s) * p.y() );

    return result;
  }


};

#endif
