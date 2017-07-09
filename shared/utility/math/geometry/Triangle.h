#ifndef UTILITY_MATH_GEOMETRY_TRIANGLE_H
#define UTILITY_MATH_GEOMETRY_TRIANGLE_H

#include <Eigen/Core>

namespace utility {
namespace math {
    namespace geometry {
        class Triangle {
        private:
            Eigen::Vector3d P0, P1, P2, normal;
            double epsilon, area;

        public:
            Triangle()
                : P0(Eigen::Vector3d::Zero())
                , P1(Eigen::Vector3d::Zero())
                , P2(Eigen::Vector3d::Zero())
                , epsilon(1e-6) {}

            Triangle(const Eigen::Vector3d& P0,
                     const Eigen::Vector3d& P1,
                     const Eigen::Vector3d& P2,
                     double epsilon = 1e-6) {
                this->P0      = P0;
                this->P1      = P1;
                this->P2      = P2;
                this->epsilon = epsilon;
                calculateNormal();
                calculateArea();
            }

            Triangle(const Eigen::Vector3d& normal,
                     const Eigen::Vector3d& P0,
                     const Eigen::Vector3d& P1,
                     const Eigen::Vector3d& P2,
                     double epsilon = 1e-6) {
                this->normal  = normal;
                this->P0      = P0;
                this->P1      = P1;
                this->P2      = P2;
                this->epsilon = epsilon;
                calculateArea();
            }

            Eigen::Vector3d calculateNormal(bool CCW = true) {
                if (CCW == true) {
                    normal = (P1 - P0).cross(P2 - P0).normalize();
                }

                else {
                    normal = (P2 - P0).cross(P1 - P0).normalize();
                }

                return (normal);
            }

            double calculateArea() {
                area = 0.5 * std::abs((P1 - P0).cross(P2 - P0).norm());
                return (area);
            }

            double getArea() const {
                return (area);
            }

            Eigen::Vector3d getNormal() const {
                return (normal);
            }

            Eigen::Vector3d getP0() const {
                return (P0);
            }

            Eigen::Vector3d getP1() const {
                return (P1);
            }

            Eigen::Vector3d getP2() const {
                return (P2);
            }

            void applyTransform(const Eigen::Matrix4d& transform) {
                Eigen::Vector4d norm(Eigen::Vector3d::Zero()), R0(Eigen::Vector4d::Ones()), R1(Eigen::Vector4d::Ones()),
                    R2(Eigen::Vector4d::Ones());
                norm.head(3) = normal;
                R0.head(3)   = P0;
                R1.head(3)   = P1;
                R2.head(3)   = P2;

                norm = (transform * norm).normalize();
                R0   = transform * R0;
                R1   = transform * R1;
                R2   = transform * R2;

                normal = normal.head(3);
                P0     = R0.head(3);
                P1     = R1.head(3);
                P2     = R2.head(3);
            }

            Eigen::Vector3d getRandomPoint() const {
                // Randomly select a point from inside a triangle.
                // https://wiki.csiro.au/display/AutonomousSystems/Fitting+CAD+models+in+a+Point+Cloud
                //      ----> Papers
                //            ----> Graphics Gems I
                //                  ----> Method 1
                const double SCALE = (1.0 / (RAND_MAX + 1.0));

                double sqrtT = std::sqrt(rand() * SCALE);
                double s     = rand() * SCALE;

                Eigen::Vector3d scale = {(1 - sqrtT), ((1 - s) * sqrtT), (s * sqrtT)};

                Eigen::Matrix3d points;
                points.col(0) = P0;
                points.col(1) = P2;
                points.col(2) = P2;

                return (points * scale);
            }

            // Taken from
            // https://github.com/erich666/jgt-code/blob/master/Volume_02/Number_1/Moller1997a/raytri.cm
            bool rayIntersect(const Eigen::Vector3d& orig, const Eigen::Vector3d& dir) {
                return (rayIntersect(orig, dir, P0, P1, P2, epsilon));
            }

            static bool rayIntersect(const Eigen::Vector3d& orig,
                                     const Eigen::Vector3d& dir,
                                     const Eigen::Matrix3d& points,
                                     double epsilon = 1e-6) {
                return (rayIntersect(orig, dir, points.col(0), points.col(1), points.col(2), epsilon));
            }

            // Taken from
            // https://github.com/erich666/jgt-code/blob/master/Volume_02/Number_1/Moller1997a/raytri.cm
            static bool rayIntersect(const Eigen::Vector3d& orig,
                                     const Eigen::Vector3d& dir,
                                     const Eigen::Vector3d& P0,
                                     const Eigen::Vector3d& P1,
                                     const Eigen::Vector3d& P2,
                                     double epsilon = 1e-6) {
                /* find vectors for two edges sharing vert0 */
                Eigen::Vector3d edge1 = P1 - P0;
                Eigen::Vector3d edge2 = P2 - P0;

                /* begin calculating determinant - also used to calculate U parameter */
                Eigen::Vector3d pvec = dir.cross(edge2);

                /* if determinant is near zero, ray lies in plane of triangle */
                double det = edge1.dot(pvec);

                if (std::abs(det) < epsilon) {
                    return (false);
                }

                double inv_det = 1.0 / det;

                /* calculate distance from vert0 to ray origin */
                Eigen::Vector3d tvec = orig - P0;

                /* calculate U parameter and test bounds */
                double u = tvec.dot(pvec) * inv_det;

                if ((u < 0.0) || (u > 1.0)) {
                    return (false);
                }

                /* prepare to test V parameter */
                Eigen::Vector3d qvec = tvec.cross(edge1);

                /* calculate V parameter and test bounds */
                double v = dir.dot(qvec) * inv_det;

                if ((v < 0.0) || ((u + v) > 1.0)) {
                    return (false);
                }

                double t = edge2.dot(qvec) * inv_det;

                return (t > epsilon);
            }
        };
    }  // namespace geometry
}  // namespace math
}  // namespace utility

#endif  // UTILITY_MATH_GEOMETRY_TRIANGLE_H
