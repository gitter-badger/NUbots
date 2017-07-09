/*
 * This file is part of the Autocalibration Codebase.
 *
 * The Autocalibration Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The Autocalibration Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Autocalibration Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_MATRIX_TRANSFORM2D_H
#define UTILITY_MATH_MATRIX_TRANSFORM2D_H

#include <Eigen/Core>

#include "Rotation2D.h"

namespace utility {
namespace math {
    namespace matrix {

        template <int Dimensions>
        class Transform;

        using Transform2D = Transform<2>;

        /**
         * Represents a 2D point including its rotation. Uses a vec3 of the form {x, y, angle}.
         *
         * See: Special Euclidean group SE(2).
         * http://en.wikipedia.org/wiki/Euclidean_group
         *
         * @author Brendan Annable
         */
        template <>
        class Transform<2> : public Eigen::Vector3d {
        public:
            using Eigen::Vector3d::Matrix;

            /**
             * @brief Default constructor initialises values to zero
             */
            Transform() {
                setZero();
            }

            /**
             * @brief Construct transform from a position and an angle.
             */
            Transform(const Eigen::Vector2d xy_, double angle_);

            /**
             * Construct a transform that represents the position and
             * orientation of a camera positioned at 'from' and facing toward
             * 'to'.
             */
            static Transform2D lookAt(const Eigen::Vector2d from, Eigen::Vector2d to);

            /**
             * @brief Transforms position from local coordinates relative to 'reference', to world coordinates
             *
             * @param reference A position to become relatively local to
             * @return The new position
             */
            Transform2D localToWorld(const Transform2D& reference) const;

            /**
             * @brief Transforms position from world coordinates to be local to 'reference'
             *
             * @param reference The position that the current position is relative to
             * @return The new position
             */
            Transform2D worldToLocal(const Transform2D& reference) const;


            /**
             * Interpolate between itself and given target vector
             *
             * @param t A value between 0-1 to interpolate between the two,
             * outside these bounds will extrapolate
             * @param target The target vector
             * @return The interpolated vector
             */
            Transform2D interpolate(double t, const Transform2D& target) const;

            Transform2D inverse() const;

            inline double x() const {
                return operator[](0);
            }
            inline double& x() {
                return operator[](0);
            }

            inline double y() const {
                return operator[](1);
            }
            inline double& y() {
                return operator[](1);
            }

            inline double angle() const {
                return operator[](2);
            }
            inline double& angle() {
                return operator[](2);
            }

            inline Rotation2D rotation() {
                return Rotation2D::createRotation(angle());
            }

            inline const Eigen::Vector2d xy() const {
                return head<2>();
            }
            inline Eigen::Vector2d xy() {
                return head<2>();
            }
        };

    }  // namespace matrix
}  // namespace math
}  // namespace utility

#endif  // UTILITY_MATH_MATRIX_TRANSFORM2D_H
