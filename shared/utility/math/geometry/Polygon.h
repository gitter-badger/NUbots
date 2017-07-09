/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_GEOMETRY_POLYGON_H
#define UTILITY_MATH_GEOMETRY_POLYGON_H

#include <Eigen/Core>
#include <vector>

#include "ParametricLine.h"

namespace utility {
namespace math {
    namespace geometry {

        class Polygon {
        private:
            std::vector<ParametricLine<2>> edges;

        public:
            Polygon() : edges() {}
            Polygon(const std::vector<Eigen::Vector2d>& vertices);

            void set(const std::vector<Eigen::Vector2d>& vertices);

            /*! @brief Checks if the point lies within the boundary of the polygon
            */
            bool pointContained(const Eigen::Vector2d& p) const;
            /*! @brief Gets the closest point in the polygon to the specified point
            */
            Eigen::Vector2d projectPointToPolygon(const Eigen::Vector2d& p) const;
        };
    }  // namespace geometry
}  // namespace math
}  // namespace utility

#endif
