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

#include "RobotModel.h"

#include <iostream>
#include <nuclear>

#include "message/input/Sensors.h"
#include "message/input/ServoID.h"
#include "message/localisation/FieldObject.h"
#include "utility/localisation/transform.h"
#include "utility/math/angle.h"
#include "utility/math/coordinates.h"

namespace module {
namespace localisation {
    namespace robot {

        using message::input::Sensors;
        using message::input::ServoID;
        using utility::localisation::transform::SphericalRobotObservation;
        using utility::localisation::transform::WorldToRobotTransform;
        using utility::localisation::transform::RobotToWorldTransform;
        using utility::localisation::transform::ImuToWorldHeadingTransform;
        using utility::math::coordinates::cartesianToRadial;
        using utility::math::coordinates::cartesianToSpherical;

        Eigen::Matrix<double, RobotModel::size, 1> RobotModel::timeUpdate(
            const Eigen::Matrix<double, RobotModel::size, 1>& state,
            double /*deltaT*/,
            const Sensors& /*sensors*/) {
            Eigen::Matrix<double, RobotModel::size, 1> new_state = state;

            return new_state;
        }


        /// Return the predicted observation of an object at the given position
        Eigen::VectorXd RobotModel::predictedObservation(const Eigen::Matrix<double, RobotModel::size, 1>& state,
                                                         const Eigen::Vector3d& actual_position,
                                                         const Sensors& sensors) {

            // Rewrite:
            double rmHeading                  = sensors.world.rotation().yaw() - state(robot::kImuOffset);
            Eigen::Vector2d robotModelHeading = {std::cos(-rmHeading), std::sin(-rmHeading)};

            auto obs = SphericalRobotObservation(
                sensors.world.translation().rows(0, 1) - state.rows(kX, kY), robotModelHeading, actual_position);
            return obs;
        }

        // Angle between goals - NOTE: CURRENTLY UNUSED
        double RobotModel::predictedObservation(const Eigen::Matrix<double, RobotModel::size, 1>& state,
                                                const std::vector<Eigen::Vector2d>& actual_positions,
                                                const Sensors& /*sensors*/) {

            // TODO: needs to incorporate new motion model position data
            Eigen::Vector2d diff_1   = actual_positions[0].head<2>() - state.middleRows<kY - kX + 1>(kX);
            Eigen::Vector2d diff_2   = actual_positions[1].head<2>() - state.middleRows<kY - kX + 1>(kX);
            Eigen::Vector2d radial_1 = cartesianToRadial(diff_1);
            Eigen::Vector2d radial_2 = cartesianToRadial(diff_2);

            double angle_diff = utility::math::angle::difference(radial_1[1], radial_2[1]);

            return {std::abs(angle_diff)};
        }

        Eigen::VectorXd RobotModel::observationDifference(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
            if (a.size() == 1) {
                return a - b;
            }
            if (a.size() == 2) {
                return a - b;
            }
            else {
                // Spherical coordinates
                Eigen::Vector3d result = a - b;
                result(1) =
                    /*utility::math::angle::normalizeAngle*/ (result(1)) * cfg_.observationDifferenceBearingFactor;
                result(2) =
                    /*utility::math::angle::normalizeAngle*/ (result(2)) * cfg_.observationDifferenceElevationFactor;
                return result;
            }
        }

        Eigen::Matrix<double, RobotModel::size, 1> RobotModel::limitState(
            const Eigen::Matrix<double, RobotModel::size, 1>& state) {

            // TODO: Clip robot's state to the field.
            return state;
        }

        Eigen::Matrix<double, RobotModel::size, RobotModel::size> RobotModel::processNoise() {
            Eigen::Matrix<double, RobotModel::size, RobotModel::size> noise =
                Eigen::Matrix<double, RobotModel::size, RobotModel::size>::Identity();
            noise(kX, kX) *= cfg_.processNoisePositionFactor;
            noise(kY, kY) *= cfg_.processNoisePositionFactor;
            noise(kImuOffset, kImuOffset) *= cfg_.processNoiseHeadingFactor;
            // std::cout << "process noise = \n" << noise << std::endl;
            return noise;
        }
    }
}  // namespace robot
}  // namespace localisation
}  // namespace module
