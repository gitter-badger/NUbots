/*
 * This file is part of KFBallLocalisation.
 *
 * KFBallLocalisation is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * KFBallLocalisation is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with KFBallLocalisation.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_KFBALLLOCALISATIONENGINE_H
#define MODULES_KFBALLLOCALISATIONENGINE_H

#include <nuclear>
#include <armadillo>
#include <chrono>

#include "utility/math/kalman/UKF.h"
#include "messages/vision/VisionObjects.h"
#include "messages/localisation/FieldObject.h"
#include "BallModel.h"

namespace modules {
namespace localisation {

    class KFBallLocalisationEngine {
        public:

        KFBallLocalisationEngine() :
            ball_filter_(
                {0, 0, 0, 0}, // mean
                // {0, 0, 3.141},
                arma::eye(ball::BallModel::size, ball::BallModel::size) * 1, // cov
                1) // alpha
                {
            last_time_update_time_ = NUClear::clock::now();
        }

        void TimeUpdate(std::chrono::system_clock::time_point current_time);

        void TimeUpdate(std::chrono::system_clock::time_point current_time,
                        const messages::localisation::FakeOdometry& odom);

        double MeasurementUpdate(const messages::vision::VisionObject& observed_object);

        utility::math::kalman::UKF<ball::BallModel> ball_filter_;

    private:
        double SecondsSinceLastTimeUpdate(std::chrono::system_clock::time_point current_time);

        std::chrono::system_clock::time_point last_time_update_time_;
    };
}
}
#endif