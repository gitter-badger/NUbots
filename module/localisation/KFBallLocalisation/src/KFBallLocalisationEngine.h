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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_KFBALLLOCALISATIONENGINE_H
#define MODULES_KFBALLLOCALISATIONENGINE_H

#include <chrono>
#include <nuclear>

#include "BallModel.h"
#include "extension/Configuration.h"
#include "message/localisation/FieldObject.h"
#include "message/vision/VisionObjects.h"
#include "utility/math/filter/ParticleFilter.h"
#include "utility/math/filter/UKF.h"

namespace module {
namespace localisation {

    class KFBallLocalisationEngine {
    public:
        KFBallLocalisationEngine()
            : ball_filter_({3, 2, 0, 0},  // mean
                           // {0, 0, 3.141},
                           Eigen::Matrix<double, ball::BallModel::size, ball::BallModel::size>::Identity() * 1,  // cov
                           0.1)  // alpha
            , cfg_()
            , last_time_update_time_(NUClear::clock::now()) {}

        void TimeUpdate(NUClear::clock::time_point current_time);

        // void TimeUpdate(std::chrono::system_clock::time_point current_time,
        //                 const message::localisation::FakeOdometry& odom);

        double MeasurementUpdate(const message::vision::VisionObject& observed_object);

        void UpdateConfiguration(const extension::Configuration& config);

        bool CanEmitFieldObjects();

        // utility::math::kalman::ParticleFilter<ball::BallModel> ball_filter_;
        utility::math::filter::UKF<ball::BallModel> ball_filter_;

    private:
        struct Config {
            Config() : emitBallFieldobjects(false) {}
            Config(bool emit) : emitBallFieldobjects(emit) {}

            bool emitBallFieldobjects;
        } cfg_;

        double SecondsSinceLastTimeUpdate(std::chrono::system_clock::time_point current_time);

        NUClear::clock::time_point last_time_update_time_;
    };
}
}
#endif
