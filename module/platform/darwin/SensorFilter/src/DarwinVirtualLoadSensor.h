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

#ifndef MODULES_PLATFORM_DARWIN_DARWINVIRTUALLOADSENSOR_H
#define MODULES_PLATFORM_DARWIN_DARWINVIRTUALLOADSENSOR_H

#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform3D.h"


namespace module {
namespace platform {
    namespace darwin {

        class DarwinVirtualLoadSensor {
            // this implements a linear model (trained by logistic regression) with a bayes filter on the output
        private:
            double noiseFactor;
            double currentNoise;
            double certaintyThreshold;
            double uncertaintyThreshold;

            Eigen::MatrixXd hiddenWeights;
            Eigen::VectorXd hiddenBias;
            Eigen::MatrixXd outputWeights;
            Eigen::VectorXd outputBias;

        public:
            double state     = 0.5;
            bool outputState = true;


            DarwinVirtualLoadSensor();

            DarwinVirtualLoadSensor(const Eigen::MatrixXd& hiddenWeights,
                                    const Eigen::VectorXd& hiddenBias,
                                    const Eigen::MatrixXd& outputWeights,
                                    const Eigen::VectorXd& outputBias,
                                    double noiseFactor,
                                    double certaintyThreshold,
                                    double uncertaintyThreshold);

            bool updateFoot(const Eigen::VectorXd& legMotors);
        };
    }  // namespace darwin
}  // namespace platform
}  // namespace module

#endif
