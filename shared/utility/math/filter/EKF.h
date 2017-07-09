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

#ifndef UTILITY_MATH_FILTER_EKF_H
#define UTILITY_MATH_FILTER_EKF_H

#include <Eigen/Core>
#include <nuclear>

namespace utility {
namespace math {
    namespace filter {

        template <typename Model>  // model is is a template parameter that Kalman also inherits
        class EKF {
        public:
            // The model
            Model model;

        private:
            using StateVec = Eigen::Matrix<double, Model::size, 1>;
            using StateMat = Eigen::Matrix<double, Model::size, Model::size>;

            // the internal UKF variables
            StateMat processNoise, processNoisePartial;

            // the current state estimate
            StateVec state;


        public:
            EKF(StateVec initialMean       = Eigen::Matrix<double, Model::size, 1>::Zero(),
                StateMat initialCovariance = Eigen::Matrix<double, Model::size, Model::size>::Identity() * 0.1 StateMat
                    initialJacobian        = model.timeUpdateJacobian(0.0)) {
                // strictly speaking, a time update should be called straight away with a non-zero time delta
                // but.... we don't know what the delta is so we must trust the user to do this
                reset(initialMean, initialCovariance, initialJacobian);
            }

            void reset(StateVec initialMean, StateMat initialCovariance, StateMat initialJacobian) {
                state      = initialMean;
                covariance = initialCovariance;
                jacobian   = initialJacobian;

                // re-initialize covariance
                processNoise        = Eigen::Matrix<double, Model::size, Model::size>::Identity();
                processNoisePartial = Eigen::Matrix<double, Model::size, Model::size>::Identity();
            }

            void setState(StateVec initialMean) {
                // this is for hard resets where covariance data should be kept. Again, call timeUpdate immediately.
                state = initialMean;
            }

            template <typename... TAdditionalParameters>
            void timeUpdate(double deltaT, const TAdditionalParameters&... additionalParameters) {
                // timeUpdate sets the new jacobian as well as updating parameters
                state             = model.timeUpdate(state, deltaT, additionalParameters...);
                StateMat jacobian = model.timeUpdateJacobian(state, additionalParameters...);

                // this is the original
                // processNoise = jacobian * processNoise * jacobian.transpose() + model.processNoise();

                // this is steve's out-of-order update (backported from UKF)
                processNoise =
                    jacobian * (processNoisePartial * processNoise) * jacobian.transpose() + model.processNoise();

                processNoisePartial = Eigen::Matrix<double, Model::size, Model::size>::Identity();
            }

            template <typename TMeasurement, typename... TMeasurementArgs>
            double measurementUpdate(const TMeasurement& measurement,
                                     const Eigen::MatrixXd& measurementVariance,
                                     const TMeasurementArgs&... measurementArgs) {
                Eigen::MatrixXd measurementTransform =
                    model.StateToMeasurementTransform(measurement, measurementArgs...);

                Eigen::MatrixXd kalmanGain =
                    processNoise * measurementTransform
                    * ((measurementTransform * processNoisePartial * processNoise * measurementTransform.transpose()
                        + measurementVariance)
                           .triangularView<Eigen::Upper>())
                          .inverse();

                state += kalmanGain * (measurement - measurementTransform * state);

                // original
                // processNoise = (Eigen::Matrix<double, Model::size, Model::size>::Identity() - kalmanGain * H) *
                // processNoise;

                // steve's backported out-of-order update
                processNoisePartial -= kalmanGain * measurementTransform;
            }

            StateVec get() const {
                return state;
            }

            StateMat getCovariance() const {
                return processNoisePartial * processNoise;
            }
        };
    }  // namespace filter
}  // namespace math
}  // namespace utility


#endif
