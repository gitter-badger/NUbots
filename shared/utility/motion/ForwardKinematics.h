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

#ifndef UTILITY_MOTION_FORWARDKINEMATICS_H
#define UTILITY_MOTION_FORWARDKINEMATICS_H

#include <armadillo>
#include <cmath>
#include <nuclear>
#include <vector>

#include "message/input/Sensors.h"
#include "message/motion/KinematicsModel.h"

#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/math/angle.h"
#include "utility/math/geometry/Line.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/support/eigen_armadillo.h"

namespace utility {
namespace motion {
    namespace kinematics {

        using LimbID  = utility::input::LimbID;
        using ServoID = utility::input::ServoID;
        using message::input::Sensors;
        using message::motion::KinematicsModel;
        using BodySide = message::motion::BodySide::Value;


        inline std::map<ServoID, utility::math::matrix::Transform3D> calculateHeadJointPosition(
            const KinematicsModel& model,
            const float& HEAD_PITCH,
            const float& HEAD_YAW,
            ServoID servoID) {
            std::map<ServoID, utility::math::matrix::Transform3D> positions;

            utility::math::matrix::Transform3D runningTransform;
            arma::vec3 NECK_POS       = {model.head.NECK_BASE_POS_FROM_ORIGIN_X,
                                   model.head.NECK_BASE_POS_FROM_ORIGIN_Y,
                                   model.head.NECK_BASE_POS_FROM_ORIGIN_Z};
            float NECK_LENGTH         = model.head.NECK_LENGTH;
            arma::vec3 NECK_TO_CAMERA = {
                model.head.NECK_TO_CAMERA_X, model.head.NECK_TO_CAMERA_Y, model.head.NECK_TO_CAMERA_Z};

            // Translate to base of neck from origin
            runningTransform = runningTransform.translate(NECK_POS);
            // Rotate to face out of base of neck
            runningTransform = runningTransform.rotateY(-M_PI_2);
            // Rotate head in yaw axis
            runningTransform = runningTransform.rotateX(HEAD_YAW);
            // Translate to top of neck (i.e. next motor axle)
            runningTransform = runningTransform.translateX(NECK_LENGTH);
            // YAW
            // Return the basis pointing out of the top of the torso with z pointing out the back of the neck. Pos is
            // top of neck (at hip pitch motor)
            positions[ServoID::HEAD_YAW] = runningTransform;
            if (servoID == ServoID::HEAD_YAW) {
                return positions;
            }

            // Rotate to face forward direction of neck
            runningTransform = runningTransform.rotateY(M_PI_2);
            // Rotate pitch
            runningTransform = runningTransform.rotateY(HEAD_PITCH);
            // Translate to camera
            runningTransform = runningTransform.translate(NECK_TO_CAMERA);
            // Rotate to set x to camera vector
            runningTransform = runningTransform.rotateY(model.head.CAMERA_DECLINATION_ANGLE_OFFSET);
            // PITCH
            // Return basis pointing along camera vector (ie x is camera vector, z out of top of head). Pos at camera
            // position
            positions[ServoID::HEAD_PITCH] = runningTransform;
            return positions;
        }

        inline std::map<ServoID, utility::math::matrix::Transform3D>
        calculateHeadJointPosition(const KinematicsModel& model, const Sensors& sensors, ServoID servoID) {
            return calculateHeadJointPosition(model,
                                              sensors.servo[static_cast<int>(ServoID::HEAD_PITCH)].presentPosition,
                                              sensors.servo[static_cast<int>(ServoID::HEAD_YAW)].presentPosition,
                                              servoID);
        }
        /*! @brief
            @NOTE read " runningTransform *= utility::math::matrix::_RotationMatrix(angle, 4); " as "Rotate the running
           transform about its local _ coordinate by angle."
            @return Returns basis matrix for position of end of limb controlled by the specified motor.

            The basis 'faces' down its x axis.
        */
        inline std::map<ServoID, utility::math::matrix::Transform3D> calculateLegJointPosition(
            const KinematicsModel& model,
            const Sensors& sensors,
            ServoID servoID,
            BodySide isLeft) {
            std::map<ServoID, utility::math::matrix::Transform3D> positions;
            utility::math::matrix::Transform3D runningTransform;
            // Variables to mask left and right leg differences:
            ServoID HIP_YAW, HIP_ROLL, HIP_PITCH, KNEE, ANKLE_PITCH, ANKLE_ROLL;
            int negativeIfRight = 1;

            if (static_cast<bool>(isLeft)) {
                HIP_YAW     = ServoID::L_HIP_YAW;
                HIP_ROLL    = ServoID::L_HIP_ROLL;
                HIP_PITCH   = ServoID::L_HIP_PITCH;
                KNEE        = ServoID::L_KNEE;
                ANKLE_PITCH = ServoID::L_ANKLE_PITCH;
                ANKLE_ROLL  = ServoID::L_ANKLE_ROLL;
            }
            else {
                HIP_YAW         = ServoID::R_HIP_YAW;
                HIP_ROLL        = ServoID::R_HIP_ROLL;
                HIP_PITCH       = ServoID::R_HIP_PITCH;
                KNEE            = ServoID::R_KNEE;
                ANKLE_PITCH     = ServoID::R_ANKLE_PITCH;
                ANKLE_ROLL      = ServoID::R_ANKLE_ROLL;
                negativeIfRight = -1;
            }

            // Hip pitch
            runningTransform = runningTransform.translate(
                {model.leg.HIP_OFFSET_X, negativeIfRight * model.leg.HIP_OFFSET_Y, -model.leg.HIP_OFFSET_Z});
            // Rotate to face down the leg (see above for definitions of terms, including 'facing')
            runningTransform = runningTransform.rotateY(M_PI_2);
            // Using right hand rule along global z gives positive direction of yaw:
            runningTransform = runningTransform.rotateX(-sensors.servo[static_cast<int>(HIP_YAW)].presentPosition);
            // Return basis facing from body to hip centre (down) with z aligned with the axis of the hip roll motor
            // axis. Position at hip joint
            positions[HIP_YAW] = runningTransform;
            if (servoID == HIP_YAW) {
                return positions;
            }

            runningTransform = runningTransform.rotateZ(sensors.servo[static_cast<int>(HIP_ROLL)].presentPosition);
            // Return basis facing down leg plane, with z oriented through axis of roll motor. Position still hip joint
            positions[HIP_ROLL] = runningTransform;
            if (servoID == HIP_ROLL) {
                return positions;
            }

            // Rotate to face down upper leg
            runningTransform = runningTransform.rotateY(sensors.servo[static_cast<int>(HIP_PITCH)].presentPosition);
            // Translate down upper leg
            runningTransform = runningTransform.translateX(model.leg.UPPER_LEG_LENGTH);
            // Return basis faces down upper leg, with z out of front of thigh. Pos = knee axis centre
            positions[HIP_PITCH] = runningTransform;
            if (servoID == HIP_PITCH) {
                return positions;
            }


            // Rotate to face down lower leg
            runningTransform = runningTransform.rotateY(sensors.servo[static_cast<int>(KNEE)].presentPosition);
            // Translate down lower leg
            runningTransform = runningTransform.translateX(model.leg.UPPER_LEG_LENGTH);
            // Return basis facing down lower leg, with z out of front of shin. Pos = ankle axis centre
            positions[KNEE] = runningTransform;
            if (servoID == KNEE) {
                return positions;
            }


            // Rotate to face down foot (pitch)
            runningTransform = runningTransform.rotateY(sensors.servo[static_cast<int>(ANKLE_PITCH)].presentPosition);
            // Return basis facing pitch down to foot with z out the front of the foot. Pos = ankle axis centre
            positions[ANKLE_PITCH] = runningTransform;
            if (servoID == ANKLE_PITCH) {
                return positions;
            }

            // Rotate to face down foot (roll)
            runningTransform = runningTransform.rotateZ(sensors.servo[static_cast<int>(ANKLE_ROLL)].presentPosition);
            // Rotate so x faces toward toes
            runningTransform = runningTransform.rotateY(-M_PI_2);
            // Translate to ground
            runningTransform = runningTransform.translateZ(-model.leg.FOOT_HEIGHT);
            // Return basis with x out of the front of the toe and z out the top of foot. Pos = ankle axis centre
            // projected to ground
            positions[ANKLE_ROLL] = runningTransform;
            return positions;
        }

        /*! @brief
            @NOTE read " runningTransform *= utility::math::matrix::_RotationMatrix(angle, 4); " as "Rotate the running
           transform about its local _ coordinate by angle."
            @return Returns basis matrix for position of end of limb controlled by the specified motor.

            The basis 'faces' down its x axis.
        */
        inline std::map<ServoID, utility::math::matrix::Transform3D> calculateArmJointPosition(
            const KinematicsModel& model,
            const Sensors& sensors,
            ServoID servoID,
            BodySide isLeft) {
            std::map<ServoID, utility::math::matrix::Transform3D> positions;
            utility::math::matrix::Transform3D runningTransform;
            // Variables to mask left and right differences:
            ServoID SHOULDER_PITCH, SHOULDER_ROLL, ELBOW;
            int negativeIfRight = 1;

            if (static_cast<bool>(isLeft)) {
                SHOULDER_PITCH = ServoID::L_SHOULDER_PITCH;
                SHOULDER_ROLL  = ServoID::L_SHOULDER_ROLL;
                ELBOW          = ServoID::L_ELBOW;
            }
            else {
                SHOULDER_PITCH  = ServoID::R_SHOULDER_PITCH;
                SHOULDER_ROLL   = ServoID::R_SHOULDER_ROLL;
                ELBOW           = ServoID::R_ELBOW;
                negativeIfRight = -1;
            }

            float shoulder_pitch = sensors.servo[static_cast<int>(SHOULDER_PITCH)].presentPosition;
            float shoulder_roll  = sensors.servo[static_cast<int>(SHOULDER_ROLL)].presentPosition;
            float elbow          = sensors.servo[static_cast<int>(ELBOW)].presentPosition;

            // std::cout << "shoulder_pitch = " << shoulder_pitch << std::endl;
            // std::cout << "shoulder_roll = " << shoulder_roll << std::endl;
            // std::cout << "elbow = " << elbow << std::endl;

            // Translate to shoulder
            runningTransform = runningTransform.translate({model.arm.SHOULDER_X_OFFSET,
                                                           negativeIfRight * model.arm.DISTANCE_BETWEEN_SHOULDERS / 2.0,
                                                           model.arm.SHOULDER_Z_OFFSET});
            // Rotate about shoulder pitch with zero position Zombie arms
            runningTransform = runningTransform.rotateY(shoulder_pitch - M_PI_2);
            // Translate to end of shoulder part
            runningTransform = runningTransform.translate(
                {model.arm.SHOULDER_LENGTH, negativeIfRight * model.arm.SHOULDER_WIDTH, -model.arm.SHOULDER_HEIGHT});
            // Return matrix pointing forward out of shoulder, y same as global y. Pos = at centre of shoulder roll
            // joint
            positions[SHOULDER_PITCH] = runningTransform;
            if (servoID == SHOULDER_PITCH) {
                return positions;
            }

            // Rotate by the shoulder roll
            runningTransform = runningTransform.rotateX(shoulder_roll);
            // Translate to centre of next joint
            runningTransform = runningTransform.translate({model.arm.UPPER_ARM_X_OFFSET,
                                                           negativeIfRight * model.arm.UPPER_ARM_Y_OFFSET,
                                                           -model.arm.UPPER_ARM_LENGTH});
            // Rotate to face down arm
            runningTransform = runningTransform.rotateY(M_PI_2);

            positions[SHOULDER_ROLL] = runningTransform;
            if (servoID == SHOULDER_ROLL) {
                return positions;
            }

            // Rotate by the elbow angle
            runningTransform = runningTransform.rotateY(elbow);
            // Translate to centre of end of arm, in line with joint
            runningTransform = runningTransform.translate({model.arm.LOWER_ARM_LENGTH,
                                                           negativeIfRight * model.arm.LOWER_ARM_Y_OFFSET,
                                                           -model.arm.LOWER_ARM_Z_OFFSET});
            positions[ELBOW] = runningTransform;
            return positions;
        }

        /*! @brief
         */
        inline std::map<ServoID, utility::math::matrix::Transform3D> calculatePosition(const KinematicsModel& model,
                                                                                       const Sensors& sensors,
                                                                                       ServoID servoID) {
            switch (servoID.value) {
                case ServoID::HEAD_YAW:
                case ServoID::HEAD_PITCH: return calculateHeadJointPosition(model, sensors, servoID);
                case ServoID::R_SHOULDER_PITCH:
                case ServoID::R_SHOULDER_ROLL:
                case ServoID::R_ELBOW: return calculateArmJointPosition(model, sensors, servoID, BodySide::RIGHT);
                case ServoID::L_SHOULDER_PITCH:
                case ServoID::L_SHOULDER_ROLL:
                case ServoID::L_ELBOW: return calculateArmJointPosition(model, sensors, servoID, BodySide::LEFT);
                case ServoID::R_HIP_YAW:
                case ServoID::R_HIP_ROLL:
                case ServoID::R_HIP_PITCH:
                case ServoID::R_KNEE:
                case ServoID::R_ANKLE_PITCH:
                case ServoID::R_ANKLE_ROLL: return calculateLegJointPosition(model, sensors, servoID, BodySide::RIGHT);
                case ServoID::L_HIP_YAW:
                case ServoID::L_HIP_ROLL:
                case ServoID::L_HIP_PITCH:
                case ServoID::L_KNEE:
                case ServoID::L_ANKLE_PITCH:
                case ServoID::L_ANKLE_ROLL: return calculateLegJointPosition(model, sensors, servoID, BodySide::LEFT);
                default: return std::map<ServoID, utility::math::matrix::Transform3D>();
            }
        }


        inline std::map<ServoID, utility::math::matrix::Transform3D> calculateAllPositions(const KinematicsModel& model,
                                                                                           const Sensors& sensors) {
            std::map<ServoID, utility::math::matrix::Transform3D> result =
                calculatePosition(model, sensors, ServoID::L_ANKLE_ROLL);
            std::map<ServoID, utility::math::matrix::Transform3D> leftLegPositions =
                calculatePosition(model, sensors, ServoID::L_ANKLE_ROLL);
            std::map<ServoID, utility::math::matrix::Transform3D> rightLegPositions =
                calculatePosition(model, sensors, ServoID::R_ANKLE_ROLL);
            std::map<ServoID, utility::math::matrix::Transform3D> headPositions =
                calculatePosition(model, sensors, ServoID::HEAD_PITCH);
            std::map<ServoID, utility::math::matrix::Transform3D> leftArm =
                calculatePosition(model, sensors, ServoID::L_ELBOW);
            std::map<ServoID, utility::math::matrix::Transform3D> rightArm =
                calculatePosition(model, sensors, ServoID::R_ELBOW);
            result.insert(leftArm.begin(), leftArm.end());
            result.insert(rightArm.begin(), rightArm.end());
            result.insert(leftLegPositions.begin(), leftLegPositions.end());
            result.insert(rightLegPositions.begin(), rightLegPositions.end());
            result.insert(headPositions.begin(), headPositions.end());
            return result;
        }
        /*! @brief Adds up the mass vectors stored in the robot model and normalises the resulting position
            @return [x_com, y_com, z_com, total_mass] relative to the torso basis
        */
        inline arma::vec4 calculateCentreOfMass(
            const message::motion::KinematicsModel& model,
            const std::array<Eigen::Matrix<double, 4, 4, Eigen::DontAlign>, 20>& jointPositions,
            bool includeTorso) {
            arma::vec4 totalMassVector = arma::zeros(4);

            for (size_t j = 0; j < 20; j++) {
                const auto& joint = jointPositions[j];
                arma::vec4 massVector;
                for (size_t i = 0; i < 4; i++) {
                    massVector[i] = model.massModel.masses[j][i];
                }
                // NUClear::log<NUClear::DEBUG>("calculateCentreOfMass - reading mass ",
                // message::input::stringFromId(j), massVector);
                double jointMass = massVector[3];

                utility::math::matrix::Transform3D massScaler;
                massScaler.submat(0, 0, 2, 2) *= jointMass;

                // = m * local centre of mass in global robot coords
                totalMassVector += convert<double, 4, 4>(joint) * massScaler * massVector;
            }

            if (includeTorso) {
                arma::vec4 massVector;
                for (size_t i = 0; i < 4; i++) {
                    massVector[i] = model.massModel.masses[20][i];
                }
                // NUClear::log<NUClear::DEBUG>("calculateCentreOfMass - reading mass Torso", massVector);
                double jointMass = massVector[3];
                utility::math::matrix::Transform3D massScaler;
                massScaler.submat(0, 0, 2, 2) *= jointMass;
                totalMassVector += massScaler * massVector;  // = m * local centre of mass in global robot coords
            }

            utility::math::matrix::Transform3D normaliser;
            if (totalMassVector[3] > 0) {
                normaliser.submat(0, 0, 2, 2) *= 1 / totalMassVector[3];
                return normaliser * totalMassVector;
            }
            else {
                NUClear::log<NUClear::ERROR>(
                    "ForwardKinematics::calculateCentreOfMass - Empty centre of mass request or no mass in mass "
                    "model.");
                return arma::vec4();
            }
        }

        inline utility::math::geometry::Line calculateHorizon(const math::matrix::Rotation3D Rcw,
                                                              double cameraDistancePixels) {

            // Normal of the line is the y and z of the z axis, however in the image the y axis is negated
            arma::vec2 normal = -arma::normalise(Rcw.submat(1, 2, 2, 2));
            double distance   = cameraDistancePixels * std::tan(utility::math::angle::acos_clamped(Rcw(0, 2)) - M_PI_2);

            return utility::math::geometry::Line(normal, distance);
        }

        inline utility::math::matrix::Transform3D calculateBodyToGround(arma::vec3 groundNormal_body,
                                                                        double bodyHeight) {
            arma::vec3 X            = arma::vec{1, 0, 0};
            double projectXOnNormal = groundNormal_body[0];

            arma::vec3 groundMatrixX;
            arma::vec3 groundMatrixY;

            if (std::fabs(projectXOnNormal) == 1) {
                // Then x is parallel to the ground normal and we need to use projection onto +/-z instead
                // If x parallel to normal, then use -z, if x antiparallel use z
                arma::vec3 Z            = arma::vec3{0, 0, (projectXOnNormal > 0 ? -1.0 : 1.0)};
                double projectZOnNormal = arma::dot(Z, groundNormal_body);
                groundMatrixX           = arma::normalise(Z - projectZOnNormal * groundNormal_body);
                groundMatrixY           = arma::cross(groundNormal_body, groundMatrixX);
            }
            else {
                groundMatrixX = arma::normalise(X - projectXOnNormal * groundNormal_body);
                groundMatrixY = arma::cross(groundNormal_body, groundMatrixX);
            }

            utility::math::matrix::Transform3D groundToBody;
            groundToBody.submat(0, 0, 2, 0) = groundMatrixX;
            groundToBody.submat(0, 1, 2, 1) = groundMatrixY;
            groundToBody.submat(0, 2, 2, 2) = groundNormal_body;
            groundToBody.submat(0, 3, 2, 3) = arma::vec{0, 0, -bodyHeight};
            return groundToBody.i();
        }

        inline arma::mat22 calculateRobotToIMU(math::matrix::Rotation3D orientation) {
            arma::vec3 xRobotImu  = orientation.i().col(0);
            arma::vec2 projXRobot = arma::normalise(xRobotImu.rows(0, 1));
            arma::vec2 projYRobot = arma::vec2({-projXRobot(1), projXRobot(0)});

            arma::mat22 robotToImu;
            robotToImu.col(0) = projXRobot;
            robotToImu.col(1) = projYRobot;

            return robotToImu;
        }

        inline arma::vec4 fsrCentreInBodyCoords(const KinematicsModel& model,
                                                const Sensors& sensors,
                                                const arma::vec2& foot,
                                                bool left) {
            // sensors.bodyToGround

            int negativeIfRight = left ? 1 : -1;

            arma::vec2 position = foot % arma::vec2({model.leg.FOOT_LENGTH / 2, model.leg.FOOT_WIDTH / 2});
            arma::vec4 centerFoot =
                arma::vec4({position[0], position[1] + negativeIfRight * model.leg.FOOT_CENTRE_TO_ANKLE_CENTRE, 0, 1});

            return ((left) ? convert<double, 4, 4>(sensors.forwardKinematics[ServoID::L_ANKLE_ROLL]) * centerFoot
                           : convert<double, 4, 4>(sensors.forwardKinematics[ServoID::R_ANKLE_ROLL]) * centerFoot);
        }

        inline arma::vec3 calculateCentreOfPressure(const KinematicsModel& model, const Sensors& sensors) {
            arma::vec4 CoP            = {0, 0, 0, 1};
            float number_of_feet_down = 0;
            if (sensors.leftFootDown) {
                CoP += fsrCentreInBodyCoords(
                    model, sensors, convert<double, 2>(sensors.fsr[LimbID::LEFT_LEG - 1].centre), true);
                number_of_feet_down += 1.0f;
            }
            if (sensors.rightFootDown) {
                CoP += fsrCentreInBodyCoords(
                    model, sensors, convert<double, 2>(sensors.fsr[LimbID::RIGHT_LEG - 1].centre), false);
                number_of_feet_down += 1.0f;
            }
            if (number_of_feet_down == 2) {
                CoP = CoP / number_of_feet_down;
            }
            // reset homogeneous coordinate
            CoP(3)              = 1;
            arma::vec4 CoP_body = convert<double, 4, 4>(sensors.bodyToGround) * CoP;
            return CoP_body.rows(0, 2);
        }

        /*! @return matrix J such that \overdot{X} = J * \overdot{theta}
         */
        inline arma::mat33 calculateArmJacobian(const KinematicsModel& model, const arma::vec3& a, bool isLeft) {
            int negativeIfRight = isLeft ? 1 : -1;

            const arma::vec3 t1 = {
                model.arm.SHOULDER_LENGTH, negativeIfRight * model.arm.SHOULDER_WIDTH, -model.arm.SHOULDER_HEIGHT};
            const arma::vec3 t2 = {model.arm.UPPER_ARM_X_OFFSET,
                                   negativeIfRight * model.arm.UPPER_ARM_Y_OFFSET,
                                   -model.arm.UPPER_ARM_LENGTH};
            const arma::vec3 t3 = {model.arm.LOWER_ARM_LENGTH,
                                   negativeIfRight * model.arm.LOWER_ARM_Y_OFFSET,
                                   -model.arm.LOWER_ARM_Z_OFFSET};

            arma::mat33 jRY1 = utility::math::matrix::Rotation3D::createRotationYJacobian(a[0] - M_PI_2);
            arma::mat33 jRX2 = utility::math::matrix::Rotation3D::createRotationXJacobian(a[1]);
            arma::mat33 jRY3 = utility::math::matrix::Rotation3D::createRotationXJacobian(a[2]);

            arma::mat33 RY1 = utility::math::matrix::Rotation3D::createRotationY(a[0] - M_PI_2);
            arma::mat33 RX2 = utility::math::matrix::Rotation3D::createRotationX(a[1]);
            arma::mat33 RY3 = utility::math::matrix::Rotation3D::createRotationY(a[2]);

            arma::mat33 RY_PI_2 = utility::math::matrix::Rotation3D::createRotationY(M_PI_2);

            arma::vec3 col1 = jRY1 * RX2 * RY_PI_2 * RY3 * t3 + jRY1 * RX2 * t2 + jRY1 * t1;

            arma::vec3 col2 = RY1 * jRX2 * RY_PI_2 * RY3 * t3 + RY1 * jRX2 * t2;

            arma::vec3 col3 = RY1 * RX2 * RY_PI_2 * jRY3 * t3;

            return arma::join_rows(col1, arma::join_rows(col2, col3));
        }
        /*! @return matrix J such that \overdot{X} = J * \overdot{theta}
         */
        inline arma::vec3 calculateArmPosition(const KinematicsModel& model, const arma::vec3& a, bool isLeft) {
            int negativeIfRight = isLeft ? 1 : -1;

            const arma::vec3 t0 = {model.arm.SHOULDER_X_OFFSET,
                                   negativeIfRight * model.arm.DISTANCE_BETWEEN_SHOULDERS / 2.0,
                                   model.arm.SHOULDER_Z_OFFSET};

            const arma::vec3 t1 = {
                model.arm.SHOULDER_LENGTH, negativeIfRight * model.arm.SHOULDER_WIDTH, -model.arm.SHOULDER_HEIGHT};
            const arma::vec3 t2 = {model.arm.UPPER_ARM_X_OFFSET,
                                   negativeIfRight * model.arm.UPPER_ARM_Y_OFFSET,
                                   -model.arm.UPPER_ARM_LENGTH};
            const arma::vec3 t3 = {model.arm.LOWER_ARM_LENGTH,
                                   negativeIfRight * model.arm.LOWER_ARM_Y_OFFSET,
                                   -model.arm.LOWER_ARM_Z_OFFSET};

            arma::mat33 RY_PI_2 = utility::math::matrix::Rotation3D::createRotationY(M_PI_2);

            arma::mat33 RY1 = utility::math::matrix::Rotation3D::createRotationY(a[0] - M_PI_2);
            arma::mat33 RX2 = utility::math::matrix::Rotation3D::createRotationX(a[1]);
            arma::mat33 RY3 = utility::math::matrix::Rotation3D::createRotationY(a[2]);

            arma::vec3 pos = RY1 * RX2 * RY_PI_2 * RY3 * t3 + RY1 * RX2 * t2 + RY1 * t1 + t0;

            return pos;
        }


    }  // namespace kinematics
}  // namespace motion
}  // namespace utility

#endif  // UTILITY_MOTION_FORWARDKINEMATICS_H
