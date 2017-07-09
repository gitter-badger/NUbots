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

#include "Transform3D.h"

namespace utility {
namespace math {
    namespace matrix {

        using geometry::UnitQuaternion;

        Transform3D::Transform(const UnitQuaternion& q) : Transform(Rotation3D(q)) {}

        Transform3D::Transform(const Rotation3D& rotation) : Transform() {
            topLeftCorner<3, 3>() = rotation;
        }

        Transform3D::Transform(const Rotation3D& rotation, const Eigen::Vector3d& translation) : Transform() {
            topLeftCorner<3, 3>() = rotation;
            this->translation() = translation;
        }

        Transform3D::Transform(const Transform2D& transform)
            : Transform(Transform3D().translate({transform.x(), transform.y(), 0}).rotateZ(transform.angle())) {}

        Transform3D::Transform(const Eigen::Matrix<double, 6, 1>& in)
            : Transform(Transform3D().translate(in.head<3>()).rotateZ(in[5]).rotateY(in[4]).rotateX(in[3])) {}

        Transform3D::Transform(const Eigen::Vector3d& in) : Transform(Transform3D().translate(in)) {}

        Transform3D Transform3D::translate(const Eigen::Vector3d& translation) const {
            return *this * createTranslation(translation);
        }

        Transform3D Transform3D::translateX(double translation) const {
            return translate({translation, 0, 0});
        }

        Transform3D Transform3D::translateY(double translation) const {
            return translate({0, translation, 0});
        }

        Transform3D Transform3D::translateZ(double translation) const {
            return translate({0, 0, translation});
        }

        Transform3D Transform3D::rotateX(double radians) const {
            return *this * createRotationX(radians);
        }

        Transform3D Transform3D::rotateY(double radians) const {
            return *this * createRotationY(radians);
        }

        Transform3D Transform3D::rotateZ(double radians) const {
            return *this * createRotationZ(radians);
        }

        Transform3D Transform3D::scale(const Eigen::Vector3d& v) const {
            return *this * createScale(v);
        }

        Transform3D Transform3D::rotateLocal(const Rotation3D& rotation, const Transform3D& local) const {
            return Transform3D(Transform3D(rotation) * worldToLocal(local)).localToWorld(local);
        }

        Transform3D Transform3D::rotateXLocal(double radians, const Transform3D& local) const {
            return Transform3D(createRotationX(radians) * worldToLocal(local)).localToWorld(local);
        }

        Transform3D Transform3D::rotateYLocal(double radians, const Transform3D& local) const {
            return Transform3D(createRotationY(radians) * worldToLocal(local)).localToWorld(local);
        }

        Transform3D Transform3D::rotateZLocal(double radians, const Transform3D& local) const {
            return Transform3D(createRotationZ(radians) * worldToLocal(local)).localToWorld(local);
        }

        Transform3D Transform3D::worldToLocal(const Transform3D& reference) const {
            // http://en.wikipedia.org/wiki/Change_of_basis
            return reference.inverse() * (*this);
        }

        Transform3D Transform3D::localToWorld(const Transform3D& reference) const {
            // http://en.wikipedia.org/wiki/Change_of_basis
            return reference * (*this);
        }

        Eigen::Vector3d Transform3D::transformPoint(const Eigen::Vector3d& p) {
            Eigen::Vector4d p4;
            p4.head<3>() = p;
            p4[3]        = 1;

            Eigen::Vector4d result4 = *this * p4;
            return result4.head<3>();
        }

        Eigen::Vector3d Transform3D::transformVector(const Eigen::Vector3d& p) {
            Eigen::Vector4d p4;
            p4.head<3>() = p;
            p4[3]        = 0;

            Eigen::Vector4d result4 = *this * p4;
            return result4.head<3>();
        }


        Transform3D Transform3D::i() const {
            // Create a new transform
            Transform3D inverseTransform3D;
            // Transpose the rotation submatrix (top-left 3x3), this is equivalent to taking the inverse of the rotation
            // matrix
            inverseTransform3D.topLeftCorner<3, 3>() = topLeftCorner<3, 3>().transpose();
            // Multiply translation vector (top-right column vector) by the negated inverse rotation matrix

            inverseTransform3D.topRightCorner<3, 1>() =
                -inverseTransform3D.topLeftCorner<3, 3>() * topRightCorner<3, 1>();

            return inverseTransform3D;
        }

        float Transform3D::norm(Transform3D T) {
            float pos_norm = T.translation().norm();
            // return Rotation3D::norm(T.rotation());
            // TODO: how to weight these two?
            return pos_norm + Rotation3D::norm(T.rotation());
        }

        float Transform3D::random(float a, float b) {
            float alpha = rand() / float(RAND_MAX);
            return a * alpha + b * (1 - alpha);
        }

        Transform3D Transform3D::getRandomU(float max_angle, float max_displacement) {
            UnitQuaternion q = UnitQuaternion::getRandomU(max_angle);
            Rotation3D R(q);

            // Get displacement:

            float phi      = random(0, 2 * M_PI);
            float costheta = random(-1, 1);
            float u        = random(0, 1);

            float theta = std::acos(costheta);
            float r     = max_displacement * std::pow(u, 1 / 3.0);

            float x = r * std::sin(theta) * std::cos(phi);
            float y = r * std::sin(theta) * std::sin(phi);
            float z = r * std::cos(theta);

            return Transform3D(R, Eigen::Vector3d(x, y, z));
        }

        Transform3D Transform3D::getRandomN(float stddev_angle, float stddev_disp) {
            UnitQuaternion q = UnitQuaternion::getRandomN(stddev_angle);
            Rotation3D R(q);

            // Get displacement:
            std::random_device rd;
            std::mt19937 gen(rd());
            std::normal_distribution<double> d(0.0, static_cast<double>(stddev_disp));

            Eigen::Vector3d displacement{d(gen), d(gen), d(gen)};

            return Transform3D(R, displacement);
        }

        Transform3D Transform3D::createTranslation(const Eigen::Vector3d& translation) {
            Transform3D transform;
            transform.translation() = translation;
            return transform;
        }

        Transform3D Transform3D::createRotationX(double radians) {
            Transform3D transform;
            transform.rotation() = Rotation3D::createRotationX(radians);
            return transform;
        }

        Transform3D Transform3D::createRotationY(double radians) {
            Transform3D transform;
            transform.rotation() = Rotation3D::createRotationY(radians);
            return transform;
        }

        Transform3D Transform3D::createRotationZ(double radians) {
            Transform3D transform;
            transform.rotation() = Rotation3D::createRotationZ(radians);
            return transform;
        }

        Transform3D Transform3D::createScale(const Eigen::Vector3d& v) {
            Transform3D transform;
            transform.rotation() = v.asDiagonal();
            return transform;
        }

        Transform3D Transform3D::interpolate(Transform3D T1, Transform3D T2, float alpha) {
            Rotation3D r1     = T1.rotation();
            UnitQuaternion q1 = UnitQuaternion(r1);
            Rotation3D r2     = T2.rotation();
            UnitQuaternion q2 = UnitQuaternion(r2);

            Eigen::Vector3d t1 = T1.translation();
            Eigen::Vector3d t2 = T2.translation();

            UnitQuaternion qResult  = q1.slerp(q2, alpha);
            Eigen::Vector3d tResult = alpha * (t2 - t1) + t1;

            Transform3D TResult   = Transform3D(Rotation3D(qResult));
            TResult.translation() = tResult;

            return TResult;
        }

        Transform2D Transform3D::projectTo2D(const Eigen::Vector3d& yawAxis, const Eigen::Vector3d& forwardAxis) const {
            Transform2D result;

            // Translation
            Eigen::Vector3d orthoForwardAxis = yawAxis.cross(forwardAxis.cross(yawAxis)).normalized();
            Eigen::Vector3d r                = translation();
            Rotation3D newSpaceToWorld;
            newSpaceToWorld.x()        = orthoForwardAxis;
            newSpaceToWorld.y()        = yawAxis.cross(orthoForwardAxis);
            newSpaceToWorld.z()        = yawAxis;
            Rotation3D worldToNewSpace = newSpaceToWorld.inverse();
            Eigen::Vector3d rNewSpace  = worldToNewSpace * r;
            result.xy()                = rNewSpace.head<2>();

            // Rotation
            Rotation3D rot       = rotation();
            Eigen::Vector3d x    = rot.x();
            Eigen::Vector3d xNew = worldToNewSpace * x;
            float theta_x_from_f = std::atan2(xNew[1], xNew[0]);  // sin/cos
            result.angle()       = theta_x_from_f;

            // std::cerr << "in = \n" << *this << std::endl;
            // std::cerr << "out = \n" << result << std::endl;
            return result;
        }
    }  // namespace matrix
}  // namespace math
}  // namespace utility
