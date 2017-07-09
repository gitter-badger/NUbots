/*
 * This file is part of NUbots Codebase.
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

#include "GoalDetector.h"

#include "extension/Configuration.h"

#include "RansacGoalModel.h"

#include "message/input/CameraParameters.h"
#include "message/vision/ClassifiedImage.h"
#include "message/vision/LookUpTable.h"
#include "message/vision/VisionObjects.h"

#include "utility/math/geometry/Line.h"
#include "utility/math/geometry/Plane.h"
#include "utility/math/geometry/Quad.h"

#include "utility/math/coordinates.h"
#include "utility/math/ransac/NPartiteRansac.h"
#include "utility/math/statistics/running_stats.h"
#include "utility/math/vision.h"

#include "utility/vision/ClassifiedImage.h"
#include "utility/vision/LookUpTable.h"
#include "utility/vision/Vision.h"
#include "utility/vision/fourcc.h"


namespace module {
namespace vision {

    using extension::Configuration;

    using message::input::CameraParameters;

    using utility::math::coordinates::cartesianToSpherical;

    using utility::math::geometry::Line;
    using Plane = utility::math::geometry::Plane<3>;
    using utility::math::geometry::Quad;

    using utility::math::ransac::NPartiteRansac;

    using utility::math::statistics::running_stat;

    using utility::math::vision::widthBasedDistanceToCircle;
    using utility::math::vision::projectCamToPlane;
    using utility::math::vision::imageToScreen;
    using utility::math::vision::getCamFromScreen;
    using utility::math::vision::getParallaxAngle;
    using utility::math::vision::projectCamSpaceToScreen;
    using utility::math::vision::distanceToVerticalObject;

    using message::vision::LookUpTable;
    using message::vision::ClassifiedImage;
    using SegmentClass = message::vision::ClassifiedImage::SegmentClass::Value;
    using message::vision::Goal;

    // TODO the system is too generous with adding segments above and below the goals and makes them too tall, stop it
    // TODO the system needs to throw out the kinematics and height based measurements when it cannot be sure it saw the
    // tops and bottoms of the goals

    GoalDetector::GoalDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , MINIMUM_POINTS_FOR_CONSENSUS(0)
        , MAXIMUM_ITERATIONS_PER_FITTING(0)
        , MAXIMUM_FITTED_MODELS(0)
        , CONSENSUS_ERROR_THRESHOLD(0.0)
        , MAXIMUM_ASPECT_RATIO(0.0)
        , MINIMUM_ASPECT_RATIO(0.0)
        , VISUAL_HORIZON_BUFFER(0.0)
        , MAXIMUM_GOAL_HORIZON_NORMAL_ANGLE(0.0)
        , MAXIMUM_ANGLE_BETWEEN_GOALS(0.0)
        , MAXIMUM_VERTICAL_GOAL_PERSPECTIVE_ANGLE(0.0)
        , MEASUREMENT_LIMITS_LEFT(10)
        , MEASUREMENT_LIMITS_RIGHT(10)
        , MEASUREMENT_LIMITS_TOP(10)
        , MEASUREMENT_LIMITS_BASE(10) {


        // Trigger the same function when either update
        on<Configuration, Trigger<CameraParameters>>("GoalDetector.yaml")
            .then([this](const Configuration& config, const CameraParameters& cam) {

                MINIMUM_POINTS_FOR_CONSENSUS   = config["ransac"]["minimum_points_for_consensus"].as<uint>();
                CONSENSUS_ERROR_THRESHOLD      = config["ransac"]["consensus_error_threshold"].as<double>();
                MAXIMUM_ITERATIONS_PER_FITTING = config["ransac"]["maximum_iterations_per_fitting"].as<uint>();
                MAXIMUM_FITTED_MODELS          = config["ransac"]["maximum_fitted_models"].as<uint>();

                MINIMUM_ASPECT_RATIO = config["aspect_ratio_range"][0].as<double>();
                MAXIMUM_ASPECT_RATIO = config["aspect_ratio_range"][1].as<double>();
                VISUAL_HORIZON_BUFFER =
                    std::max(1, int(cam.focalLengthPixels * tan(config["visual_horizon_buffer"].as<double>())));
                MAXIMUM_GOAL_HORIZON_NORMAL_ANGLE =
                    std::cos(config["minimum_goal_horizon_angle"].as<double>() - M_PI_2);
                MAXIMUM_ANGLE_BETWEEN_GOALS = std::cos(config["maximum_angle_between_goals"].as<double>());
                MAXIMUM_VERTICAL_GOAL_PERSPECTIVE_ANGLE =
                    std::sin(-config["maximum_vertical_goal_perspective_angle"].as<double>());

                MEASUREMENT_LIMITS_LEFT  = config["measurement_limits"]["left"].as<uint>();
                MEASUREMENT_LIMITS_RIGHT = config["measurement_limits"]["right"].as<uint>();
                MEASUREMENT_LIMITS_TOP   = config["measurement_limits"]["top"].as<uint>();
                MEASUREMENT_LIMITS_BASE  = config["measurement_limits"]["base"].as<uint>();
            });

        on<Trigger<ClassifiedImage>, With<CameraParameters>, With<LookUpTable>, Single>().then(
            "Goal Detector",
            [this](
                std::shared_ptr<const ClassifiedImage> rawImage, const CameraParameters& cam, const LookUpTable& lut) {

                const auto& image = *rawImage;
                // Our segments that may be a part of a goal
                std::vector<RansacGoalModel::GoalSegment> segments;
                auto goals = std::make_unique<std::vector<Goal>>();

                // Get our goal segments
                for (const auto& segment : image.horizontalSegments) {

                    // We throw out points if they are:
                    // Less the full quality (subsampled)
                    // Do not have a transition on the other side
                    if ((segment.segmentClass == SegmentClass::GOAL) && (segment.subsample == 1)
                        && (segment.previous > -1)
                        && (segment.next > -1)) {
                        segments.push_back({{double(segment.start[0]), double(segment.start[1])},
                                            {double(segment.end[0]), double(segment.end[1])}});
                    }
                }

                // Partition our segments so that they are split between above and below the horizon
                auto split = std::partition(
                    std::begin(segments), std::end(segments), [image](const RansacGoalModel::GoalSegment& segment) {
                        // Is the midpoint above or below the horizon?
                        utility::math::geometry::Line horizon(image.horizon.normal, image.horizon.distance);
                        return horizon.distanceToPoint(segment.left + segment.right / 2) > 0;
                    });

                // Make an array of our partitions
                std::array<std::vector<RansacGoalModel::GoalSegment>::iterator, RansacGoalModel::REQUIRED_POINTS + 1>
                    points = {segments.begin(), split, segments.end()};

                // Ransac for goals
                auto models = NPartiteRansac<RansacGoalModel>::fitModels(points,
                                                                         MINIMUM_POINTS_FOR_CONSENSUS,
                                                                         MAXIMUM_ITERATIONS_PER_FITTING,
                                                                         MAXIMUM_FITTED_MODELS,
                                                                         CONSENSUS_ERROR_THRESHOLD);

                // Look at our results
                for (auto& result : models) {

                    // Get our left, right and midlines
                    Line& left  = result.model.left;
                    Line& right = result.model.right;
                    Line mid;

                    // Normals in same direction
                    if (left.normal.dot(right.normal) > 0) {
                        mid.normal   = (right.normal + left.normal).normalized();
                        mid.distance = ((right.distance / right.normal.dot(mid.normal))
                                        + (left.distance / left.normal.dot(mid.normal)))
                                       * 0.5;
                    }
                    // Normals opposed
                    else {
                        mid.normal   = (right.normal - left.normal).normalized();
                        mid.distance = ((right.distance / right.normal.dot(mid.normal))
                                        - (left.distance / left.normal.dot(mid.normal)))
                                       * 0.5;
                    }

                    // Find a point that should work to start searching down
                    Eigen::Vector2d midpoint{0, 0};
                    int i = 0;
                    for (auto& m : result) {
                        midpoint += m.left;
                        midpoint += m.right;
                        i += 2;
                    }
                    midpoint /= i;

                    // Work out which direction to go
                    Eigen::Vector2d direction = mid.tangent();
                    direction *= direction[1] > 0 ? 1 : -1;
                    double theta = std::acos(direction[0]);
                    if (std::abs(theta) < M_PI_4) {
                        direction[0] = 1;
                        direction[1] = -std::tan(theta);
                    }
                    else {
                        direction[0] = std::tan(M_PI_2 - std::abs(theta));
                        direction[1] = 1;
                    }

                    // Classify until we reach green
                    Eigen::Vector2d basePoint({0, 0});
                    int notWhiteLen = 0;
                    for (Eigen::Vector2d point = mid.orthogonalProjection(midpoint);
                         (point[0] < image.dimensions[0]) && (point[0] > 0) && (point[1] < image.dimensions[1]);
                         point += direction) {

                        char c = static_cast<char>(utility::vision::getPixelColour(
                            lut,
                            utility::vision::getPixel(int(point[0]),
                                                      int(point[1]),
                                                      image.image->dimensions[0],
                                                      image.image->dimensions[1],
                                                      image.image->data,
                                                      static_cast<utility::vision::FOURCC>(image.image->format))));

                        if (c != 'y') {
                            ++notWhiteLen;
                            if (notWhiteLen > 4) {
                                basePoint = point;
                                break;
                            }
                        }
                        else if (c == 'g') {
                            basePoint = point;
                            break;
                        }
                        else if (c == 'y') {
                            notWhiteLen = 0;
                        }
                    }

                    running_stat<double> stat;

                    // Look through our segments to find endpoints
                    for (auto& point : result) {
                        // Project left and right onto midpoint keep top and bottom
                        stat(mid.tangentialDistanceToPoint(point.left));
                        stat(mid.tangentialDistanceToPoint(point.right));
                    }

                    // Get our endpoints from the min and max points on the line
                    Eigen::Vector2d midP1 = mid.pointFromTangentialDistance(stat.min());
                    Eigen::Vector2d midP2 = mid.orthogonalProjection(basePoint);

                    // Project those points outward onto the quad
                    Eigen::Vector2d p1 = midP1 - left.distanceToPoint(midP1) * left.normal.dot(mid.normal) * mid.normal;
                    Eigen::Vector2d p2 = midP2 - left.distanceToPoint(midP2) * left.normal.dot(mid.normal) * mid.normal;
                    Eigen::Vector2d p3 =
                        midP1 - right.distanceToPoint(midP1) * right.normal.dot(mid.normal) * mid.normal;
                    Eigen::Vector2d p4 =
                        midP2 - right.distanceToPoint(midP2) * right.normal.dot(mid.normal) * mid.normal;

                    // Make a quad
                    Goal goal;
                    goal.visObject.sensors = image.sensors;
                    goal.side              = Goal::Side::UNKNOWN_SIDE;

                    // Seperate tl and bl
                    Eigen::Vector2d tl = p1[1] > p2[1] ? p2 : p1;
                    Eigen::Vector2d bl = p1[1] > p2[1] ? p1 : p2;
                    Eigen::Vector2d tr = p3[1] > p4[1] ? p4 : p3;
                    Eigen::Vector2d br = p3[1] > p4[1] ? p3 : p4;

                    goal.quad.bl = bl;
                    goal.quad.tl = tl;
                    goal.quad.tr = tr;
                    goal.quad.br = br;

                    goals->push_back(std::move(goal));
                }

                utility::math::geometry::Line horizon(image.horizon.normal, image.horizon.distance);

                // Throwout invalid quads
                for (auto it = goals->begin(); it != goals->end();) {

                    utility::math::geometry::Quad quad(it->quad.bl, it->quad.tl, it->quad.tr, it->quad.br);

                    Eigen::Vector2d lhs = (quad.getTopLeft() - quad.getBottomLeft()).normalized();
                    Eigen::Vector2d rhs = (quad.getTopRight() - quad.getBottomRight()).normalized();

                    // Check if we are within the aspect ratio range
                    bool valid =
                        quad.aspectRatio() > MINIMUM_ASPECT_RATIO && quad.aspectRatio() < MAXIMUM_ASPECT_RATIO

                        // Check if we are close enough to the visual horizon
                        && (utility::vision::visualHorizonAtPoint(image, quad.getBottomLeft()[0])
                                < quad.getBottomLeft()[1] + VISUAL_HORIZON_BUFFER
                            || utility::vision::visualHorizonAtPoint(image, quad.getBottomRight()[0])
                                   < quad.getBottomRight()[1] + VISUAL_HORIZON_BUFFER)

                        // Check we finish above the kinematics horizon or or kinematics horizon is off the screen
                        && (horizon.y(quad.getTopLeft()[0]) > quad.getTopLeft()[1]
                            || horizon.y(quad.getTopLeft()[0]) < 0)
                        && (horizon.y(quad.getTopRight()[0]) > quad.getTopRight()[1]
                            || horizon.y(quad.getTopRight()[0]) < 0)

                        // Check that our two goal lines are perpendicular with the horizon must use greater than rather
                        // then less than because of the cos
                        && std::abs(rhs.dot(horizon.normal)) > MAXIMUM_GOAL_HORIZON_NORMAL_ANGLE
                        && std::abs(lhs.dot(horizon.normal)) > MAXIMUM_GOAL_HORIZON_NORMAL_ANGLE

                        // Check that our two goal lines are approximatly parallel
                        && std::abs(lhs.dot(rhs)) > MAXIMUM_ANGLE_BETWEEN_GOALS;

                    // Check that our goals don't form too much of an upward cup (not really possible for us)
                    //&& lhs.at(0) * rhs.at(1) - lhs.at(1) * rhs.at(0) > MAXIMUM_VERTICAL_GOAL_PERSPECTIVE_ANGLE;


                    if (!valid) {
                        it = goals->erase(it);
                    }
                    else {
                        ++it;
                    }
                }

                // Merge close goals
                for (auto a = goals->begin(); a != goals->end(); ++a) {
                    utility::math::geometry::Quad aquad(a->quad.bl, a->quad.tl, a->quad.tr, a->quad.br);

                    for (auto b = std::next(a); b != goals->end();) {

                        utility::math::geometry::Quad bquad(b->quad.bl, b->quad.tl, b->quad.tr, b->quad.br);


                        if (aquad.overlapsHorizontally(bquad)) {
                            // Get outer lines.
                            Eigen::Vector2d tl, tr, bl, br;

                            tl = {std::min(aquad.getTopLeft()[0], bquad.getTopLeft()[0]),
                                  std::min(aquad.getTopLeft()[1], bquad.getTopLeft()[1])};
                            tr = {std::max(aquad.getTopRight()[0], bquad.getTopRight()[0]),
                                  std::min(aquad.getTopRight()[1], bquad.getTopRight()[1])};
                            bl = {std::min(aquad.getBottomLeft()[0], bquad.getBottomLeft()[0]),
                                  std::max(aquad.getBottomLeft()[1], bquad.getBottomLeft()[1])};
                            br = {std::max(aquad.getBottomRight()[0], bquad.getBottomRight()[0]),
                                  std::max(aquad.getBottomRight()[1], bquad.getBottomRight()[1])};

                            // Replace original two quads with the new one.
                            aquad.set(bl, tl, tr, br);
                            a->quad.bl = bl;
                            a->quad.tl = tl;
                            a->quad.tr = tr;
                            a->quad.br = br;
                            b          = goals->erase(b);
                        }
                        else {
                            b++;
                        }
                    }
                }

                // Store our measurements
                for (auto it = goals->begin(); it != goals->end(); ++it) {
                    utility::math::geometry::Quad quad(it->quad.bl, it->quad.tl, it->quad.tr, it->quad.br);

                    // Get the quad points in screen coords
                    Eigen::Vector2d tl               = imageToScreen(quad.getTopLeft(), image.dimensions);
                    Eigen::Vector2d tr               = imageToScreen(quad.getTopRight(), image.dimensions);
                    Eigen::Vector2d bl               = imageToScreen(quad.getBottomLeft(), image.dimensions);
                    Eigen::Vector2d br               = imageToScreen(quad.getBottomRight(), image.dimensions);
                    Eigen::Vector2d screenGoalCentre = (tl + tr + bl + br) * 0.25;

                    // Get vectors for TL TR BL BR;
                    Eigen::Vector3d ctl = getCamFromScreen(tl, cam.focalLengthPixels);
                    Eigen::Vector3d ctr = getCamFromScreen(tr, cam.focalLengthPixels);
                    Eigen::Vector3d cbl = getCamFromScreen(bl, cam.focalLengthPixels);
                    Eigen::Vector3d cbr = getCamFromScreen(br, cam.focalLengthPixels);

                    // Get our four normals for each edge
                    // BL TL cross product gives left side
                    Eigen::Vector3d left = cbl.cross(ctl).normalized();
                    it->measurement.push_back(Goal::Measurement(Goal::MeasurementType::LEFT_NORMAL, left));

                    // TR BL cross product gives right side
                    Eigen::Vector3d right = ctr.cross(cbl).normalized();
                    it->measurement.push_back(Goal::Measurement(Goal::MeasurementType::RIGHT_NORMAL, right));

                    // Check that the points are not too close to the edges of the screen
                    if (std::min(cbr[0], cbl[0]) > MEASUREMENT_LIMITS_LEFT
                        && std::min(cbr[1], cbl[1]) > MEASUREMENT_LIMITS_TOP
                        && cam.imageSizePixels[0] - std::max(cbr[0], cbl[0]) < MEASUREMENT_LIMITS_TOP
                        && cam.imageSizePixels[1] - std::max(cbr[1], cbl[1]) < MEASUREMENT_LIMITS_BASE) {

                        // BR BL cross product gives the bottom side
                        Eigen::Vector3d bottom = cbr.cross(cbl).normalized();
                        it->measurement.push_back(Goal::Measurement(Goal::MeasurementType::BASE_NORMAL, bottom));
                    }

                    // Check that the points are not too close to the edges of the screen
                    if (std::min(ctr[0], ctl[0]) > MEASUREMENT_LIMITS_LEFT
                        && std::min(ctr[1], ctl[1]) > MEASUREMENT_LIMITS_TOP
                        && cam.imageSizePixels[0] - std::max(ctr[0], ctl[0]) < MEASUREMENT_LIMITS_TOP
                        && cam.imageSizePixels[1] - std::max(ctr[1], ctl[1]) < MEASUREMENT_LIMITS_BASE) {

                        // TL TR cross product gives the top side
                        Eigen::Vector3d top = ctl.cross(ctr).normalized();
                        it->measurement.push_back(Goal::Measurement(Goal::MeasurementType::TOP_NORMAL, top));
                    }

                    // Angular positions from the camera
                    Eigen::Vector2d pixelsToTanThetaFactor = cam.pixelsToTanThetaFactor;
                    it->visObject.screenAngular =
                        Eigen::atan(pixelsToTanThetaFactor.cwiseProduct(screenGoalCentre).array()).matrix();
                    Eigen::Vector2d brAngular = Eigen::atan(pixelsToTanThetaFactor.cwiseProduct(br).array()).matrix();
                    Eigen::Vector2d trAngular = Eigen::atan(pixelsToTanThetaFactor.cwiseProduct(tr).array()).matrix();
                    Eigen::Vector2d blAngular = Eigen::atan(pixelsToTanThetaFactor.cwiseProduct(bl).array()).matrix();
                    Eigen::Vector2d tlAngular = Eigen::atan(pixelsToTanThetaFactor.cwiseProduct(tl).array()).matrix();
                    Quad angularQuad(blAngular, tlAngular, trAngular, brAngular);
                    it->visObject.angularSize = angularQuad.getSize();
                }


                // Assign leftness and rightness to goals
                if (goals->size() == 2) {
                    utility::math::geometry::Quad quad0(
                        goals->at(0).quad.bl, goals->at(0).quad.tl, goals->at(0).quad.tr, goals->at(0).quad.br);

                    utility::math::geometry::Quad quad1(
                        goals->at(1).quad.bl, goals->at(1).quad.tl, goals->at(1).quad.tr, goals->at(1).quad.br);

                    if (quad0.getCentre()(0) < quad1.getCentre()(0)) {
                        goals->at(0).side = Goal::Side::LEFT;
                        goals->at(1).side = Goal::Side::RIGHT;
                    }
                    else {
                        goals->at(0).side = Goal::Side::RIGHT;
                        goals->at(1).side = Goal::Side::LEFT;
                    }
                }

                emit(std::move(goals));

            });
    }
}  // namespace vision
}  // namespace module
