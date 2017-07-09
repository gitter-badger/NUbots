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

#ifndef NUHELPERS_H
#define NUHELPERS_H

#include <Eigen/Core>
#include <nuclear>

#include "message/support/nubugger/DataPoint.h"
#include "message/support/nubugger/DrawObjects.h"
#include "message/vision/VisionObjects.h"
#include "utility/math/geometry/Circle.h"
#include "utility/math/geometry/RotatedRectangle.h"
#include "utility/math/matrix/Rotation2D.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/matrix/Transform3D.h"

namespace utility {
namespace nubugger {
    using utility::math::geometry::RotatedRectangle;

    using utility::math::geometry::Circle;
    using utility::math::matrix::Rotation2D;
    using utility::math::matrix::Rotation3D;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;

    using message::support::nubugger::DataPoint;
    using message::support::nubugger::DrawObject;
    using message::support::nubugger::DrawObjects;

    namespace {

        constexpr float TIMEOUT = 2.5;

        template <typename T>
        struct is_iterable {
        private:
            typedef std::true_type yes;
            typedef std::false_type no;

            template <typename U>
            static auto test_begin(int) -> decltype(std::declval<U>().begin(), yes());
            template <typename>
            static no test_begin(...);

            template <typename U>
            static auto test_end(int) -> decltype(std::declval<U>().end(), yes());
            template <typename>
            static no test_end(...);

        public:
            static constexpr bool value = std::is_same<decltype(test_begin<T>(0)), yes>::value
                                          && std::is_same<decltype(test_end<T>(0)), yes>::value;
        };

        inline void buildGraph(DataPoint&) {}

        template <typename First, typename... Remainder>
        typename std::enable_if<!is_iterable<First>::value>::type buildGraph(DataPoint& dataPoint,
                                                                             First first,
                                                                             Remainder... remainder) {
            dataPoint.value.push_back(first);
            buildGraph(dataPoint, remainder...);
        }

        template <typename First, typename... Remainder>
        typename std::enable_if<is_iterable<First>::value>::type buildGraph(DataPoint& dataPoint,
                                                                            First first,
                                                                            Remainder... remainder) {
            for (const auto& value : first) {
                dataPoint.value.push_back(value);
            }
            buildGraph(dataPoint, remainder...);
        }
    }

    template <typename... Values>
    inline std::unique_ptr<message::support::nubugger::DataPoint> graph(std::string label, Values... values) {
        auto dataPoint   = std::make_unique<DataPoint>();
        dataPoint->label = label;
        dataPoint->type  = DataPoint::Type::Value::FLOAT_LIST;
        buildGraph(*dataPoint, values...);
        return dataPoint;
    }

    template <typename T, std::enable_if_t<((T::RowsAtCompileTime == 2) && (T::ColsAtCompileTime == 2))>* = nullptr>
    inline std::unique_ptr<message::support::nubugger::DataPoint> graph(std::string label, Rotation2D rotation) {
        auto dataPoint   = std::make_unique<DataPoint>();
        dataPoint->label = label;
        dataPoint->type  = DataPoint::Type::Value::ROTATION_2D;
        dataPoint->value.resize(rotation.size());
        Eigen::Matrix2f::Map(dataPoint->value.data(), rotation.rows(), rotation.cols()) = rotation.cast<float>();
        return dataPoint;
    }

    template <typename T, std::enable_if_t<((T::RowsAtCompileTime == 3) && (T::ColsAtCompileTime == 3))>* = nullptr>
    inline std::unique_ptr<message::support::nubugger::DataPoint> graph(std::string label, Rotation3D rotation) {
        auto dataPoint   = std::make_unique<DataPoint>();
        dataPoint->label = label;
        dataPoint->type  = DataPoint::Type::Value::ROTATION_3D;
        dataPoint->value.resize(rotation.size());
        Eigen::Matrix3f::Map(dataPoint->value.data(), rotation.rows(), rotation.cols()) = rotation.cast<float>();
        return dataPoint;
    }

    template <typename T, std::enable_if_t<((T::RowsAtCompileTime == 3) && (T::ColsAtCompileTime == 1))>* = nullptr>
    inline std::unique_ptr<message::support::nubugger::DataPoint> graph(std::string label, Transform2D transform) {
        auto dataPoint   = std::make_unique<DataPoint>();
        dataPoint->label = label;
        dataPoint->type  = DataPoint::Type::Value::TRANSFORM_2D;
        dataPoint->value.resize(transform.size());
        Eigen::Vector3f::Map(dataPoint->value.data(), transform.size()) = transform.cast<float>();
        return dataPoint;
    }

    template <typename T, std::enable_if_t<((T::RowsAtCompileTime == 4) && (T::ColsAtCompileTime == 4))>* = nullptr>
    inline std::unique_ptr<message::support::nubugger::DataPoint> graph(std::string label, Transform3D transform) {
        auto dataPoint   = std::make_unique<DataPoint>();
        dataPoint->label = label;
        dataPoint->type  = DataPoint::Type::Value::TRANSFORM_3D;
        dataPoint->value.resize(transform.size());
        Eigen::Matrix4f::Map(dataPoint->value.data(), transform.rows(), transform.cols()) = transform.cast<float>();
        return dataPoint;
    }

    template <typename T, std::enable_if_t<((T::RowsAtCompileTime == 2) && (T::ColsAtCompileTime == 2))>* = nullptr>
    inline std::unique_ptr<message::support::nubugger::DataPoint> graph(std::string label, T rotation) {
        return graph(label, static_cast<Rotation2D>(rotation.template cast<double>()));
    }

    template <typename T, std::enable_if_t<((T::RowsAtCompileTime == 3) && (T::ColsAtCompileTime == 3))>* = nullptr>
    inline std::unique_ptr<message::support::nubugger::DataPoint> graph(std::string label, T rotation) {
        return graph(label, static_cast<Rotation3D>(rotation.template cast<double>()));
    }

    template <typename T, std::enable_if_t<((T::RowsAtCompileTime == 3) && (T::ColsAtCompileTime == 1))>* = nullptr>
    inline std::unique_ptr<message::support::nubugger::DataPoint> graph(std::string label, T transform) {
        return graph(label, static_cast<Transform2D>(transform.template cast<double>()));
    }

    template <typename T, std::enable_if_t<((T::RowsAtCompileTime == 4) && (T::ColsAtCompileTime == 4))>* = nullptr>
    inline std::unique_ptr<message::support::nubugger::DataPoint> graph(std::string label, T transform) {
        return graph(label, static_cast<Transform3D>(transform.template cast<double>()));
    }

    template <typename T,
              std::enable_if_t<(((T::RowsAtCompileTime != 4) && (T::ColsAtCompileTime != 4))
                                && ((T::RowsAtCompileTime != 3) && (T::ColsAtCompileTime != 1))
                                && ((T::RowsAtCompileTime != 3) && (T::ColsAtCompileTime != 3))
                                && ((T::RowsAtCompileTime != 2) && (T::ColsAtCompileTime != 2)))>* = nullptr>
    inline std::unique_ptr<message::support::nubugger::DataPoint> graph(std::string label, T matrix) {
        auto dataPoint   = std::make_unique<DataPoint>();
        dataPoint->label = label;
        dataPoint->type  = DataPoint::Type::Value::MATRIX;
        dataPoint->value.resize(matrix.size());
        Eigen::Matrix<float, T::RowsAtCompileTime, T::ColsAtCompileTime>::Map(
            dataPoint->value.data(), matrix.rows(), matrix.cols()) = matrix.template cast<float>();
        return dataPoint;
    }

    inline std::unique_ptr<DrawObjects> drawArrow(std::string name,
                                                  Eigen::Vector3d position,
                                                  float length,
                                                  Eigen::Vector3d direction,
                                                  float timeout = TIMEOUT) {

        DrawObject object;
        object.name      = name;
        object.shape     = DrawObject::Shape::ARROW;
        object.timeout   = timeout;
        object.position  = position;
        object.direction = direction;
        object.length    = length;

        auto drawObjects = std::make_unique<DrawObjects>();
        drawObjects->objects.push_back(object);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawArrow(std::string name,
                                                  Eigen::Vector3d position,
                                                  Eigen::Vector3d target,
                                                  float timeout = TIMEOUT) {

        DrawObject object;
        object.name     = name;
        object.shape    = DrawObject::Shape::ARROW;
        object.timeout  = timeout;
        object.position = position;
        object.target   = target;

        auto drawObjects = std::make_unique<DrawObjects>();
        drawObjects->objects.push_back(object);

        return std::move(drawObjects);
    }


    inline std::unique_ptr<DrawObjects> drawArrow(std::string name,
                                                  Eigen::Vector2d position,
                                                  Eigen::Vector2d target,
                                                  float timeout = TIMEOUT) {

        DrawObject object;
        object.name     = name;
        object.shape    = DrawObject::Shape::ARROW;
        object.timeout  = timeout;
        object.position = Eigen::Vector3d(position(0), position(1), 0);
        object.target   = Eigen::Vector3d(target(0), target(1), 0);

        auto drawObjects = std::make_unique<DrawObjects>();
        drawObjects->objects.push_back(object);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawArrow(std::string name,
                                                  Transform2D position,
                                                  Eigen::Vector3d colour,
                                                  float length,
                                                  float timeout = TIMEOUT) {

        DrawObject object;
        object.name      = name;
        object.shape     = DrawObject::Shape::ARROW;
        object.timeout   = timeout;
        object.position  = Eigen::Vector3d(position.x(), position.y(), 0);
        object.direction = Eigen::Vector3d(std::cos(position.angle()), std::sin(position.angle()), 0);
        object.colour    = colour;
        object.length    = length;

        auto drawObjects = std::make_unique<DrawObjects>();
        drawObjects->objects.push_back(object);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawBox(std::string name,
                                                Eigen::Vector3d position,
                                                float width,
                                                float height,
                                                float depth,
                                                float timeout = TIMEOUT) {

        DrawObject object;
        object.name     = name;
        object.shape    = DrawObject::Shape::BOX;
        object.timeout  = timeout;
        object.position = position;
        object.width    = width;
        object.height   = height;
        object.depth    = depth;

        auto drawObjects = std::make_unique<DrawObjects>();
        drawObjects->objects.push_back(object);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawCircle(std::string name,
                                                   Eigen::Vector3d position,
                                                   Eigen::Vector3d rotation,
                                                   float width,
                                                   float height,
                                                   float timeout = TIMEOUT) {

        DrawObject object;
        object.name     = name;
        object.shape    = DrawObject::Shape::CIRCLE;
        object.timeout  = timeout;
        object.position = position;
        object.rotation = rotation;
        object.width    = width;
        object.height   = height;

        auto drawObjects = std::make_unique<DrawObjects>();
        drawObjects->objects.push_back(object);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawCircle(std::string name,
                                                   Circle circle,
                                                   float z                = 0,
                                                   Eigen::Vector3d colour = {1, 1, 0},
                                                   float timeout          = TIMEOUT) {

        DrawObject object;
        object.name     = name;
        object.shape    = DrawObject::Shape::CIRCLE;
        object.timeout  = timeout;
        object.position = Eigen::Vector3d(circle.centre(0), circle.centre(1), z);
        object.rotation = Eigen::Vector3d(0.0, 0.0, 0.0);
        object.colour   = colour;
        object.width    = circle.radius * 2.0f;
        object.height   = circle.radius * 2.0f;

        auto drawObjects = std::make_unique<DrawObjects>();
        drawObjects->objects.push_back(object);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawCylinder(std::string name,
                                                     Eigen::Vector3d position,
                                                     Eigen::Vector3d rotation,
                                                     float topRadius,
                                                     float bottomRadius,
                                                     float height,
                                                     float timeout = TIMEOUT) {

        DrawObject object;
        object.name          = name;
        object.shape         = DrawObject::Shape::CYLINDER;
        object.timeout       = timeout;
        object.position      = position;
        object.rotation      = rotation;
        object.height        = height;
        object.top_radius    = topRadius;
        object.bottom_radius = bottomRadius;

        auto drawObjects = std::make_unique<DrawObjects>();
        drawObjects->objects.push_back(object);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawPyramid(std::string name,
                                                    Eigen::Vector3d position,
                                                    Eigen::Vector3d rotation,
                                                    float height,
                                                    float faces,
                                                    float timeout = TIMEOUT) {

        DrawObject object;
        object.name     = name;
        object.shape    = DrawObject::Shape::PYRAMID;
        object.timeout  = timeout;
        object.position = position;
        object.rotation = rotation;
        object.height   = height;
        object.faces    = faces;

        auto drawObjects = std::make_unique<DrawObjects>();
        drawObjects->objects.push_back(object);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawRectangle(std::string name,
                                                      Eigen::Vector3d position,
                                                      float height,
                                                      float length,
                                                      float timeout = TIMEOUT) {

        DrawObject object;
        object.name     = name;
        object.shape    = DrawObject::Shape::RECTANGLE;
        object.timeout  = timeout;
        object.position = position;
        object.height   = height;
        object.length   = length;

        auto drawObjects = std::make_unique<DrawObjects>();
        drawObjects->objects.push_back(object);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawRectangle(std::string name,
                                                      RotatedRectangle rect,
                                                      float z       = 0.08,
                                                      float timeout = TIMEOUT) {

        DrawObject object;
        object.name     = name;
        object.shape    = DrawObject::Shape::RECTANGLE;
        object.timeout  = timeout;
        object.position = Eigen::Vector3d(rect.getPosition()(0), rect.getPosition()(1), z);
        object.rotation = Eigen::Vector3d(0.0, 0.0, rect.getRotation());
        object.length   = rect.getSize()(0);
        object.width    = rect.getSize()(1);

        auto drawObjects = std::make_unique<DrawObjects>();
        drawObjects->objects.push_back(object);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawRectangle(std::string name,
                                                      RotatedRectangle rect,
                                                      Eigen::Vector3d colour,
                                                      float z       = 0.08,
                                                      float timeout = TIMEOUT) {

        auto drawObjects               = drawRectangle(name, rect, z, timeout);
        drawObjects->objects[0].colour = colour;

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawSphere(std::string name,
                                                   Eigen::Vector3d position,
                                                   float radius,
                                                   float timeout = TIMEOUT) {

        DrawObject object;
        object.name     = name;
        object.shape    = DrawObject::Shape::SPHERE;
        object.timeout  = timeout;
        object.position = position;
        object.radius   = radius;

        auto drawObjects = std::make_unique<DrawObjects>();
        drawObjects->objects.push_back(object);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawSphere(std::string name,
                                                   Eigen::Vector3d position,
                                                   float radius,
                                                   Eigen::Vector3d colour,
                                                   float timeout = TIMEOUT) {

        auto drawObjects               = drawSphere(name, position, radius, timeout);
        drawObjects->objects[0].colour = colour;

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawTree(std::string name,
                                                 std::vector<Eigen::Vector2d> positions,
                                                 std::vector<uint> parentIndices,
                                                 float line_width,
                                                 Eigen::Vector3d colour,
                                                 float timeout = TIMEOUT) {

        DrawObject object;
        object.name    = name;
        object.shape   = DrawObject::Shape::POLYLINE;
        object.timeout = timeout;
        object.width   = line_width;
        object.colour  = colour;

        for (uint i = 0; i < positions.size(); i++) {

            DrawObject::Path node;
            node.position     = positions[i];
            node.parent_index = parentIndices[i];
            object.path.push_back(node);
        }

        auto drawObjects = std::make_unique<DrawObjects>();
        drawObjects->objects.push_back(object);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawPolyline(std::string name,
                                                     std::vector<Eigen::Vector2d> positions,
                                                     float line_width,
                                                     Eigen::Vector3d colour,
                                                     float timeout = TIMEOUT) {

        std::vector<uint> parentIndices;
        parentIndices.reserve(positions.size());

        for (uint i = 0; i < positions.size(); i++) {
            parentIndices.push_back(std::max(0, int(i) - 1));
        }

        return drawTree(name, positions, parentIndices, line_width, colour, timeout);
    }

    inline std::unique_ptr<DrawObjects> drawPath(std::string name,
                                                 std::vector<Transform2D> states,
                                                 float line_width,
                                                 Eigen::Vector3d colour,
                                                 float timeout = TIMEOUT) {

        std::vector<Eigen::Vector2d> positions;

        for (auto state : states) {
            positions.push_back(state.xy());
        }

        return utility::nubugger::drawPolyline(name, positions, line_width, colour, timeout);
    }

    inline std::unique_ptr<std::vector<message::vision::Line>> drawVisionLines(
        std::vector<std::tuple<Eigen::Vector2i, Eigen::Vector2i, Eigen::Vector4d>,
                    Eigen::aligned_allocator<std::tuple<Eigen::Vector2i, Eigen::Vector2i, Eigen::Vector4d>>> lines) {

        auto msg = std::make_unique<std::vector<message::vision::Line>>();

        for (const auto& line : lines) {
            message::vision::Line objLine;
            objLine.visObject.camera_id = 0;  // TODO
            objLine.start               = std::get<0>(line);
            objLine.end                 = std::get<1>(line);
            objLine.colour              = std::get<2>(line);
            msg->push_back(objLine);
        }

        return std::move(msg);
    }

    inline std::unique_ptr<std::vector<message::vision::Line>> drawVisionLines(
        std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>> lines,
        Eigen::Vector4d colour = Eigen::Vector4d({1, 1, 1, 1})) {

        std::vector<std::tuple<Eigen::Vector2i, Eigen::Vector2i, Eigen::Vector4d>,
                    Eigen::aligned_allocator<std::tuple<Eigen::Vector2i, Eigen::Vector2i, Eigen::Vector4d>>>
            colouredLines;
        colouredLines.reserve(lines.size());

        for (auto const line : lines) {
            colouredLines.push_back(std::make_tuple(line.first, line.second, colour));
        }

        return drawVisionLines(colouredLines);
    }

    inline std::unique_ptr<std::vector<message::vision::Line>>
    drawVisionLine(Eigen::Vector2i start, Eigen::Vector2i end, Eigen::Vector4d colour = Eigen::Vector4d({1, 1, 1, 1})) {

        return drawVisionLines({std::make_tuple(start, end, colour)});
    }
}
}

#endif
