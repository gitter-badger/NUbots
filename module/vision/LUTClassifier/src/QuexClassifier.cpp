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

#include "QuexClassifier.h"

#include <Eigen/Core>

#include "utility/vision/fourcc.h"
#include "utility/vision/LookUpTable.h"
#include "utility/vision/Vision.h"

namespace module {
    namespace vision {
        using message::input::Image;
        using message::vision::LookUpTable;
        using message::vision::ClassifiedImage;
        using SegmentClass = message::vision::ClassifiedImage::SegmentClass::Value;
        using quex::Token;
        using FOURCC = utility::vision::FOURCC;

        QuexClassifier::QuexClassifier()
            : lexer(buffer, BUFFER_SIZE, buffer + 1)
            , tknNumber(lexer.token_p()->number) {
        }

        std::vector<ClassifiedImage::Segment> QuexClassifier::classify(const Image& image, const LookUpTable& lut, const Eigen::Vector2i& start, const Eigen::Vector2i& end, const uint& subsample) {

            // Start reading data
            lexer.buffer_fill_region_prepare();

            // For vertical runs
            if(start[0] == end[0]) {

                size_t length = end[1] - start[1] + 1;

                for(uint i = 0; i < length / subsample; ++i) {
                    buffer[i + 1] = utility::vision::getPixelColour(lut,
                        utility::vision::getPixel(start[0], start[1] + (i * subsample), image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)));
                }

                lexer.buffer_fill_region_finish(length / subsample);
            }

            // For horizontal runs
            else if(start[1] == end[1]) {

                size_t length = end[0] - start[0] + 1;

                for(uint i = 0; i < length / subsample; ++i) {
                    buffer[i + 1] = utility::vision::getPixelColour(lut,
                        utility::vision::getPixel(start[0] + (i * subsample), start[1], image.dimensions[0], image.dimensions[1], image.data, static_cast<FOURCC>(image.format)));
                }

                lexer.buffer_fill_region_finish(length / subsample);
            }

            // Diagonal run
            else {
                // TODO not implemented (and probably won't implement)
            }

            // Our output
            std::vector<ClassifiedImage::Segment> output;
            output.reserve(64);

            // Our vector of position
            Eigen::Vector2i position = start;

            // A reference to the relevant movement direction
            int& movement = start[1] == end[1] ? position[0] : position[1];

            for(uint32_t typeID = lexer.receive(); typeID != QUEX_TKN_TERMINATION; typeID = lexer.receive()) {

                // Update our position
                Eigen::Vector2i s = position;
                uint len = tknNumber * subsample;
                movement += len;
                Eigen::Vector2i m = (s + position) / 2;

                switch(typeID) {
                    case QUEX_TKN_FIELD:
                        output.push_back(ClassifiedImage::Segment(SegmentClass::FIELD, len, subsample, s, position, m, -1, -1));
                        break;

                    case QUEX_TKN_BALL:
                        output.push_back(ClassifiedImage::Segment(SegmentClass::BALL, len, subsample, s, position, m, -1, -1));
                        break;

                    case QUEX_TKN_GOAL:
                        output.push_back(ClassifiedImage::Segment(SegmentClass::GOAL, len, subsample, s, position, m, -1, -1));
                        break;

                    case QUEX_TKN_LINE:
                        output.push_back(ClassifiedImage::Segment(SegmentClass::LINE, len, subsample, s, position, m, -1, -1));
                        break;

                    case QUEX_TKN_CYAN_TEAM:
                        output.push_back(ClassifiedImage::Segment(SegmentClass::CYAN_TEAM, len, subsample, s, position, m, -1, -1));
                        break;

                    case QUEX_TKN_MAGENTA_TEAM:
                        output.push_back(ClassifiedImage::Segment(SegmentClass::MAGENTA_TEAM, len, subsample, s, position, m, -1, -1));
                        break;

                    case QUEX_TKN_UNCLASSIFIED:
                        output.push_back(ClassifiedImage::Segment(SegmentClass::UNKNOWN_CLASS, len, subsample, s, position, m, -1, -1));
                        break;
                }
            }

            return output;

        }
    }
}
