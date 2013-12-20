#include "BallDetector.h"

namespace modules {
    namespace vision {
        
        using messages::vision::ColourSegment;

        BallDetector::BallDetector() {
            // Empty constructor. 
        }

        BallDetector::~BallDetector() {
            // Empty destructor. 
        }

        void BallDetector::setParameters(int BALL_EDGE_THRESHOLD_, int BALL_ORANGE_TOLERANCE_, float BALL_MIN_PERCENT_ORANGE_) {
            BALL_MIN_PERCENT_ORANGE = BALL_EDGE_THRESHOLD_;
            BALL_ORANGE_TOLERANCE = BALL_ORANGE_TOLERANCE_;
            BALL_EDGE_THRESHOLD = BALL_MIN_PERCENT_ORANGE_;
        }

        // BROKEN
        // COLOUR_CLASS = BALL_COLOUR
        std::vector<Ball> BallDetector::run(const std::vector<ColourSegment>& horizontalMatchedSegments, 
                                            const std::vector<ColourSegment>& verticalMatchedSegments, 
                                            const std::vector<arma::vec2>& greenHorizonInterpolatedPoints) {
            // TODO: Add original image and LUT information.
            //NUImage img = vbb->getOriginalImage();
            //const LookUpTable& lut = vbb->getLUT();

            // BEGIN BALL DETECTION -----------------------------------------------------------------

            std::list<arma::vec2> edges;
            std::vector<Ball> balls; //will only ever hold one

            appendEdgesFromSegments(horizontalMatchedSegments, edges, greenHorizonInterpolatedPoints);
            appendEdgesFromSegments(verticalMatchedSegments, edges, greenHorizonInterpolatedPoints);

            int height = 10; //img.getHeight();
            int width = 10; //img.getWidth();

            // Arithmetic mean
            arma::vec2 avg, stddev;
            arma::running_stat_vec<double> acc;

            for(const arma::vec2& point : edges) {
                acc(point);
            }

            avg = acc.mean();
            stddev = acc.stddev();

            // Statistical throw-out
            for (std::list<arma::vec2>::iterator it = edges.begin(); it != edges.end(); /* Iteration done by for loop */) {
                if ((std::abs((*it)[0] - avg[0]) > stddev[0]) || (std::abs((*it)[1] - avg[1]) > stddev[1])) {
                    it = edges.erase(it);
                }

                else {
                    it++;
                }
            }

            if (!edges.empty()) {
                // Geometric mean
                arma::Col<long double> pos;
                pos << 1.0 << 1.0;
                long double root_order = 1.0 / edges.size();

                for(const arma::vec2& p : edges) {
                    pos[0] *= std::pow(p[0], root_order);
                    pos[1] *= std::pow(p[1], root_order);
                }

                pos[0] = std::min(pos[0], width - 1.0L);
                pos[1] = std::min(pos[1], height - 1.0L);

                // Find ball centre (not occluded)
                int top = pos[1];
                int bottom = pos[1];
                int left = pos[0];
                int right = pos[0];
                int not_orange_count = 0;

                // FIND BALL CENTRE (single iteration approach; doesn't deal great with occlusion)
                for (top = pos[1]; ((top > 0) && (not_orange_count <= BALL_ORANGE_TOLERANCE)); top--) {
                    if (getColourFromIndex(lut.classifyPixel(img((int)pos[0], top))) != orange) {
                        not_orange_count++;
                    }

                    else {
                        not_orange_count = 0;
                    }
                }

                top += not_orange_count;

                not_orange_count = 0;

                for (bottom = pos[1]; ((bottom < height) && (not_orange_count <= BALL_ORANGE_TOLERANCE)); bottom++) {
                    if (getColourFromIndex(lut.classifyPixel(img((int)pos[0], bottom))) != orange) {
                        not_orange_count++;
                    }

                    else {
                        not_orange_count = 0;
                    }
                }

                bottom -= not_orange_count;

                not_orange_count = 0;

                for (left = pos[0]; ((left > 0) && (not_orange_count <= BALL_ORANGE_TOLERANCE)); left--) {
                    if (getColourFromIndex(lut.classifyPixel(img(left, (int)pos[1]))) != orange) {
                        not_orange_count++;
                    }

                    else {
                        not_orange_count = 0;
                    }
                }

                left += not_orange_count;

                not_orange_count = 0;

                for (right = pos[0]; ((right < width) && (not_orange_count <= BALL_ORANGE_TOLERANCE)); right++) {
                    if (getColourFromIndex(lut.classifyPixel(img(right, (int)pos[1]))) != orange) {
                        not_orange_count++;
                    }

                    else {
                        not_orange_count = 0;
                    }
                }

                right -= not_orange_count;

                // CHECK IF POINT IS ON EDGE OF BALL (OR OCCLUDED)
                // OCCLUSION CHECK / COMPENSATION
                bool top_edge = false;
                bool bottom_edge = false;
                bool left_edge = false;
                bool right_edge = false;

                for (int i = left; ((i > (left - BALL_EDGE_THRESHOLD)) && (i >= 0)); i--) {
                    if (getColourFromIndex(lut.classifyPixel(img(i, (int)pos[1]))) == green) {
                        left_edge = true;
                        break;
                    }
                }

                for (int i = right; ((i < (right + BALL_EDGE_THRESHOLD)) && (i < width)); i++) {
                    if (getColourFromIndex(lut.classifyPixel(img(i, (int)pos[1]))) == green) {
                        right_edge = true;
                        break;
                    }
                }
                
                for (int i = bottom; ((i < (bottom + BALL_EDGE_THRESHOLD)) && (i < height)); i++) {
                    if (getColourFromIndex(lut.classifyPixel(img((int)pos[0], i))) == green) {
                        bottom_edge = true;
                        break;
                    }
                }

                for (int i = top; ((i > (top - BALL_EDGE_THRESHOLD)) && (i >= 0)); i--) {
                    if (getColourFromIndex(lut.classifyPixel(img((int)pos[0], i))) == green) {
                        top_edge = true;
                        break;
                    }
                }

                top_edge = true;

                // DETERMINE CENTRE
                arma::vec2 center;

                // only bottom occluded
                if (left_edge && right_edge && top_edge && !bottom_edge) {
                    center << ((right + left) / 2) << std::min((top + ((top + right - left)) / 2), height - 1);
                }

                // only top occluded
                else if (left_edge && right_edge && !top_edge && bottom_edge) {
                    center << ((right + left) / 2) << std::max((bottom + ((bottom - right + left)) / 2), 0);
                }

                // only right occluded
                else if (left_edge && !right_edge && top_edge && bottom_edge) {
                    center << std::min((left + ((left + bottom - top)) / 2), width - 1) << ((top + bottom) / 2);
                }

                // only left occluded
                else if (!left_edge && right_edge && top_edge && bottom_edge) {
                    center << std::max((right + ((right - bottom + top)) / 2), 0) << ((top + bottom) / 2);
                }

                else {
                    center << ((right + left) / 2) << ((top + bottom) / 2);
                }

                // CHECK FOR SUCCESS
                if (((center[0] != 1) && (center[1] != 1)) && (bottom > top) && (right > left)) {
                    // expensive check - only do if we want to
                    if (BALL_MIN_PERCENT_ORANGE > 0) {
                        // CHECK FOR PIXEL DENSITY
                        int count = 0;

                        double min_dimension = std::min(right - left, bottom - top);

                        int box_left = std::max(center[0] - min_dimension / 2, 0.0);
                        int box_right = std::min(center[0] + min_dimension / 2, width - 1.0);
                        int box_top = std::max(center[1] - min_dimension / 2, 0.0);
                        int box_bottom = std::min(center[1] + min_dimension / 2, height - 1.0);

                        for (int i = box_left; i < box_right; i++) {
                            for (int j = box_top; j < box_bottom; j++) {
                                if (getColourFromIndex(lut.classifyPixel(img(i, j))) == orange)
                                    count++;
                            }
                        }

                        if ((count / (min_dimension * min_dimension)) >= BALL_MIN_PERCENT_ORANGE) {
                            balls.push_back(Ball(center, std::max((right - left), (bottom - top))));
                        }

                        else {
                            NUClear::log<NUClear::DEBUG>("BallDetector::detectBall - ball thrown out on percentage contained orange");
                        }
                    }

                    else {
                        balls.push_back(Ball(center, std::max((right - left), (bottom - top))));
                    }
                }
                else {
                    NUClear::log<NUClear::DEBUG>("BallDetector::detectBall - (1, 1) ball thrown out");
                }
            }

            return balls;
        }

        void BallDetector::appendEdgesFromSegments(const std::vector<ColourSegment>& segments, std::list<arma::vec2>& pointList, const std::vector<arma::vec2>& greenHorizonInterpolatedPoints) {
            for (const ColourSegment& segment : segments) {
                const arma::vec2& start = segment.m_start;
                const arma::vec2& end = segment.m_end;

                if (start[1] > greenHorizonInterpolatedPoints.at(start[0])[1]) {
                    pointList.push_back(start);
                }

                if (end[1] > greenHorizonInterpolatedPoints.at(end[0])[1]) {
                    pointList.push_back(end);
                }
            }
        }

    }
}
