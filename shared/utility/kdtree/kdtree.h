/*
 * This is free and unencumbered software released into the public domain.
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 * For more information, please refer to <http://unlicense.org/>
 *
 * @author: Josiah Walker
 */

#ifndef KDTREE_H
#define KDTREE_H

#include <sys/types.h>
#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

namespace utility {
namespace kdtree {
    template <typename T = float>
    struct KDTreeNode {
        int splitDim;
        double splitVal;
        Eigen::Matrix<T, Eigen::Dynamic, 1> LowerBounds;
        Eigen::Matrix<T, Eigen::Dynamic, 1> UpperBounds;
        Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> dataIndices;
        Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> spillDataIndices;
        uint32_t leftChild;   // std::reference_wrapper<KDTreeNode> leftChild;
        uint32_t rightChild;  // std::reference_wrapper<KDTreeNode> rightChild;
    };

    template <typename T = float>
    class KDTree {
    private:
        uint32_t root;
        std::vector<KDTreeNode<T>> nodes;
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> data;

        double getDistSq(const uint32_t& i, const Eigen::Matrix<T, Eigen::Dynamic, 1>& v) {
            return (data.col(i) - v).squaredNorm();
        }

        void addNeighbour(std::vector<std::pair<double, uint64_t>>& nHeap, double dist, uint64_t ind) {
            static auto neighbourCmp = ([](const std::pair<double, uint64_t>& a, const std::pair<double, uint64_t>& b) {
                return std::get<0>(a) < std::get<0>(b);
            });

            if (dist < std::get<0>(nHeap.front())) {
                std::pop_heap(nHeap.begin(), nHeap.end(), neighbourCmp);
                nHeap.back() = std::make_pair(dist, ind);
                std::push_heap(nHeap.begin(), nHeap.end(), neighbourCmp);
            }
        }

        uint32_t mkNodeRecursive(const Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> dataIndices,
                                 const Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> spillDataIndices,
                                 const Eigen::Matrix<T, Eigen::Dynamic, 1>& LowerBounds,
                                 const Eigen::Matrix<T, Eigen::Dynamic, 1>& UpperBounds,
                                 const uint& minLeafSize,
                                 const double& epsilon) {
            // we need to determine the split point first, because if the data is unsplittable we should put it in a
            // leaf
            int splitDim = 0;
            const Eigen::Matrix<T, Eigen::Dynamic, 1> boundGap = UpperBounds - LowerBounds;

            for (uint i = 0; i < boundGap.n_elem; ++i) {
                if (boundGap[splitDim] < boundGap[i]) {
                    splitDim = i;
                }
            }

            double splitVal = (UpperBounds[splitDim] + LowerBounds[splitDim]) / 2.0f;

            // slide the split point if one child is empty
            Eigen::Matrix<T, 1, Eigen::Dynamic> vals =
                Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>(data.cols(dataIndices)).row(splitDim);
            double maxv = arma::max(vals);
            double minv = arma::min(vals);

            if (maxv <= splitVal) {
                splitVal = maxv - 0.01f;
            }

            else if (minv > splitVal) {
                splitVal = minv;
            }

            Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> leftInds  = arma::find(vals <= splitVal + epsilon);
            Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> rightInds = arma::find(vals > splitVal + epsilon);

            if ((dataIndices.n_elem <= minLeafSize) || (leftInds.n_elem == 0) || (rightInds.n_elem == 0)) {
                // return a leaf node
                const uint32_t empty = 0;

                KDTreeNode<T> node = {-1,
                                      0.0f,
                                      Eigen::Matrix<T, Eigen::Dynamic, 1>(),
                                      Eigen::Matrix<T, Eigen::Dynamic, 1>(),
                                      dataIndices,
                                      spillDataIndices,
                                      empty,
                                      empty};
                nodes.push_back(node);
            }

            else {
                // separate the child data
                Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> leftDataIndices  = dataIndices(leftInds);
                Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> rightDataIndices = dataIndices(rightInds);

                // separate the spill data
                Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> leftSpillData, rightSpillData;

                if (epsilon > 0.0f) {
                    Eigen::Matrix<T, Eigen::Dynamic, 1> spillDists =
                        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>(data.cols(spillDataIndices))
                            .row(splitDim)
                            .transpose();

                    leftSpillData =
                        arma::join_cols(spillDataIndices(arma::find(spillDists <= splitVal + epsilon)),
                                        dataIndices(rightInds(arma::find(vals(rightInds) <= splitVal + epsilon))));
                    rightSpillData =
                        arma::join_cols(spillDataIndices(arma::find(spillDists > splitVal - epsilon)),
                                        dataIndices(leftInds(arma::find(vals(leftInds) > splitVal - epsilon))));
                }

                // create the child bounds
                Eigen::Matrix<T, Eigen::Dynamic, 1> leftBounds = UpperBounds;
                leftBounds[splitDim] = splitVal;
                Eigen::Matrix<T, Eigen::Dynamic, 1> rightBounds = LowerBounds;
                rightBounds[splitDim] = splitVal;

                // make a new node with children
                KDTreeNode<T> node = {
                    splitDim,
                    splitVal,
                    LowerBounds,
                    UpperBounds,
                    Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>(),
                    Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>(),
                    mkNodeRecursive(leftDataIndices, leftSpillData, LowerBounds, leftBounds, minLeafSize, epsilon),
                    mkNodeRecursive(rightDataIndices, rightSpillData, rightBounds, UpperBounds, minLeafSize, epsilon)};

                nodes.push_back(node);
            }

            return (nodes.size() - 1);
        }

    public:
        KDTree(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& d,
               const uint& minLeafSize,
               const double epsilon = 0.0f) {
            nodes.push_back(KDTreeNode<T>());
            data = d;
            Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> inds =
                arma::linspace<Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>>(0, data.n_cols - 1, data.n_cols);
            root = mkNodeRecursive(inds,
                                   Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>(),
                                   arma::min(data, 1),
                                   arma::max(data, 1),
                                   minLeafSize,
                                   epsilon);
        }

        std::vector<std::pair<double, uint64_t>> query(const Eigen::Matrix<T, Eigen::Dynamic, 1>& val,
                                                       const int& kneighbours,
                                                       const uint64_t& maxLeaves    = 10000000,
                                                       const double lowDimProjError = 0.0f) {
            // initialise heaps
            std::vector<std::tuple<double, uint32_t>> candidateHeap(1, std::make_tuple(0.0, root));
            candidateHeap.reserve(4096);  // XXX: this is a dirty way of avoiding memory copies
            std::vector<std::pair<double, uint64_t>> neighbourHeap(kneighbours,
                                                                   {std::numeric_limits<double>::max(), 0});

            // initialise heap comparisons
            auto candidateCmp = ([](const std::tuple<double, uint32_t>& a, const std::tuple<double, uint32_t>& b) {
                return std::get<0>(a) > std::get<0>(b);
            });

            uint64_t leafCounter = 0;

            // keep going until the closest KD-tree node is further than the furthest neighbour
            while (candidateHeap.size() > 0
                   and std::get<0>(candidateHeap[0]) < std::get<0>(neighbourHeap[0]) + lowDimProjError
                   and leafCounter < maxLeaves) {

                // get a new candidate KD-tree node
                std::pop_heap(candidateHeap.begin(), candidateHeap.end(), candidateCmp);
                auto current = candidateHeap.back();
                candidateHeap.resize(candidateHeap.size() - 1);


                while (nodes[std::get<1>(current)].dataIndices.n_elem == 0) {
                    KDTreeNode<T>& cnode = nodes[std::get<1>(current)];
                    // descend until we reach a leaf,
                    double d     = cnode.splitVal - val[cnode.splitDim];
                    double pDist = std::get<0>(current) + d * d;
                    uint32_t farChild;
                    if (val[cnode.splitDim] > cnode.splitVal) {
                        farChild = cnode.leftChild;
                        current  = std::make_tuple(std::get<0>(current), cnode.rightChild);
                    }
                    else {
                        farChild = cnode.rightChild;
                        current  = std::make_tuple(std::get<0>(current), cnode.leftChild);
                    }

                    // put the more distant decision node onto the heap for later
                    candidateHeap.push_back(std::make_pair(pDist, farChild));
                    std::push_heap(candidateHeap.begin(), candidateHeap.end(), candidateCmp);
                }

                // add any near neighbours to the result
                KDTreeNode<T>& cnode = nodes[std::get<1>(current)];
                for (uint i = 0; i < cnode.dataIndices.n_elem; ++i) {
                    double dist = getDistSq(cnode.dataIndices[i], val);
                    addNeighbour(neighbourHeap, dist, cnode.dataIndices[i]);
                }
                for (uint i = 0; i < cnode.spillDataIndices.n_elem; ++i) {
                    double dist = getDistSq(cnode.spillDataIndices[i], val);
                    addNeighbour(neighbourHeap, dist, cnode.spillDataIndices[i]);
                }
                ++leafCounter;
            }
            // std::cout << leafCounter << std::endl;
            return neighbourHeap;
        }

        uint64_t getTotalNumItems() {
            uint64_t total = 0;

            for (int n = 0; n < nodes.size(); n++) {
                total += nodes.at(n).dataIndices.n_elem;
                total += nodes.at(n).spillDataIndices.n_elem;
            }

            return (total);
        }

        void replaceData(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& d) {
            /*
            Replaces the internal data store with d.
            Used when constructing tree on low-dimensional projections.
            */
            data = d;
        }
    };
}
}

#endif
