#ifndef MODULE_MOTION_WALK_FOOTSTEP_H
#define MODULE_MOTION_WALK_FOOTSTEP_H

#include <Eigen/Core>
#include <nuclear>

namespace module {
namespace motion {
    namespace walk {

        class FootStep : public NUClear::Reactor {
        private:
            double f_x(const Eigen::Vector3d& vec);
            double f_y(const Eigen::Vector3d& vec);

            size_t subsumptionId;

            double step_height;
            double step_steep;
            double well_width;
            double c;

        public:
            /// @brief Called by the powerplant to build and setup the FootStep reactor.
            explicit FootStep(std::unique_ptr<NUClear::Environment> environment);
        };
    }
}
}

#endif  // MODULE_MOTION_WALK_FOOTSTEP_H
