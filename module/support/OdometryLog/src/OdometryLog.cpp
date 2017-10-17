#include "OdometryLog.h"

#include "extension/Configuration.h"

#include "message/behaviour/Nod.h"
#include "message/input/Sensors.h"
#include "message/localisation/Field.h"
#include "message/platform/darwin/DarwinSensors.h"

#include "utility/math/matrix/Transform3D.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"

#include "utility/input/ServoID.h"

namespace module {
namespace support {

    using extension::Configuration;
    using message::input::Sensors;
    using message::localisation::Field;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using message::platform::darwin::ButtonLeftDown;
    using message::behaviour::Nod;
    using utility::nubugger::graph;
    using ServoID = utility::input::ServoID;
    using namespace std::chrono;

    OdometryLog::OdometryLog(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), logFile(), logFilePath("odom_data.CSV") {

        on<Configuration>("OdometryLog.yaml").then([this](const Configuration& config) {
            // Use configuration here from file OdometryLog.yaml

            localisationOffset = config["localisationOffset"].as<arma::vec>();
            log(__LINE__);
            logFilePath = "odom_data.csv";
        });

        on<Startup>().then("Leg Loads Logger Startup", [this]() {
            log(__LINE__);
            logFile.open(logFilePath, std::ios::out | std::ios::binary);

            /* get time , x, y, angle and leg positions from robot */

            if ((logFile.is_open() == true) && (logFile.good() == true)) {
                logFile << "Time, x_pos, y_pos, angle ,"
                        << "RightHipPitchPosition, RightHipRollPosition, RightHipYawPosition,"
                        << "LeftHipPitchPosition, LeftHipRollPosition,LeftHipYawPosition, "
                        << "RightKneePosition, LeftKneePosition, "
                        << "RightAnklePitchPosition, RightAnkleRollPosition ,"
                        << "LeftAnklePitchPosition, LeftAnkleRollPosition " << std::endl;
            }

            else {
                NUClear::log<NUClear::ERROR>("Failed to open log file '", logFilePath, "'.");
            }
            logFile.close();
        });

        on<Trigger<ButtonLeftDown>, Single, With<Sensors>, Sync<OdometryLog>>().then([this](const Sensors& sensors) {
            log(__LINE__);
            NUClear::log("Localisation Orientation reset. This direction is now forward.");
            emit(std::make_unique<Nod>(true));
            Transform2D Trw    = Transform3D(convert<double, 4, 4>(sensors.world)).projectTo2D();
            localisationOffset = Trw;
            logFile.open(logFilePath, std::ios::out | std::ios::binary);
            log(__LINE__);
            /* "blank" is to keep line length the same in the output csv*/
            if ((logFile.is_open() == true) && (logFile.good() == true)) {
                logFile << "Localisation Orientation reset This direction is now forward," << Trw.x() << "," << Trw.y()
                        << "," << Trw.angle() << ","
                        << "blank , blank , blank, blank, blank, blank , blank, blank, blank,"
                        << "blank , blank, blank" << std::endl;
            }
            else {
                NUClear::log<NUClear::ERROR>("Failed to open log file '", logFilePath, "'.");
            }
            logFile.close();
        });


        on<Every<30, Per<std::chrono::seconds>>, With<Sensors>, Sync<OdometryLog>, Single>().then(
            "Odometry Loc", [this](const Sensors& sensors) {
                log(__LINE__);
                logFile.open(logFilePath, std::ios::out | std::ios::binary | std::ios::app);

                /* save odometry info to csv type file */
                // If the file isn't open skip
                if (!logFile.is_open()) {
                    { return; }
                }
                /* get current time */
                auto timeNow   = std::chrono::steady_clock::now();
                auto timeDur   = timeNow.time_since_epoch();
                auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(timeDur).count();
                /* get current joint angles */
                float RightHipPitchPosition = sensors.servo[ServoID::R_HIP_PITCH].presentPosition;
                float RightHipRollPosition  = sensors.servo[ServoID::R_HIP_ROLL].presentPosition;
                float RightHipYawPosition   = sensors.servo[ServoID::R_HIP_YAW].presentPosition;

                float LeftHipPitchPosition = sensors.servo[ServoID::L_HIP_PITCH].presentPosition;
                float LeftHipRollPosition  = sensors.servo[ServoID::L_HIP_ROLL].presentPosition;
                float LeftHipYawPosition   = sensors.servo[ServoID::L_HIP_YAW].presentPosition;

                float RightKneePosition = sensors.servo[ServoID::R_KNEE].presentPosition;
                float LeftKneePosition  = sensors.servo[ServoID::L_KNEE].presentPosition;

                float RightAnklePitchPosition = sensors.servo[ServoID::R_ANKLE_PITCH].presentPosition;
                float RightAnkleRollPosition  = sensors.servo[ServoID::R_ANKLE_ROLL].presentPosition;


                float LeftAnklePitchPosition = sensors.servo[ServoID::L_ANKLE_PITCH].presentPosition;
                float LeftAnkleRollPosition  = sensors.servo[ServoID::L_ANKLE_ROLL].presentPosition;


                /* get current x,y and angle values */
                Transform2D Trw = Transform3D(convert<double, 4, 4>(sensors.world)).projectTo2D();
                Transform2D Twr = Trw.i();

                // Transform2D state = localisationOffset.localToWorld(Twr);
                log(__LINE__);
                logFile << timestamp << "," << Trw.x() << "," << Trw.y() << "," << Trw.angle() << ","
                        << RightHipPitchPosition << "," << RightHipRollPosition << "," << RightHipYawPosition << ","
                        << LeftHipPitchPosition << "," << LeftHipRollPosition << "," << LeftHipYawPosition << ","
                        << RightKneePosition << "," << LeftKneePosition << "," << RightAnklePitchPosition << ","
                        << RightAnkleRollPosition << "," << LeftAnklePitchPosition << "," << LeftAnkleRollPosition
                        << std::endl;
                logFile.close();
                log(__LINE__);
            });


        on<Shutdown>().then("OdometryLog Shutdown", [this]() {
            log(__LINE__);
            logFile.close();
        });


    }  // OdometryLog


}  // namespace localisation
}  // namespace module
