#include <libusb-1.0/libusb.h>

#include "Camera.h"

#include "message/input/Image.h"
#include "message/input/Sensors.h"
#include "message/motion/KinematicsModel.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"

namespace module {
namespace input {

    uint Camera::cameraCount = 0;

    using extension::Configuration;
    using message::input::Image;
    using message::input::Sensors;
    using message::motion::KinematicsModel;
    using message::input::CameraParameters;

    Camera::Camera(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , dumpImages(false)
        , V4L2FrameRateHandle()
        , V4L2SettingsHandle()
        , V4L2Cameras()
        , SpinnakerSystem(Spinnaker::System::GetInstance())
        , SpinnakerCamList(SpinnakerSystem->GetCameras(true, true))
        , SpinnakerLoggingCallback(*this)
        , SpinnakerCameras() {

        on<Configuration>("Camera.yaml").then("Camera Module Configuration", [this](const Configuration& config) {
            dumpImages = config["dump_images"].as<bool>();
        });

        on<Configuration>("Cameras").then("Camera driver loader", [this](const Configuration& config) {
            // Monitor camera config directory for files.
            // Each file MUST define a "driver", we use this driver to load the appropriate handler for the camera.

            if (config.config["driver"] && config.config["deviceID"]) {
                auto driver   = config["driver"].as<std::string>();
                auto deviceID = config["deviceID"].as<std::string>();

                log("Searching for", driver, "camera with deviceID", deviceID);

                if (driver == "V4L2") {
                    auto cam = V4L2Cameras.find(deviceID);

                    if (cam == V4L2Cameras.end()) {
                        V4L2Cameras.insert(std::make_pair(deviceID, std::move(initiateV4L2Camera(config))));
                        cameraCount++;
                    }

                    else {
                        cam->second.setConfig(config);
                    }
                }

                else if (driver == "Spinnaker") {
                    auto cam = SpinnakerCameras.find(deviceID);

                    if (cam == SpinnakerCameras.end()) {
                        initiateSpinnakerCamera(config);
                        cameraCount++;
                    }

                    else {
                        resetSpinnakerCamera(cam, config);
                    }
                }

                else {
                    log<NUClear::FATAL>("Unsupported camera driver:", driver);
                }
            }
        });

        on<Trigger<ImageData>, Optional<With<Sensors>>, Optional<With<KinematicsModel>>>().then(
            [this](const ImageData& image_data,
                   std::shared_ptr<const Sensors> sensors,
                   std::shared_ptr<const KinematicsModel> model) {
                static uint8_t count = 0;


                // NOTE: we only emit direct the ImageData messages and steal their data
                // Make sure you do not trigger on them anywhere else
                ImageData& i = const_cast<ImageData&>(image_data);

                // Copy across our data
                auto msg           = std::make_unique<Image>();
                msg->format        = i.format;
                msg->dimensions    = i.dimensions;
                msg->data          = std::move(i.data);
                msg->camera_id     = i.camera_id;
                msg->serial_number = i.serial_number;
                msg->timestamp     = i.timestamp;

                // Calculate our transform if we have information
                if (sensors && model) {

                    Eigen::Affine3d Htc(sensors->forwardKinematics[utility::input::ServoID::HEAD_PITCH]);
                    Htc(1, 3) += model->head.INTERPUPILLARY_DISTANCE * 0.5f * (i.isLeft ? 1.0f : -1.0f);

                    msg->Hcw = Htc.inverse() * sensors->world;
                }
                else {
                    msg->Hcw.setIdentity();
                }

                if (dumpImages) {
                    utility::vision::saveImage(fmt::format("image-{}.ppm", count++), *msg);
                }

                emit(msg);
            });

        on<Shutdown>().then([this] {
            ShutdownV4L2Camera();
            ShutdownSpinnakerCamera();
        });
    }

    // http://www.roman10.net/2011/06/14/how-to-reset-usb-device-in-linuxusing-libusb/
    bool Camera::resetUSBDevice(int bus, int device) {
        //     libusb_device_handle* devh = NULL;
        //     libusb_device* dev         = NULL;
        //     libusb_device** devs       = NULL;

        //     int ret;

        //     ret = libusb_init(NULL);

        //     if (ret < 0) {
        //         log<NUClear::WARN>("Failed to initalise libusb.");
        //         return false;
        //     }

        //     ret = libusb_get_device_list(NULL, &devs);

        //     if (ret < 0) {
        //         log<NUClear::WARN>("Failed to get device list from libusb.");
        //         return false;
        //     }

        //     for (int i = 0; devs[i - 1] != NULL; i++) {
        //         dev = devs[i];

        //         int busNum = (int) libusb_get_bus_number(dev);
        //         int devNum = (int) libusb_get_device_address(dev);

        //         if ((busNum == bus) && (devNum == device)) {
        //             break;
        //         }
        //     }

        //     if (dev == NULL) {
        //         log<NUClear::WARN>("Failed to find usb device", device, "on bus", bus);
        //         return false;
        //     }

        //     ret = libusb_open(dev, &devh);

        //     if (ret != 0) {
        //         log<NUClear::WARN>("Failed to open usb device", device, "on bus", bus);
        //         return false;
        //     }

        //     bool success = true;
        //     ret          = -1;

        //     for (int i = 0; (i < 100) && (ret < 0); i++) {
        //         ret = libusb_reset_device(devh);

        //         if (i > 98) {
        //             success = false;
        //             break;
        //         }
        //     }

        //     // free the device list
        //     libusb_free_device_list(devs, 1);
        //     libusb_exit(NULL);

        //     if (!success) {
        //         log<NUClear::WARN>("Failed to reset usb device", device, "on bus", bus);
        //         return false;
        //     }

        //     return true;
    }

    void Camera::initiateSpinnakerCamera(const Configuration& config) {
        log<NUClear::DEBUG>("Found ", SpinnakerCamList.GetSize(), " cameras.");

        if (SpinnakerCamList.GetSize() < 1) {
            return;
        }

        std::string serialNumber = config["deviceID"].as<std::string>();

        log<NUClear::DEBUG>("Processing camera", config.fileName, "with serial number", serialNumber);

        // See if we already have this camera
        auto camera = SpinnakerCameras.find(serialNumber);

        if (camera == SpinnakerCameras.end()) {
            try {
                Spinnaker::CameraPtr newCamera = SpinnakerCamList.GetBySerial(serialNumber);

                // Ensure we found the camera.
                if (newCamera) {
                    // Initlise the camera.
                    newCamera->Init();

                    // Add camera to list.
                    FOURCC fourcc =
                        utility::vision::getFourCCFromDescription(config["format"]["pixel"].as<std::string>());
                    camera =
                        SpinnakerCameras
                            .insert(std::make_pair(serialNumber,
                                                   std::make_unique<SpinnakerImageEvent>(config.fileName,
                                                                                         serialNumber,
                                                                                         std::move(newCamera),
                                                                                         *this,
                                                                                         fourcc,
                                                                                         cameraCount,
                                                                                         config["is_left"].as<bool>())))
                            .first;
                    log("Camera ", serialNumber, " added to map with local id", cameraCount);
                }

                else {
                    log<NUClear::WARN>("Failed to find camera", config.fileName, " with serial number: ", serialNumber);
                    return;
                }
            }
            catch (const Spinnaker::Exception& ex) {
                log<NUClear::WARN>("Failed to find camera", config.fileName, " with serial number: ", serialNumber);
                return;
            }
        }

        else {
            camera->second->camera->EndAcquisition();
        }

        // Get device node map.
        auto& nodeMap = camera->second->camera->GetNodeMap();

        // Set the pixel format.
        Spinnaker::GenApi::CEnumerationPtr ptrPixelFormat = nodeMap.GetNode("PixelFormat");

        if (IsAvailable(ptrPixelFormat) && IsWritable(ptrPixelFormat)) {
            // Retrieve the desired entry node from the enumeration node
            std::string format                              = config["format"]["pixel"].as<std::string>();
            Spinnaker::GenApi::CEnumEntryPtr newPixelFormat = ptrPixelFormat->GetEntryByName(format.c_str());

            if (IsAvailable(newPixelFormat) && IsReadable(newPixelFormat)) {
                ptrPixelFormat->SetIntValue(newPixelFormat->GetValue());

                log("Pixel format for camera ", camera->first, " set to ", format);
            }

            else {
                log("Failed to set pixel format to ", format, " for camera ", camera->first);
                log("PixelFormat enum entry is '", format, "' available? ", IsAvailable(newPixelFormat) ? "yes" : "no");
                log("PixelFormat enum entry is '", format, "' writable? ", IsWritable(newPixelFormat) ? "yes" : "no");
            }
        }

        else {
            log("Failed to retrieve pixel format for camera ", camera->first);
            log("PixelFormat enum entry is available? ", IsAvailable(ptrPixelFormat) ? "yes" : "no");
            log("PixelFormat enum entry is writable? ", IsWritable(ptrPixelFormat) ? "yes" : "no");
        }

        // Set the width and height of the image.
        Spinnaker::GenApi::CIntegerPtr ptrWidth = nodeMap.GetNode("Width");

        if (IsAvailable(ptrWidth) && IsWritable(ptrWidth)) {
            int64_t width = config["format"]["width"].as<int>();

            // Ensure the width is a multiple of the increment.
            if ((width % ptrWidth->GetInc()) != 0) {
                width =
                    std::min(ptrWidth->GetMax(), std::max(ptrWidth->GetMin(), width - (width % ptrWidth->GetInc())));
            }

            ptrWidth->SetValue(width);
            log("Image width for camera ", camera->first, " set to ", width);
        }

        else {
            log("Failed to retrieve image width for camera ", camera->first);
            log("Width entry is available? ", IsAvailable(ptrWidth) ? "yes" : "no");
            log("Width entry is writable? ", IsWritable(ptrWidth) ? "yes" : "no");
        }

        Spinnaker::GenApi::CIntegerPtr ptrHeight = nodeMap.GetNode("Height");

        if (IsAvailable(ptrHeight) && IsWritable(ptrHeight)) {
            int64_t height = config["format"]["height"].as<int>();

            // Ensure the height is a multiple of the increment.
            if ((height % ptrHeight->GetInc()) != 0) {
                height = std::min(ptrHeight->GetMax(),
                                  std::max(ptrHeight->GetMin(), height - (height % ptrHeight->GetInc())));
            }

            ptrHeight->SetValue(height);
            log("Image height for camera ", camera->first, " set to ", height);
        }

        else {
            log("Failed to retrieve image height for camera ", camera->first);
            log("Height entry is available? ", IsAvailable(ptrHeight) ? "yes" : "no");
            log("Height entry is writable? ", IsWritable(ptrHeight) ? "yes" : "no");
        }

        // Set exposure to auto/continuous
        Spinnaker::GenApi::CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");

        if (IsAvailable(ptrExposureAuto) || IsWritable(ptrExposureAuto)) {
            Spinnaker::GenApi::CEnumEntryPtr ptrExposureAutoContinuous = ptrExposureAuto->GetEntryByName("Continuous");

            if (IsAvailable(ptrExposureAutoContinuous) || IsReadable(ptrExposureAutoContinuous)) {
                ptrExposureAuto->SetIntValue(ptrExposureAutoContinuous->GetValue());
                log(camera->first, " set to continuous auto exposure.");
            }

            else {
                log("Unable to enable automatic exposure (enum entry retrieval). Non-fatal error...");
                return;
            }
        }

        else {
            log("Unable to enable automatic exposure (node retrieval). Non-fatal error...");
            return;
        }

        setExposure(nodeMap, config["settings"]["exposure"].as<double>());
        setGain(nodeMap, config["settings"]["gain"].as<double>());

        Spinnaker::GenApi::CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");

        if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode)) {
            log("Failed to retrieve acquisition mode for camera ", camera->first);
            return;
        }

        Spinnaker::GenApi::CEnumEntryPtr ptrAcquisitionModeContinuous =
            ptrAcquisitionMode->GetEntryByName("Continuous");

        if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous)) {
            log("Failed to retrieve continuous acquisition mode entry for camera ", camera->first);
            return;
        }

        ptrAcquisitionMode->SetIntValue(ptrAcquisitionModeContinuous->GetValue());

        log("Camera ", camera->first, " set to continuous acquisition mode.");

        // Setup the event handler for image acquisition.
        camera->second->camera->RegisterEvent(*camera->second);

        log("Camera ", camera->first, " image event handler registered.");

        // Begin acquisition.
        camera->second->camera->BeginAcquisition();

        log("Camera ", camera->first, " image acquisition started.");

        auto cameraParameters = std::make_unique<CameraParameters>();

        // Generic camera parameters
        cameraParameters->imageSizePixels << config["format"]["width"].as<uint>(),
            config["format"]["height"].as<uint>();
        cameraParameters->FOV << config["lens"]["FOV"].as<double>(), config["lens"]["FOV"].as<double>();

        // Radial specific
        cameraParameters->lens                   = CameraParameters::LensType::RADIAL;
        cameraParameters->radial.radiansPerPixel = config["lens"]["radiansPerPixel"].as<float>();
        cameraParameters->centreOffset           = convert<int, 2>(config["lens"]["centreOffset"].as<arma::ivec>());

        emit<Scope::DIRECT>(std::move(cameraParameters));

        log("Emitted radial camera parameters for camera", config["deviceID"].as<std::string>());
    }

    void Camera::setExposure(Spinnaker::GenApi::INodeMap& nodeMap, double exposure) {
        Spinnaker::GenApi::CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
        if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto)) {
            log("Unable to disable automatic exposure (node retrieval). Aborting...");
            return;
        }

        Spinnaker::GenApi::CEnumEntryPtr ptrExposureAutoOff = ptrExposureAuto->GetEntryByName("Off");
        if (!IsAvailable(ptrExposureAutoOff) || !IsReadable(ptrExposureAutoOff)) {
            log("Unable to disable automatic exposure (enum entry retrieval). Aborting...");
            return;
        }

        ptrExposureAuto->SetIntValue(ptrExposureAutoOff->GetValue());

        log("Automatic exposure disabled...");

        //
        // Set exposure time manually; exposure time recorded in microseconds
        //
        // *** NOTES ***
        // The node is checked for availability and writability prior to the
        // setting of the node. Further, it is ensured that the desired exposure
        // time does not exceed the maximum. Exposure time is counted in
        // microseconds. This information can be found out either by
        // retrieving the unit with the GetUnit() method or by checking SpinView.
        //
        Spinnaker::GenApi::CFloatPtr ptrExposureTime = nodeMap.GetNode("ExposureTime");
        if (!IsAvailable(ptrExposureTime) || !IsWritable(ptrExposureTime)) {
            log("Unable to set exposure time. Aborting...");
            return;
        }

        // Ensure desired exposure time does not exceed the maximum
        const double exposureTimeMax = ptrExposureTime->GetMax();
        double exposureTimeToSet     = exposure;

        if (exposureTimeToSet > exposureTimeMax) {
            exposureTimeToSet = exposureTimeMax;
        }

        ptrExposureTime->SetValue(exposureTimeToSet);

        log("Exposure set to ", ptrExposureTime->GetValue());
    }

    void Camera::setGain(Spinnaker::GenApi::INodeMap& nodeMap, double gain) {
        Spinnaker::GenApi::CEnumerationPtr ptrGainAuto = nodeMap.GetNode("GainAuto");
        if (!IsAvailable(ptrGainAuto) || !IsWritable(ptrGainAuto)) {
            log("Unable to disable automatic gain (node retrieval). Aborting...");
            return;
        }

        Spinnaker::GenApi::CEnumEntryPtr ptrGainAutoOff = ptrGainAuto->GetEntryByName("Off");
        if (!IsAvailable(ptrGainAutoOff) || !IsReadable(ptrGainAutoOff)) {
            log("Unable to disable automatic gain (enum entry retrieval). Aborting...");
            return;
        }

        ptrGainAuto->SetIntValue(static_cast<int64_t>(ptrGainAutoOff->GetValue()));

        log("Automatic gain disabled...");

        // Set gain; gain recorded in decibels
        Spinnaker::GenApi::CFloatPtr ptrGain = nodeMap.GetNode("Gain");
        if (!IsAvailable(ptrGain) || !IsWritable(ptrGain)) {
            log("Unable to set gain (node retrieval). Aborting...");
            return;
        }

        double gainMax = ptrGain->GetMax();

        if (gain > gainMax) {
            gain = gainMax;
        }

        ptrGain->SetValue(gain);

        log("Gain set to ", ptrGain->GetValue());
    }

    // When we shutdown, we must tell our camera class to close (stop streaming)
    void Camera::ShutdownV4L2Camera() {
        for (auto& camera : V4L2Cameras) {
            camera.second.closeCamera();
            camera.second.disableHandles();
        }

        V4L2Cameras.clear();
    }
}  // namespace input
}  // namespace module
