#ifndef __ROBOTSIMSETTINGS_H__
#define __ROBOTSIMSETTINGS_H__

#include <string>
#include <vector>
#include <exception>
#include <functional>
#include "Settings.hpp"
#include "CommonStructs.hpp"
#include "common_utils/Utils.hpp"
#include "Common.hpp"
#include "ImageCaptureBase.hpp"

namespace RobotSim
{

/**
 * @brief
 *
 */
struct RobotSimSettings
{
private:
    typedef common_utils::Utils Utils;
    typedef ImageCaptureBase::ImageType ImageType;

public:                                       // types
    static constexpr int kSubwindowCount = 3; // must be >= 3 for now
    static constexpr char const* kVehicleTypeUrdfBot = "urdfbot";
    static constexpr char const* kVehicleTypeSimpleVehicle = "simplevehicle";

    /**
     * @brief Subwindow settings: window subscript, image type, whether
    // to visualize, camera name
     *
     */
    struct SubwindowSetting
    {
        int window_index;
        ImageType image_type;
        bool visible;
        std::string camera_name;

        SubwindowSetting(int window_index_val = 0,
                         ImageType image_type_val = ImageType::Scene,
                         bool visible_val = false,
                         const std::string& camera_name_val = "")
            : window_index(window_index_val), image_type(image_type_val),
              visible(visible_val), camera_name(camera_name_val)
        {
        }
    };

    /**
     * @brief
     *
     */
    struct PawnPath
    {
        std::string pawn_bp;
        std::string slippery_mat;
        std::string non_slippery_mat;
        std::string urdf_path;
        std::string end_effector_link;
        float scale_factor;
        bool enable_keyboard;

        PawnPath(
            const std::string& pawn_bp_val = "",
            const std::string& slippery_mat_val =
                "/RobotSim/VehicleAdv/PhysicsMaterials/Slippery.Slippery",
            const std::string& non_slippery_mat_val =
                "/RobotSim/VehicleAdv/PhysicsMaterials/NonSlippery.NonSlippery",
            const std::string& urdf_path_val = "",
            const std::string& end_effector_link_val = "",
            const float& scale_factor_val = 1.0f,
            const bool& enable_keyboard_val = false)
            : pawn_bp(pawn_bp_val), slippery_mat(slippery_mat_val),
              non_slippery_mat(non_slippery_mat_val), urdf_path(urdf_path_val),
              end_effector_link(end_effector_link_val),
              scale_factor(scale_factor_val),
              enable_keyboard(enable_keyboard_val)
        {
        }
    };

    /**
     * @brief
     *
     */
    struct Rotation
    {
        float yaw = 0;
        float pitch = 0;
        float roll = 0;

        Rotation()
        {
        }

        Rotation(float yaw_val, float pitch_val, float roll_val)
            : yaw(yaw_val), pitch(pitch_val), roll(roll_val)
        {
        }

        bool hasNan()
        {
            std::isnan(yaw) || std::isnan(pitch) || std::isnan(roll);
        }

        static Rotation nanRotation()
        {
            static const Rotation val(Utils::nan<float>(), Utils::nan<float>(),
                                      Utils::nan<float>());
            return val;
        }
    };

    /**
     * @brief
     *
     */
    struct GimbalSetting
    {
        float stabilization = 0;
        // bool is_world_frame = false;
        Rotation rotation = Rotation::nanRotation();
    };

    /**
     * @brief
     *
     */
    struct CaptureSetting
    {
        typedef common_utils::Utils Utils;
        static constexpr float kSceneTargetGamma = 1.4f;

        int image_type = 0;

        unsigned int width = 256, height = 144;                   // 960 X 540
        float fov_degrees = Utils::nan<float>();                  // 90.0f
        int auto_exposure_method = -1;                            // histogram
        float auto_exposure_speed = Utils::nan<float>();          // 100.0f;
        float auto_exposure_bias = Utils::nan<float>();           // 0;
        float auto_exposure_max_brightness = Utils::nan<float>(); // 0.64f;
        float auto_exposure_min_brightness = Utils::nan<float>(); // 0.03f;
        float auto_exposure_low_percent = Utils::nan<float>();    // 80.0f;
        float auto_exposure_high_percent = Utils::nan<float>();   // 98.3f;
        float auto_exposure_histogram_log_min = Utils::nan<float>(); // -8;
        float auto_exposure_histogram_log_max = Utils::nan<float>(); // 4;
        float motion_blur_amount = Utils::nan<float>();
        float target_gamma =
            Utils::nan<float>(); // 1.0f; //This would be reset to
                                 // kSceneTargetGamma for scene as default
        int projection_mode = 0; // ECameraProjectionMode::Perspective
        float ortho_width = Utils::nan<float>();
    };

    /**
     * @brief This structure contains the noise parameters of the different
     * camera objects.
     *
     */
    struct NoiseSetting
    {
        int ImageType = 0;

        bool Enabled = false;

        float RandContrib = 0.2f;
        float RandSpeed = 100000.0f;
        float RandSize = 500.0f;
        float RandDensity = 2.0f;

        float HorzWaveContrib = 0.03f;
        float HorzWaveStrength = 0.08f;
        float HorzWaveVertSize = 1.0f;
        float HorzWaveScreenSize = 1.0f;

        float HorzNoiseLinesContrib = 1.0f;
        float HorzNoiseLinesDensityY = 0.01f;
        float HorzNoiseLinesDensityXY = 0.5f;

        float HorzDistortionContrib = 1.0f;
        float HorzDistortionStrength = 0.002f;
    };

    /**
     * @brief
     *
     */
    struct CameraSetting
    {
        // nan means keep the default values set in components
        Vector3r position = VectorMath::nanVector();
        Rotation rotation = Rotation::nanRotation();

        std::string attach_link = "";

        GimbalSetting gimbal;
        std::map<int, CaptureSetting> capture_settings;
        std::map<int, NoiseSetting> noise_settings;

        CameraSetting()
        {
            initializeCaptureSettings(capture_settings);
            initializeNoiseSettings(noise_settings);
        }
    };

    /**
     * @brief
     *
     */
    struct CameraDirectorSetting
    {
        Vector3r position = VectorMath::nanVector();
        Rotation rotation = Rotation::nanRotation();
        float follow_distance = Utils::nan<float>();
    };

    /**
     * @brief
     *
     */
    struct ControlSetting
    {
        std::string LeftWheelJoint;
        std::string RightWheelJoint;
        std::string EndEffectorLink;
        std::vector<std::string> ManipulatorJoints;
    };

    /**
     * @brief This data structure contains the different values characterizing
     * the transmission chain of a robot or vehicle.
     *
     */
    struct ActuationSetting
    {
        float gearRatio = 1.0f; // Gear ratio of the OpenBot motors.
        float motorVelocityConstant =
            1.0f; // Motor torque constant in [rad/s/V]
        float controlDeadZone =
            1.0f; // Below this command threshold, the torque is set to
                  // zero if the motor velocity is "small enough"
        float motorTorqueMax =
            1.0f; // Maximum torque a motor (+ gearbox system)
                  // can apply to one of the wheels in [N.m].
        float electricalResistance = 1.0f; // Electrical resistance of the DC
                                           // motor windings in [Ohms]
        float electricalInductance = 1.0f; // Electrical inductance of the DC
                                           // motor windings in [Henry]
        float actionScale =
            1.0f; // Actions generated by a neural network or a RL algorithm
                  // might not be normalized. Therefore a scale factor needs to
                  // be applied to the input action to bound it to the
                  // [-1.0, 1.0] range. not be normalized. Therefore a scale
                  // factor needs to be applied to the input action to bound it
                  // to the [-1.0, 1.0] range.
        float batteryVoltage = 12.0f; // Expressed in [V]
    };

    /**
     * @brief
     *
     */
    struct VehicleSetting
    {
        // required
        std::string vehicle_name;
        std::string vehicle_type;

        // optional
        std::string default_vehicle_state;
        std::string pawn_path;
        bool allow_api_always = true;
        bool auto_create = true;
        bool enable_collision_passthrough = false;
        bool enable_trace = false;
        bool enable_collisions = true;
        bool is_fpv_vehicle = false;
        float debug_symbol_scale = 0.0f;
        ControlSetting controlSetting;
        ActuationSetting actuationSetting;

        // nan means use player start
        Vector3r position;
        Rotation rotation;
        // whether use box tracing to find position close to ground for spawn
        bool enable_spawn_tracing_ground = true;
        // whether use NavMesh to find random spawn location
        bool enable_random_spawn = true;
        Vector3r bounding_box;
        Vector3r bounding_box_offset;

        //相机输出不需要
        // Translation: Camera output is not required
        std::map<std::string, CameraSetting> cameras;
        // std::map<std::string, std::unique_ptr<SensorSetting>> sensors;
        std::vector<std::pair<std::string, std::string>> collision_blacklist;
    };

    /**
     * @brief
     *
     */
    struct SegmentationSetting
    {
        enum class InitMethodType
        {
            None,
            CommonObjectsRandomIDs
        };

        enum class MeshNamingMethodType
        {
            OwnerName,
            StaticMeshName
        };

        InitMethodType init_method = InitMethodType::CommonObjectsRandomIDs;
        bool override_existing = false;
        MeshNamingMethodType mesh_naming_method =
            MeshNamingMethodType::OwnerName;
    };

    /**
     * @brief
     *
     */
    struct TimeOfDaySetting
    {
        bool enabled = false;
        std::string start_datetime = ""; // format: %Y-%m-%d %H:%M:%S
        bool is_start_datetime_dst = false;
        float celestial_clock_speed = 1;
        float update_interval_secs = 60;
    };

    struct ControlledMotionComponentSetting
    {
        std::string name = "";
        std::map<std::string, std::string> configuration;
    };

private: // fields
    float settings_version_actual;
    float settings_version_minimum = 1.2f;

public: // fields
    std::string simmode_name = "";

    std::vector<SubwindowSetting> subwindow_settings;
    // RecordingSetting recording_setting;
    SegmentationSetting segmentation_setting;
    TimeOfDaySetting tod_setting;

    std::vector<std::string> warning_messages;
    std::vector<std::string> error_messages;

    bool is_record_ui_visible = false;
    int initial_view_mode =
        3; // ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FLY_WITH_ME
    bool enable_rpc = true;
    std::string api_server_address = "";
    std::string physics_engine_name = "";

    std::string clock_type = "";
    float clock_speed = 1.0f;
    bool engine_sound = false;
    bool log_messages_visible = true;
    HomeGeoPoint origin_geopoint{GeoPoint(
        47.641468,
        -122.140165,
        122)}; // The geo-coordinate assigned to Unreal coordinate 0,0,0
    std::map<std::string, PawnPath> pawn_paths; // path for pawn blueprint
    std::map<std::string, std::unique_ptr<VehicleSetting>> vehicles;
    CameraSetting camera_defaults;
    CameraDirectorSetting camera_director;
    float speed_unit_factor = 1.0f;
    std::string speed_unit_label = "m\\s";

public: // methods
    static RobotSimSettings& singleton()
    {
        static RobotSimSettings instance;
        return instance;
    }

    RobotSimSettings()
    {
        initializeSubwindowSettings(subwindow_settings);
        // initializePawnPaths(pawn_paths);
        // No need to load the default robotsimSettings.json
        initializeVehicleSettings(vehicles);
    }

    /**
     * @brief
     * returns number of warnings
     *
     * @param simmode_getter
     */
    void load(std::function<std::string(void)> simmode_getter)
    {
        warning_messages.clear();
        error_messages.clear();
        const Settings& settings_json = Settings::singleton();
        checkSettingsVersion(settings_json);

        loadCoreSimModeSettings(settings_json, simmode_getter);
        loadCameraDirectorSetting(settings_json, camera_director, simmode_name);
        loadSubWindowsSettings(settings_json, subwindow_settings);
        loadViewModeSettings(settings_json);
        loadPawnPaths(settings_json, pawn_paths);
        loadVehicleSettings(simmode_name, settings_json, vehicles);

        // this should be done last because it depends on type of vehicles we
        // have
        loadClockSettings(settings_json);
    }

    /**
     * @brief
     * Dependent only on settings.hpp
     *
     * @param json_settings_text
     */
    static void initializeSettings(const std::string& json_settings_text)
    {
        Settings& settings_json = Settings::loadJSonString(json_settings_text);
        if (!settings_json.isLoadSuccess())
            throw std::invalid_argument(
                "Cannot parse JSON settings_json string.");
    }

    /**
     * @brief Create a Default Settings File object
     * Dependent only on settings.hpp
     *
     */
    static void createDefaultSettingsFile()
    {
        std::string settings_filename =
            Settings::getUserDirectoryFullPath("settings.json");
        Settings& settings_json = Settings::loadJSonString("{}");
        // https://answers.unrealengine.com/questions/664905/unreal-crashes-on-two-lines-of-extremely-simple-st.html
        settings_json.saveJSonFile(settings_filename);
    }
    // Dependent only on string
    const VehicleSetting*
    getVehicleSetting(const std::string& vehicle_name) const
    {
        auto it = vehicles.find(vehicle_name);
        if (it == vehicles.end())
            throw std::invalid_argument(
                Utils::stringf("VehicleSetting for vehicle name %s was "
                               "requested but not found",
                               vehicle_name.c_str())
                    .c_str());
        else
            return it->second.get();
    }

private:
    /**
     * @brief
     * Dependent only on settings.hpp
     *
     * @param settings_json
     */
    void checkSettingsVersion(const Settings& settings_json)
    {
        bool has_default_settings =
            hasDefaultSettings(settings_json, settings_version_actual);
        bool upgrade_required =
            settings_version_actual < settings_version_minimum;
        if (upgrade_required)
        {
            bool auto_upgrade = false;

            // if we have default setting file not modified by user then we will
            // just auto-upgrade it
            if (has_default_settings)
            {
                auto_upgrade = true;
            }
            else
            {
                // check if auto-upgrade is possible
                if (settings_version_actual == 1)
                {
                    const std::vector<std::string> all_changed_keys = {
                        "AdditionalCameras", "CaptureSettings", "NoiseSettings",
                        "UsageScenario",     "SimpleFlight",    "PX4"};
                    std::stringstream detected_keys_ss;
                    for (const auto& changed_key : all_changed_keys)
                    {
                        if (settings_json.hasKey(changed_key))
                            detected_keys_ss << changed_key << ",";
                    }
                    std::string detected_keys = detected_keys_ss.str();
                    if (detected_keys.length())
                    {
                        std::string error_message =
                            "You are using newer version of RobotSim with "
                            "older version of settings.json. ";

                        error_messages.push_back(error_message + detected_keys);
                    }
                    else
                        auto_upgrade = true;
                }
                else
                    auto_upgrade = true;
            }

            if (auto_upgrade)
            {
                warning_messages.push_back(
                    "You are using newer version of RobotSim with older "
                    "version of settings.json. "
                    "You should delete your settings.json file and restart "
                    "RobotSim.");
            }
        }
        // else no action necessary
    }

    /**
     * @brief
     * Dependent only on settings.hpp
     *
     * @param settings_json
     * @param version
     * @return true
     * @return false
     */
    bool hasDefaultSettings(const Settings& settings_json, float& version)
    {
        // if empty settings file
        bool has_default = settings_json.size() == 0;

        bool has_docs = settings_json.getString("SeeDocsAt", "") != "" ||
                        settings_json.getString("see_docs_at", "") != "";
        // we had spelling mistake so we are currently supporting
        // SettingsVersion or SettingdVersion :(
        version = settings_json.getFloat(
            "SettingsVersion", settings_json.getFloat("SettingdVersion", 0));

        // If we have pre-V1 settings and only element is docs link
        has_default |= settings_json.size() == 1 && has_docs;

        // if we have V1 settings and only elements are docs link and version
        has_default |= settings_json.size() == 2 && has_docs && version > 0;

        return has_default;
    }

    /**
     * @brief
     *
     * @param settings_json
     * @param simmode_getter
     */
    void
    loadCoreSimModeSettings(const Settings& settings_json,
                            std::function<std::string(void)> simmode_getter)
    {
        // get the simmode from user if not specified
        simmode_name = settings_json.getString("SimMode", "");
        if (simmode_name == "")
        {
            if (simmode_getter)
                simmode_name = simmode_getter();
            else
                throw std::invalid_argument(
                    "simmode_name is not expected empty in SimModeBase");
        }
        physics_engine_name =
            "PhysX"; // this value is only informational for now

        UE_LOG(LogTemp, Warning, TEXT("simmode_name:%s"),
               *FString(simmode_name.c_str()));
    }

    /**
     * @brief
     *
     * @param settings_json
     */
    void loadViewModeSettings(const Settings& settings_json)
    {
        std::string view_mode_string = settings_json.getString("ViewMode", "");

        if (view_mode_string == "")
        {
            if (simmode_name == "Multirotor")
                view_mode_string = "FlyWithMe";
            else if (simmode_name == "ComputerVision")
                view_mode_string = "Fpv";
            else
                view_mode_string = "SpringArmChase";
        }

        if (view_mode_string == "Fpv")
            initial_view_mode =
                1; // ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FPV;
        else if (view_mode_string == "GroundObserver")
            initial_view_mode =
                2; // ECameraDirectorMode::CAMERA_DIRECTOR_MODE_GROUND_OBSERVER;
        else if (view_mode_string == "FlyWithMe")
            initial_view_mode =
                3; // ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FLY_WITH_ME;
        else if (view_mode_string == "Manual")
            initial_view_mode =
                4; // ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL;
        else if (view_mode_string == "SpringArmChase")
            initial_view_mode =
                5; // ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE;
        else if (view_mode_string == "Backup")
            initial_view_mode =
                6; // ECameraDirectorMode::CAMREA_DIRECTOR_MODE_BACKUP;
        else if (view_mode_string == "NoDisplay")
            initial_view_mode =
                7; // ECameraDirectorMode::CAMREA_DIRECTOR_MODE_NODISPLAY;
        else if (view_mode_string == "Front")
            initial_view_mode =
                8; // ECameraDirectorMode::CAMREA_DIRECTOR_MODE_FRONT;
        else
            error_messages.push_back("ViewMode setting is not recognized: " +
                                     view_mode_string);
    }

    static std::string getCameraName(const Settings& settings_json)
    {
        return settings_json.getString(
            "CameraName",
            // TODO: below exist only due to legacy reason and can be replaced
            // by "" in future
            std::to_string(settings_json.getInt("CameraID", 0)));
    }

    /**
     * @brief
     *
     * @param capture_settings
     */
    static void
    initializeCaptureSettings(std::map<int, CaptureSetting>& capture_settings)
    {
        capture_settings.clear();
        for (int i = -1; i < Utils::toNumeric(ImageType::Count); ++i)
        {
            capture_settings[i] = CaptureSetting();
        }
        capture_settings.at(Utils::toNumeric(ImageType::Scene)).target_gamma =
            CaptureSetting::kSceneTargetGamma;
    }

    /**
     * @brief
     *
     * @param settings_json
     * @param capture_settings
     */
    static void
    loadCaptureSettings(const Settings& settings_json,
                        std::map<int, CaptureSetting>& capture_settings)
    {
        initializeCaptureSettings(capture_settings);

        Settings json_parent;
        if (settings_json.getChild("CaptureSettings", json_parent))
        {
            for (size_t child_index = 0; child_index < json_parent.size();
                 ++child_index)
            {
                Settings json_settings_child;
                if (json_parent.getChild(child_index, json_settings_child))
                {
                    CaptureSetting capture_setting;
                    createCaptureSettings(json_settings_child, capture_setting);
                    capture_settings[capture_setting.image_type] =
                        capture_setting;
                }
            }
        }
    }

    /**
     * @brief Create a Vector Setting object
     *
     * @param settings_json
     * @param default_vec
     * @return Vector3r
     */
    static Vector3r createVectorSetting(const Settings& settings_json,
                                        const Vector3r& default_vec)
    {
        return Vector3r(settings_json.getFloat("X", default_vec.x()),
                        settings_json.getFloat("Y", default_vec.y()),
                        settings_json.getFloat("Z", default_vec.z()));
    }
    static Rotation createRotationSetting(const Settings& settings_json,
                                          const Rotation& default_rot)
    {
        return Rotation(settings_json.getFloat("Yaw", default_rot.yaw),
                        settings_json.getFloat("Pitch", default_rot.pitch),
                        settings_json.getFloat("Roll", default_rot.roll));
    }

    /**
     * @brief Create a Vehicle Setting object
     *
     * @param simmode_name
     * @param settings_json
     * @param vehicle_name
     * @return std::unique_ptr<VehicleSetting>
     */
    static std::unique_ptr<VehicleSetting>
    createVehicleSetting(const std::string& simmode_name,
                         const Settings& settings_json,
                         const std::string vehicle_name)
    {
        auto vehicle_type =
            Utils::toLower(settings_json.getString("VehicleType", ""));
        std::unique_ptr<VehicleSetting> vehicle_setting;
        vehicle_setting = std::unique_ptr<VehicleSetting>(new VehicleSetting());
        if (vehicle_type == kVehicleTypeUrdfBot)
        {
            vehicle_setting->debug_symbol_scale =
                settings_json.getFloat("DebugSymbolScale", 0.0f);

            Settings collisionBlacklist;

            std::vector<std::string> keys;
            std::string botMeshKey = "BotMesh";
            std::string externalMeshRegexKey = "ExternalActorRegex";
            keys.emplace_back(botMeshKey);
            keys.emplace_back(externalMeshRegexKey);

            std::vector<std::map<std::string, std::string>>
                collision_blacklist_pairs =
                    settings_json.getArrayOfKeyValuePairs("CollisionBlacklist",
                                                          keys);

            vehicle_setting->collision_blacklist.clear();
            for (unsigned int i = 0; i < collision_blacklist_pairs.size(); i++)
            {
                std::pair<std::string, std::string> pair;
                pair.first = collision_blacklist_pairs[i][botMeshKey];
                pair.second =
                    collision_blacklist_pairs[i][externalMeshRegexKey];

                vehicle_setting->collision_blacklist.emplace_back(pair);
            }
            // control setting
            Settings controlSetting;
            settings_json.getChild("Control", controlSetting);
            vehicle_setting->controlSetting =
                createControlSetting(controlSetting);
        }

        // Gear transmission setting:
        Settings actuationSetting;
        settings_json.getChild("ActuationParameters", actuationSetting);
        createActuationSetting(actuationSetting,
                               vehicle_setting->actuationSetting);

        // required settings_json
        vehicle_setting->vehicle_type = vehicle_type;

        // optional settings_json
        vehicle_setting->pawn_path = settings_json.getString("PawnPath", "");
        vehicle_setting->default_vehicle_state =
            settings_json.getString("DefaultVehicleState", "");
        vehicle_setting->allow_api_always = settings_json.getBool(
            "AllowAPIAlways", vehicle_setting->allow_api_always);
        vehicle_setting->auto_create =
            settings_json.getBool("AutoCreate", vehicle_setting->auto_create);
        vehicle_setting->enable_collision_passthrough = settings_json.getBool(
            "EnableCollisionPassthrogh",
            vehicle_setting->enable_collision_passthrough);
        vehicle_setting->enable_trace =
            settings_json.getBool("EnableTrace", vehicle_setting->enable_trace);
        vehicle_setting->enable_collisions = settings_json.getBool(
            "EnableCollisions", vehicle_setting->enable_collisions);
        vehicle_setting->is_fpv_vehicle = settings_json.getBool(
            "IsFpvVehicle", vehicle_setting->is_fpv_vehicle);

        Settings positionSetting;
        settings_json.getChild("SpawnTransform", positionSetting);
        vehicle_setting->position =
            createVectorSetting(positionSetting, vehicle_setting->position);
        vehicle_setting->rotation =
            createRotationSetting(positionSetting, vehicle_setting->rotation);

        vehicle_setting->enable_spawn_tracing_ground =
            settings_json.getBool("EnableSpawnTracingGround",
                                  vehicle_setting->enable_spawn_tracing_ground);
        vehicle_setting->enable_random_spawn = settings_json.getBool(
            "EnableRandomSpawn", vehicle_setting->enable_random_spawn);

        Settings boundingBoxSetting;
        settings_json.getChild("BoundingBox", boundingBoxSetting);
        vehicle_setting->bounding_box = createVectorSetting(
            boundingBoxSetting, vehicle_setting->bounding_box);

        Settings boundingBoxOffsetSetting;
        settings_json.getChild("BoundingBoxOffset", boundingBoxOffsetSetting);
        vehicle_setting->bounding_box_offset = createVectorSetting(
            boundingBoxOffsetSetting, vehicle_setting->bounding_box_offset);

        loadCameraSettings(settings_json, vehicle_setting->cameras);
        // TODO: create suitable sensor description files
        // loadSensorSettings(settings_json, "Sensors",
        // vehicle_setting->sensors);

        return vehicle_setting;
    }

    /**
     * @brief
     *
     * @param vehicles
     */
    static void initializeVehicleSettings(
        std::map<std::string, std::unique_ptr<VehicleSetting>>& vehicles)
    {
        vehicles.clear();

        // create default urdf bot vehicle
        auto urdf_bot_setting =
            std::unique_ptr<VehicleSetting>(new VehicleSetting());
        urdf_bot_setting->vehicle_name = "UrdfBot";
        urdf_bot_setting->vehicle_type = kVehicleTypeUrdfBot;
        urdf_bot_setting->is_fpv_vehicle =
            true; // TODO: Should this be defined somewhere else??
        vehicles[urdf_bot_setting->vehicle_name] = std::move(urdf_bot_setting);
    }

    /**
     * @brief
     *
     * @param simmode_name
     * @param settings_json
     * @param vehicles
     */
    static void loadVehicleSettings(
        const std::string& simmode_name,
        const Settings& settings_json,
        std::map<std::string, std::unique_ptr<VehicleSetting>>& vehicles)
    {
        initializeVehicleSettings(vehicles);

        RobotSim::Settings vehicles_child;
        if (settings_json.getChild("Vehicles", vehicles_child))
        {
            std::vector<std::string> keys;
            vehicles_child.getChildNames(keys);

            // remove default vehicles, if values are specified in settings
            if (keys.size())
                vehicles.clear();

            for (const auto& key : keys)
            {
                RobotSim::Settings child;
                vehicles_child.getChild(key, child);
                vehicles[key] = createVehicleSetting(simmode_name, child, key);
            }
        }
    }

    // static void initializePawnPaths(std::map<std::string, PawnPath>&
    // pawn_paths)
    //{
    //    pawn_paths.clear();
    //    pawn_paths.emplace("BareboneCar",
    //        PawnPath("Class'/RobotSim/VehicleAdv/Vehicle/VehicleAdvPawn.VehicleAdvPawn_C'"));
    //    pawn_paths.emplace("DefaultCar",
    //        PawnPath("Class'/RobotSim/VehicleAdv/SUV/SuvCarPawn.SuvCarPawn_C'"));
    //    pawn_paths.emplace("DefaultQuadrotor",
    //        PawnPath("Class'/RobotSim/Blueprints/BP_FlyingPawn.BP_FlyingPawn_C'"));
    //    pawn_paths.emplace("DefaultComputerVision",
    //        PawnPath("Class'/RobotSim/Blueprints/BP_ComputerVisionPawn.BP_ComputerVisionPawn_C'"));
    //
    //}

    /**
     * @brief
     *
     * @param settings_json
     * @param pawn_paths
     */
    static void loadPawnPaths(const Settings& settings_json,
                              std::map<std::string, PawnPath>& pawn_paths)
    {
        // initializePawnPaths(pawn_paths);

        RobotSim::Settings pawn_paths_child;
        if (settings_json.getChild("PawnPaths", pawn_paths_child))
        {
            std::vector<std::string> keys;
            pawn_paths_child.getChildNames(keys);

            for (const auto& key : keys)
            {
                RobotSim::Settings child;
                pawn_paths_child.getChild(key, child);
                pawn_paths[key] = createPathPawn(child);
            }
        }
    }

    /**
     * @brief Create a Path Pawn object
     *
     * @param settings_json
     * @return PawnPath
     */
    static PawnPath createPathPawn(const Settings& settings_json)
    {
        auto paths = PawnPath();
        paths.pawn_bp = settings_json.getString("PawnBP", "");
        auto slippery_mat = settings_json.getString("SlipperyMat", "");
        auto non_slippery_mat = settings_json.getString("NonSlipperyMat", "");
        auto urdf_path = settings_json.getString("UrdfFile", "");
        auto end_effector_link = settings_json.getString("EndEffectorLink", "");
        auto scale_factor = settings_json.getFloat("ScaleFactor", 1.0f);
        auto enable_keyboard = settings_json.getBool("EnableKeyboard", true);

        if (slippery_mat != "")
            paths.slippery_mat = slippery_mat;
        if (non_slippery_mat != "")
            paths.non_slippery_mat = non_slippery_mat;
        if (urdf_path != "")
            paths.urdf_path = RobotSim::Settings::getAnyPossiblePath(urdf_path);
        if (end_effector_link != "")
            paths.end_effector_link = end_effector_link;

        paths.scale_factor = scale_factor;
        paths.enable_keyboard = enable_keyboard;

        return paths;
    }

    /**
     * @brief
     *
     * @param settings_json
     * @param segmentation_setting
     */
    static void
    loadSegmentationSetting(const Settings& settings_json,
                            SegmentationSetting& segmentation_setting)
    {
        Settings json_parent;
        if (settings_json.getChild("SegmentationSettings", json_parent))
        {
            std::string init_method =
                Utils::toLower(json_parent.getString("InitMethod", ""));
            if (init_method == "" || init_method == "commonobjectsrandomids")
                segmentation_setting.init_method =
                    SegmentationSetting::InitMethodType::CommonObjectsRandomIDs;
            else if (init_method == "none")
                segmentation_setting.init_method =
                    SegmentationSetting::InitMethodType::None;
            else
                // TODO: below exception doesn't actually get raised right now
                // because of issue in Unreal Engine?
                throw std::invalid_argument(
                    std::string("SegmentationSetting init_method has invalid "
                                "value in settings_json ") +
                    init_method);

            segmentation_setting.override_existing =
                json_parent.getBool("OverrideExisting", false);

            std::string mesh_naming_method =
                Utils::toLower(json_parent.getString("MeshNamingMethod", ""));
            if (mesh_naming_method == "" || mesh_naming_method == "ownername")
                segmentation_setting.mesh_naming_method =
                    SegmentationSetting::MeshNamingMethodType::OwnerName;
            else if (mesh_naming_method == "staticmeshname")
                segmentation_setting.mesh_naming_method =
                    SegmentationSetting::MeshNamingMethodType::StaticMeshName;
            else
                throw std::invalid_argument(
                    std::string("SegmentationSetting MeshNamingMethod has "
                                "invalid value in settings_json ") +
                    mesh_naming_method);
        }
    }

    static void
    initializeNoiseSettings(std::map<int, NoiseSetting>& noise_settings)
    {
        int image_count = Utils::toNumeric(ImageType::Count);
        noise_settings.clear();
        for (int i = -1; i < image_count; ++i)
            noise_settings[i] = NoiseSetting();
    }

    /**
     * @brief Parses the input json string and fills the noise parameters of
     * the robot camera.
     *
     * @param settings_json
     * @param noise_settings
     */
    static void loadNoiseSettings(const Settings& settings_json,
                                  std::map<int, NoiseSetting>& noise_settings)
    {
        initializeNoiseSettings(noise_settings);

        Settings json_parent;
        if (settings_json.getChild("NoiseSettings", json_parent))
        {
            for (size_t child_index = 0; child_index < json_parent.size();
                 ++child_index)
            {
                Settings json_settings_child;
                if (json_parent.getChild(child_index, json_settings_child))
                {
                    NoiseSetting noise_setting;
                    loadNoiseSetting(json_settings_child, noise_setting);
                    noise_settings[noise_setting.ImageType] = noise_setting;
                }
            }
        }
    }

    /**
     * @brief Parses the input json string and fills the noise parameters of
     * the robot camera.
     *
     * @param settings_json
     * @param noise_setting
     */
    static void loadNoiseSetting(const RobotSim::Settings& settings_json,
                                 NoiseSetting& noise_setting)
    {
        noise_setting.Enabled =
            settings_json.getBool("Enabled", noise_setting.Enabled);
        noise_setting.ImageType =
            settings_json.getInt("ImageType", noise_setting.ImageType);

        noise_setting.HorzWaveStrength = settings_json.getFloat(
            "HorzWaveStrength", noise_setting.HorzWaveStrength);
        noise_setting.RandSpeed =
            settings_json.getFloat("RandSpeed", noise_setting.RandSpeed);
        noise_setting.RandSize =
            settings_json.getFloat("RandSize", noise_setting.RandSize);
        noise_setting.RandDensity =
            settings_json.getFloat("RandDensity", noise_setting.RandDensity);
        noise_setting.RandContrib =
            settings_json.getFloat("RandContrib", noise_setting.RandContrib);
        noise_setting.HorzWaveContrib = settings_json.getFloat(
            "HorzWaveContrib", noise_setting.HorzWaveContrib);
        noise_setting.HorzWaveVertSize = settings_json.getFloat(
            "HorzWaveVertSize", noise_setting.HorzWaveVertSize);
        noise_setting.HorzWaveScreenSize = settings_json.getFloat(
            "HorzWaveScreenSize", noise_setting.HorzWaveScreenSize);
        noise_setting.HorzNoiseLinesContrib = settings_json.getFloat(
            "HorzNoiseLinesContrib", noise_setting.HorzNoiseLinesContrib);
        noise_setting.HorzNoiseLinesDensityY = settings_json.getFloat(
            "HorzNoiseLinesDensityY", noise_setting.HorzNoiseLinesDensityY);
        noise_setting.HorzNoiseLinesDensityXY = settings_json.getFloat(
            "HorzNoiseLinesDensityXY", noise_setting.HorzNoiseLinesDensityXY);
        noise_setting.HorzDistortionStrength = settings_json.getFloat(
            "HorzDistortionStrength", noise_setting.HorzDistortionStrength);
        noise_setting.HorzDistortionContrib = settings_json.getFloat(
            "HorzDistortionContrib", noise_setting.HorzDistortionContrib);
    }

    /**
     * @brief Parses the input json string and fills a GimbalSettings
     * object with the suitable parameter values.
     *
     * @param settings_json
     * @return GimbalSetting
     */
    static GimbalSetting createGimbalSetting(const Settings& settings_json)
    {
        GimbalSetting gimbal;
        // capture_setting.gimbal.is_world_frame =
        // settings_json.getBool("IsWorldFrame", false);
        gimbal.stabilization = settings_json.getFloat("Stabilization", false);
        gimbal.rotation = createRotationSetting(settings_json, gimbal.rotation);
        return gimbal;
    }

    /**
     * @brief Parses the input json string and fills a CameraSettings
     * object with the suitable parameter values.
     *
     * @param settings_json
     * @return CameraSetting
     */
    static CameraSetting createCameraSetting(const Settings& settings_json)
    {
        CameraSetting setting;

        setting.position = createVectorSetting(settings_json, setting.position);
        setting.rotation =
            createRotationSetting(settings_json, setting.rotation);

        loadCaptureSettings(settings_json, setting.capture_settings);
        loadNoiseSettings(settings_json, setting.noise_settings);
        Settings json_gimbal;
        if (settings_json.getChild("Gimbal", json_gimbal))
            setting.gimbal = createGimbalSetting(json_gimbal);

        setting.attach_link = settings_json.getString("AttachLink", "");

        return setting;
    }

    /**
     * @brief Parses the input json string and fills a CameraSettings
     * object for each of the camera objects described in the parameter file.
     *
     * @param settings_json
     * @param cameras
     */
    static void
    loadCameraSettings(const Settings& settings_json,
                       std::map<std::string, CameraSetting>& cameras)
    {
        cameras.clear();

        Settings json_parent;
        if (settings_json.getChild("Cameras", json_parent))
        {
            std::vector<std::string> keys;
            json_parent.getChildNames(keys);

            for (const auto& key : keys)
            {
                RobotSim::Settings child;
                json_parent.getChild(key, child);
                cameras[key] = createCameraSetting(child);
            }
        }
    }

    /**
     * @brief Parses the input json string and fills a ControlSetting
     * object with the suitable parameter values.
     *
     * @param settings_json
     * @return ControlSetting
     */
    static ControlSetting createControlSetting(const Settings& settings_json)
    {
        ControlSetting setting;

        setting.LeftWheelJoint = settings_json.getString("LeftWheelJoint", "");
        setting.RightWheelJoint =
            settings_json.getString("RightWheelJoint", "");
        setting.EndEffectorLink =
            settings_json.getString("EndEffectorLink", "");
        setting.ManipulatorJoints =
            settings_json.getStringArray("ManipulatorJoints");

        return setting;
    }

    /**
     * @brief Parses the input json string and fills a ActuationSetting
     * object with the suitable parameter values.
     *
     * @param settings_json
     * @return ActuationSetting
     */
    static void createActuationSetting(const RobotSim::Settings& settings_json,
                                       ActuationSetting& actuation_setting)
    {
        actuation_setting.gearRatio =
            settings_json.getFloat("GearRatio", actuation_setting.gearRatio);
        actuation_setting.motorVelocityConstant = settings_json.getFloat(
            "MotorVelocityConstant", actuation_setting.motorVelocityConstant);
        actuation_setting.controlDeadZone = settings_json.getFloat(
            "ControlDeadZone", actuation_setting.controlDeadZone);
        actuation_setting.motorTorqueMax = settings_json.getFloat(
            "MaximumMotorTorque", actuation_setting.motorTorqueMax);
        actuation_setting.electricalResistance = settings_json.getFloat(
            "WindingResistance", actuation_setting.electricalResistance);
        actuation_setting.electricalInductance = settings_json.getFloat(
            "WindingInductance", actuation_setting.electricalInductance);
        actuation_setting.actionScale = settings_json.getFloat(
            "ActionScaling", actuation_setting.actionScale);
        actuation_setting.batteryVoltage = settings_json.getFloat(
            "BatteryVoltage", actuation_setting.batteryVoltage);
    }

    /**
     * @brief Parses the input json string and fills a CaptureSetting
     * object with the suitable parameter values.
     *
     * @param settings_json
     * @param capture_setting
     */
    static void createCaptureSettings(const RobotSim::Settings& settings_json,
                                      CaptureSetting& capture_setting)
    {
        capture_setting.width =
            settings_json.getInt("Width", capture_setting.width);
        capture_setting.height =
            settings_json.getInt("Height", capture_setting.height);
        capture_setting.fov_degrees =
            settings_json.getFloat("FOV_Degrees", capture_setting.fov_degrees);
        capture_setting.auto_exposure_speed = settings_json.getFloat(
            "AutoExposureSpeed", capture_setting.auto_exposure_speed);
        capture_setting.auto_exposure_bias = settings_json.getFloat(
            "AutoExposureBias", capture_setting.auto_exposure_bias);
        capture_setting.auto_exposure_max_brightness = settings_json.getFloat(
            "AutoExposureMaxBrightness",
            capture_setting.auto_exposure_max_brightness);
        capture_setting.auto_exposure_min_brightness = settings_json.getFloat(
            "AutoExposureMinBrightness",
            capture_setting.auto_exposure_min_brightness);
        capture_setting.motion_blur_amount = settings_json.getFloat(
            "MotionBlurAmount", capture_setting.motion_blur_amount);
        capture_setting.image_type = settings_json.getInt("ImageType", 0);
        capture_setting.target_gamma = settings_json.getFloat(
            "TargetGamma", capture_setting.image_type == 0
                               ? CaptureSetting::kSceneTargetGamma
                               : Utils::nan<float>());

        std::string projection_mode =
            Utils::toLower(settings_json.getString("ProjectionMode", ""));
        if (projection_mode == "" || projection_mode == "perspective")
            capture_setting.projection_mode = 0; // Perspective
        else if (projection_mode == "orthographic")
            capture_setting.projection_mode = 1; // Orthographic
        else
            throw std::invalid_argument(
                std::string("CaptureSettings projection_mode has invalid value "
                            "in settings_json ") +
                projection_mode);

        capture_setting.ortho_width =
            settings_json.getFloat("OrthoWidth", capture_setting.ortho_width);
    }

    /**
     * @brief Parses the input json string and fills a vector of
     * SubwindowSetting objects with the suitable parameter values.
     *
     * @param settings_json
     * @param subwindow_settings
     */
    static void
    loadSubWindowsSettings(const Settings& settings_json,
                           std::vector<SubwindowSetting>& subwindow_settings)
    {
        // load default subwindows
        initializeSubwindowSettings(subwindow_settings);

        Settings json_parent;
        if (settings_json.getChild("SubWindows", json_parent))
        {
            for (size_t child_index = 0; child_index < json_parent.size();
                 ++child_index)
            {
                Settings json_settings_child;
                if (json_parent.getChild(child_index, json_settings_child))
                {
                    int window_index =
                        json_settings_child.getInt("WindowID", 0);
                    SubwindowSetting& subwindow_setting =
                        subwindow_settings.at(window_index);
                    subwindow_setting.window_index = window_index;
                    subwindow_setting.image_type = Utils::toEnum<ImageType>(
                        json_settings_child.getInt("ImageType", 0));
                    subwindow_setting.visible =
                        json_settings_child.getBool("Visible", false);
                    subwindow_setting.camera_name =
                        getCameraName(json_settings_child);
                }
            }
        }
    }

    /**
     * @brief
     *
     * @param subwindow_settings
     */
    static void initializeSubwindowSettings(
        std::vector<SubwindowSetting>& subwindow_settings)
    {
        subwindow_settings.clear();
        subwindow_settings.push_back(
            SubwindowSetting(0, ImageType::DepthVis, false, "")); // depth
        subwindow_settings.push_back(
            SubwindowSetting(0, ImageType::Segmentation, false, "")); // seg
        subwindow_settings.push_back(
            SubwindowSetting(0, ImageType::Scene, false, "")); // vis
    }

    /**
     * @brief Loads default camera parameters
     *
     * @param settings_json
     * @param camera_defaults
     */
    static void loadDefaultCameraSetting(const Settings& settings_json,
                                         CameraSetting& camera_defaults)
    {
        Settings child_json;
        if (settings_json.getChild("CameraDefaults", child_json))
        {
            UE_LOG(LogTemp, Warning, TEXT("loadDefaultCameraSetting!"));
            camera_defaults = createCameraSetting(child_json);
        }
    }

    /**
     * @brief
     *
     * @param settings_json
     * @param camera_director
     * @param simmode_name
     */
    static void
    loadCameraDirectorSetting(const Settings& settings_json,
                              CameraDirectorSetting& camera_director,
                              const std::string& simmode_name)
    {
        camera_director = CameraDirectorSetting();

        if (std::isnan(camera_director.follow_distance))
            camera_director.follow_distance = -3;
        if (std::isnan(camera_director.position.x()))
            camera_director.position.x() = camera_director.follow_distance;
        if (std::isnan(camera_director.position.y()))
            camera_director.position.y() = 0;
        if (std::isnan(camera_director.position.z()))
            camera_director.position.z() = -2;
    }

    /**
     * @brief
     *
     * @param settings_json
     */
    void loadClockSettings(const Settings& settings_json)
    {
        clock_type = settings_json.getString("ClockType", "");

        if (clock_type == "")
        {
            // default value
            clock_type = "ScalableClock";

            // override if multirotor simmode with simple_flight
            if (simmode_name == "Multirotor")
            {

                clock_type = "SteppableClock";
            }
        }

        clock_speed = settings_json.getFloat("ClockSpeed", 1.0f);
    }
};

} // namespace RobotSim

#endif // __ROBOTSIMSETTINGS_H__