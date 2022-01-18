// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

//#include "CoreMinimal.h"

#ifndef msr_airlib_settings_hpp
#define msr_airlib_settings_hpp

#include "common_utils/Utils.hpp"

STRICT_MODE_OFF
// this json library is not strict clean
// TODO: HACK!! below are added temporariliy because something is defining min,
// max macros #undef max
#undef min
#include "common_utils/json.hpp"
STRICT_MODE_ON

#include <string>
#include <mutex>
#include "common_utils/FileSystem.hpp"

/**
 *
 */
namespace RobotSim
{

class Settings
{
private:
    std::string full_filepath_;
    nlohmann::json doc_;
    bool load_success_ = false;

private:
    static std::mutex& getFileAccessMutex()
    {
        static std::mutex file_access;
        return file_access;
    }

public:
    static Settings& singleton()
    {
        static Settings instance;
        return instance;
    }

    std::string getFullFilePath()
    {
        return full_filepath_;
    }

    static std::string getUserDirectoryFullPath(std::string fileName)
    {
        std::string path = common_utils::FileSystem::getAppDataFolder();
        return common_utils::FileSystem::combine(path, fileName);
    }

    static std::string getPorjectDirectoryFullPath(std::string fileName)
    {
        FString path = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir());
        return common_utils::FileSystem::combine(TCHAR_TO_UTF8(*path),
                                                 fileName);
    }

    static std::string getPluginDirectoryFullPath(std::string fileName)
    {
        FString path =
            FPaths::ConvertRelativePathToFull(FPaths::ProjectPluginsDir());
        return common_utils::FileSystem::combine(TCHAR_TO_UTF8(*path),
                                                 fileName);
    }

    static std::string getExecutableFullPath(std::string fileName)
    {
        std::string path = common_utils::FileSystem::getExecutableFolder();
        return common_utils::FileSystem::combine(path, fileName);
    }
    static std::string getLaunchDir(std::string fileName)
    {
        FString path = FPaths::ConvertRelativePathToFull(FPaths::LaunchDir());
        return common_utils::FileSystem::combine(TCHAR_TO_UTF8(*path),
                                                 fileName);
    }
    static std::string getAnyPossiblePath(std::string fileName)
    {
        // check if absolute path
        if (FPaths::FileExists(FString(fileName.c_str())))
        {
            return fileName;
        }
        // check if relative path from launch directory
        std::string fileRelativeToLaunchDir = getLaunchDir(fileName);
        if (FPaths::FileExists(FString(fileRelativeToLaunchDir.c_str())))
        {
            return fileRelativeToLaunchDir;
        }
        std::string fileInSetting("setting/" + fileName);
        // check if in <user directory>/setting/
        std::string fileInUserDir = getUserDirectoryFullPath(fileInSetting);
        if (FPaths::FileExists(FString(fileInUserDir.c_str())))
        {
            return fileInUserDir;
        }
        // check if in <launch dir>/setting/ (for standalone executable)
        std::string fileInLaunchSettingDir = getLaunchDir(fileInSetting);
        if (FPaths::FileExists(FString(fileInLaunchSettingDir.c_str())))
        {
            return fileInLaunchSettingDir;
        }
        // check if in <path/to/ue4project>/Plugins/RobotSim/setting/
        std::string fileInPluginDir = getPluginDirectoryFullPath(
            common_utils::FileSystem::getProductFolderName() + "/" +
            fileInSetting);
        if (FPaths::FileExists(FString(fileInPluginDir.c_str())))
        {
            return fileInPluginDir;
        }
        // if nothing match, return original fileName
        return fileName;
    }

    static Settings& loadJSonString(const std::string& json_str)
    {
        singleton().full_filepath_ = "";
        singleton().load_success_ = false;

        if (json_str.length() > 0)
        {
            std::stringstream ss;
            ss << json_str;
            ss >> singleton().doc_;
            singleton().load_success_ = true;
        }

        return singleton();
    }
    std::string saveJSonString()
    {
        std::lock_guard<std::mutex> guard(getFileAccessMutex());
        std::stringstream ss;
        ss << std::setw(2) << singleton().doc_ << std::endl;

        return ss.str();
    }

    static Settings& loadJSonFile(std::string full_filepath)
    {
        std::lock_guard<std::mutex> guard(getFileAccessMutex());
        singleton().full_filepath_ = full_filepath;

        singleton().load_success_ = false;

        std::ifstream s;
        common_utils::FileSystem::openTextFile(full_filepath, s);
        if (!s.fail())
        {
            s >> singleton().doc_;
            singleton().load_success_ = true;
        }

        return singleton();
    }

    bool isLoadSuccess()
    {
        return load_success_;
    }

    bool hasFileName()
    {
        return !getFullFilePath().empty();
    }

    void saveJSonFile(std::string full_filepath)
    {
        std::lock_guard<std::mutex> guard(getFileAccessMutex());
        singleton().full_filepath_ = full_filepath;
        std::ofstream s;
        common_utils::FileSystem::createTextFile(full_filepath, s);
        s << std::setw(2) << doc_ << std::endl;
    }

    bool getChild(const std::string& name, Settings& child) const
    {
        if (doc_.count(name) == 1 &&
            (doc_[name].type() == nlohmann::detail::value_t::object ||
             doc_[name].type() == nlohmann::detail::value_t::array))
        {
            child.doc_ = doc_[name].get<nlohmann::json>();
            return true;
        }
        return false;
    }

    size_t size() const
    {
        return doc_.size();
    }

    template <typename Container> void getChildNames(Container& c) const
    {
        for (auto it = doc_.begin(); it != doc_.end(); ++it)
        {
            c.push_back(it.key());
        }
    }

    bool getChild(size_t index, Settings& child) const
    {
        if (doc_.size() > index &&
            (doc_[index].type() == nlohmann::detail::value_t::object ||
             doc_[index].type() == nlohmann::detail::value_t::array))
        {

            child.doc_ = doc_[index].get<nlohmann::json>();
            return true;
        }
        return false;
    }

    std::string getString(const std::string& name,
                          std::string defaultValue) const
    {
        if (doc_.count(name) == 1)
        {
            return doc_[name].get<std::string>();
        }
        else
        {
            return defaultValue;
        }
    }

    double getDouble(const std::string& name, double defaultValue) const
    {
        if (doc_.count(name) == 1)
        {
            return doc_[name].get<double>();
        }
        else
        {
            return defaultValue;
        }
    }

    float getFloat(const std::string& name, float defaultValue) const
    {
        if (doc_.count(name) == 1)
        {
            return doc_[name].get<float>();
        }
        else
        {
            return defaultValue;
        }
    }

    bool getBool(const std::string& name, bool defaultValue) const
    {
        if (doc_.count(name) == 1)
        {
            return doc_[name].get<bool>();
        }
        else
        {
            return defaultValue;
        }
    }

    std::vector<std::map<std::string, std::string>>
    getArrayOfKeyValuePairs(const std::string& name,
                            const std::vector<std::string> keys) const
    {
        std::vector<std::map<std::string, std::string>> return_value;
        if (doc_.count(name) == 1 && doc_[name].is_array())
        {
            auto arr = doc_[name].get<nlohmann::json::array_t>();
            for (size_t i = 0; i < arr.size(); i++)
            {
                std::map<std::string, std::string> kvp;
                auto obj = arr[i].get<nlohmann::json::object_t>();
                for (size_t j = 0; j < keys.size(); j++)
                {
                    auto val = obj[keys[j]];
                    kvp[keys[j]] = val.get<std::string>();
                }
                return_value.emplace_back(kvp);
            }
        }
        return return_value;
    }

    std::vector<std::string> getStringArray(const std::string& name) const
    {
        std::vector<std::string> return_value;
        if (doc_.count(name) == 1 && doc_[name].is_array())
        {
            auto arr = doc_[name].get<nlohmann::json::array_t>();
            for (size_t i = 0; i < arr.size(); i++)
            {
                auto obj = arr[i].get<std::string>();
                // auto obj2 = arr[i].get<nlohmann::json::object_t>();
                return_value.emplace_back(obj);
            }
        }
        return return_value;
    }

    bool hasKey(const std::string& key) const
    {
        return doc_.find(key) != doc_.end();
    }

    int getInt(const std::string& name, int defaultValue) const
    {
        if (doc_.count(name) == 1)
        {
            return doc_[name].get<int>();
        }
        else
        {
            return defaultValue;
        }
    }

    bool setString(const std::string& name, std::string value)
    {
        if (doc_.count(name) != 1 ||
            doc_[name].type() != nlohmann::detail::value_t::string ||
            doc_[name] != value)
        {
            doc_[name] = value;
            return true;
        }
        return false;
    }
    bool setDouble(const std::string& name, double value)
    {
        if (doc_.count(name) != 1 ||
            doc_[name].type() != nlohmann::detail::value_t::number_float ||
            static_cast<double>(doc_[name]) != value)
        {
            doc_[name] = value;
            return true;
        }
        return false;
    }
    bool setBool(const std::string& name, bool value)
    {
        if (doc_.count(name) != 1 ||
            doc_[name].type() != nlohmann::detail::value_t::boolean ||
            static_cast<bool>(doc_[name]) != value)
        {
            doc_[name] = value;
            return true;
        }
        return false;
    }
    bool setInt(const std::string& name, int value)
    {
        if (doc_.count(name) != 1 ||
            doc_[name].type() != nlohmann::detail::value_t::number_integer ||
            static_cast<int>(doc_[name]) != value)
        {
            doc_[name] = value;
            return true;
        }
        return false;
    }

    void setChild(const std::string& name, Settings& value)
    {
        doc_[name] = value.doc_;
    }
};
} // namespace RobotSim
#endif
