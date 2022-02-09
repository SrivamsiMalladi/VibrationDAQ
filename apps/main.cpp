/* Copyright (c) 2020, Jonas Lauener & Wingtra AG
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#include <iostream>
#include <vibration_daq/VibrationSensorModule.hpp>
#include <vibration_daq/utils/HexUtils.hpp>
#include <filesystem>
#include <vibration_daq/ConfigModule.hpp>
#include "loguru/loguru.hpp"
#include <fstream>
#include <date/date.h>
#include <chrono>
#include <date/tz.h>
#include <vibration_daq/StorageModule.hpp>
#include "chrono"
#include "thread"
//#include "yaml-cpp/yaml.h"
#include "date/date.h"

using namespace vibration_daq;
using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds

static const int SPI_SPEED = 14000000;
gpio_t *gpioTrigger;
gpio_t *gpioStatusLed;

std::vector<VibrationSensorModule> vibrationSensorModules;
ConfigModule configModule;
StorageModule storageModule;

bool setupVibrationSensorModules(const bool &externalTriggerActivated);

system_clock::time_point triggerVibrationSensors(const bool &externalTrigger);

int main(int argc, char *argv[]) {
    loguru::g_preamble_uptime = false;
    loguru::g_preamble_thread = false;
    loguru::init(argc, argv);

    std::string configFilePath = "";
    if (argc > 1) {
        configFilePath = argv[1];
    } else {
        LOG_S(ERROR) << "No config file as program argument specified!";
        return EXIT_FAILURE;
    }

    LOG_S(INFO) << "Loading from config file path: " << configFilePath;
    // if (!configModule.setup(configFilePath)) {
    //     LOG_S(ERROR) << "Could not setup ConfigModule.";
    //     return EXIT_FAILURE;
    // }

    bool externalTriggerActivated = false;
    int externalTriggerPin = -1;
    // if (!configModule.readExternalTriggerConfig(externalTriggerActivated, externalTriggerPin)) {
    //     LOG_S(ERROR) << "Could not retrieve externalTrigger config.";
    //     return EXIT_FAILURE;
    // }
    if (externalTriggerActivated) {
        gpioTrigger = gpio_new();
        if (gpio_open(gpioTrigger, "/dev/gpiochip0", externalTriggerPin, GPIO_DIR_OUT_LOW) < 0) {
            LOG_F(ERROR, "gpio_open(): %s", gpio_errmsg(gpioTrigger));
            return EXIT_FAILURE;
        }
    }

    bool statusLedActivated = false;
    int statusLedPin = -1;
    // if (!configModule.readStatusLedConfig(statusLedActivated, statusLedPin)) {
    //     LOG_S(ERROR) << "Could not retrieve externalTrigger config.";
    //     return EXIT_FAILURE;
    // }
    if (statusLedActivated) {
        gpioStatusLed = gpio_new();
        if (gpio_open(gpioStatusLed, "/dev/gpiochip0", statusLedPin, GPIO_DIR_OUT_LOW) < 0) {
            LOG_F(ERROR, "gpio_open(): %s", gpio_errmsg(gpioStatusLed));
            return EXIT_FAILURE;
        }
    }


    if (!setupVibrationSensorModules(externalTriggerActivated)) {
        return EXIT_FAILURE;
    }

    std::string storageDirectoryPath;
    if (!configModule.readStorageDirectoryPath(storageDirectoryPath)) {
        LOG_S(ERROR) << "Could not retrieve storage_directory from config.";
        return EXIT_FAILURE;
    }
    if (!storageModule.setup({storageDirectoryPath})) {
        LOG_S(ERROR) << "Could not setup StorageModule.";
        return EXIT_FAILURE;
    }

    int recordingsCount;
    if (!configModule.readRecordingsCount(recordingsCount)) {
        recordingsCount = 1;
    }

    if (statusLedActivated && gpio_write(gpioStatusLed, true) < 0) {
        fprintf(stderr, "gpio_write(): %s", gpio_errmsg(gpioStatusLed));
        exit(1);
    }

    // run indefinitely if recordingsCount == 0
    for (int i = 0; i < recordingsCount || recordingsCount == 0; ++i) {
        system_clock::time_point triggerTime = triggerVibrationSensors(externalTriggerActivated);

        for (const auto &vibrationSensorModule : vibrationSensorModules) {
            auto vibrationData = vibrationSensorModule.retrieveVibrationData();

            if (statusLedActivated && gpio_write(gpioStatusLed, false) < 0) {
                fprintf(stderr, "gpio_write(): %s", gpio_errmsg(gpioStatusLed));
                exit(1);
            }

            bool storedVibrationData = storageModule.storeVibrationData(vibrationData,
                                                                        vibrationSensorModule.getSensorName(),
                                                                        triggerTime);

            if (statusLedActivated && gpio_write(gpioStatusLed, true) < 0) {
                fprintf(stderr, "gpio_write(): %s", gpio_errmsg(gpioStatusLed));
                exit(1);
            }

            LOG_IF_F(ERROR, !storedVibrationData, "Could not store vibration data.");
        }


    }

    for (auto &vibrationSensorModule : vibrationSensorModules) {
        vibrationSensorModule.close();
    }

    if (externalTriggerActivated) {
        gpio_close(gpioTrigger);
        gpio_free(gpioTrigger);
    }

    if (statusLedActivated) {
        if (gpio_write(gpioStatusLed, false) < 0) {
            fprintf(stderr, "gpio_write(): %s", gpio_errmsg(gpioStatusLed));
            exit(1);
        }

        gpio_close(gpioStatusLed);
        gpio_free(gpioStatusLed);
    }

    return EXIT_SUCCESS;
}

system_clock::time_point triggerVibrationSensors(const bool &externalTrigger) {
    system_clock::time_point triggerTime;
    if (externalTrigger) {
        if (gpio_write(gpioTrigger, true) < 0) {
            fprintf(stderr, "gpio_write(): %s", gpio_errmsg(gpioTrigger));
            exit(1);
        }

        triggerTime = system_clock::now();
        LOG_S(INFO) << "Triggered over PIN.";
        sleep_for(5ms);

        if (gpio_write(gpioTrigger, false) < 0) {
            fprintf(stderr, "gpio_write(): %s", gpio_errmsg(gpioTrigger));
            exit(1);
        }
    } else {
        for (const auto &vibrationSensorModule : vibrationSensorModules) {
            // start recording
            LOG_S(INFO) << vibrationSensorModule.getSensorName() << " triggered over SPI.";
            vibrationSensorModule.triggerRecording();
        }
        triggerTime = system_clock::now();
    }

    return triggerTime;
}

bool setupVibrationSensorModules(const bool &externalTriggerActivated) {
    std::vector<VibrationSensorConfig> vibrationSensorConfigs;
    VibrationSensorConfig vibrationSensor;
        
    vibrationSensor.name = "sensor1";
    vibrationSensor.busyPin = 22;
    vibrationSensor.resetPin = 27;
    vibrationSensor.spiPath = "/dev/spidev0.0";

    std::string recordingModeString = "MFFT";
    if (!Enum::convert(recordingModeString, vibrationSensor.recordingMode)) {
        LOG_S(WARNING) << "could not convert decimation_factor to enum: " << recordingModeString;
        return false;
    }

    switch (vibrationSensor.recordingMode) {
        case RecordingMode::MFFT:
            std::string decimationFactorString = "Factor_2";
            if (!Enum::convert(decimationFactorString, mfftConfig.decimationFactor)) {
                LOG_S(WARNING) << "could not convert decimation_factor to enum: " << decimationFactorString;
                return false;
            }

            std::string firFilterString="NO_FILTER";
            if (!Enum::convert(firFilterString, mfftConfig.firFilter)) {
                LOG_S(WARNING) << "could not convert fir_filter to enum: " << firFilterString;
                return false;
            }

            if (recordingConfig.firFilter == FIRFilter::CUSTOM) {
                LOG_S(WARNING) << "could not read custom_filter_taps from config";
                return false;
            }

            mfftConfig.spectralAvgCount = 2;
            if (mfftConfig.spectralAvgCount < 1 || mfftConfig.spectralAvgCount > 255) {
                LOG_S(WARNING) << "spectral_avg_count is not in range (1-255): " << mfftConfig.spectralAvgCount;
                return false;
            }

            std::string windowSettingString = "HANNING";
            if (!Enum::convert(windowSettingString, mfftConfig.windowSetting)) {
                LOG_S(WARNING) << "could not convert window_setting to enum: " << windowSettingString;
                return false;
            }
            
            return true;
        case RecordingMode::MTC:
            LOG_S(WARNING) << "could not read MTC_config from config";
            return false;
            //return true;
        case RecordingMode::AFFT:
        case RecordingMode::RTS:
            //TODO
            if (!readRTSConfig(node["RTS_config"], vibrationSensor.rtsConfig)) {
                LOG_S(WARNING) << "could not read MTC_config from config";
                return false;
            }
            return true;
        default:
            LOG_S(WARNING) << "only MFFT supported.";
            return false;
    }
    vibrationSensorConfigs.push_back(vibrationSensor);


    for (const auto &vibrationSensorConfig : vibrationSensorConfigs) {
        VibrationSensorModule vibrationSensorModule(vibrationSensorConfig.name);
        if (!vibrationSensorModule.setup(vibrationSensorConfig.resetPin, vibrationSensorConfig.busyPin,
                                         vibrationSensorConfig.spiPath,
                                         SPI_SPEED)) {
            LOG_S(ERROR) << "Could not setup vibration sensor: " << vibrationSensorConfig.name;
            return false;
        }

        if (externalTriggerActivated) {
            vibrationSensorModule.activateExternalTrigger();
        }

//        vibrationSensorModule.triggerAutonull();
//        vibrationSensorModule.restoreFactorySettings();

        switch (vibrationSensorConfig.recordingMode) {
            case RecordingMode::MFFT:
                vibrationSensorModule.activateMode(vibrationSensorConfig.mfftConfig);
                break;
            case RecordingMode::MTC:
                vibrationSensorModule.activateMode(vibrationSensorConfig.mtcConfig);
                break;
            case RecordingMode::RTC:
                vibrationSensorModule.activateMode(vibrationSensorConfig.rtcConfig);
                break;
        }

        vibrationSensorModules.push_back(vibrationSensorModule);
        LOG_S(INFO) << vibrationSensorModule.getSensorName() << " setup done";
    }

    return true;
}
