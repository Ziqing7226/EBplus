/**********************************************************************************************************************
 * Copyright (c) Prophesee S.A.                                                                                       *
 *                                                                                                                    *
 * Licensed under the Apache License, Version 2.0 (the "License");                                                    *
 * you may not use this file except in compliance with the License.                                                   *
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0                                 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed   *
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.                      *
 * See the License for the specific language governing permissions and limitations under the License.                 *
 **********************************************************************************************************************/

#ifndef METAVISION_HAL_GEN31_CCAM5_TZ_DEVICE_H
#define METAVISION_HAL_GEN31_CCAM5_TZ_DEVICE_H

#include "metavision/psee_hw_layer/devices/treuzell/tz_regmap_device.h"
#include "metavision/psee_hw_layer/devices/treuzell/tz_psee_fpga_device.h"
#include "devices/treuzell/tz_issd_device.h"
#include "metavision/psee_hw_layer/devices/treuzell/tz_main_device.h"
#include "metavision/psee_hw_layer/facilities/tz_monitoring.h"

namespace Metavision {

class DeviceBuilder;
class DeviceBuilderParameters;
class DeviceConfig;
class TzLibUSBBoardCommand;

class TzCcam5Gen31 : public TzPseeFpgaDevice, public TzIssdDevice, public TzMainDevice, public IlluminationProvider {
public:
    TzCcam5Gen31(std::shared_ptr<TzLibUSBBoardCommand> cmd, uint32_t dev_id, std::shared_ptr<TzDevice> parent);
    virtual ~TzCcam5Gen31();
    static std::shared_ptr<TzDevice> build(std::shared_ptr<TzLibUSBBoardCommand> cmd, uint32_t dev_id,
                                           std::shared_ptr<TzDevice> parent);

    virtual std::list<StreamFormat> get_supported_formats() const override;
    StreamFormat get_output_format() const override;
    long get_system_id() const override;
    bool set_mode_standalone() override;
    bool set_mode_master() override;
    bool set_mode_slave() override;
    I_CameraSynchronization::SyncMode get_mode() const override;
    virtual I_HW_Identification::SensorInfo get_sensor_info() override {
        return {3, 1, "Gen3.1"};
    }
    virtual int get_illumination() override;

    // TzPseeFpgaDevice and TzIssdDevice both derive from TzDevice; the
    // compiler already resolves these via dominance - the explicit
    // delegating overrides only spell that out (silences C4250). Note the
    // split: TzIssdDevice overrides the lifecycle hooks, TzPseeFpgaDevice
    // overrides get_device_info (adding the FPGA system ID / version
    // entries) - each delegation targets the base that actually overrides.
    void start() override { TzIssdDevice::start(); }
    void stop() override { TzIssdDevice::stop(); }
    void initialize() override { TzIssdDevice::initialize(); }
    void destroy() override { TzIssdDevice::destroy(); }
    void get_device_info(I_HW_Identification::SystemInfo &info, std::string prefix) override {
        TzPseeFpgaDevice::get_device_info(info, prefix);
    }

    /// get the type of sensor
    /// 0x90100402h uniform TD feedback PPD VGA
    /// 0x90100403h uniform EM HVGA
    long long get_sensor_id();

protected:
    void spawn_facilities(DeviceBuilder &device_builder, const DeviceConfig &device_config) override;

private:
    I_CameraSynchronization::SyncMode sync_mode_;
};

} // namespace Metavision

#endif // METAVISION_HAL_GEN31_CCAM5_TZ_DEVICE_H
