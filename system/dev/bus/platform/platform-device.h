// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddktl/device.h>
#include <ddktl/protocol/platform-device.h>
#include <fbl/vector.h>

typedef struct pdev_req pdev_req_t;

namespace platform_bus {

class PlatformBus;

class PlatformDevice;
using PlatformDeviceType = ddk::Device<PlatformDevice, ddk::GetProtocolable, ddk::Rxrpcable>;

// This class represents a platform device attached to the platform bus.
class PlatformDevice : public PlatformDeviceType, public ddk::PlatformDevProtocol<PlatformDevice> {
public:
    explicit PlatformDevice(zx_device_t* parent, PlatformBus* bus, uint32_t flags);
    zx_status_t Init(const pbus_dev_t* pdev);

    inline uint32_t vid() const { return vid_; }
    inline uint32_t pid() const { return pid_; }
    inline uint32_t did() const { return did_; }

    // Device protocol implementation.
    zx_status_t DdkGetProtocol(uint32_t proto_id, void* out);
    void DdkRelease();
    zx_status_t DdkRxrpc(zx_handle_t channel);

    // Platform device protocol implementation,
    zx_status_t MapMmio(uint32_t index, uint32_t cache_policy, void** out_vaddr, size_t* out_size,
                        zx_paddr_t* out_paddr, zx_handle_t* out_handle);
    zx_status_t MapInterrupt(uint32_t index, uint32_t flags, zx_handle_t* out_handle);
    zx_status_t GetBti(uint32_t index, zx_handle_t* out_handle);
    zx_status_t GetDeviceInfo(pdev_device_info_t* out_info);

    zx_status_t Enable(bool enable);

private:
    zx_status_t RpcGetMmio(uint32_t index, zx_paddr_t* out_paddr, size_t *out_length,
                           zx_handle_t* out_handle, uint32_t* out_handle_count);
    zx_status_t RpcGetInterrupt(uint32_t index, uint32_t* out_irq, uint32_t* out_mode,
                                zx_handle_t* out_handle, uint32_t* out_handle_count);
    zx_status_t RpcGetBti(uint32_t index, zx_handle_t* out_handle, uint32_t* out_handle_count);
    zx_status_t RpcUmsSetMode(usb_mode_t mode);
    zx_status_t RpcGpioConfig(uint32_t index, uint32_t flags);
    zx_status_t RpcGpioSetAltFunction(uint32_t index, uint64_t function);
    zx_status_t RpcGpioRead(uint32_t index, uint8_t* out_value);
    zx_status_t RpcGpioWrite(uint32_t index, uint8_t value);
    zx_status_t RpcGpioGetInterrupt(uint32_t index, uint32_t flags, zx_handle_t* out_handle,
                                    uint32_t* out_handle_count);
    zx_status_t RpcGpioReleaseInterrupt(uint32_t index);
    zx_status_t RpcGpioSetPolarity(uint32_t index, uint32_t flags);
    zx_status_t RpcCanvasConfig(zx_handle_t vmo, size_t offset, canvas_info_t* info, uint8_t* canvas_idx);
    zx_status_t RpcCanvasFree(uint8_t canvas_idx);
    zx_status_t RpcScpiGetSensor(char* name, uint32_t *sensor_id);
    zx_status_t RpcScpiGetSensorValue(uint32_t sensor_id, uint32_t* sensor_value);
    zx_status_t RpcScpiGetDvfsInfo(uint8_t power_domain, scpi_opp_t* opps);
    zx_status_t RpcScpiGetDvfsIdx(uint8_t power_domain, uint16_t* idx);
    zx_status_t RpcScpiSetDvfsIdx(uint8_t power_domain, uint16_t idx);
    zx_status_t RpcI2cTransact(pdev_req_t* req, uint8_t* data, zx_handle_t channel);
    zx_status_t RpcI2cGetMaxTransferSize(uint32_t index, size_t* out_size);
    zx_status_t RpcClkEnable(uint32_t index);
    zx_status_t RpcDisable(uint32_t index);

    zx_status_t AddMetaData(uint32_t index);

private:
    PlatformBus* bus_;
    char name_[ZX_DEVICE_NAME_MAX + 1];
    uint32_t flags_;
    uint32_t vid_;
    uint32_t pid_;
    uint32_t did_;
    serial_port_info_t serial_port_info_;
    bool enabled_;

    fbl::Vector<pbus_mmio_t> mmios_;
    fbl::Vector<pbus_irq_t> irqs_;
    fbl::Vector<pbus_gpio_t> gpios_;
    fbl::Vector<pbus_i2c_channel_t> i2c_channels_;
    fbl::Vector<pbus_clk_t> clks_;
    fbl::Vector<pbus_bti_t> btis_;
    fbl::Vector<pbus_metadata_t> metadata_;
};

} // namespace platform_bus
