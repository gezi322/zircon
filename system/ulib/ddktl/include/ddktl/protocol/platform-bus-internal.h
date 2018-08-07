// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <fbl/type_support.h>

namespace ddk {
namespace internal {

DECLARE_HAS_MEMBER_FN_WITH_SIGNATURE(has_pbus_set_protocol, SetProtocol,
        zx_status_t (C::*)(uint32_t proto_id, void* protocol));
DECLARE_HAS_MEMBER_FN_WITH_SIGNATURE(has_pbus_wait_protocol, WaitProtocol,
        zx_status_t (C::*)(uint32_t proto_id));
DECLARE_HAS_MEMBER_FN_WITH_SIGNATURE(has_pbus_device_add, DeviceAdd,
        zx_status_t (C::*)(const pbus_dev_t* dev, uint32_t flags));
DECLARE_HAS_MEMBER_FN_WITH_SIGNATURE(has_pbus_device_enable, DeviceEnable,
        zx_status_t (C::*)(uint32_t vid, uint32_t pid, uint32_t did, bool enable));
DECLARE_HAS_MEMBER_FN_WITH_SIGNATURE(has_pbus_get_board_name, GetBoardName,
        const char* (C::*)());

template <typename D>
constexpr void CheckPlatformBusProtocolSubclass() {
    static_assert(internal::has_pbus_set_protocol<D>::value,
                  "PlatformBusProtocol subclasses must implement "
                  "MapMmio(uint32_t index, uint32_t cache_policy, void** out_vaddr, "
                  "size_t* out_size, zx_paddr_t* out_paddr, zx_handle_t* out_handle)");
    static_assert(internal::has_pbus_wait_protocol<D>::value,
                  "PlatformBusProtocol subclasses must implement "
                  "MapMmio(uint32_t index, uint32_t cache_policy, void** out_vaddr, "
                  "size_t* out_size, zx_paddr_t* out_paddr, zx_handle_t* out_handle)");
    static_assert(internal::has_pbus_device_add<D>::value,
                  "PlatformBusProtocol subclasses must implement "
                  "MapMmio(uint32_t index, uint32_t cache_policy, void** out_vaddr, "
                  "size_t* out_size, zx_paddr_t* out_paddr, zx_handle_t* out_handle)");
    static_assert(internal::has_pbus_device_enable<D>::value,
                  "PlatformBusProtocol subclasses must implement "
                  "MapMmio(uint32_t index, uint32_t cache_policy, void** out_vaddr, "
                  "size_t* out_size, zx_paddr_t* out_paddr, zx_handle_t* out_handle)");
    static_assert(internal::has_pbus_get_board_name<D>::value,
                  "PlatformBusProtocol subclasses must implement "
                  "MapMmio(uint32_t index, uint32_t cache_policy, void** out_vaddr, "
                  "size_t* out_size, zx_paddr_t* out_paddr, zx_handle_t* out_handle)");
 }

}  // namespace internal
}  // namespace ddk
