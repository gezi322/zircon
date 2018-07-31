// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/protocol/i2c.h>

class PlatformI2cBus {
public:
    explicit PlatformI2cBus(i2c_impl_protocol_t* i2c, uint32_t bus_id);
    zx_status_t Start();

    zx_status_t Transact(pdev_req_t* req, uint16_t address, const void* write_buf,
                         zx_handle_t channel_handle);
private:
    // struct representing an I2C transaction.
    struct I2cTxn {
        uint32_t txid;
        zx_handle_t channel_handle;

        list_node_t node;
        size_t write_length;
        size_t read_length;
        uint16_t address;
        i2c_complete_cb complete_cb;
        void* cookie;
        uint8_t write_buffer[];
    };

    void Complete(I2cTxn* txn, zx_status_t status, const uint8_t* data,
                     size_t data_length);
    void I2cThread();
    static int I2cThread(void* arg);

    i2c_impl_protocol_t i2c_;
    uint32_t bus_id_;
    size_t max_transfer_;

    list_node_t queued_txns_;
    list_node_t free_txns_;
    sync_completion_t txn_signal_;

    thrd_t thread_;
    fbl::Mutex mutex_;
};
