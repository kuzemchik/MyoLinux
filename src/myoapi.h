/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

/* Copyright (c) 2015, Thalmic Labs Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the copyright holder(s) nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER(S) BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#pragma once
#ifndef MYOAPI_H
#define MYOAPI_H
#define PACKED __attribute__ ((__packed__))

#include "myolinux.h"

#include <array>
#include <cinttypes>
#include <vector>

namespace MYOLINUX_NAMESPACE {
namespace myo {

/** UUID of the info service.
 *  The UUID is used to identify a Myo device when scanning. This string appears at the end
 *  of the vendor specific part of the packet. */
const std::vector<std::uint8_t> MyoUuid = {
    0x42, 0x48, 0x12, 0x4a,
    0x7f, 0x2c, 0x48, 0x47,
    0xb9, 0xde, 0x04, 0xa9,
    0x01, 0x00, 0x06, 0xd5
};

constexpr float OrientationScale = 16384.0f;  ///< Orientation data multiplier, see \ref OrientationSample
constexpr float AccelerometerScale = 2048.0f; ///< Accelerometer data multiplier, see \ref AccelerometerSample
constexpr float GyroscopeScale = 16.0f;       ///< Gyroscope data multiplier, see \ref GyroscopeSample

/// EMG modes.
enum class EmgMode {
    None       = 0x00, ///< Do not send EMG data.
    SendEmg    = 0x02, ///< Send filtered EMG data.
    SendEmgRaw = 0x03, ///< Send raw (unfiltered) EMG data.
};

/// IMU modes.
enum class ImuMode {
    None       = 0x00, ///< Do not send IMU data or events.
    SendData   = 0x01, ///< Send IMU data streams (accelerometer, gyroscope, and orientation).
    SendEvents = 0x02, ///< Send motion events detected by the IMU (e.g. taps).
    SendAll    = 0x03, ///< Send both IMU data streams and motion events.
    SendRaw    = 0x04, ///< Send raw IMU data streams.
};

/// Classifier modes.
enum class ClassifierMode  {
    Disabled = 0x00, ///< Disable and reset the internal state of the onboard classifier.
    Enabled  = 0x01, ///< Send classifier events (poses and arm events).
};

/// Sleep modes.
enum class SleepMode {
    Normal     = 0, ///< Normal sleep mode; Myo will sleep after a period of inactivity.
    NeverSleep = 1, ///< Never go to sleep.
};

/// Unlock modes.
enum class UnlockMode {
    Lock  = 0x00, ///< Re-lock immediately.
    Timed = 0x01, ///< Unlock now and re-lock after a fixed timeout.
    Hold  = 0x02, ///< Unlock now and remain unlocked until a lock command is received.
};

/// Kinds of vibrations.
enum class Vibration {
    None   = 0x00, ///< Do not vibrate.
    Short  = 0x01, ///< Vibrate for a short amount of time.
    Medium = 0x02, ///< Vibrate for a medium amount of time.
    Long   = 0x03, ///< Vibrate for a long amount of time.
};

/** Various parameters that may affect the behaviour of this Myo armband.
 *  The Myo library reads this attribute when a connection is established. */
struct PACKED FwInfo {
    uint8_t serial_number[6];        ///< Unique serial number of this Myo.
    uint16_t unlock_pose;            ///< Pose that should be interpreted as the unlock pose.
    uint8_t active_classifier_type;  ///< Whether Myo is currently using a built-in or a custom classifier.
    uint8_t active_classifier_index; ///< Index of the classifier that is currently active.
    uint8_t has_custom_classifier;   ///< Whether Myo contains a valid custom classifier. 1 if it does, otherwise 0.
    uint8_t stream_indicating;       ///< Set if the Myo uses BLE indicates to stream data, for reliable capture.
    uint8_t sku;                     ///< SKU value of the device.
    uint8_t reserved[7];             ///< Reserved for future use; populated with zeros.
};

/** Version information for the Myo firmware.
 *  Value layout for the myohw_att_handle_fw_version attribute.
 *  Minor version is incremented for changes in this interface.
 *  Patch version is incremented for firmware changes that do not introduce changes in this interface. */
struct PACKED FwVersion {
    uint16_t major;
    uint16_t minor;
    uint16_t patch;
    uint16_t hardware_rev; ///< Myo hardware revision. See myohw_hardware_rev_t.
};

/// Types of motion events.
enum class MotionEventType: std::uint8_t {
    Tap = 0x00
};

/// Motion event data received in a myohw_att_handle_motion_event attribute.
struct PACKED MotionEvent {
    MotionEventType type; /// Type type of motion event that occurred. See myohw_motion_event_type_t.
    /// Event-specific data.
    union PACKED {
        /// For myohw_motion_event_tap events.
        struct PACKED {
            uint8_t tap_direction;
            uint8_t tap_count;
        };
    };
};

/// Types of classifier events.
enum class ClassifierEventType: std::uint8_t {
    ArmSynced   = 0x01,
    ArmUnsynced = 0x02,
    Pose        = 0x03,
    Unlocked    = 0x04,
    Locked      = 0x05,
    SyncFailed  = 0x06
};

/// Enumeration identifying a right arm or left arm.
enum class Arm: std::uint8_t {
    Right   = 0x01,
    Left    = 0x02,
    Unknown = 0xff
};

/// Possible directions for Myo's +x axis relative to a user's arm.
enum class XDirection: std::uint8_t {
    TowardsWrist = 0x01,
    TowardsElbow = 0x02,
    Unknown      = 0xff
};

/// Supported poses.
enum class Pose: std::uint16_t {
    Rest          = 0x0000,
    Fist          = 0x0001,
    WaveIn        = 0x0002,
    WaveOut       = 0x0003,
    FingersSpread = 0x0004,
    DoubleTap     = 0x0005,
    Unknown       = 0xffff
};

/// Possible outcomes when the user attempts a sync gesture.
enum class SyncResult {
    TooHard     = 0x01 ///< Sync geture was performed too hard
};

/// Classifier event data received in a myohw_att_handle_classifier_event attribute.
struct PACKED ClassifierEvent {
    ClassifierEventType type; ///< See myohw_classifier_event_type_t

    /// Event-specific data
    union PACKED {
        /// For myohw_classifier_event_arm_synced events.
        struct PACKED {
            Arm arm; ///< See myohw_arm_t
            XDirection x_direction; ///< See myohw_x_direction_t
        };

        /// For myohw_classifier_event_pose events.
        Pose pose; ///< See myohw_pose_t

        /// For myohw_classifier_event_sync_failed events.
        SyncResult sync_result; ///< See myohw_sync_result_t.
    };
};

/// EmgSample
using EmgSample = std::array<std::int8_t, 8>;

/** OrientationSample
 *  Orientation data, represented as a unit quaternion. Values are multiplied by \ref OrientationScale. */
using OrientationSample = std::array<std::int16_t, 4>;

/** AccelerometerSample
 *  Accelerometer data. In units of g. Range of + -16.
 *  Values are multiplied by \ref AccelerometerScale. */
using AccelerometerSample = std::array<std::int16_t, 3>;

/** GyroscopeSample
 *  Gyroscope data. In units of deg/s. Range of + -2000.
 *  Values are multiplied by \ref GyroscopeScale. */
using GyroscopeSample = std::array<std::int16_t, 3>;

}
}

#undef PACKED
#endif // MYOAPI_H
