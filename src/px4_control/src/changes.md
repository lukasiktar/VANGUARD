# Changes log

## How to rewind

Full restore of the original file (backup taken 2026-10-07 before any edit):

```bash
cp src/px4_control/src/px4_control_node_manual_v2.cpp.orig src/px4_control/src/px4_control_node_manual_v2.cpp
```

To undo a single change, replace its **After** block with its **Before** block.

---

## 1. Cross button arms only, keeps current mode — 2026-10-07

**File:** `px4_control_node_manual_v2.cpp`, `arm()` (orig lines 197–226, now 197–206)
**Reason:** Cross used to switch to POSITION mode before arming. Now it only arms and leaves PX4 in whatever mode it is in. This also fixes a compile error: the mode-switch call had a trailing comma and a missing argument.

**Before:**
```cpp
    void arm()
    {
        RCLCPP_INFO(this->get_logger(), "[Cross] ARM requested");
        // PX4 boots into Manual mode by default, which requires the throttle stick
        // pulled to the bottom (< -0.8) to arm -- a self-centering gamepad stick can't
        // do that. Position mode only requires throttle <= 0.2 (centered is fine), so
        // switch there first and only arm once that's confirmed.
        auto mode_command = create_vehicle_command(
            VEHICLE_CMD_DO_SET_MODE,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            );//PX4_CUSTOM_MAIN_MODE_POSCTL);

        send_vehicle_command(mode_command, [this](bool accepted)
        {
            if (!accepted)
            {
                RCLCPP_ERROR(this->get_logger(), "Could not switch to POSITION mode, aborting ARM");
                return;
            }

            arm_delay_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                [this]()
                {
                    arm_delay_timer_->cancel();
                    auto arm_command = create_vehicle_command(VEHICLE_CMD_ARM, 1.0f);
                    send_vehicle_command(arm_command);
                });
        });
    }
```

**After:**
```cpp
    void arm()
    {
        RCLCPP_INFO(this->get_logger(), "[Cross] ARM requested (keeping current mode)");
        // Arms in whatever mode PX4 is already in. Manual/Stabilized require the
        // throttle stick at the bottom (< -0.8) to arm, which a self-centering gamepad
        // stick can't do, so PX4 will reject it there -- pick Position (R1) or
        // Altitude (L2) first, both accept a centered throttle.
        auto arm_command = create_vehicle_command(VEHICLE_CMD_ARM, 1.0f);
        send_vehicle_command(arm_command);
    }
```

## 2. Removed unused `arm_delay_timer_` member — 2026-10-07

**File:** `px4_control_node_manual_v2.cpp`, member variables (orig line 378)
**Reason:** It was only used by the old arm-after-mode-switch delay, which change 1 removed.

**Before:**
```cpp
    rclcpp::TimerBase::SharedPtr manual_control_timer_;
    rclcpp::TimerBase::SharedPtr arm_delay_timer_;

    std::vector<int32_t> prev_buttons_;
```

**After:**
```cpp
    rclcpp::TimerBase::SharedPtr manual_control_timer_;

    std::vector<int32_t> prev_buttons_;
```

**Note:** If you undo change 1, undo this one too.

## 3. L1 switches to STABILIZED instead of MANUAL — 2026-10-07

**File:** `px4_control_node_manual_v2.cpp` (startup log line 46, constants lines 68 and 79, `joy_callback` line 151, mode function line 252)
**Reason:** L1 should switch PX4 to STABILIZED mode (PX4 custom main mode 7) instead of MANUAL (1). Button index 3 for L1 is unchanged.

**Before → After:**
```cpp
// line 46 (startup help text)
"... L1=MANUAL mode  L2=ALTITUDE mode"
→ "... L1=STABILIZED mode  L2=ALTITUDE mode"

// line 68
static constexpr float PX4_CUSTOM_MAIN_MODE_MANUAL = 1.0f;
→ static constexpr float PX4_CUSTOM_MAIN_MODE_STABILIZED = 7.0f;

// line 79
static constexpr size_t BTN_MANUAL = 3;   // L1
→ static constexpr size_t BTN_STABILIZED = 3; // L1

// line 151-153
else if (is_rising_edge(msg, BTN_MANUAL)) { switch_to_manual_mode(); }
→ else if (is_rising_edge(msg, BTN_STABILIZED)) { switch_to_stabilized_mode(); }
```

**Before (mode function):**
```cpp
    void switch_to_manual_mode()
    {
        // Manual has no position/altitude hold: throttle is direct thrust (centered
        // stick != hover) and roll/pitch is self-leveled attitude only, no velocity
        // or position lock. Expect it to climb/sink and drift once switched.
        RCLCPP_WARN(this->get_logger(), "[L1] Switching to MANUAL mode (no altitude/position hold!)");
        auto command = create_vehicle_command(
            VEHICLE_CMD_DO_SET_MODE,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_CUSTOM_MAIN_MODE_MANUAL);
        send_vehicle_command(command);
    }
```

**After (mode function):**
```cpp
    void switch_to_stabilized_mode()
    {
        // Stabilized has no position/altitude hold: throttle is direct thrust (centered
        // stick != hover) and roll/pitch is self-leveled attitude only, no velocity
        // or position lock. Expect it to climb/sink and drift once switched.
        RCLCPP_WARN(this->get_logger(), "[L1] Switching to STABILIZED mode (no altitude/position hold!)");
        auto command = create_vehicle_command(
            VEHICLE_CMD_DO_SET_MODE,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_CUSTOM_MAIN_MODE_STABILIZED);
        send_vehicle_command(command);
    }
```

**Note:** The button remap (Cross=1, Circle=0, R1=2, L1=3, Square=4, Triangle=5) and the LOITER→HOLD rename were made by hand before this change, not by Claude, so they are not logged here. The `.orig` backup predates them, so restoring it undoes them too.

## 4. Button constants sorted by index, terminal text matches the new mapping — 2026-10-07

**File:** `px4_control_node_manual_v2.cpp` (startup log lines 46 and 49, constants lines 75–81, `arm()` comment line 202, button tags in the log messages of each command function)
**Reason:** The button comments were corrected by hand (Square=0, Cross=1, Circle=2, Triangle=3, L1=4, R1=5). This change sorts the constants by index and updates every terminal message to name the right button. No button numbers changed.

**Before (constants):**
```cpp
    static constexpr size_t BTN_ARM = 1;      // Cross
    static constexpr size_t BTN_HOLD = 0;   // Circle
    static constexpr size_t BTN_TAKEOFF = 5;  // Triangle
    static constexpr size_t BTN_LAND = 4;     // Square
    static constexpr size_t BTN_STABILIZED = 3; // L1
    static constexpr size_t BTN_POSCTL = 2;   // R1
    static constexpr size_t BTN_DISARM = 9;   // Options (kill switch)
```

**After (constants):**
```cpp
    static constexpr size_t BTN_HOLD = 0;       // Square
    static constexpr size_t BTN_ARM = 1;        // Cross
    static constexpr size_t BTN_POSCTL = 2;     // Circle
    static constexpr size_t BTN_STABILIZED = 3; // Triangle
    static constexpr size_t BTN_LAND = 4;       // L1
    static constexpr size_t BTN_TAKEOFF = 5;    // R1
    static constexpr size_t BTN_DISARM = 9;     // Options (kill switch)
```

**Terminal text, Before → After:**
```
line 46:  "Cross=ARM  Triangle=TAKEOFF  Square=LAND  Options=DISARM (kill)  R1=POSITION mode  Circle=HOLD mode  L1=STABILIZED mode  L2=ALTITUDE mode"
       →  "Square=HOLD mode  Cross=ARM  Circle=POSITION mode  Triangle=STABILIZED mode  L1=LAND  R1=TAKEOFF  L2=ALTITUDE mode  Options=DISARM (kill)"
line 49:  "... in a manual flight mode (R1). "  →  "... in a manual flight mode (Circle). "
takeoff():                    "[Triangle] TAKEOFF requested ..."         →  "[R1] TAKEOFF requested ..."
land():                       "[Square] LAND requested"                  →  "[L1] LAND requested"
switch_to_position_mode():    "[R1] Switching to POSITION mode ..."      →  "[Circle] Switching to POSITION mode ..."
switch_to_stabilized_mode():  "[L1] Switching to STABILIZED mode ..."    →  "[Triangle] Switching to STABILIZED mode ..."
HOLD():                       "[Circle] Switching to HOLD mode ..."      →  "[Square] Switching to HOLD mode ..."
arm() comment line 202:       "pick Position (R1) or"                    →  "pick Position (Circle) or"
```
Unchanged: `[Cross] ARM`, `[Options] DISARM`, `[L2] ALTITUDE`.

## 5. Button checks in `joy_callback` follow button order, kill switch first — 2026-10-07

**File:** `px4_control_node_manual_v2.cpp`, `joy_callback()` (lines ~127–155)
**Reason:** The `if / else if` chain now follows the same 0, 1, 2, ... order as the constants. Options (DISARM) moved to the top: because it is an `else if` chain, only the first matching button runs, so the kill switch now wins if it is pressed together with another button. Before, it lost to ARM, TAKEOFF and LAND.

**Before:**
```cpp
        if (is_rising_edge(msg, BTN_ARM))
        {
            arm();
        }
        else if (is_rising_edge(msg, BTN_TAKEOFF))
        {
            takeoff(TARGET_ALTITUDE);
        }
        else if (is_rising_edge(msg, BTN_LAND))
        {
            land();
        }
        else if (is_rising_edge(msg, BTN_DISARM))
        {
            disarm();
        }
        else if (is_rising_edge(msg, BTN_POSCTL))
        {
            switch_to_position_mode();
        }
        else if (is_rising_edge(msg, BTN_HOLD))
        {
            HOLD();
        }
        else if (is_rising_edge(msg, BTN_STABILIZED))
        {
            switch_to_stabilized_mode();
        }
```

**After:**
```cpp
        // Kill switch is checked first so it always wins if pressed together with another button.
        if (is_rising_edge(msg, BTN_DISARM))
        {
            disarm();
        }
        else if (is_rising_edge(msg, BTN_HOLD))
        {
            HOLD();
        }
        else if (is_rising_edge(msg, BTN_ARM))
        {
            arm();
        }
        else if (is_rising_edge(msg, BTN_POSCTL))
        {
            switch_to_position_mode();
        }
        else if (is_rising_edge(msg, BTN_STABILIZED))
        {
            switch_to_stabilized_mode();
        }
        else if (is_rising_edge(msg, BTN_LAND))
        {
            land();
        }
        else if (is_rising_edge(msg, BTN_TAKEOFF))
        {
            takeoff(TARGET_ALTITUDE);
        }
```

**Note on 4 and 5 (2026-10-07):** These changes were undone once, when VS Code saved an older copy of the file over them, and then redone exactly as described above. The only difference from entry 4's Before block is that the hand-edited comments read `//  Square`, `//  R1`, `//  L1`, `//  Triangle`, `//  Circle` (with two spaces).
