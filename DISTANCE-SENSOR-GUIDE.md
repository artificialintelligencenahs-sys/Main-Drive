# Distance Sensor Guide — PROS + LemLib

This guide covers how to set up and use the distance sensor position + heading reset system in your PROS + LemLib code.

## Your Sensor Layout

              Front (no sensor)
                    ↑
                    |

Left (1 sensor) ← 🤖 → Right (1 sensor)
|
↓ ↓ ↓
Back Left Back Right (2 sensors)

- **Back**: 2 sensors → resets position AND heading
- **Left / Right**: 1 sensor each → resets position only (with trig correction)

Total: 4 distance sensors, 4 smart ports used

## Step 1: Configure Ports

In `src/robot_config.cpp`, your sensors are defined using the `pros::Distance` class. Set the correct ports for each sensor:

```cpp
pros::Distance back_sensor_left(19);   // ← Change to your actual port
pros::Distance back_sensor_right(17);  // ← Change to your actual port
pros::Distance left_sensor(18);        // ← Change to your actual port
pros::Distance right_sensor(16);       // ← Change to your actual port
```

## Step 2: Measure and Set Offsets

You need to measure 5 values on your robot. All measurements must be in **inches**.

> [!IMPORTANT]
> **Where is the Tracking Center?**
> The tracking center is the exact origin point (0,0) of your robot's coordinate system.
> If you are using LemLib's default tracking, this is the **center of rotation** of your robot.
> This is exactly the middle point between your left and right drive wheels, and the middle point between your front and back drive wheels.
>
> **How to measure to the sensors:**
> Measure from the **face** of the distance sensor (the flat part that emits the signal) to that exact center point. Do **NOT** measure to the edge of the wheels or the edge of the robot frame.

### Back Sensor Offsets

Measure the distance from the face of each back sensor along the Y-axis to the center of the robot.

```text
           ← spacing →
    [sensorL]       [sensorR]
         ↕ offset       ↕ offset
         ● (robot center)
```

In `src/robot_config.cpp` AND `src/helper_functions.cpp`, set these values:

```cpp
const double back_sensor_left_offset  = 4.875;   // ← Distance from left back sensor face to robot center Y-axis
const double back_sensor_right_offset = 4.875;   // ← Distance from right back sensor face to robot center Y-axis
const double back_sensor_spacing      = 9.125;   // ← Center-to-center horizontal distance between the two back sensors
```

> [!NOTE]
> Ensure the back sensors are perfectly parallel to each other.

### Side Sensor Offsets

Measure the distance from the face of each side sensor along the X-axis to the center of the robot.

```cpp
const double left_sensor_offset       = 5.8125;  // ← Distance from left sensor face to robot center X-axis
const double right_sensor_offset      = 5.8125;  // ← Distance from right sensor face to robot center X-axis
```

## Step 3: Use in Autonomous

### `driveUntilDistance()` — Lining Up to a Wall

Before calling a reset, you must get the robot close to the wall. `driveUntilDistance()` drives the robot based on the sensor reading until it hits a threshold:

```cpp
// Back up slowly (speed 50/127) until 3 inches from the wall behind you, timeout 3000ms
driveUntilDistance(back_sensor_left, 3.0, 50, false, 3000);
```

### Back Reset (Dual Sensor — Position + Heading)

Call this when the back of the robot is near/against a wall. It corrects both your position relative to that wall and your robot's heading.

```cpp
void autonomous() {
    chassis.setPose(-48, -60, 0); // initial pose

    // Move to some position...

    // 1. Back up to the wall and align
    driveUntilDistance(back_sensor_left, 3.0, 50, false, 3000);

    // 2. Reset odometry (pos + heading)
    resetPositionAndHeadingBack(back_sensor_left, back_sensor_right, back_sensor_spacing, back_sensor_left_offset, back_sensor_right_offset, field_half_size);

    // Now odometry is highly accurate — continue routine
}
```

### Side Resets (Single Sensor — Position Only)

Call these when the left or right side of the robot is near a wall. These only correct position, using your IMU heading to do trig correction for angles.

```cpp
// Left side facing a wall
resetPositionLeft(left_sensor, left_sensor_offset, field_half_size);

// Right side facing a wall
resetPositionRight(right_sensor, right_sensor_offset, field_half_size);
```

## Best Practices

1.  **Reset early and often during skills** — Odometry drifts more over a 60-second routine.
2.  **Always reset after long sequences** — After 4-5 major movements, drift accumulates.
3.  **Use `driveUntilDistance` before resetting** — This ensures you are close to the wall at a controlled speed and within sensor range.
4.  **When combining back and side resets, ORDER MATTERS!**
    - ALWAYS do `driveUntilDistance` first.
    - THEN call `resetPositionAndHeadingBack()` to get perfect heading.
    - THEN call your side reset (`resetPositionLeft()` or `Right()`).
    - _Why?_ The side reset relies on the heading being perfect to calculate its own trig correction. If you do the side reset first, its calculation will be entirely wrong!
5.  **Lower speed for `driveUntilDistance`** — A speed of ~50 is ideal. Full speed will cause overshoot and inaccuracy.
6.  **Maintain rough perpendicularity** — The robot should be roughly perpendicular to the wall (within ~20°). The dual sensor setup corrects for small angle errors, but large angle approaches will fail.

## Troubleshooting

| Issue                                                    | Fix                                                                                                                                              |
| :------------------------------------------------------- | :----------------------------------------------------------------------------------------------------------------------------------------------- |
| Reset puts robot at wrong position                       | Check sensor offsets — they might be measured wrong. Remember, it's from the sensor face to the drive center.                                    |
| Reset heading is way off                                 | Check `back_sensor_spacing` — measure center-to-center between the back sensors again.                                                           |
| "Invalid sensor reading" in terminal                     | Sensor might not be detecting a wall. Make sure it's pointing at a flat surface and within range (0-200 inches).                                 |
| Position resets X when it should reset Y (or vice versa) | Check that the robot is actually facing the wall you think it is — the code uses current heading to determine which field wall it is looking at. |
| Side resets are slightly off                             | This is normal for single-sensor resets — they depend entirely on IMU heading accuracy.                                                          |
