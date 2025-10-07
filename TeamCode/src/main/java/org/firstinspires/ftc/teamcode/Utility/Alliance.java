/*
 * MIT License
 *
 * Copyright (c) 2024 ParkCircus Productions; All Rights Reserved
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * Alliance: Defines the possible alliances for an FTC match.
 * * This enum is used to share the determined alliance (Red or Blue)
 * between the Autonomous OpMode and the TeleOp OpMode using the SharedData class.
 */
package org.firstinspires.ftc.teamcode.Utility;

public enum Alliance {
    /**
     * Represents the Red Alliance that is positioned to left of the field
     * when viewed from the audience perspective.
     * For the DECODE 2025-2026 season, the Red Alliance GOAL is on the right of the field
     * when viewed from the audience perspective.
     */
    RED,
    /**
     * Represents the Blue Alliance that is positioned to right of the field
     * when viewed from the audience perspective.
     * For the DECODE 2025-2026 season, the Blue Alliance GOAL is on the left of the field
     * when viewed from the audience perspective.
     */
    BLUE,
    /**
     * Represents a default, safe, or unselected state.
     * This should be the initial value in SharedData and serves as
     * a check for whether the Autonomous OpMode successfully ran and set the value.
     */
    UNSELECTED // A safe, neutral default state
}
