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

package org.firstinspires.ftc.teamcode.Holonomic;

public class GridManager {
    private final boolean[][] grid;
    private final int rows, cols;

    public GridManager(int rows, int cols) {
        this.rows = rows;
        this.cols = cols;
        this.grid = new boolean[rows][cols];
    }

    public void addObstacle(int row, int col) {
        if (isValid(row, col)) {
            grid[row][col] = true;
        }
    }

    public void removeObstacle(int row, int col) {
        if (isValid(row, col)) {
            grid[row][col] = false;
        }
    }

    public void clearObstacles() {
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                grid[r][c] = false;
            }
        }
    }

    public boolean isObstacle(int row, int col) {
        return !isValid(row, col) || grid[row][col];
    }

    private boolean isValid(int row, int col) {
        return row >= 0 && row < rows && col >= 0 && col < cols;
    }
}