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