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

import java.util.*;

public class MecanumPathPlanner {

    private final GridManager gridManager;

    private static class Node implements Comparable<Node> {
        public int row, col;
        public double gScore, fScore;
        public Node parent;

        public Node(int row, int col, double gScore, double fScore, Node parent) {
            this.row = row;
            this.col = col;
            this.gScore = gScore;
            this.fScore = fScore;
            this.parent = parent;
        }

        @Override
        public int compareTo(Node other) {
            return Double.compare(this.fScore, other.fScore);
        }
    }

    public MecanumPathPlanner(int gridRows, int gridCols) {
        this.gridManager = new GridManager(gridRows, gridCols);
    }

    public GridManager getGridManager() {
        return gridManager;
    }

    public List<int[]> findPath(int startRow, int startCol, int endRow, int endCol) {
        PriorityQueue<Node> openSet = new PriorityQueue<>();
        Map<String, Node> allNodes = new HashMap<>();

        Node startNode = new Node(startRow, startCol, 0, heuristic(startRow, startCol, endRow, endCol), null);
        openSet.add(startNode);
        allNodes.put(startRow + "," + startCol, startNode);

        while (!openSet.isEmpty()) {
            Node currentNode = openSet.poll();

            if (currentNode.row == endRow && currentNode.col == endCol) {
                return reconstructPath(currentNode);
            }

            for (int dRow = -1; dRow <= 1; dRow++) {
                for (int dCol = -1; dCol <= 1; dCol++) {
                    if (dRow == 0 && dCol == 0) continue;

                    int neighborRow = currentNode.row + dRow;
                    int neighborCol = currentNode.col + dCol;

                    if (gridManager.isObstacle(neighborRow, neighborCol)) {
                        continue;
                    }

                    double newGScore = currentNode.gScore + distance(currentNode.row, currentNode.col, neighborRow, neighborCol);
                    String key = neighborRow + "," + neighborCol;
                    Node neighborNode = allNodes.get(key);

                    if (neighborNode == null) {
                        neighborNode = new Node(neighborRow, neighborCol, newGScore, newGScore + heuristic(neighborRow, neighborCol, endRow, endCol), currentNode);
                        allNodes.put(key, neighborNode);
                        openSet.add(neighborNode);
                    } else if (newGScore < neighborNode.gScore) {
                        neighborNode.gScore = newGScore;
                        neighborNode.fScore = newGScore + heuristic(neighborRow, neighborCol, endRow, endCol);
                        neighborNode.parent = currentNode;
                        openSet.remove(neighborNode);
                        openSet.add(neighborNode);
                    }
                }
            }
        }
        return null;
    }

    private List<int[]> reconstructPath(Node endNode) {
        List<int[]> path = new ArrayList<>();
        Node current = endNode;
        while (current != null) {
            path.add(0, new int[]{current.row, current.col});
            current = current.parent;
        }
        return path;
    }

    private double heuristic(int currentRow, int currentCol, int endRow, int endCol) {
        double dRow = endRow - currentRow;
        double dCol = endCol - currentCol;
        return Math.sqrt(dRow * dRow + dCol * dCol);
    }

    private double distance(int row1, int col1, int row2, int col2) {
        double dRow = row2 - row1;
        double dCol = col2 - col1;
        return Math.sqrt(dRow * dRow + dCol * dCol);
    }
}