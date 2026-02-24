"""
gen_test_json.py — Generate N random 101x101 mazes and save as mazes.json. Uses same algorithm as maze_generator.py.

Usage:
    python gen_test_json.py [--num_mazes N] [--seed S] [--output FILE]
"""
import json
import random
import argparse
import random
from constants import ROWS
from tqdm import tqdm
import argparse

# set random seed for reproducibility
random.seed(42)

def create_maze() -> list:
    # TODO: Implement this function to generate and return a random maze as a 2D list of 0s and 1s.
    size = ROWS
    blockProb = 0.3
    grid = [[0] * size for _ in range(size)]
    visited = [[False] * size for _ in range(size)]
    stack = []
    
    startX, startY = random.randint(0, size - 1), random.randint(0, size - 1)
    visited[startX][startY] = True
    grid[startX][startY] = 0
    stack.append((startX, startY))
    
    totalCells = size ** 2
    visitedCount = 1
    while visitedCount < totalCells:
        if stack:
            currentX, currentY = stack[-1]
            
            neighbors = []
            for dX, dY in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                newX, newY = currentX + dX, currentY + dY
                if 0 <= newX < size and 0 <= newY < size and not visited[newX][newY]:
                    neighbors.append((newX, newY))
                
            
            if neighbors:
                newX, newY = random.choice(neighbors)
                visited[newX][newY] = True
                visitedCount += 1
                
                if random.random() < blockProb:
                    grid[newX][newY] = 1
                else:
                    grid[newX][newY] = 0
                    stack.append((newX, newY))
                    
            else:
                stack.pop()
        else:
            found = False
            for i in range(size):
                for j in range(size):
                    if not visited[i][j]:
                        visited[i][j] = True
                        visitedCount += 1
                        grid[i][j] = 0
                        stack.append((i, j))
                        found = True
                        break
                if found: 
                    break
    return grid
                    

def main():
    parser = argparse.ArgumentParser(description="Generate random mazes as JSON")
    parser.add_argument("--num_mazes", type=int, default=50,
                        help="Number of mazes to generate")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed for reproducibility")
    parser.add_argument("--output", type=str, default="mazes.json",
                        help="Output JSON file path")
    args = parser.parse_args()

    random.seed(args.seed)
    
    mazes = []
    for _ in tqdm(range(args.num_mazes), desc="Generating mazes"):  
        mazes.append(create_maze())

    with open(args.output, "w") as fp:
        json.dump(mazes, fp)
    print(f"Generated {args.num_mazes} mazes (seed={args.seed}) -> {args.output}")

if __name__ == "__main__":
    main()
