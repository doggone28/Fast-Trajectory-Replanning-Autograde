"""
q5.py — Adaptive A* with tie-breaking variants + Pygame visualization

Renders TWO views side-by-side:
- LEFT  : full (ground-truth) maze used for the run
- RIGHT : agent knowledge + search visualization

Controls:
- R : generate a new random maze and run again (max-g by default)
- 1 : run MAX-G Adaptive A* on the current maze
- 2 : run MIN-G Adaptive A* on the current maze
- ESC or close window : quit

Maze file format helper:
- readFile(fname) reads 0/1 space-separated tokens, 1=blocked, 0=free, one row per line.

Legend (colors):
GREY   = expanded / frontier / unknown (unseen)
PATH   = executed path
YELLOW = start + agent position
BLUE   = goal
WHITE  = known free
BLACK  = known blocked
"""

from __future__ import annotations

import heapq
import argparse
import json
import time
from typing import Callable, Dict, List, Optional, Tuple
from tqdm import tqdm
import pygame
from q2 import repeated_forward_astar
from constants import ROWS, START_NODE, END_NODE, BLACK, WHITE, GREY, YELLOW, BLUE, PATH, NODE_LENGTH, GRID_LENGTH, WINDOW_W, WINDOW_H, GAP
from custom_pq import CustomPQ_maxG


# ---------------- FILE LOADER ----------------
def readMazes(fname: str) -> List[List[List[int]]]:
    """
    Reads a JSON file containing a list of mazes.
    Each maze is a list of ROWS lists, each with ROWS int values (0=free, 1=blocked).
    Returns a list of maze[r][c] grids.
    """
    with open(fname, "r", encoding="utf-8") as fp:
        data = json.load(fp)
    mazes: List[List[List[int]]] = []
    for idx, grid in enumerate(data):
        if len(grid) != ROWS or any(len(row) != ROWS for row in grid):
            raise ValueError(f"Maze {idx}: expected {ROWS}x{ROWS}, got {len(grid)}x{len(grid[0]) if grid else 0}")
        maze = [[int(v) for v in row] for row in grid]
        maze[START_NODE[0]][START_NODE[1]] = 0
        maze[END_NODE[0]][END_NODE[1]] = 0
        mazes.append(maze)
    return mazes

def adaptive_astar(
    actual_maze: List[List[int]],
    start: Tuple[int, int] = START_NODE,
    goal: Tuple[int, int] = END_NODE,
    visualize_callbacks: Optional[Dict[str, Callable[[Tuple[int, int]], None]]] = None,
) -> Tuple[bool, List[Tuple[int, int]], int, int]:
    """
    Adaptive A* with max_g tie-breaking.

    Identical to Repeated Forward A* (max_g) except that after each ComputePath
    the h-values of all expanded states are updated to  h(s) := g(goal) - g(s).
    These tighter (yet still consistent) h-values are reused in future searches,
    reducing the number of expansions over time.

    Returns (found, executed_path, total_expanded, replans).
    """
    ROWS_N = len(actual_maze)
    COLS_N = len(actual_maze[0])
    INF = float('inf')
    DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    def manhattan(s: Tuple[int, int]) -> int:
        return abs(s[0] - goal[0]) + abs(s[1] - goal[1])

    # h_val persists across searches; defaults to Manhattan distance for unseen states.
    h_val: Dict[Tuple[int, int], int] = {}

    def h(s: Tuple[int, int]) -> int:
        return h_val.get(s, manhattan(s))

    # Agent's world model under the freespace assumption.
    known_blocked: set = set()

    def observe(pos: Tuple[int, int]) -> None:
        r, c = pos
        for dr, dc in DIRS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < ROWS_N and 0 <= nc < COLS_N:
                if actual_maze[nr][nc] == 1:
                    known_blocked.add((nr, nc))

    def get_neighbors(s: Tuple[int, int]) -> List[Tuple[int, int]]:
        r, c = s
        result = []
        for dr, dc in DIRS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < ROWS_N and 0 <= nc < COLS_N and (nr, nc) not in known_blocked:
                result.append((nr, nc))
        return result

    # Per-cell A* bookkeeping (lazy-initialised via search_stamp)
    g: Dict[Tuple[int, int], float] = {}
    search_stamp: Dict[Tuple[int, int], int] = {}
    tree: Dict[Tuple[int, int], Tuple[int, int]] = {}

    counter = 0
    agent = start
    executed: List[Tuple[int, int]] = [start]
    total_expanded = 0
    replans = 0

    observe(agent)

    while agent != goal:
        counter += 1
        replans += 1

        g[agent] = 0
        search_stamp[agent] = counter
        g[goal] = INF
        search_stamp[goal] = counter

        open_list = CustomPQ_maxG()
        open_list.push(agent, h(agent), 0)

        closed: set = set()

        # ComputePath (same as Repeated Forward A*, max_g variant)
        while not open_list.is_empty():
            s, f_s, _ = open_list.pop()

            if g[goal] <= f_s:
                break

            if s in closed:
                continue
            closed.add(s)
            total_expanded += 1

            for nb in get_neighbors(s):
                if search_stamp.get(nb, 0) < counter:
                    g[nb] = INF
                    search_stamp[nb] = counter

                if g[nb] > g[s] + 1:
                    g[nb] = g[s] + 1
                    tree[nb] = s
                    f_nb = g[nb] + h(nb)
                    if open_list.contains(nb):
                        open_list.decrease_key(nb, f_nb, g[nb])
                    else:
                        open_list.push(nb, f_nb, g[nb])

        if g[goal] == INF:
            return False, executed, total_expanded, replans

        # --- Adaptive A* h-value update (the key difference) ---
        # For every state expanded in this search, tighten its heuristic.
        # h_new(s) = g(goal) - g(s)  [proven consistent and >= previous h]
        g_goal = g[goal]
        for s in closed:
            h_val[s] = int(g_goal - g[s])

        # Reconstruct path via tree-pointers from goal back to agent
        path: List[Tuple[int, int]] = []
        cur = goal
        while cur != agent:
            path.append(cur)
            cur = tree[cur]
        path.append(agent)
        path.reverse()

        # Move agent along path until goal reached or a step turns out blocked
        for i in range(1, len(path)):
            next_cell = path[i]
            if next_cell in known_blocked:
                break
            agent = next_cell
            executed.append(agent)
            observe(agent)
            if agent == goal:
                return True, executed, total_expanded, replans

    return True, executed, total_expanded, replans

def show_astar_search(win: pygame.Surface, actual_maze: List[List[int]], algo: str, fps: int = 240, step_delay_ms: int = 0, save_path: Optional[str] = None) -> None:
    # [BONUS] TODO: Place your visualization code here.
    # This function should display the maze used, the agent's knowledge, and the search process as the agent plans and executes.
    # As a reference, this function takes pygame Surface 'win' to draw on, the actual maze grid, the algorithm name for labeling, 
    # and optional parameters for controlling the visualization speed and saving a screenshot.
    # You are free to use other visualization libraries other than pygame. 
    # You can call repeated_backward_astar with visualize_callbacks that update the Pygame display as the agent plans and executes.
    # In the end it should store the visualization as a PNG file if save_path is provided, or default to "vis_{algo}.png".
    # print(f"[{algo}] found={found}  executed_steps={len(executed)-1}  expanded={expanded}  replans={replans}")

    if save_path is None:
        save_path = f"vis_{algo}.png"

    # If 'win' is the display surface (it is), this works:
    pygame.image.save(win, save_path)
    print(f"Saved the visualization -> {save_path}")

def main() -> None:
    parser = argparse.ArgumentParser(description="Q5: Adaptive A*")
    parser.add_argument("--maze_file", type=str, required=True,
                        help="Path to input JSON file containing a list of mazes")
    parser.add_argument("--output", type=str, default="results_q5.json",
                        help="Path to output JSON results file")
    parser.add_argument("--show_vis", action="store_true",
                        help="[Bonus] If set, show Pygame visualization for the selected maze")
    parser.add_argument("--maze_vis_id", type=int, default=0,
                        help="[Bonus] maze_id (index) 0 ... 49 among 50 grid worlds")
    parser.add_argument("--save_vis_path", type=str, default="q5-vis-max-g.png",
                        help="[Bonus] If set, save visualization to this PNG file")
    args = parser.parse_args()

    mazes = readMazes(args.maze_file)
    results: List[Dict] = []

    for maze_id in tqdm(range(len(mazes)), desc="Processing mazes"):
        entry: Dict = {"maze_id": maze_id}

        t0 = time.perf_counter()
        found, executed, expanded, replans = adaptive_astar(
            actual_maze=mazes[maze_id],
            start=START_NODE,
            goal=END_NODE,
        )
        t1 = time.perf_counter()

        entry["adaptive"] = {
            "found": found,
            "path_length": len(executed) - 1 if found else -1,
            "expanded": expanded,
            "replans": replans,
            "runtime_ms": (t1 - t0) * 1000,
        }

        t0 = time.perf_counter()
        found, executed, expanded, replans = repeated_forward_astar(
            actual_maze=mazes[maze_id],
            start=START_NODE,
            goal=END_NODE,
            tie_braking="max_g",
        )
        t1 = time.perf_counter()

        entry["fwd"] = {
            "found": found,
            "path_length": len(executed) - 1 if found else -1,
            "expanded": expanded,
            "replans": replans,
            "runtime_ms": (t1 - t0) * 1000,
        }

        results.append(entry)

    if args.show_vis:
        # In case, PyGame is used for visualization, this code initializes a window and runs the visualization for the selected maze and algorithm.
        # Feel free to modify this code if you use a different visualization library or approach.
        pygame.init()
        win = pygame.display.set_mode((WINDOW_W, WINDOW_H))
        pygame.display.set_caption("Adaptive A* Visualization")
        clock = pygame.time.Clock()
        selected_maze = mazes[args.maze_vis_id]
        current_algo = "adaptive"
        show_astar_search(win, selected_maze, algo=current_algo, fps=240, step_delay_ms=0, save_path=args.save_vis_path)
        running = True
        while running:
            clock.tick(30)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_r:
                        current_algo = "adaptive"
                        show_astar_search(win, selected_maze, algo=current_algo, fps=240, step_delay_ms=0, save_path=args.save_vis_path)
                    elif event.key == pygame.K_1:
                        current_algo = "adaptive"
                        show_astar_search(win, selected_maze, algo=current_algo, fps=240, step_delay_ms=0, save_path=args.save_vis_path)
                    elif event.key == pygame.K_2:
                        current_algo = "fwd"
                        show_astar_search(win, selected_maze, algo=current_algo, fps=240, step_delay_ms=0, save_path=args.save_vis_path)
            pygame.display.flip()

        pygame.quit()

    with open(args.output, "w") as fp:
        json.dump(results, fp, indent=2)
    print(f"Results for {len(results)} mazes written to {args.output}")


if __name__ == "__main__":
    main()