"""
q2.py — Repeated Forward A* (Forward Replanning) with tie-breaking variants + Pygame visualization

Renders TWO views side-by-side:
- LEFT  : full (ground-truth) maze used for the run
- RIGHT : agent knowledge + search visualization

Controls:
- R : generate a new random maze and run again (max-g by default)
- 1 : run MAX-G on the current maze
- 2 : run MIN-G on the current maze
- L : load maze from text file (see readFile format) and run MAX-G
- ESC or close window : quit

Maze file format (readFile):
- Space-separated 0/1 values, 1 = blocked, 0 = free, one row per line.

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
from time import time
from typing import Callable, Dict, List, Optional, Tuple
from tqdm import tqdm
import pygame
from constants import ROWS, START_NODE, END_NODE, BLACK, WHITE, GREY, YELLOW, BLUE, PATH, NODE_LENGTH, GRID_LENGTH, WINDOW_W, WINDOW_H, GAP
from custom_pq import CustomPQ_maxG, CustomPQ_minG

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

def repeated_forward_astar(
    actual_maze: List[List[int]],
    start: Tuple[int, int] = START_NODE,
    goal: Tuple[int, int] = END_NODE,
    tie_breaking: str = "max_g", # "min_g"
    visualize_callbacks: Optional[Dict[str, Callable[[Tuple[int, int]], None]]] = None,
) -> Tuple[bool, List[Tuple[int, int]], int, int]:
    """
    Repeated Forward A* with freespace assumption.

    Returns (found, executed_path, total_expanded, replans).
    Tie-breaking:
      'max_g' — prefer larger g-values among equal f  (uses CustomPQ_maxG)
      'min_g' — prefer smaller g-values among equal f  (uses CustomPQ_minG)
    """
    ROWS_N = len(actual_maze)
    COLS_N = len(actual_maze[0])
    INF = float('inf')
    DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    def h(s: Tuple[int, int]) -> int:
        """Manhattan distance heuristic to goal."""
        return abs(s[0] - goal[0]) + abs(s[1] - goal[1])

    # Agent's world model: only cells confirmed blocked are excluded.
    # Unknown cells are treated as unblocked (freespace assumption).
    known_blocked: set = set()

    def observe(pos: Tuple[int, int]) -> None:
        """Reveal the true blockage status of the 4 neighbours of pos."""
        r, c = pos
        for dr, dc in DIRS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < ROWS_N and 0 <= nc < COLS_N:
                if actual_maze[nr][nc] == 1:
                    known_blocked.add((nr, nc))

    def get_neighbors(s: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Passable neighbours under the freespace assumption."""
        r, c = s
        result = []
        for dr, dc in DIRS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < ROWS_N and 0 <= nc < COLS_N and (nr, nc) not in known_blocked:
                result.append((nr, nc))
        return result

    # Per-cell A* bookkeeping (lazy-initialised via search_stamp)
    g: Dict[Tuple[int, int], float] = {}          # g-values
    search_stamp: Dict[Tuple[int, int], int] = {}  # last search that touched each cell
    tree: Dict[Tuple[int, int], Tuple[int, int]] = {}  # tree-pointers for path reconstruction

    counter = 0
    agent = start
    executed: List[Tuple[int, int]] = [start]
    total_expanded = 0
    replans = 0

    # Agent sees its neighbours before the first search (Figure 4, implicit)
    observe(agent)

    # ---------- Main() from Figure 4 ----------
    while agent != goal:
        counter += 1
        replans += 1

        # Initialise start and goal for this search (lines 20-25 of pseudocode)
        g[agent] = 0
        search_stamp[agent] = counter
        g[goal] = INF
        search_stamp[goal] = counter

        if tie_breaking == "max_g":
            open_list = CustomPQ_maxG()
            open_list.push(agent, h(agent), 0)
        else:
            open_list = CustomPQ_minG()
            open_list.insert(agent, h(agent), 0)

        closed: set = set()

        # ---------- ComputePath() from Figure 4 ----------
        while not open_list.is_empty():
            # Pop state with smallest f (ties broken by the chosen strategy)
            if tie_breaking == "max_g":
                s, f_s, _ = open_list.pop()
            else:
                s = open_list.extract_min()
                f_s = g.get(s, INF) + h(s)

            # Termination condition (line 2): g(goal) <= min f in OPEN
            # After popping s, f_s is the minimum f that was in OPEN.
            if g[goal] <= f_s:
                break

            # Skip already-expanded states (consistent h guarantees this rarely fires)
            if s in closed:
                continue
            closed.add(s)
            total_expanded += 1

            for nb in get_neighbors(s):
                # Lazy initialisation (lines 6-8 of pseudocode)
                if search_stamp.get(nb, 0) < counter:
                    g[nb] = INF
                    search_stamp[nb] = counter

                # Relax edge (lines 9-13)
                if g[nb] > g[s] + 1:
                    g[nb] = g[s] + 1
                    tree[nb] = s
                    f_nb = g[nb] + h(nb)
                    if tie_breaking == "max_g":
                        if open_list.contains(nb):
                            open_list.decrease_key(nb, f_nb, g[nb])
                        else:
                            open_list.push(nb, f_nb, g[nb])
                    else:
                        if open_list.contains(nb):
                            open_list.update(nb, f_nb, g[nb])
                        else:
                            open_list.insert(nb, f_nb, g[nb])

        # If goal was never reached, no path exists (line 27-29)
        if g[goal] == INF:
            return False, executed, total_expanded, replans

        # Reconstruct path: follow tree-pointers from goal back to agent (line 30)
        path: List[Tuple[int, int]] = []
        cur = goal
        while cur != agent:
            path.append(cur)
            cur = tree[cur]
        path.append(agent)
        path.reverse()

        # Move agent along path until it reaches goal or a step is blocked (line 30-32)
        for i in range(1, len(path)):
            next_cell = path[i]
            # next_cell is adjacent to agent, so already observed — check known blockage
            if next_cell in known_blocked:
                break  # path is blocked; outer loop will replan
            agent = next_cell
            executed.append(agent)
            observe(agent)  # reveal neighbours from new position
            if agent == goal:
                return True, executed, total_expanded, replans

    return True, executed, total_expanded, replans

def show_astar_search(win: pygame.Surface, actual_maze: List[List[int]], algo: str, fps: int = 240, step_delay_ms: int = 0, save_path: Optional[str] = None) -> None:
    # [BONUS] TODO: Place your visualization code here.
    # This function should display the maze used, the agent's knowledge, and the search process as the agent plans and executes.
    # As a reference, this function takes pygame Surface 'win' to draw on, the actual maze grid, the algorithm name for labeling, 
    # and optional parameters for controlling the visualization speed and saving a screenshot.
    # You are free to use other visualization libraries other than pygame. 
    # You can call repeated_forward_astar with visualize_callbacks that update the Pygame display as the agent plans and executes.
    # In the end it should store the visualization as a PNG file if save_path is provided, or default to "vis_{algo}.png".
    # print(f"[{algo}] found={found}  executed_steps={len(executed)-1}  expanded={expanded}  replans={replans}")

    if save_path is None:
        save_path = f"vis_{algo}.png"

    # If 'win' is the display surface (it is), this works:
    pygame.image.save(win, save_path)
    print(f"Saved the visualization -> {save_path}")

def main() -> None:
    parser = argparse.ArgumentParser(description="Q2: Repeated Forward A*")
    parser.add_argument("--maze_file", type=str, required=True,
                        help="Path to input JSON file containing a list of mazes")
    parser.add_argument("--output", type=str, default="results_q2.json",
                        help="Path to output JSON results file")
    parser.add_argument("--tie_braking", type=str, choices=["max_g", "min_g", "both"], default="both",
                        help="Tie-breaking variant to run (default: both)")
    parser.add_argument("--show_vis", action="store_true",
                        help="[Bonus] If set, show Pygame visualization for the selected maze")
    parser.add_argument("--maze_vis_id", type=int, default=0,
                        help="[Bonus] maze_id (index) 0 ... 49 among 50 grid worlds")
    parser.add_argument("--save_vis_path", type=str, default="q2-vis-max-g.png",
                        help="[Bonus] If set, save visualization to this PNG file")
    args = parser.parse_args()

    mazes = readMazes(args.maze_file)
    results: List[Dict] = []

    for maze_id in tqdm(range(len(mazes)), desc="Processing mazes"):
        entry: Dict = {"maze_id": maze_id}

        if args.tie_braking in ("max_g", "both"):
            t0 = time.perf_counter()
            found, executed, expanded, replans = repeated_forward_astar(
                actual_maze=mazes[maze_id],
                start=START_NODE,
                goal=END_NODE,
                tie_breaking=args.tie_braking
            )
            t1 = time.perf_counter()

            entry["max_g"] = {
                "found": found,
                "path_length": len(executed) - 1 if found else -1,
                "expanded": expanded,
                "replans": replans,
                "runtime_ms": (t1 - t0) * 1000,
            }

        if args.tie_braking in ("min_g", "both"):
            t0 = time.perf_counter()
            found, executed, expanded, replans = repeated_forward_astar(
                actual_maze=mazes[maze_id],
                start=START_NODE,
                goal=END_NODE,
                tie_breaking=args.tie_braking
            )
            t1 = time.perf_counter()

            entry["min_g"] = {
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
        pygame.display.set_caption("Repeated Forward A* Visualization")
        clock = pygame.time.Clock()
        selected_maze = mazes[args.maze_vis_id]
        current_algo = "max_g"
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
                        current_algo = "max_g"
                        show_astar_search(win, selected_maze, algo=current_algo, fps=240, step_delay_ms=0, save_path=args.save_vis_path)
                    elif event.key == pygame.K_1:
                        current_algo = "max_g"
                        show_astar_search(win, selected_maze, algo=current_algo, fps=240, step_delay_ms=0, save_path=args.save_vis_path)
                    elif event.key == pygame.K_2:
                        current_algo = "min_g"
                        show_astar_search(win, selected_maze, algo=current_algo, fps=240, step_delay_ms=0, save_path=args.save_vis_path)
            pygame.display.flip()

        pygame.quit()

    with open(args.output, "w") as fp:
        json.dump(results, fp, indent=2)
    print(f"Results for {len(results)} mazes written to {args.output}")


if __name__ == "__main__":
    main()