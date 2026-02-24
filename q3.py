"""
q3.py — Repeated BACKWARD A* (Backward Replanning) with tie-breaking variants + Pygame visualization

Renders TWO views side-by-side:
- LEFT  : full (ground-truth) maze used for the run
- RIGHT : agent knowledge + search visualization

Controls:
- R : generate a new random maze and run again (max-g by default)
- 1 : run MAX-G on the current maze
- 2 : run MIN-G on the current maze
- ESC or close window : quit

Maze file loader (optional helper): readFile(fname) reads 0/1 tokens (space-separated), 1=blocked, 0=free.

Legend (colors):
GREY   = expanded / frontier / unknown (unseen)
PATH   = executed path (agent actually walked)
YELLOW = start + current agent position
BLUE   = goal
WHITE  = known free
BLACK  = known blocked
"""

from __future__ import annotations

import heapq
import argparse
import json
from typing import Callable, Dict, List, Optional, Tuple
from tqdm import tqdm
import time
import pygame
from constants import ROWS, START_NODE, END_NODE, BLACK, WHITE, GREY, YELLOW, BLUE, PATH, NODE_LENGTH, GRID_LENGTH, WINDOW_W, WINDOW_H, GAP
from custom_pq import CustomPQ_maxG, CustomPQ_minG
from q2 import repeated_forward_astar


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

def repeated_backward_astar(
    actual_maze: List[List[int]],
    start: Tuple[int, int] = START_NODE,
    goal: Tuple[int, int] = END_NODE,
    visualize_callbacks: Optional[Dict[str, Callable[[Tuple[int, int]], None]]] = None,
) -> Tuple[bool, List[Tuple[int, int]], int, int]:
    
    # TODO: Implement Backward A* with max_g tie-braking strategy.
    
    INF = float('inf')
    DIRS = [(-1, 0), (1,0),(0, -1), (0, 1)]
    
    knownBlocked: set = set()
    
    def observe(pos: Tuple[int, int]) -> None:
        row, col = pos
        for dRow, dCol in DIRS:
            nr, nc = row + dRow, col + dCol
            if 0 <= nr < ROWS and 0 <= nc < ROWS:
                if actual_maze[nr][nc] == 1:
                    knownBlocked.add((nr, nc))
    
    def getNeighbors(s: Tuple[int, int]) -> List[Tuple[int, int]]:
        row, col = s
        return [
                    (row + dr, col + dc)
                    for dr, dc in DIRS
                    if 0 <= row + dr < ROWS and 0 <= col + dc < ROWS and (row + dr, col + dc) not in knownBlocked
                ]
        
        
        
    g: Dict[Tuple[int, int], float] = {}
    searchStamp: Dict[Tuple[int, int], int] = {}
    tree: Dict[Tuple[int, int], Tuple[int, int]] = {}
    counter = 0
    agent = start
    executed: List[Tuple[int, int]] = [start]
    totalExpanded = 0
    replans = 0
    
    observe(agent)
    
    while agent != goal:
        counter += 1
        replans += 1
        
        #?h(S) is Manhattan distance
        
        def h(s: Tuple[int, int]) -> int:
            return abs(s[0] - agent[0]) + abs(s[1] - agent[1])
    
    searchStart = goal
    searchEnd = agent
    
    g[searchStart] = 0 
    searchStamp[searchStart] = counter
    g[searchEnd] = INF
    searchStamp[searchEnd] = counter
    
    openList = CustomPQ_maxG()
    openList.push(searchStart, h(searchStart), 0)
    closed: set = set()
    
    
    while not openList.is_empty():
        s, f_s, _ = openList.pop()
        
        if g[searchEnd] <= f_s:
            break
        
        if s in closed:
            continue
        closed.add(s)
        totalExpanded += 1
        
        for nb in getNeighbors(s):
            if searchStamp.get(nb, 0):
                g[nb] = INF
                searchStamp[nb] = counter
                
            if g[nb] > g[s] + 1:
                g[nb] = g[s] + 1
                tree[nb] = s
                f_nb = g[nb] + h(nb)
                if openList.contains(nb):
                    openList.decrease_key(nb, f_nb, g[nb])
                    
                else:
                    openList.push(nb, f_nb, g[nb])
                    
    if g[searchEnd] == INF:
        return False, executed, totalExpanded, replans
    
    path: List[Tuple[int, int]] = []
    current = searchEnd
    while current != searchStart:
        path.append(current)
        current = tree[current]
    path.append(searchStart)
    
    
    for i in range(1, len(path)):
        nextCell = path[i]
        if nextCell in knownBlocked:
            break
        agent = nextCell
        executed.append(agent)
        observe(agent)
        if agent == goal:
            return True, executed, totalExpanded, replans
        
        if any(path[j] in knownBlocked for j in range(i + 1, len(path))):
            break
        
    return True, executed, totalExpanded, replans
        
    
    
    # Use heapq for standard priority queue implementation and name your max_g heap class as `CustomPQ_maxG` and use it. 
    pass

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
        
    NL = NODE_LENGTH
    OFF = GRID_LENGTH + GAP
    DIRS = [(-1, 0), (1,0), (0, -1), (0, 1)]
    clock = pygame.time.Clock()
    
    
    
    #?helper methods
    
    def fill(row: int, col: int, color, pane_x: int = 0) -> None:
        pygame.draw.rect(win, color, (pane_x + col * NL, NL, NL))
    
    def gridLines(pane_x: int = 0) -> None:
        for i in range(ROWS + 1):
            pygame.draw.line(win, GREY, (pane_x, i * NL), (pane_x + GRID_LENGTH, i * NL))
            pygame.draw.line(win, GREY, (pane_x + i*NL, 0), (pane_x + i*NL, GRID_LENGTH))
    
    
    def draw_actual() -> None:
        for r in range(ROWS):
            for c in range(ROWS):
                fill(r, c, BLACK if actual_maze[r][c] else WHITE)
        gridLines(0)
        
        
    def drawKnowledge(knownBlocked, openSet, closedSet, pathCells, agent) -> None:
        for r in range(ROWS):
            for c in range(ROWS):
                cell = (r, c)
                if cell in knownBlocked: color = BLACK
                elif cell in closedSet: color = GREY
                elif cell in openSet: color = YELLOW
                else: color = WHITE
                fill(r, c, color, OFF)
                
            
        for cell in pathCells:
            fill(cell[0], cell[1], PATH, OFF)
            
        fill(START_NODE[0], START_NODE[1]. YELLOW, OFF)
        fill(END_NODE[0], END_NODE[1], BLUE, OFF)
        fill(agent[0], agent[1], YELLOW, OFF)
        
        fill(agent[0], agent[1], YELLOW, 0)
        gridLines(OFF)
        
        
    def tick() -> None:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); raise SystemExit
        
        pygame.display.flip()
        clock.tick(fps)
        if step_delay_ms:
            pygame.time.delay(step_delay_ms)
            
            
    
    knownBlocked: set = set()
    
    
    def observe(pos):
        r, c = pos
        for dr, dc in DIRS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < ROWS and 0 <= nc < ROWS:
                if actual_maze[nr][nc] == 1:
                    knownBlocked.add((nr, nc))
                    
    def h(s): return abs(s[0] - -END_NODE[0]) + abs(s[1] - END_NODE[1])
    
    
    def neighbors(s):
        r, c = s
        
        return [(r + dr, c + dc) for dr, dc in DIRS
                if 0 <= r + dr < ROWS and 0 <= c + dc < ROWS
                and (r+dr, c+dc) not in knownBlocked]
        
        
        
    tie = "min_g" if "min" in algo.lower() else "max_g"
    agent = tuple(START_NODE)
    goal = tuple(END_NODE)
    executed = [agent]
    expanded = replans = 0
    found = False
    
    observe(agent)
    draw_actual()
    drawKnowledge(knownBlocked, set(), set(), [], agent)
    tick()
    
    
    INF = float("inf")
    gVal: Dict[Tuple[int, int], float] = {}
    
    while agent != goal:
        replans += 1
        gVal[agent] = 0
        gVal[goal] = INF
        tree: Dict = {}
        
        
        counter_pq = 0
        open_heap  = [(h(agent), 0 if tie=="max_g" else 0, counter_pq, agent)]
        open_set   = {agent}
        closed_set: set = set()

        path = None

        while open_heap:
            f_s, _, _, s = heapq.heappop(open_heap)
            if s in closed_set:
                continue
            if gVal.get(goal, INF) <= f_s:
                break

            closed_set.add(s)
            open_set.discard(s)
            expanded += 1

            # live draw every expansion
            draw_actual()
            drawKnowledge(knownBlocked, open_set, closed_set, [], agent)
            tick()

            for nb in neighbors(s):
                if nb in closed_set:
                    continue
                if nb not in gVal:
                    gVal[nb] = INF
                ng = gVal[s] + 1
                if ng < gVal.get(nb, INF):
                    gVal[nb] = ng
                    tree[nb]  = s
                    f_nb  = ng + h(nb)
                    # tie-break: max_g → negate g so larger g wins
                    tb = -ng if tie == "max_g" else ng
                    counter_pq += 1
                    heapq.heappush(open_heap, (f_nb, tb, counter_pq, nb))
                    open_set.add(nb)

        if gVal.get(goal, INF) == INF:
            print(f"[{algo}] Cannot reach the target.")
            break

        # reconstruct path
        path = []
        cur  = goal
        while cur != agent:
            path.append(cur); cur = tree[cur]
        path.append(agent); path.reverse()

        # draw planned path
        draw_actual()
        drawKnowledge(knownBlocked, set(), closed_set, path, agent)
        tick()

        # follow path
        for step in path[1:]:
            if step in knownBlocked:
                break
            agent = step
            executed.append(agent)
            observe(agent)
            draw_actual()
            drawKnowledge(knownBlocked, set(), closed_set, path, agent)
            tick()
            if agent == goal:
                found = True; break
            if any(p in knownBlocked for p in path[path.index(step)+1:]):
                break
        if found:
            break

    # ── final frame ───────────────────────────────────────────────────────
    draw_actual()
    drawKnowledge(knownBlocked, set(), executed, agent)
    tick()

    print(f"[{algo}] found={found}  executed_steps={len(executed)-1}"
          f"  expanded={expanded}  replans={replans}")

    pygame.image.save(win, save_path)
    print(f"Saved visualization -> {save_path}")
        
        
        
    

    # If 'win' is the display surface (it is), this works:
    pygame.image.save(win, save_path)
    print(f"Saved the visualization -> {save_path}")

def main() -> None:
    parser = argparse.ArgumentParser(description="Q3: Repeated Backward A*")
    parser.add_argument("--maze_file", type=str, required=True,
                        help="Path to input JSON file containing a list of mazes")
    parser.add_argument("--output", type=str, default="results_q3.json",
                        help="Path to output JSON results file")
    parser.add_argument("--show_vis", action="store_true",
                        help="[Bonus] If set, show Pygame visualization for the selected maze")
    parser.add_argument("--maze_vis_id", type=int, default=0,
                        help="[Bonus] maze_id (index) 0 ... 49 among 50 grid worlds")
    parser.add_argument("--save_vis_path", type=str, default="q3-vis-max-g.png",
                        help="[Bonus] If set, save visualization to this PNG file")
    args = parser.parse_args()

    mazes = readMazes(args.maze_file)
    results: List[Dict] = []

    for maze_id in tqdm(range(len(mazes)), desc="Processing mazes"):
        entry: Dict = {"maze_id": maze_id}

        t0 = time.perf_counter()
        found, executed, expanded, replans = repeated_backward_astar(
            actual_maze=mazes[maze_id],
            start=START_NODE,
            goal=END_NODE,
        )
        t1 = time.perf_counter()

        entry["bwd"] = {
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
            tie_breaking="max_g",
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
        pygame.display.set_caption("Repeated Backward A* Visualization")
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