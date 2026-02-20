# Homework 1 Submission Instructions

## Directory Structure

Your submission **must** follow the directory structure shown below:

```
submitting-student-name-netid/
└── hw1/
    ├── team_details.json
    ├── report.pdf
    ├── constants.py            # Grid world constants (START, END, ROWS, colors)
    ├── create_grid_worlds.py   # TODO: Implement maze creation  
    ├── q2.py                   # TODO: Implement Repeated Forward A*
    ├── q3.py                   # TODO: Implement Repeated Backward A*
    └── q5.py                   # TODO: Implement Adaptive A*
```

> **Group submissions:**
> Only use the **name and NetID of the student submitting on behalf of the group** for the top-level directory.
> The names and NetIDs of **all group members** must be listed in `team_details.json` and also in the `report.pdf` file.

---

## File Descriptions

* **team_details.json**

  Contains identifying information for all members of the group.

  ```json
  {
    "names": ["student1-name", "student2-name", "..."],
    "netids": ["student1-netid", "student2-netid", "..."]
  }
  ```

* **report.pdf**
  Contains explanations, observations, and analysis for:

  * Part 1a
  * Part 1b
  * Part 2
  * Part 3
  * Part 4a
  * Part 4b
  * Part 5
  * Part 6

* **create_grid_worlds.py**
    Creates a single JSON file containing k (e.g. k=50) mazes.

* **q2.py**
  Implements Repeated Forward A* with different tie-breaking strategies.

* **q3.py**
  Implements Repeated Backward A* (max_g variant) and compares Repeated Forward A* vs. Repeated Backward A* using max_g tie-braking strategy.

* **q5.py**
  Implements Adaptive A* (max_g variant) and compares Repeated Forward A* vs. Adaptive A* using max_g tie-braking strategy.


## Command-Line Interface (All Python Files)

The script (`q2.py`) must support the following command-line arguments.

### Example usage (q2.py)

```bash
python q2.py \
  --maze_file mazes.json \
  --tie_braking both \
  --show_vis \
  --maze_vis_id 3 \
  --save_vis_path q2-vis-max-g.png \
  --output results_q2.json
```
For, `q3.py` and, `q5.py`, `tie_braking` argument is not present as we are comparing only `max_g` variant of Repeated Backward & Adaptive A* with `max_g` variant of Repeated Forward A*.

#### Argument Descriptions

* `--maze_file`

    Path to an input **JSON file** containing a list of mazes.

    * The file should contain **k grid worlds** (e.g., `k = 50`)
    * Each maze is represented as a **2D list of integers**
    
* `--output`
    
    Path to the output JSON file where experiment results will be saved.
    
    **Default:** `results_q2.json`

* `--tie_braking`
    
    Tie-breaking strategy to run for **Repeated Forward A***.

    **Choices:**

    * `max_g` — break ties by larger g-values
    * `min_g` — break ties by smaller g-values
    * `both` — run and compare both variants

    **Default:** `both`

* `--show_vis` *(Bonus)*

    If set, enables **Pygame visualization** of the search process for a single maze.

    If not set, runs in **headless mode** (no visualization).

    Pygame is for reference only, feel free to use other libraries.

* `--maze_vis_id` *(Bonus)*

    Index of the maze to visualize.

    * Must be in the range `0 ... (k - 1)`
    * Only used if `--show_vis` is enabled

    **Default:** `0`

* `--save_vis_path` *(Bonus)*

    Path to save a **PNG image** of the final visualization for the selected maze.

    **Default:** `q2-vis-max-g.png`

#### Notes

* Visualization is optional
* Runtime statistics are collected **without visualization**
* Output JSON includes **path length**, **expanded nodes**, **replans**, and **runtime (seconds)**


## Expected Output Format for `q2.py`

`q2.py` **must return** a JSON object of the following format:

```json
[
    {
      "maze_id": 0,
      "max_g": {
          "found": false,
          "path_length": -1,
          "expanded": 22949,
          "replans": 148,
          "runtime_ms": 61.949231079779565
      },
      "min_g": {
          "found": false,
          "path_length": -1,
          "expanded": 324397,
          "replans": 148,
          "runtime_ms": 682.2988139465451
      }
    },
    {
      "maze_id": 1,
      "max_g": {
          "found": true,
          "path_length": 392,
          "expanded": 9483,
          "replans": 95,
          "runtime_ms": 30.266137095168233
      },
      "min_g": {
          "found": true,
          "path_length": 392,
          "expanded": 291218,
          "replans": 95,
          "runtime_ms": 612.963492050767
      }
    }
]
```

## Expected Output Format for `q3.py`

`q3.py` **must return** a JSON object of the following format:

```json
[
    {
      "maze_id": 0,
      "bwd": {
          "found": false,
          "path_length": -1,
          "expanded": 22949,
          "replans": 148,
          "runtime_ms": 61.949231079779565
      },
      "fwd": {
          "found": false,
          "path_length": -1,
          "expanded": 324397,
          "replans": 148,
          "runtime_ms": 682.2988139465451
      }
    },
    {
      "maze_id": 1,
      "bwd": {
          "found": true,
          "path_length": 392,
          "expanded": 9483,
          "replans": 95,
          "runtime_ms": 30.266137095168233
      },
      "fwd": {
          "found": true,
          "path_length": 392,
          "expanded": 291218,
          "replans": 95,
          "runtime_ms": 612.963492050767
      }
    }
]
```

## Expected Output Format for `q5.py`

`q5.py` **must return** a JSON object of the following format:

```json
[
    {
      "maze_id": 0,
      "adaptive": {
          "found": false,
          "path_length": -1,
          "expanded": 22949,
          "replans": 148,
          "runtime_ms": 61.949231079779565
      },
      "fwd": {
          "found": false,
          "path_length": -1,
          "expanded": 324397,
          "replans": 148,
          "runtime_ms": 682.2988139465451
      }
    },
    {
      "maze_id": 1,
      "adaptive": {
          "found": true,
          "path_length": 392,
          "expanded": 9483,
          "replans": 95,
          "runtime_ms": 30.266137095168233
      },
      "fwd": {
          "found": true,
          "path_length": 392,
          "expanded": 291218,
          "replans": 95,
          "runtime_ms": 612.963492050767
      }
    }
]
```
#### Notes
* Reporting number of `replans` is optional.
* If you are reporting only one of the metrics `runtime_sec` or `cells_expanded`, please return `None` for the remaining metrics.

### Custom Heap Implementation

Please name the custom heap file as `custom_pq.py` with minG and maxG heap classes named as `CustomPQ_minG`, `CustomPQ_maxG`.
