/**
 * =============================================
 * PathFinder AI — script.js
 * Algorithms: A* (A-Star), Dijkstra, BFS
 * Features: animated visualization, obstacles,
 *           heuristics, diagonal movement
 * =============================================
 */

// ── Configuration ──────────────────────────
const ROWS = 22;
const COLS = 40;

// Animation delay in ms per step (indexed 1–5)
const SPEED_MAP = { 1: 80, 2: 40, 3: 20, 4: 8, 5: 2 };

// ── State ───────────────────────────────────
let grid = [];          // 2D array of cell objects
let startNode = null;   // {row, col}
let endNode = null;     // {row, col}
let drawMode = 'start'; // 'start' | 'end' | 'wall' | 'erase'
let algorithm = 'astar';
let isRunning = false;
let isMouseDown = false;
let diagonalAllowed = false;
let showHeuristic = true;
let animSpeed = 3;
let animationCancelled = false;

// ── DOM References ──────────────────────────
const gridEl = document.getElementById('grid');
const gridInfo = document.getElementById('grid-info');
const statsEl = document.getElementById('stats');
const statVisited = document.getElementById('stat-visited');
const statPath = document.getElementById('stat-path');
const statLength = document.getElementById('stat-length');
const btnStart = document.getElementById('btn-start');
const btnReset = document.getElementById('btn-reset');
const btnClear = document.getElementById('btn-clear');
const speedSlider = document.getElementById('speed-slider');
const speedDisplay = document.getElementById('speed-display');

// ─────────────────────────────────────────────
// 1. GRID CREATION
// ─────────────────────────────────────────────

/**
 * Build the 2D grid array and render cells into DOM.
 */
function createGrid() {
    gridEl.style.gridTemplateColumns = `repeat(${COLS}, var(--cell))`;
    gridEl.innerHTML = '';
    grid = [];

    for (let r = 0; r < ROWS; r++) {
        grid[r] = [];
        for (let c = 0; c < COLS; c++) {
            const node = {
                row: r, col: c,
                isWall: false,
                isStart: false,
                isEnd: false,
                // Pathfinding state
                g: Infinity, h: 0, f: Infinity,
                dist: Infinity,
                parent: null,
                visited: false,
                inQueue: false,
                el: null,
            };
            grid[r][c] = node;

            const cell = document.createElement('div');
            cell.className = 'cell';
            cell.dataset.row = r;
            cell.dataset.col = c;

            // Mouse events for painting
            cell.addEventListener('mousedown', (e) => {
                if (isRunning) return;
                isMouseDown = true;
                handleCellInteraction(r, c, e.shiftKey);
                e.preventDefault();
            });
            cell.addEventListener('mouseenter', (e) => {
                if (!isMouseDown || isRunning) return;
                // shift+drag = erase
                const erasingMode = e.shiftKey ? 'erase' : drawMode;
                if (erasingMode === 'wall' || erasingMode === 'erase') {
                    handleCellInteraction(r, c, e.shiftKey);
                }
            });

            node.el = cell;
            gridEl.appendChild(cell);
        }
    }

    // Stop dragging when mouse leaves grid
    gridEl.addEventListener('mouseleave', () => { isMouseDown = false; });
    document.addEventListener('mouseup', () => { isMouseDown = false; });
}

/**
 * Handle a cell click/drag interaction based on current draw mode.
 */
function handleCellInteraction(r, c, shiftHeld) {
    const node = grid[r][c];
    const currentMode = shiftHeld ? 'erase' : drawMode;

    if (currentMode === 'start') {
        // Clear existing start
        if (startNode) {
            const prev = grid[startNode.row][startNode.col];
            prev.isStart = false;
            updateCellClass(prev);
        }
        node.isStart = true;
        node.isWall = false;
        startNode = { row: r, col: c };
        updateCellClass(node);
        setInfo('Start set. Now place End node or draw walls.');

    } else if (currentMode === 'end') {
        if (endNode) {
            const prev = grid[endNode.row][endNode.col];
            prev.isEnd = false;
            updateCellClass(prev);
        }
        node.isEnd = true;
        node.isWall = false;
        endNode = { row: r, col: c };
        updateCellClass(node);
        setInfo('End set. Draw walls or click "Start Visualization".');

    } else if (currentMode === 'wall') {
        if (node.isStart || node.isEnd) return;
        node.isWall = true;
        updateCellClass(node);

    } else if (currentMode === 'erase') {
        if (node.isStart) { startNode = null; node.isStart = false; }
        if (node.isEnd) { endNode = null; node.isEnd = false; }
        node.isWall = false;
        clearVisualState(node);
        updateCellClass(node);
    }
}

/**
 * Derive and apply the CSS class from node state.
 */
function updateCellClass(node) {
    const cl = node.el.classList;
    cl.remove('start', 'end', 'wall', 'visited', 'open', 'path');
    if (node.isStart) cl.add('start');
    else if (node.isEnd) cl.add('end');
    else if (node.isWall) cl.add('wall');
}

/**
 * Reset pathfinding visual state on a node (visited/path classes).
 */
function clearVisualState(node) {
    node.el.classList.remove('visited', 'open', 'path');
}

// ─────────────────────────────────────────────
// 2. UTILITY HELPERS
// ─────────────────────────────────────────────

/** Manhattan distance heuristic */
function manhattan(a, b) {
    return Math.abs(a.row - b.row) + Math.abs(a.col - b.col);
}

/** Euclidean distance (used for diagonal A*) */
function euclidean(a, b) {
    return Math.sqrt((a.row - b.row) ** 2 + (a.col - b.col) ** 2);
}

/** Get valid neighbors for a node */
function getNeighbors(node) {
    const neighbors = [];
    const { row, col } = node;

    const dirs = [
        [-1, 0, 1], [1, 0, 1], [0, -1, 1], [0, 1, 1],  // cardinal
    ];
    if (diagonalAllowed) {
        dirs.push(
            [-1, -1, 1.414], [-1, 1, 1.414],
            [1, -1, 1.414], [1, 1, 1.414],
        );
    }

    for (const [dr, dc, cost] of dirs) {
        const nr = row + dr, nc = col + dc;
        if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS) {
            const n = grid[nr][nc];
            if (!n.isWall) neighbors.push({ node: n, cost });
        }
    }
    return neighbors;
}

/** Reconstruct path from end back to start */
function reconstructPath(endNode) {
    const path = [];
    let cur = endNode;
    while (cur !== null) {
        path.unshift(cur);
        cur = cur.parent;
    }
    return path;
}

/** Set info bar message */
function setInfo(msg) {
    gridInfo.textContent = msg;
}

/** Simple min-heap (priority queue) */
class MinHeap {
    constructor() { this.data = []; }
    push(item, priority) { this.data.push({ item, priority }); this.data.sort((a, b) => a.priority - b.priority); }
    pop() { return this.data.shift().item; }
    get size() { return this.data.length; }
    get isEmpty() { return this.data.length === 0; }
}

/** Sleep for animation timing */
function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

// ─────────────────────────────────────────────
// 3. ALGORITHMS
// ─────────────────────────────────────────────

/**
 * A* Search
 * Uses g (cost so far) + h (heuristic) = f (estimated total cost)
 * Returns { visited, path } arrays of nodes for animation.
 */
function runAstar(start, end) {
    // Reset node state
    resetNodeState();

    const openSet = new MinHeap();
    start.g = 0;
    start.h = showHeuristic ? manhattan(start, end) : 0;
    start.f = start.g + start.h;
    openSet.push(start, start.f);
    start.inQueue = true;

    const visited = [];

    while (!openSet.isEmpty) {
        const current = openSet.pop();
        if (current.visited) continue;
        current.visited = true;
        visited.push(current);

        // Found the end!
        if (current === end) break;

        for (const { node: neighbor, cost } of getNeighbors(current)) {
            if (neighbor.visited) continue;

            const tentativeG = current.g + cost;
            if (tentativeG < neighbor.g) {
                neighbor.parent = current;
                neighbor.g = tentativeG;
                neighbor.h = showHeuristic ? manhattan(neighbor, end) : 0;
                neighbor.f = neighbor.g + neighbor.h;
                openSet.push(neighbor, neighbor.f);
                neighbor.inQueue = true;
            }
        }
    }

    const path = end.parent !== null || end === start ? reconstructPath(end) : [];
    return { visited, path };
}

/**
 * Dijkstra's Algorithm
 * Explores by minimum cumulative cost (no heuristic).
 */
function runDijkstra(start, end) {
    resetNodeState();

    const pq = new MinHeap();
    start.dist = 0;
    pq.push(start, 0);

    const visited = [];

    while (!pq.isEmpty) {
        const current = pq.pop();
        if (current.visited) continue;
        current.visited = true;
        visited.push(current);

        if (current === end) break;

        for (const { node: neighbor, cost } of getNeighbors(current)) {
            if (neighbor.visited) continue;

            const newDist = current.dist + cost;
            if (newDist < neighbor.dist) {
                neighbor.dist = newDist;
                neighbor.parent = current;
                pq.push(neighbor, newDist);
            }
        }
    }

    const path = end.parent !== null || end === start ? reconstructPath(end) : [];
    return { visited, path };
}

/**
 * Breadth-First Search (BFS)
 * Explores layer by layer — finds shortest path in unweighted graphs.
 */
function runBFS(start, end) {
    resetNodeState();

    const queue = [start];
    start.visited = true;

    const visited = [];

    while (queue.length > 0) {
        const current = queue.shift();
        visited.push(current);

        if (current === end) break;

        for (const { node: neighbor } of getNeighbors(current)) {
            if (!neighbor.visited) {
                neighbor.visited = true;
                neighbor.parent = current;
                queue.push(neighbor);
            }
        }
    }

    const path = end.parent !== null || end === start ? reconstructPath(end) : [];
    return { visited, path };
}

/** Reset per-node pathfinding data before a new run */
function resetNodeState() {
    for (let r = 0; r < ROWS; r++) {
        for (let c = 0; c < COLS; c++) {
            const n = grid[r][c];
            n.g = Infinity;
            n.h = 0;
            n.f = Infinity;
            n.dist = Infinity;
            n.parent = null;
            n.visited = false;
            n.inQueue = false;
        }
    }
}

// ─────────────────────────────────────────────
// 4. ANIMATION ENGINE
// ─────────────────────────────────────────────

/**
 * Animate the visited nodes one by one, then animate the path.
 */
async function animate(visited, path) {
    const delay = SPEED_MAP[animSpeed] || 20;

    // Animate visited nodes (exclude start and end)
    for (let i = 0; i < visited.length; i++) {
        if (animationCancelled) return;
        const node = visited[i];
        if (!node.isStart && !node.isEnd) {
            node.el.classList.add('visited');
        }
        if (i % 3 === 0) await sleep(delay); // batch a few steps for speed
    }

    statVisited.textContent = `Visited: ${visited.length}`;

    if (path.length === 0) {
        showNoPath();
        return;
    }

    // Animate path nodes
    for (let i = 0; i < path.length; i++) {
        if (animationCancelled) return;
        const node = path[i];
        if (!node.isStart && !node.isEnd) {
            node.el.classList.remove('visited');
            node.el.classList.add('path');
        }
        await sleep(delay * 3);
    }

    // Calculate path length (sum of costs, rounded)
    const length = path.length - 1;
    const costLabel = diagonalAllowed
        ? (path.reduce((acc, n, i) => {
            if (i === 0) return 0;
            const prev = path[i - 1];
            const isDiag = prev.row !== n.row && prev.col !== n.col;
            return acc + (isDiag ? 1.414 : 1);
        }, 0)).toFixed(1)
        : length;

    statPath.textContent = `Path: ${length} steps`;
    statLength.textContent = `Length: ${costLabel}`;
    statsEl.style.display = 'flex';
}

function showNoPath() {
    setInfo('⚠ No path found! The end is completely blocked by walls.');
    const msg = document.createElement('div');
    msg.className = 'no-path-msg';
    msg.textContent = '✕  No path exists';
    document.body.appendChild(msg);
    setTimeout(() => msg.remove(), 2600);
}

// ─────────────────────────────────────────────
// 5. MAIN CONTROL FLOW
// ─────────────────────────────────────────────

async function startVisualization() {
    if (isRunning) return;

    if (!startNode || !endNode) {
        setInfo('⚠ Please place both a Start and End node first.');
        return;
    }

    // Clear previous visual state
    resetVisuals();

    isRunning = true;
    animationCancelled = false;
    document.body.classList.add('running');
    btnStart.disabled = true;
    statsEl.style.display = 'none';
    statVisited.textContent = 'Visited: 0';
    statPath.textContent = 'Path: —';
    statLength.textContent = 'Length: —';

    const sNode = grid[startNode.row][startNode.col];
    const eNode = grid[endNode.row][endNode.col];

    setInfo(`Running ${algorithm.toUpperCase()}...`);

    let result;
    if (algorithm === 'astar') {
        result = runAstar(sNode, eNode);
        setInfo(`A* explored ${result.visited.length} nodes.`);
    } else if (algorithm === 'dijkstra') {
        result = runDijkstra(sNode, eNode);
        setInfo(`Dijkstra explored ${result.visited.length} nodes.`);
    } else {
        result = runBFS(sNode, eNode);
        setInfo(`BFS explored ${result.visited.length} nodes.`);
    }

    await animate(result.visited, result.path);

    if (!animationCancelled) {
        if (result.path.length > 0) {
            setInfo(`✓ Path found! ${result.path.length - 1} steps · ${result.visited.length} nodes explored.`);
        }
        statsEl.style.display = 'flex';
    }

    isRunning = false;
    document.body.classList.remove('running');
    btnStart.disabled = false;
}

/** Reset only the path visualization, keep walls/start/end */
function resetPath() {
    if (isRunning) {
        animationCancelled = true;
        isRunning = false;
        document.body.classList.remove('running');
        btnStart.disabled = false;
    }
    resetVisuals();
    statsEl.style.display = 'none';
    setInfo('Path cleared. Adjust walls or re-run visualization.');
}

/** Remove visited/path classes from all non-start/end cells */
function resetVisuals() {
    for (let r = 0; r < ROWS; r++) {
        for (let c = 0; c < COLS; c++) {
            const node = grid[r][c];
            node.el.classList.remove('visited', 'open', 'path');
        }
    }
}

/** Wipe entire grid back to empty */
function clearGrid() {
    if (isRunning) {
        animationCancelled = true;
        isRunning = false;
        document.body.classList.remove('running');
        btnStart.disabled = false;
    }
    startNode = null;
    endNode = null;
    for (let r = 0; r < ROWS; r++) {
        for (let c = 0; c < COLS; c++) {
            const node = grid[r][c];
            node.isWall = false;
            node.isStart = false;
            node.isEnd = false;
            node.el.className = 'cell';
        }
    }
    statsEl.style.display = 'none';
    setInfo('Grid cleared. Click to place Start, End and Walls — then visualize!');
}

// ─────────────────────────────────────────────
// 6. EVENT LISTENERS
// ─────────────────────────────────────────────

// Buttons
btnStart.addEventListener('click', startVisualization);
btnReset.addEventListener('click', resetPath);
btnClear.addEventListener('click', clearGrid);

// Algorithm selector
document.querySelectorAll('.algo-btn').forEach(btn => {
    btn.addEventListener('click', () => {
        if (isRunning) return;
        document.querySelectorAll('.algo-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        algorithm = btn.dataset.algo;
        resetPath();
    });
});

// Draw mode selector
document.querySelectorAll('.mode-btn').forEach(btn => {
    btn.addEventListener('click', () => {
        document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        drawMode = btn.dataset.mode;
    });
});

// Diagonal toggle
document.getElementById('diagonal-toggle').addEventListener('change', (e) => {
    diagonalAllowed = e.target.checked;
    if (!isRunning) resetPath();
});

// Heuristic toggle
document.getElementById('heuristic-toggle').addEventListener('change', (e) => {
    showHeuristic = e.target.checked;
    if (!isRunning) resetPath();
});

// Speed slider
speedSlider.addEventListener('input', (e) => {
    animSpeed = parseInt(e.target.value);
    speedDisplay.textContent = `Speed: ${animSpeed}×`;
});

// Keyboard shortcuts
document.addEventListener('keydown', (e) => {
    if (e.key === ' ' || e.key === 'Enter') {
        e.preventDefault();
        if (!isRunning) startVisualization();
    }
    if (e.key === 'r' || e.key === 'R') resetPath();
    if (e.key === 'c' || e.key === 'C') clearGrid();
    // Quick mode switch
    if (e.key === 's') setDrawMode('start');
    if (e.key === 'e') setDrawMode('end');
    if (e.key === 'w') setDrawMode('wall');
    if (e.key === 'x') setDrawMode('erase');
});

function setDrawMode(mode) {
    drawMode = mode;
    document.querySelectorAll('.mode-btn').forEach(b => {
        b.classList.toggle('active', b.dataset.mode === mode);
    });
}

// Touch support (mobile drag for walls)
let touchDrawing = false;
gridEl.addEventListener('touchstart', (e) => {
    touchDrawing = true;
    const touch = e.touches[0];
    const el = document.elementFromPoint(touch.clientX, touch.clientY);
    if (el && el.classList.contains('cell')) {
        handleCellInteraction(+el.dataset.row, +el.dataset.col, false);
    }
    e.preventDefault();
}, { passive: false });

gridEl.addEventListener('touchmove', (e) => {
    if (!touchDrawing) return;
    const touch = e.touches[0];
    const el = document.elementFromPoint(touch.clientX, touch.clientY);
    if (el && el.classList.contains('cell') && (drawMode === 'wall' || drawMode === 'erase')) {
        handleCellInteraction(+el.dataset.row, +el.dataset.col, false);
    }
    e.preventDefault();
}, { passive: false });

gridEl.addEventListener('touchend', () => { touchDrawing = false; });

// ─────────────────────────────────────────────
// 7. INITIALIZATION
// ─────────────────────────────────────────────

function init() {
    createGrid();

    // Place default start and end nodes
    const defaultStart = { row: Math.floor(ROWS / 2), col: 4 };
    const defaultEnd = { row: Math.floor(ROWS / 2), col: COLS - 5 };

    handleCellInteraction(defaultStart.row, defaultStart.col, false);
    drawMode = 'end';
    handleCellInteraction(defaultEnd.row, defaultEnd.col, false);
    drawMode = 'wall';

    // Switch mode UI to 'wall'
    document.querySelectorAll('.mode-btn').forEach(b => {
        b.classList.toggle('active', b.dataset.mode === 'wall');
    });

    setInfo('Click or drag to draw walls — then press "Start Visualization"!');
}

init();