# PathFinder AI

![PathFinder AI](https://img.shields.io/badge/Status-Active-brightgreen) ![Version](https://img.shields.io/badge/Version-1.0-blue)

A real-time interactive visualization tool for exploring and comparing shortest path finding algorithms. Watch how different algorithms navigate through obstacles to find the optimal path.

## Features

✨ **Interactive Grid-based Visualization**
- Click to place start and end nodes
- Draw walls and obstacles
- Real-time algorithm execution with smooth animations
- Adjustable animation speed (1x - 5x)

🎯 **Multiple Algorithms**
- **A* (A-Star)** - Optimal pathfinding with heuristics
- **Dijkstra** - Guaranteed shortest path
- **BFS (Breadth-First Search)** - Unweighted shortest path

🛠️ **Customizable Options**
- Diagonal movement toggle
- Heuristic visualization
- Speed control
- Multiple drawing modes (walls, erase)

📊 **Statistics & Legend**
- Real-time stats: nodes visited, path length
- Visual legend for different cell types
- Instant feedback on algorithm performance

## How to Use

1. **Select Algorithm** - Choose from A*, Dijkstra, or BFS
2. **Place Start Node** - Click "Start Node" mode and click on grid
3. **Place End Node** - Click "End Node" mode and click on grid
4. **Draw Obstacles** - Click "Draw Wall" mode to create barriers
5. **Configure** - Toggle diagonal movement or heuristic display
6. **Run** - Click "▶ Start Visualization" to execute the algorithm
7. **Reset/Clear** - Use Reset Path or Clear Grid buttons as needed

## Controls

| Action | Description |
|--------|-------------|
| Left Click | Draw based on selected mode |
| Shift + Drag | Quick erase mode |
| Speed Slider | Control animation speed |
| Diagonal Toggle | Enable/disable diagonal movement |
| Heuristic Toggle | Show/hide heuristic calculation |

## Color Legend

| Color | Cell Type |
|-------|-----------|
| 🟢 Green | Start Node |
| 🔴 Red | End Node |
| 🔵 Blue | Wall/Obstacle |
| 🟣 Light Blue | Visited Nodes |
| ⭐ Yellow | Path Found |

## Algorithms Explained

### A* (A-Star)
The most efficient algorithm using heuristics to prioritize promising paths. Combines actual distance from start with estimated distance to goal.

### Dijkstra
A classic algorithm that guarantees the shortest path by exploring all directions equally. Slower than A* but doesn't rely on heuristics.

### BFS (Breadth-First Search)
Explores nodes level by level. Works well for unweighted grids and guarantees shortest path in terms of node count.

## Installation

Simply open `index.html` in your web browser. No dependencies or build process required!

```bash
# Navigate to project directory
cd SPF

# Open in browser (or just double-click index.html)
# Works offline - no internet required
```

## Technologies Used

- **HTML5** - Structure
- **CSS3** - Styling with CSS Variables, Flexbox, Grid
- **Vanilla JavaScript** - Algorithm implementation and DOM manipulation
- **Google Fonts** - Syne & Space Mono typefaces

## Project Structure

```
SPF/
├── index.html       # Main HTML structure
├── style.css        # Styling and animations
├── script.js        # Algorithm implementations and logic
└── README.md        # Documentation (this file)
```

## Browser Compatibility

Works on all modern browsers:
- Chrome/Chromium ✓
- Firefox ✓
- Safari ✓
- Edge ✓

## Tips & Tricks

💡 **Pro Tips:**
- Use A* for best performance with heuristic-enabled visualization
- Compare algorithms by running them on the same grid
- Draw complex mazes to see how algorithms handle challenging paths
- Use diagonal movement for faster pathfinding (less cells to traverse)
- Adjust speed to 5x to quickly see results

## License

Feel free to use and modify this project as needed.

---

**Made with ❤️ for algorithm visualization enthusiasts**
