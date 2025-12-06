# Robotic Path Planning Visualizer

**Created by Alex Adamov**

An interactive, 3D educational tool for understanding how robots plan movement around obstacles.

## Background

I started learning about robotic algorithms with Gemini 3. I initially built the physics and logic engine for my own understanding, but as it came together, I decided to create an interactive tutorial to share these concepts with others.

## What Does This App Do?

Imagine you have a robot arm and you want it to grab an object behind a wall. You can't just move straight there—the arm would hit the wall. You need to figure out a path that snakes "around" the obstacles.

This application visualizes how computers solve this problem. It takes you through a journey from naive approaches that fail, to advanced "tree-growing" algorithms used in real-world robotics (like self-driving cars and industrial automation) that can explore complex spaces to find a safe path.

## How It Works

The app creates a 3D simulation of a multi-jointed robot arm. You can define obstacles (walls, gates, corridors) and a target position. The app then visualizes three different strategies for reaching that target:

1.  **The Greedy Approach (Cyclic Coordinate Descent)**:

    - **Concept**: "Move every joint to get closer to the target right now."
    - **Result**: It works great in open space but fails miserably with obstacles. It gets stuck trying to go "through" a wall because it never thinks to go "around" it.

2.  **Standard RRT (Rapidly-exploring Random Tree)**:

    - **Concept**: "Let's explore blindly." The robot randomly samples a point in space and tries to extend a branch toward it. Over time, it builds a massive "tree" of safe movements.
    - **Result**: It eventually finds a path, but it's slow and explores the entire room just to move a few inches.

3.  **RRT-Connect (Bi-directional)**:
    - **Concept**: "Let's meet in the middle." One tree grows from the start, and another grows backward from the goal. They aggressively try to connect to each other.
    - **Result**: Much faster and more efficient. This is the industry-standard approach for many motion planning problems.

### Technical Details

The project is built with:

- **React**: For the UI and state management.
- **Three.js**: For the 3D rendering and simulation environment.
- **Web Workers**: To run the heavy RRT calculations off the main thread, keeping the UI smooth even while calculating thousands of path nodes.

## Contributing

This is an open educational project. I invite comments, additions, and improvements! Whether it's adding new algorithms (like RRT\* or PRM), improving the visualization, or fixing bugs, your contributions are welcome.

## Support

If you found this useful or interesting:

<a href="https://buymeacoffee.com/ysqander" target="_blank"><img src="https://cdn.buymeacoffee.com/buttons/v2/default-yellow.png" alt="Buy Me A Coffee" style="height: 60px !important;width: 217px !important;" ></a>

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
