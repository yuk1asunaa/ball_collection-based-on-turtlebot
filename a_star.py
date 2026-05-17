import heapq
import math
import matplotlib.pyplot as plt
import numpy as np


class DensityAwareAStarPlanner:
    def __init__(self, ox, oy, resolution, rr,
                 obstacle_weight=1.2,
                 density_weight=0.7,
                 min_step_cost=0.6,
                 show_animation=False):
        """
        Density-aware A* planner.

        Edge cost model (matches C++ DensityAwareAStarPlanner):
            step_cost = base_distance
                      + obstacle_weight * obstacle_cost
                      - density_weight * density_value
        step_cost is clamped to >= min_step_cost * base_distance so A*
        assumptions hold (no negative edges).

        Parameters
        ----------
        ox, oy : obstacle point coordinates
        resolution : grid resolution [m/cell]
        rr : robot radius [m]
        obstacle_weight : w_obs, default 1.2
        density_weight : w_den, default 0.7
        min_step_cost : minimum multiplier for step cost, default 0.6
        """
        self.resolution = resolution
        self.rr = rr
        self.obstacle_weight = obstacle_weight
        self.density_weight = density_weight
        self.min_step_cost = min_step_cost
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.density_map = None
        self.x_width, self.y_width = 0, 0
        self.show_animation = show_animation
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)
        self.density_map = [[0.0 for _ in range(self.y_width)]
                            for _ in range(self.x_width)]

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

    # ------------------------------------------------------------------
    # Density map helpers
    # ------------------------------------------------------------------
    def add_gaussian_blob(self, cx, cy, sigma=0.5, amplitude=1.0):
        """Add a 2D Gaussian bump to the density map (simulates a detected ball)."""
        for ix in range(self.x_width):
            wx = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                wy = self.calc_grid_position(iy, self.min_y)
                d2 = (wx - cx) ** 2 + (wy - cy) ** 2
                val = amplitude * math.exp(-d2 / (2 * sigma ** 2))
                self.density_map[ix][iy] = min(1.0, self.density_map[ix][iy] + val)

    def get_density(self, ix, iy):
        """Return density value in [0, 1] at grid cell (ix, iy)."""
        if 0 <= ix < self.x_width and 0 <= iy < self.y_width:
            return self.density_map[ix][iy]
        return 0.0

    # ------------------------------------------------------------------
    # A* planning
    # ------------------------------------------------------------------
    def planning(self, sx, sy, gx, gy):
        start_node = self.Node(
            self.calc_xy_index(sx, self.min_x),
            self.calc_xy_index(sy, self.min_y),
            0.0, -1)
        goal_node = self.Node(
            self.calc_xy_index(gx, self.min_x),
            self.calc_xy_index(gy, self.min_y),
            0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                return [], []

            c_id = min(open_set,
                       key=lambda o: open_set[o].cost +
                       self.calc_heuristic(goal_node, open_set[o]))

            current = open_set[c_id]

            if self.show_animation:
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])

                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            del open_set[c_id]
            closed_set[c_id] = current

            for i, _ in enumerate(self.motion):
                nx = current.x + self.motion[i][0]
                ny = current.y + self.motion[i][1]
                base_dist = self.motion[i][2]

                # Quick bounds check before constructing node / computing cost
                if nx < 0 or nx >= self.x_width or ny < 0 or ny >= self.y_width:
                    continue
                if self.obstacle_map[nx][ny]:
                    continue

                node = self.Node(nx, ny,
                                 current.cost + self._step_cost(nx, ny, base_dist),
                                 c_id)

                n_id = self.calc_grid_index(node)

                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node
                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)
        return rx, ry

    def _step_cost(self, ix, iy, base_distance):
        """Density-aware step cost (matches C++ DensityAwareAStarPlanner)."""
        # Obstacle cost: 0 for free cells, scaled for occupied
        obstacle_cost = 0.0
        if self.obstacle_map[ix][iy]:
            obstacle_cost = 1.0

        density_val = self.get_density(ix, iy)

        step_cost = (base_distance
                     + self.obstacle_weight * obstacle_cost
                     - self.density_weight * density_val)

        # Clamp to positive minimum so A* assumptions hold
        step_cost = max(step_cost, self.min_step_cost * base_distance)
        return step_cost

    # ------------------------------------------------------------------
    # Path reconstruction
    # ------------------------------------------------------------------
    def calc_final_path(self, goal_node, closed_set):
        rx = [self.calc_grid_position(goal_node.x, self.min_x)]
        ry = [self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index
        return rx, ry

    # ------------------------------------------------------------------
    # Motion model
    # ------------------------------------------------------------------
    @staticmethod
    def get_motion_model():
        return [[1, 0, 1],
                [0, 1, 1],
                [-1, 0, 1],
                [0, -1, 1],
                [-1, -1, math.sqrt(2)],
                [-1, 1, math.sqrt(2)],
                [1, -1, math.sqrt(2)],
                [1, 1, math.sqrt(2)]]

    # ------------------------------------------------------------------
    # Grid utilities
    # ------------------------------------------------------------------
    def calc_grid_position(self, index, min_position):
        return index * self.resolution + min_position

    def calc_xy_index(self, position, min_pos):
        return int(math.floor((position - min_pos) / self.resolution))

    def calc_grid_index(self, node):
        return node.y * self.x_width + node.x

    def calc_heuristic(self, n1, n2):
        w = 1.0
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    # ------------------------------------------------------------------
    # Map construction
    # ------------------------------------------------------------------
    def calc_obstacle_map(self, ox, oy):
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))

        self.x_width = int(math.floor((self.max_x - self.min_x) / self.resolution)) + 1
        self.y_width = int(math.floor((self.max_y - self.min_y) / self.resolution)) + 1

        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x or py < self.min_y:
            return False
        if px > self.max_x or py > self.max_y:
            return False
        if self.obstacle_map[node.x][node.y]:
            return False
        return True


# ======================================================================
# Validation: classic vs density-aware A*
# ======================================================================
def _density_at_world(planner, x, y):
    ix = planner.calc_xy_index(x, planner.min_x)
    iy = planner.calc_xy_index(y, planner.min_y)
    return planner.get_density(ix, iy)


def _plan_collect_all(planner, sx, sy, balls,
                      density_weight=1.0, distance_weight=0.2):
    current = (sx, sy)
    remaining = balls[:]
    order = []
    path_x, path_y = [], []

    while remaining:
        best_idx = None
        best_score = -1e9
        for i, (bx, by) in enumerate(remaining):
            density = _density_at_world(planner, bx, by)
            dist = math.hypot(bx - current[0], by - current[1])
            score = density_weight * density - distance_weight * dist
            if score > best_score:
                best_score = score
                best_idx = i

        bx, by = remaining.pop(best_idx)
        order.append((bx, by))

        rx, ry = planner.planning(current[0], current[1], bx, by)
        if not rx:
            current = (bx, by)
            continue
        rx = rx[::-1]
        ry = ry[::-1]
        if path_x:
            path_x.append(None)
            path_y.append(None)
        path_x.extend(rx)
        path_y.extend(ry)
        current = (bx, by)

    return path_x, path_y, order


def main():
    print(__file__ + " start!")

    rng = np.random.default_rng(2)

    sx, sy = 5.0, 5.0
    grid_size = 0.5
    robot_radius = 0.5

    world_min = -5.0
    world_max = 55.0

    # Boundary walls
    ox, oy = [], []
    for i in range(int(world_min), int(world_max) + 1):
        ox.append(i);       oy.append(world_min)
        ox.append(i);       oy.append(world_max)
        ox.append(world_min); oy.append(i)
        ox.append(world_max); oy.append(i)

    # Random ball generation with clustered density
    num_balls = 30
    cluster_centers = [(12.0, 40.0), (35.0, 35.0), (45.0, 12.0)]
    cluster_sigma = 2.5

    balls = []
    per_cluster = num_balls // len(cluster_centers)
    for cx, cy in cluster_centers:
        for _ in range(per_cluster):
            for _ in range(100):
                bx = rng.normal(cx, cluster_sigma)
                by = rng.normal(cy, cluster_sigma)
                if world_min + 2.0 <= bx <= world_max - 2.0 and world_min + 2.0 <= by <= world_max - 2.0:
                    balls.append((bx, by))
                    break

    while len(balls) < num_balls:
        bx = rng.uniform(world_min + 2.0, world_max - 2.0)
        by = rng.uniform(world_min + 2.0, world_max - 2.0)
        balls.append((bx, by))

    planner = DensityAwareAStarPlanner(
        ox, oy, grid_size, robot_radius,
        obstacle_weight=1.2, density_weight=0.7, min_step_cost=0.6,
        show_animation=False)

    for bx, by in balls:
        planner.add_gaussian_blob(bx, by, sigma=1.2, amplitude=1.0)

    rx, ry, visit_order = _plan_collect_all(planner, sx, sy, balls)

    fig, ax = plt.subplots(1, 1, figsize=(10, 8))
    ax.plot(ox, oy, ".k", alpha=0.6)
    ax.plot(sx, sy, "og", label="start")

    den = np.array(planner.density_map).T
    extent = [planner.min_x, planner.max_x, planner.min_y, planner.max_y]
    im = ax.imshow(den, origin='lower', extent=extent,
                   cmap='YlOrRd', alpha=0.5, interpolation='bilinear')
    plt.colorbar(im, ax=ax, shrink=0.8, label='density')

    bx = [p[0] for p in balls]
    by = [p[1] for p in balls]
    ax.plot(bx, by, "ob", markersize=4, label="balls")

    if rx:
        ax.plot(rx, ry, "-r", linewidth=2, label="density-aware path")

    for i, (bx_i, by_i) in enumerate(visit_order, start=1):
        ax.text(bx_i + 0.3, by_i + 0.3, str(i), fontsize=8, color="black")

    ax.set_title("Density-Aware A*: prioritize dense clusters and visit all balls")
    ax.grid(True)
    ax.axis("equal")
    ax.legend()

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
