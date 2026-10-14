// Warehouse Fleet Simulator v2 | SFML 3 | C++17
// Setup screen, configurable fleet, BFS/A*, traffic coordination, and end summary.
#include <SFML/Graphics.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <optional>
#include <queue>
#include <random>
#include <sstream>
#include <string>
#include <vector>

constexpr int CELL = 40;
constexpr int ROWS = 20;
constexpr int COLS = 30;
constexpr int TOTAL = ROWS * COLS;
constexpr int WIDTH = CELL * COLS;
constexpr int GRID_HEIGHT = CELL * ROWS;
constexpr int HEIGHT = GRID_HEIGHT + 176;
constexpr int MAX_ROBOTS = 12;
constexpr std::array<float, 3> SPEEDS = {0.24f, 0.16f, 0.09f};
constexpr std::array<int, 3> JAM_SECONDS = {6, 12, 20};

struct Cell { int row = 0; int col = 0; };
bool operator==(Cell a, Cell b) { return a.row == b.row && a.col == b.col; }
bool operator!=(Cell a, Cell b) { return !(a == b); }
int indexOf(Cell c) { return c.row * COLS + c.col; }
Cell cellOf(int id) { return {id / COLS, id % COLS}; }
bool inside(Cell c) { return c.row >= 0 && c.row < ROWS && c.col >= 0 && c.col < COLS; }
int distance(Cell a, Cell b) { return std::abs(a.row - b.row) + std::abs(a.col - b.col); }

struct Settings {
    int robotCount = 5;
    int targetOrders = 50;
    bool astar = true;
    int speedIndex = 1;
    int jamIndex = 1;
};

struct SearchResult {
    std::vector<Cell> path; // Excludes start
    int expanded = 0;
    double microseconds = 0;
};

// Search on a static grid, treating all other robot positions as temporary obstacles.
// An occupied target can be searched toward, but is never entered while occupied.
SearchResult searchPath(const int (&map)[ROWS][COLS],
                        const std::array<bool, TOTAL>& occupied,
                        Cell start, Cell target, bool astar) {
    auto started = std::chrono::steady_clock::now();
    SearchResult answer;
    constexpr int INF = std::numeric_limits<int>::max();
    std::array<int, TOTAL> cost;
    std::array<int, TOTAL> parent;
    cost.fill(INF);
    parent.fill(-1);
    const int begin = indexOf(start);
    const int end = indexOf(target);
    cost[begin] = 0;

    struct Item {
        int id, g, f;
        bool operator<(const Item& rhs) const {
            if (f != rhs.f) return f > rhs.f;
            return g < rhs.g;
        }
    };
    std::queue<int> bfs;
    std::priority_queue<Item> astarQueue;
    if (astar) astarQueue.push({begin, 0, distance(start, target)});
    else bfs.push(begin);

    const int dRow[] = {-1, 0, 1, 0};
    const int dCol[] = {0, 1, 0, -1};
    while (astar ? !astarQueue.empty() : !bfs.empty()) {
        int current = -1;
        if (astar) {
            Item item = astarQueue.top();
            astarQueue.pop();
            if (item.g != cost[item.id]) continue;
            current = item.id;
        } else {
            current = bfs.front();
            bfs.pop();
        }
        ++answer.expanded;
        if (current == end) break;
        Cell here = cellOf(current);
        for (int d = 0; d < 4; ++d) {
            Cell next{here.row + dRow[d], here.col + dCol[d]};
            if (!inside(next) || map[next.row][next.col] != 0) continue;
            int id = indexOf(next);
            if (occupied[id] && id != end) continue;
            if (cost[current] + 1 >= cost[id]) continue;
            cost[id] = cost[current] + 1;
            parent[id] = current;
            if (astar) astarQueue.push({id, cost[id], cost[id] + distance(next, target)});
            else bfs.push(id);
        }
    }
    if (cost[end] != INF) {
        for (int at = end; at != begin; at = parent[at]) {
            answer.path.push_back(cellOf(at));
        }
        std::reverse(answer.path.begin(), answer.path.end());
    }
    auto finished = std::chrono::steady_clock::now();
    answer.microseconds = std::chrono::duration<double, std::micro>(finished - started).count();
    return answer;
}

struct Robot {
    Cell position;
    Cell pickup{-1, -1};
    sf::Color color;
    bool carrying = false;
    int delivered = 0;
    int steps = 0;
    int waits = 0;
    std::vector<Cell> path;
};

struct Comparison {
    long long searches = 0;
    long long bfsExpanded = 0;
    long long astarExpanded = 0;
    double bfsMicro = 0;
    double astarMicro = 0;
    long long sameLengths = 0;
};

constexpr Cell DOCK{17, 27};
const std::array<Cell, MAX_ROBOTS> SPAWNS = {{
    {1, 1}, {1, 27}, {17, 1}, {17, 15}, {8, 27}, {8, 1},
    {1, 14}, {17, 21}, {14, 27}, {14, 1}, {8, 15}, {1, 20}
}};
const std::array<sf::Color, MAX_ROBOTS> COLORS = {{
    sf::Color(65, 145, 255), sf::Color(255, 130, 65),
    sf::Color(60, 220, 225), sf::Color(230, 100, 220),
    sf::Color(255, 205, 50), sf::Color(110, 225, 110),
    sf::Color(255, 105, 160), sf::Color(170, 180, 245),
    sf::Color(240, 170, 115), sf::Color(130, 210, 180),
    sf::Color(190, 125, 255), sf::Color(255, 220, 165)
}};

class Simulator {
public:
    Settings settings;
    int warehouse[ROWS][COLS] = {};
    std::vector<Cell> pickupCells;
    std::vector<Robot> robots;
    Comparison comparison;
    long long ticks = 0;
    long long lastMovementTick = 0;
    long long lastDeliveryTick = 0;
    int completed = 0;
    int selectedRobot = 0;
    bool paused = false;
    bool manualFirst = false;
    bool over = false;
    std::string endReason;

    void reset(const Settings& options) {
        settings = options;
        robots.clear();
        pickupCells.clear();
        comparison = {};
        ticks = lastMovementTick = lastDeliveryTick = 0;
        completed = selectedRobot = 0;
        paused = manualFirst = over = false;
        endReason.clear();
        trappedTicks = stationBlockedTicks = 0;
        rng.seed(std::random_device{}());

        for (int r = 0; r < ROWS; ++r) {
            for (int c = 0; c < COLS; ++c) {
                warehouse[r][c] = (r == 0 || c == 0 || r == ROWS-1 || c == COLS-1) ? 1 : 0;
            }
        }
        for (int sc = 5; sc <= 23; sc += 6) {
            for (int sr = 3; sr <= 10; sr += 7) {
                for (int r = sr; r < sr + 4; ++r) {
                    for (int c = sc; c < sc + 2; ++c)
                        warehouse[r][c] = 2;
                }
            }
        }
        for (int r = 1; r < ROWS - 1; ++r) {
            for (int c = 1; c < COLS - 1; ++c) {
                if (warehouse[r][c] != 0) continue;
                bool besideShelf = warehouse[r-1][c] == 2 || warehouse[r+1][c] == 2 ||
                                   warehouse[r][c-1] == 2 || warehouse[r][c+1] == 2;
                if (besideShelf) pickupCells.push_back({r,c});
            }
        }
        for (int i = 0; i < settings.robotCount; ++i) {
            Robot bot;
            bot.position = SPAWNS[i];
            bot.color = COLORS[i];
            robots.push_back(bot);
        }
        for (int i = 0; i < settings.robotCount; ++i) assignPickup(i);
    }

    float interval() const { return SPEEDS[settings.speedIndex]; }
    double seconds() const { return ticks * static_cast<double>(interval()); }
    double ordersPerMinute() const {
        return seconds() == 0.0 ? 0.0 : completed * 60.0 / seconds();
    }
    int totalSteps() const {
        int sum = 0;
        for (const Robot& r : robots) sum += r.steps;
        return sum;
    }
    int totalWaits() const {
        int sum = 0;
        for (const Robot& r : robots) sum += r.waits;
        return sum;
    }
    double utilization() const {
        const int moves = totalSteps(), waits = totalWaits();
        return moves + waits == 0 ? 0.0 : 100.0 * moves / (moves + waits);
    }
    void end(const std::string& reason) {
        over = true;
        endReason = reason;
    }

    void step(Cell manualDirection) {
        if (paused || over) return;
        ++ticks;
        bool anybodyMoved = false;
        // Priority rotates, avoiding favoritism toward lower robot IDs.
        for (int rank = 0; rank < settings.robotCount; ++rank) {
            int i = static_cast<int>((ticks + rank) % settings.robotCount);
            Robot& bot = robots[i];
            checkTask(i);
            if (completed >= settings.targetOrders) {
                end("ORDER TARGET COMPLETED");
                return;
            }
            Cell goal = bot.carrying ? DOCK : bot.pickup;
            std::array<bool, TOTAL> occupied{};
            for (int j = 0; j < settings.robotCount; ++j) {
                if (j != i) occupied[indexOf(robots[j].position)] = true;
            }
            if (i == 0 && manualFirst) {
                Cell next{bot.position.row + manualDirection.row,
                          bot.position.col + manualDirection.col};
                if (next != bot.position) {
                    if (inside(next) && warehouse[next.row][next.col] == 0 &&
                        !occupied[indexOf(next)]) {
                        bot.position = next;
                        ++bot.steps;
                        anybodyMoved = true;
                    } else {
                        ++bot.waits;
                    }
                }
                bot.path.clear();
            } else {
                // Same snapshot for BFS and A*: comparable node counts and timings.
                SearchResult bfs = searchPath(warehouse, occupied, bot.position, goal, false);
                SearchResult astar = searchPath(warehouse, occupied, bot.position, goal, true);
                ++comparison.searches;
                comparison.bfsExpanded += bfs.expanded;
                comparison.astarExpanded += astar.expanded;
                comparison.bfsMicro += bfs.microseconds;
                comparison.astarMicro += astar.microseconds;
                if (!bfs.path.empty() && !astar.path.empty() &&
                    bfs.path.size() == astar.path.size()) ++comparison.sameLengths;

                bot.path = settings.astar ? std::move(astar.path) : std::move(bfs.path);
                if (!bot.path.empty() && !occupied[indexOf(bot.path.front())]) {
                    bot.position = bot.path.front();
                    bot.path.erase(bot.path.begin());
                    ++bot.steps;
                    anybodyMoved = true;
                } else if (bot.position != goal) {
                    ++bot.waits;
                }
            }
            checkTask(i);
            if (completed >= settings.targetOrders) {
                end("ORDER TARGET COMPLETED");
                return;
            }
        }
        if (anybodyMoved) lastMovementTick = ticks;
        checkGridlock();
    }

private:
    std::mt19937 rng;
    int trappedTicks = 0;
    int stationBlockedTicks = 0;

    void assignPickup(int i) {
        std::vector<Cell> choices;
        for (Cell candidate : pickupCells) {
            bool taken = false;
            for (int j = 0; j < settings.robotCount; ++j) {
                if (j != i && !robots[j].carrying && robots[j].pickup == candidate) {
                    taken = true;
                    break;
                }
            }
            if (!taken) choices.push_back(candidate);
        }
        if (choices.empty()) choices = pickupCells;
        std::uniform_int_distribution<int> pick(0, static_cast<int>(choices.size()) - 1);
        robots[i].pickup = choices[pick(rng)];
    }

    void checkTask(int i) {
        Robot& bot = robots[i];
        if (!bot.carrying && bot.position == bot.pickup) {
            bot.carrying = true;
            bot.path.clear();
        } else if (bot.carrying && bot.position == DOCK) {
            bot.carrying = false;
            ++bot.delivered;
            ++completed;
            lastDeliveryTick = ticks;
            stationBlockedTicks = 0;
            bot.path.clear();
            assignPickup(i);
        }
    }

    bool robotOneTrapped() const {
        Cell p = robots[0].position;
        const Cell neighbors[] = {
            {p.row-1,p.col}, {p.row+1,p.col}, {p.row,p.col-1}, {p.row,p.col+1}
        };
        for (Cell c : neighbors) {
            if (!inside(c) || warehouse[c.row][c.col] != 0) continue;
            bool blocked = false;
            for (int j = 1; j < settings.robotCount; ++j) {
                if (robots[j].position == c) { blocked = true; break; }
            }
            if (!blocked) return false;
        }
        return true;
    }

    void checkGridlock() {
        const int timeout = JAM_SECONDS[settings.jamIndex];
        const int timeoutTicks = static_cast<int>(std::ceil(timeout / interval()));
        bool manualOnDock = manualFirst && robots[0].position == DOCK;
        bool carryingRobotNearby = false;
        if (manualOnDock) {
            for (int i = 1; i < settings.robotCount; ++i) {
                if (robots[i].carrying && distance(robots[i].position, DOCK) <= 5) {
                    carryingRobotNearby = true;
                    break;
                }
            }
        }
        // Actual enclosure: every neighboring accessible square is occupied.
        if (manualOnDock && robotOneTrapped()) ++trappedTicks;
        else trappedTicks = 0;
        if (trappedTicks >= static_cast<int>(std::ceil(1.5f / interval()))) {
            end("ROBOT 1 TRAPPED AT DELIVERY STATION");
            return;
        }
        // Manual robot holds the only unloading square while loaded robots queue nearby.
        if (manualOnDock && carryingRobotNearby) ++stationBlockedTicks;
        else stationBlockedTicks = 0;
        if (stationBlockedTicks >= timeoutTicks) {
            end("DELIVERY STATION GRIDLOCK");
            return;
        }
        // General fleet-wide freeze: all robots fail to move for the selected timeout.
        if (settings.robotCount > 1 && !manualFirst &&
            ticks - lastMovementTick >= timeoutTicks) {
            end("FLEET DEADLOCK: NO ROBOT CAN MOVE");
        }
    }
};

enum class Screen { Menu, Running, Results };

// Small UI functions to keep main() readable.
void rectangle(sf::RenderWindow& window, float x, float y, float w, float h,
               sf::Color color, sf::Color border = sf::Color::Transparent) {
    sf::RectangleShape box({w, h});
    box.setPosition({x,y});
    box.setFillColor(color);
    box.setOutlineThickness(border == sf::Color::Transparent ? 0.f : 1.5f);
    box.setOutlineColor(border);
    window.draw(box);
}

void label(sf::RenderWindow& window, const sf::Font& font, bool loaded,
           const std::string& value, float x, float y, unsigned size,
           sf::Color color = sf::Color(234, 241, 250)) {
    if (!loaded) return;
    sf::Text text(font, value, size);
    text.setPosition({x,y});
    text.setFillColor(color);
    window.draw(text);
}

std::string fixed(double value, int digits = 1) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(digits) << value;
    return out.str();
}

std::string settingValue(const Settings& s, int index) {
    switch (index) {
        case 0: return std::to_string(s.robotCount);
        case 1: return std::to_string(s.targetOrders);
        case 2: return s.astar ? "A*" : "BFS";
        case 3: return std::string(SPEEDS[s.speedIndex] == SPEEDS[0] ? "Slow" :
                                    SPEEDS[s.speedIndex] == SPEEDS[1] ? "Normal" : "Fast") +
                       "  (" + fixed(SPEEDS[s.speedIndex], 2) + "s / cell)";
        case 4: return std::to_string(JAM_SECONDS[s.jamIndex]) + " seconds";
    }
    return "";
}

void changeSetting(Settings& s, int index, int direction) {
    switch (index) {
        case 0: s.robotCount = std::clamp(s.robotCount + direction, 1, MAX_ROBOTS); break;
        case 1: s.targetOrders = std::clamp(s.targetOrders + 10 * direction, 10, 200); break;
        case 2: s.astar = !s.astar; break;
        case 3: s.speedIndex = std::clamp(s.speedIndex + direction, 0, 2); break;
        case 4: s.jamIndex = std::clamp(s.jamIndex + direction, 0, 2); break;
    }
}

void drawMenu(sf::RenderWindow& window, const sf::Font& font, bool hasFont,
              const Settings& settings, int selected) {
    window.clear(sf::Color(20, 26, 37));
    rectangle(window, 0, 0, WIDTH, 12, sf::Color(64, 147, 240));
    label(window, font, hasFont, "WAREHOUSE FLEET SIMULATOR", 115, 73, 42);
    label(window, font, hasFont,
          "Set up your run before launching the autonomous fleet.",
          118, 138, 20, sf::Color(160, 182, 205));
    rectangle(window, 112, 205, 976, 532, sf::Color(34, 44, 61), sf::Color(68, 88, 115));
    const char* names[] = {"Number of robots", "Orders to complete", "Pathfinding", "Robot speed", "Gridlock timeout"};
    for (int i = 0; i < 5; ++i) {
        float y = 229.f + i * 96.f;
        if (i == selected)
            rectangle(window, 132, y, 936, 80, sf::Color(45, 80, 113), sf::Color(82, 158, 244));
        else
            rectangle(window, 132, y, 936, 80, sf::Color(42, 53, 71));
        label(window, font, hasFont, names[i], 160, y + 24, 24);
        label(window, font, hasFont, "<", 672, y + 24, 24, sf::Color(133, 218, 255));
        label(window, font, hasFont, settingValue(settings, i),
              733, y + 24, 23, sf::Color(133, 218, 255));
        label(window, font, hasFont, ">", 1024, y + 24, 24, sf::Color(133, 218, 255));
    }
    rectangle(window, 308, 771, 584, 75, sf::Color(62, 153, 120));
    label(window, font, hasFont, "PRESS ENTER TO START", 408, 786, 29);
    label(window, font, hasFont,
          "Arrows or mouse: Choose settings     ENTER or click START     ESC: Exit",
          154, 888, 19, sf::Color(180, 198, 218));
    label(window, font, hasFont,
          "TIP: Park Robot 1 on the purple station in MANUAL mode to test gridlock detection.",
          128, 926, 16, sf::Color(154, 178, 200));
}

void drawSimulation(sf::RenderWindow& window, const sf::Font& font, bool hasFont,
                    const Simulator& sim) {
    window.clear(sf::Color(21, 28, 39));
    sf::RectangleShape square({35.f, 35.f});
    sf::RectangleShape robot({31.f, 31.f});
    sf::RectangleShape marker({24.f, 24.f});
    sf::CircleShape pathDot(5.f);
    pathDot.setFillColor(sf::Color(100, 230, 255));
    for (int r = 0; r < ROWS; ++r) {
        for (int c = 0; c < COLS; ++c) {
            if (sim.warehouse[r][c] == 1) square.setFillColor(sf::Color(142, 148, 158));
            else if (sim.warehouse[r][c] == 2) square.setFillColor(sf::Color(140, 82, 48));
            else if (Cell{r,c} == DOCK) square.setFillColor(sf::Color(154, 91, 224));
            else square.setFillColor(sf::Color(44, 117, 81));
            square.setPosition({c * 40.f, r * 40.f});
            window.draw(square);
        }
    }
    // Pickups are yellow centers surrounded by the requesting robot's color.
    for (const Robot& bot : sim.robots) {
        if (bot.carrying) continue;
        marker.setSize({24.f,24.f});
        marker.setFillColor(bot.color);
        marker.setPosition({bot.pickup.col*40.f+6.f, bot.pickup.row*40.f+6.f});
        window.draw(marker);
        marker.setSize({12.f,12.f});
        marker.setFillColor(sf::Color::Yellow);
        marker.setPosition({bot.pickup.col*40.f+12.f, bot.pickup.row*40.f+12.f});
        window.draw(marker);
    }
    // Selected robot route.
    for (Cell step : sim.robots[sim.selectedRobot].path) {
        pathDot.setPosition({step.col*40.f+14.f, step.row*40.f+14.f});
        window.draw(pathDot);
    }
    for (int i = 0; i < sim.settings.robotCount; ++i) {
        const Robot& bot = sim.robots[i];
        robot.setFillColor(bot.color);
        robot.setPosition({bot.position.col*40.f+2.f, bot.position.row*40.f+2.f});
        window.draw(robot);
        label(window, font, hasFont, std::to_string(i+1),
              bot.position.col*40.f+10.f, bot.position.row*40.f+3.f,
              17, sf::Color::Black);
    }
    rectangle(window, 0, GRID_HEIGHT, WIDTH, HEIGHT - GRID_HEIGHT, sf::Color(25, 34, 49));
    label(window, font, hasFont,
          "FLEET  " + std::to_string(sim.settings.robotCount) + " robots   |   " +
          (sim.settings.astar ? "A*" : "BFS") + "   |   Orders " +
          std::to_string(sim.completed) + " / " + std::to_string(sim.settings.targetOrders) +
          "   |   Time " + fixed(sim.seconds()) + "s   |   " +
          (sim.paused ? "PAUSED" : "RUNNING"), 14, 803, 21);
    label(window, font, hasFont,
          "Throughput " + fixed(sim.ordersPerMinute()) + "/min   |   Travel " +
          std::to_string(sim.totalSteps()) + " cells   |   Blocked/wait " +
          std::to_string(sim.totalWaits()) + "   |   Utilization " +
          fixed(sim.utilization()) + "%", 14, 841, 18);
    double avgBFS = sim.comparison.searches ?
        (double)sim.comparison.bfsExpanded / sim.comparison.searches : 0;
    double avgAstar = sim.comparison.searches ?
        (double)sim.comparison.astarExpanded / sim.comparison.searches : 0;
    label(window, font, hasFont,
          "BFS explored " + fixed(avgBFS) + " nodes/search   |   A* explored " +
          fixed(avgAstar) + "   |   Selected route: Robot " +
          std::to_string(sim.selectedRobot+1), 14, 873, 17,
          sf::Color(151, 212, 247));
    label(window, font, hasFont,
          "M: Manual R1 (WASD)   TAB: Route   1/2: BFS/A*   P: Pause   R: Restart   ESC: Quit",
          14, 915, 17, sf::Color(178, 196, 218));
    if (sim.manualFirst)
        label(window, font, hasFont, "MANUAL ROBOT 1", 964, 803, 17, sf::Color(255, 210, 105));
}

void drawResults(sf::RenderWindow& window, const sf::Font& font, bool hasFont,
                 const Simulator& sim) {
    window.clear(sf::Color(19, 27, 39));
    sf::Color accent = sim.completed >= sim.settings.targetOrders ?
                       sf::Color(60, 185, 130) : sf::Color(236, 135, 83);
    rectangle(window, 0, 0, WIDTH, 12, accent);
    label(window, font, hasFont, "SIMULATION OVER", 105, 61, 44);
    label(window, font, hasFont, sim.endReason, 108, 125, 24, accent);
    const std::array<std::string,4> headings = {
        "ORDERS", "SIMULATION TIME", "THROUGHPUT", "FLEET UTILIZATION"
    };
    const std::array<std::string,4> values = {
        std::to_string(sim.completed) + " / " + std::to_string(sim.settings.targetOrders),
        fixed(sim.seconds()) + " sec", fixed(sim.ordersPerMinute()) + " /min",
        fixed(sim.utilization()) + "%"
    };
    for (int i = 0; i < 4; ++i) {
        float x = 105.f + i*255.f;
        rectangle(window, x, 200, 242, 124, sf::Color(35, 48, 67));
        label(window, font, hasFont, headings[i], x+15.f, 218, 16,
              sf::Color(165, 190, 215));
        label(window, font, hasFont, values[i], x+15.f, 254, 28);
    }
    label(window, font, hasFont,
          "Total distance " + std::to_string(sim.totalSteps()) +
          " cells     Wait / blocked events " + std::to_string(sim.totalWaits()) +
          "     Final algorithm " + (sim.settings.astar ? "A*" : "BFS"),
          108, 351, 18);
    const double searches = static_cast<double>(sim.comparison.searches);
    label(window, font, hasFont,
          "BFS avg " + fixed(searches ? sim.comparison.bfsExpanded/searches : 0) +
          " nodes (" + fixed(searches ? sim.comparison.bfsMicro/searches : 0) + " us)" +
          "     A* avg " + fixed(searches ? sim.comparison.astarExpanded/searches : 0) +
          " nodes (" + fixed(searches ? sim.comparison.astarMicro/searches : 0) + " us)",
          108, 383, 18, sf::Color(151, 212, 247));
    label(window, font, hasFont, "PER-ROBOT RESULTS", 108, 431, 23);
    rectangle(window, 104, 474, 995, 30, sf::Color(58, 72, 92));
    label(window, font, hasFont, "ROBOT", 118, 476, 16);
    label(window, font, hasFont, "DELIVERIES", 322, 476, 16);
    label(window, font, hasFont, "TRAVEL (CELLS)", 560, 476, 16);
    label(window, font, hasFont, "WAIT EVENTS", 865, 476, 16);
    for (int i = 0; i < sim.settings.robotCount; ++i) {
        const Robot& b = sim.robots[i];
        float y = 510.f + i * 29.f;
        if (i % 2 == 0)
            rectangle(window, 104, y, 995, 29, sf::Color(33, 44, 61));
        label(window, font, hasFont, "Robot " + std::to_string(i+1), 118, y+1, 16, b.color);
        label(window, font, hasFont, std::to_string(b.delivered), 344, y+1, 16);
        label(window, font, hasFont, std::to_string(b.steps), 608, y+1, 16);
        label(window, font, hasFont, std::to_string(b.waits), 900, y+1, 16);
    }
    label(window, font, hasFont,
          "R: Run again with same settings      ENTER: Setup menu      ESC: Exit",
          112, 928, 20, sf::Color(174, 210, 237));
}

int main() {
    sf::RenderWindow window(sf::VideoMode({WIDTH, HEIGHT}), "Warehouse Fleet Simulator");
    window.setFramerateLimit(60);
    window.setKeyRepeatEnabled(false);

    sf::Font font;
    bool hasFont = false;
    const std::array<std::string,6> fontPaths = {{
        "C:/Windows/Fonts/segoeui.ttf", "C:/Windows/Fonts/arial.ttf",
        "arial.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
        "/System/Library/Fonts/Supplemental/Arial.ttf"
    }};
    for (const std::string& path : fontPaths) {
        // Local fonts are not packaged with this source file.
        if (font.openFromFile(path)) { hasFont = true; break; }
    }
    if (!hasFont) {
        window.setTitle("Font missing: add arial.ttf to the executable folder");
    }

    Settings settings;
    Simulator simulation;
    Screen screen = Screen::Menu;
    int selection = 0;
    sf::Clock moveClock;

    while (window.isOpen()) {
        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
                break;
            }
            // Mouse controls on the setup screen: choose row, use < and >,
            // or click START. Keyboard arrows and Enter work too.
            if (screen == Screen::Menu) {
                if (const auto* mouse = event->getIf<sf::Event::MouseButtonPressed>()) {
                    if (mouse->button == sf::Mouse::Button::Left) {
                        int x = mouse->position.x;
                        int y = mouse->position.y;
                        if (x >= 308 && x <= 892 && y >= 771 && y <= 846) {
                            simulation.reset(settings);
                            moveClock.restart();
                            screen = Screen::Running;
                        } else if (x >= 132 && x <= 1068) {
                            for (int i = 0; i < 5; ++i) {
                                int rowY = 229 + i * 96;
                                if (y >= rowY && y <= rowY + 80) {
                                    selection = i;
                                    if (x >= 645 && x <= 715) changeSetting(settings, selection, -1);
                                    else if (x >= 990 && x <= 1068) changeSetting(settings, selection, 1);
                                    break;
                                }
                            }
                        }
                    }
                }
            }
            const auto* key = event->getIf<sf::Event::KeyPressed>();
            if (!key) continue;
            if (key->code == sf::Keyboard::Key::Escape) {
                window.close();
                break;
            }
            if (screen == Screen::Menu) {
                switch (key->code) {
                    case sf::Keyboard::Key::Up: selection = (selection + 4) % 5; break;
                    case sf::Keyboard::Key::Down: selection = (selection + 1) % 5; break;
                    case sf::Keyboard::Key::Left: changeSetting(settings, selection, -1); break;
                    case sf::Keyboard::Key::Right: changeSetting(settings, selection, 1); break;
                    case sf::Keyboard::Key::Enter:
                        simulation.reset(settings);
                        moveClock.restart();
                        screen = Screen::Running;
                        break;
                    default: break;
                }
            } else if (screen == Screen::Results) {
                if (key->code == sf::Keyboard::Key::Enter) {
                    screen = Screen::Menu;
                } else if (key->code == sf::Keyboard::Key::R) {
                    simulation.reset(settings);
                    moveClock.restart();
                    screen = Screen::Running;
                }
            } else {
                switch (key->code) {
                    case sf::Keyboard::Key::Num1: simulation.settings.astar = false; break;
                    case sf::Keyboard::Key::Num2: simulation.settings.astar = true; break;
                    case sf::Keyboard::Key::Tab:
                        simulation.selectedRobot = (simulation.selectedRobot + 1) %
                                                   simulation.settings.robotCount;
                        break;
                    case sf::Keyboard::Key::M:
                        simulation.manualFirst = !simulation.manualFirst;
                        simulation.robots[0].path.clear();
                        break;
                    case sf::Keyboard::Key::P:
                        simulation.paused = !simulation.paused;
                        break;
                    case sf::Keyboard::Key::R:
                        simulation.reset(settings);
                        moveClock.restart();
                        break;
                    default: break;
                }
            }
        }
        if (!window.isOpen()) break;

        if (screen == Screen::Running) {
            if (moveClock.getElapsedTime().asSeconds() >= simulation.interval()) {
                moveClock.restart();
                Cell direction{0,0};
                if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W)) --direction.row;
                else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S)) ++direction.row;
                else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A)) --direction.col;
                else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D)) ++direction.col;
                simulation.step(direction);
                if (simulation.over) screen = Screen::Results;
            }
        }
        if (screen == Screen::Menu) drawMenu(window, font, hasFont, settings, selection);
        else if (screen == Screen::Running) drawSimulation(window, font, hasFont, simulation);
        else drawResults(window, font, hasFont, simulation);
        window.display();
    }
    return 0;
}
