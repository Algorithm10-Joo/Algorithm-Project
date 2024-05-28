#include <SFML/Graphics.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <limits>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <ctime>
#include <chrono>

class Node {
public:
    std::string code;
    std::string centralNode;
    double latitude;
    double longitude;
    std::vector<std::pair<std::string, double>> nearNodes;

    Node(std::string c, std::string cn, double lat, double lon, std::vector<std::pair<std::string, double>> nn)
        : code(c), centralNode(cn), latitude(lat), longitude(lon), nearNodes(nn) {}

    void display() const {
        std::cout << "Code: " << code << "\n";
        std::cout << "Central Node: " << centralNode << "\n";
        std::cout << "Latitude: " << latitude << "\n";
        std::cout << "Longitude: " << longitude << "\n";
        std::cout << "Nearby Nodes:\n";
        for (const auto& node : nearNodes) {
            std::cout << "  Node: " << node.first << ", Weight: " << node.second << "\n";
        }
        std::cout << "--------------------\n";
    }
};

std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

std::vector<Node> readCSV(const std::string& filename) {
    std::ifstream file(filename);
    std::string line;

    std::vector<Node> nodes;

    std::getline(file, line);

    while (std::getline(file, line)) {
        auto tokens = split(line, ',');

        std::string code = tokens[0];
        std::string centralNode = tokens[1];
        double latitude = std::stod(tokens[2]);
        double longitude = std::stod(tokens[3]);

        std::vector<std::pair<std::string, double>> nearNodes;
        for (size_t i = 4; i < tokens.size(); i += 2) {
            if (tokens[i].empty()) break;
            std::string nearNode = tokens[i];
            double weight = std::stod(tokens[i + 1]);
            nearNodes.push_back({ nearNode, weight });
        }

        nodes.emplace_back(code, centralNode, latitude, longitude, nearNodes);
    }

    return nodes;
}

void normalizeNodes(std::vector<Node>& nodes) {
    double minLat = std::numeric_limits<double>::max();
    double maxLat = std::numeric_limits<double>::lowest();
    double minLon = std::numeric_limits<double>::max();
    double maxLon = std::numeric_limits<double>::lowest();

    for (const auto& node : nodes) {
        if (node.latitude < minLat) minLat = node.latitude;
        if (node.latitude > maxLat) maxLat = node.latitude;
        if (node.longitude < minLon) minLon = node.longitude;
        if (node.longitude > maxLon) maxLon = node.longitude;
    }

    for (auto& node : nodes) {
        node.latitude = (node.latitude - minLat) / (maxLat - minLat);
        node.longitude = (node.longitude - minLon) / (maxLon - minLon);
    }
}

double heuristic(const Node& a, const Node& b) {
    return std::sqrt((a.latitude - b.latitude) * (a.latitude - b.latitude) +
                     (a.longitude - b.longitude) * (a.longitude - b.longitude));
}

std::vector<std::string> astar(const std::vector<Node>& nodes, const std::string& startCode, const std::string& exitCode, const std::unordered_set<std::string>& fireNodes) {
    std::unordered_map<std::string, double> gScore, fScore;
    std::unordered_map<std::string, std::string> cameFrom;
    auto cmp = [&fScore](const std::string& left, const std::string& right) { return fScore[left] > fScore[right]; };
    std::priority_queue<std::string, std::vector<std::string>, decltype(cmp)> openSet(cmp);

    for (const auto& node : nodes) {
        gScore[node.code] = std::numeric_limits<double>::infinity();
        fScore[node.code] = std::numeric_limits<double>::infinity();
    }
    gScore[startCode] = 0.0;
    fScore[startCode] = heuristic(nodes[0], nodes[0]); // 초기 휴리스틱 값 설정

    openSet.push(startCode);

    while (!openSet.empty()) {
        std::string current = openSet.top();
        openSet.pop();

        if (current == exitCode) {
            std::vector<std::string> path;
            for (std::string at = exitCode; !at.empty(); at = cameFrom[at]) {
                path.push_back(at);
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        const Node* currentNode = nullptr;
        for (const auto& node : nodes) {
            if (node.code == current) {
                currentNode = &node;
                break;
            }
        }
        if (!currentNode) continue;

        for (const auto& neighbor : currentNode->nearNodes) {
            if (fireNodes.find(neighbor.first) != fireNodes.end()) {
                continue;
            }
            double tentative_gScore = gScore[current] + neighbor.second;
            if (tentative_gScore < gScore[neighbor.first]) {
                cameFrom[neighbor.first] = current;
                gScore[neighbor.first] = tentative_gScore;

                const Node* neighborNode = nullptr;
                for (const auto& node : nodes) {
                    if (node.code == neighbor.first) {
                        neighborNode = &node;
                        break;
                    }
                }
                if (neighborNode) {
                    fScore[neighbor.first] = gScore[neighbor.first] + heuristic(*neighborNode, *currentNode);
                    openSet.push(neighbor.first);
                }
            }
        }
    }

    return {};
}

double normalizeWeight(double weight, double minWeight, double maxWeight) {
    return 0.5 + 4.5 * (weight - minWeight) / (maxWeight - minWeight);
}

struct Fire {
    sf::CircleShape shape;
    std::string startNode;
    std::string endNode;
    double interpolation;
    Fire(const sf::CircleShape& shape, const std::string& startNode, const std::string& endNode)
        : shape(shape), startNode(startNode), endNode(endNode), interpolation(0.0) {}
};

void resetPathEdgesColors(const std::vector<std::string>& path, std::unordered_map<std::string, sf::CircleShape>& nodeMap, std::vector<sf::VertexArray>& pathEdgesShapes, const sf::Color& color) {
    for (size_t i = 1; i < path.size(); ++i) {
        const auto& startNode = nodeMap[path[i - 1]];
        const auto& endNode = nodeMap[path[i]];
        for (auto& edge : pathEdgesShapes) {
            if ((edge[0].position == startNode.getPosition() && edge[1].position == endNode.getPosition()) ||
                (edge[1].position == startNode.getPosition() && edge[0].position == endNode.getPosition())) {
                edge[0].color = color;
                edge[1].color = color;
            }
        }
    }
}

int main() {
    sf::RenderWindow window(sf::VideoMode(780, 580), "SFML Nodes Visualization");

    std::string csvFilePath = "nodes.csv";

    std::vector<Node> nodes = readCSV(csvFilePath);
    if (nodes.empty()) {
        std::cerr << "Error: No nodes were loaded from the CSV file." << std::endl;
        return 1;
    }

    normalizeNodes(nodes);

    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    std::string playerNodeCode = nodes[std::rand() % nodes.size()].code;
    std::string exitNodeCode = nodes[std::rand() % nodes.size()].code;

    std::unordered_set<std::string> fireNodes;
    fireNodes.insert(nodes[std::rand() % nodes.size()].code);

    std::vector<std::string> path = astar(nodes, playerNodeCode, exitNodeCode, fireNodes);

    double minWeight = std::numeric_limits<double>::max();
    double maxWeight = std::numeric_limits<double>::lowest();
    for (const auto& node : nodes) {
        for (const auto& neighbor : node.nearNodes) {
            if (neighbor.second < minWeight) minWeight = neighbor.second;
            if (neighbor.second > maxWeight) maxWeight = neighbor.second;
        }
    }

    std::vector<sf::CircleShape> nodeShapes;
    std::vector<sf::VertexArray> edgesShapes;
    std::unordered_map<std::string, sf::CircleShape> nodeMap;
    std::vector<sf::VertexArray> pathEdgesShapes;

    for (const auto& node : nodes) {
        sf::CircleShape shape(5);
        if (node.centralNode == "O") {
            shape.setFillColor(sf::Color::Green);
        } else {
            shape.setFillColor(sf::Color::Yellow);
        }
        shape.setPosition(node.longitude * 760 + 10, (1.0 - node.latitude) * 560 + 10);
        if (node.code == playerNodeCode) {
            shape.setFillColor(sf::Color::Red);
        }
        if (node.code == exitNodeCode) {
            shape.setFillColor(sf::Color::Blue);
        }
        if (fireNodes.find(node.code) != fireNodes.end()) {
            shape.setFillColor(sf::Color::Magenta);
        }
        nodeShapes.push_back(shape);
        nodeMap[node.code] = shape;

        for (const auto& edge : node.nearNodes) {
            auto it = std::find_if(nodes.begin(), nodes.end(), [&edge](const Node& n) {
                return n.code == edge.first;
            });
            if (it == nodes.end()) {
                std::cerr << "Error: Edge points to a non-existing node with ID " << edge.first << std::endl;
                continue;
            }

            sf::VertexArray line(sf::Lines, 2);
            line[0].position = sf::Vector2f(node.longitude * 760 + 10, (1.0 - node.latitude) * 560 + 10);
            line[0].color = sf::Color::White;

            const auto& neighbor = *it;
            line[1].position = sf::Vector2f(neighbor.longitude * 760 + 10, (1.0 - neighbor.latitude) * 560 + 10);
            line[1].color = sf::Color::White;

            edgesShapes.push_back(line);
        }
    }

    for (size_t i = 1; i < path.size(); ++i) {
        sf::VertexArray line(sf::Lines, 2);
        const auto& startNode = nodeMap[path[i - 1]];
        const auto& endNode = nodeMap[path[i]];

        line[0].position = startNode.getPosition();
        line[0].color = sf::Color::Red;
        line[1].position = endNode.getPosition();
        line[1].color = sf::Color::Red;

        pathEdgesShapes.push_back(line);
    }

    sf::CircleShape playerShape(5);
    playerShape.setFillColor(sf::Color::Red);
    playerShape.setPosition(nodeMap[path[0]].getPosition());

    sf::Clock clock;
    sf::Clock fireClock;
    size_t currentPathIndex = 0;
    double interpolation = 0.0;
    const double maxTravelTime = 5.0;

    auto getWeight = [&](const std::string& from, const std::string& to) -> double {
        for (const auto& node : nodes) {
            if (node.code == from) {
                for (const auto& neighbor : node.nearNodes) {
                    if (neighbor.first == to) {
                        return neighbor.second;
                    }
                }
            }
        }
        return 1.0;
    };

    std::vector<Fire> fireAnimations;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        if (fireClock.getElapsedTime().asSeconds() > 0.8 * maxTravelTime) {
            fireClock.restart();
            std::unordered_set<std::string> newFireNodes;
            for (const auto& fireNode : fireNodes) {
                for (const auto& node : nodes) {
                    if (node.code == fireNode) {
                        for (const auto& neighbor : node.nearNodes) {
                            if (fireNodes.find(neighbor.first) == fireNodes.end()) {
                                newFireNodes.insert(neighbor.first);

                                sf::CircleShape fireShape(5);
                                fireShape.setFillColor(sf::Color::Magenta);
                                fireShape.setPosition(nodeMap[fireNode].getPosition());
                                fireAnimations.emplace_back(fireShape, fireNode, neighbor.first);
                            }
                        }
                        break;
                    }
                }
            }
            for (const auto& newFireNode : newFireNodes) {
                fireNodes.insert(newFireNode);
            }

            std::vector<std::string> newPath = astar(nodes, path[currentPathIndex], exitNodeCode, fireNodes);
            if (newPath.size() == 1 && newPath[0] == path[currentPathIndex]) {
                std::cout << "Game Over: All paths to the exit are blocked by fire." << std::endl;
                window.close();
            } else {
                resetPathEdgesColors(path, nodeMap, pathEdgesShapes, sf::Color::White);
                path = newPath;
                pathEdgesShapes.clear();
                for (size_t i = 1; i < path.size(); ++i) {
                    sf::VertexArray line(sf::Lines, 2);
                    const auto& startNode = nodeMap[path[i - 1]];
                    const auto& endNode = nodeMap[path[i]];

                    line[0].position = startNode.getPosition();
                    line[0].color = sf::Color::Red;
                    line[1].position = endNode.getPosition();
                    line[1].color = sf::Color::Red;

                    pathEdgesShapes.push_back(line);
                }
                currentPathIndex = 0;
                interpolation = 0.0;
            }
        }

        sf::Time elapsed = clock.restart();
        double travelTime = getWeight(path[currentPathIndex], path[currentPathIndex + 1]);
        double normalizedTime = normalizeWeight(travelTime, minWeight, maxWeight);
        interpolation += elapsed.asSeconds() / normalizedTime;

        if (interpolation >= 1.0) {
            interpolation = 0.0;
            currentPathIndex++;
            if (currentPathIndex + 1 >= path.size()) {
                playerShape.setPosition(nodeMap[exitNodeCode].getPosition());
                std::cout << "Player reached the exit!" << std::endl;
                window.close();
            } else {
                nodeMap[path[currentPathIndex]].setFillColor(sf::Color::Red);
                playerShape.setPosition(nodeMap[path[currentPathIndex]].getPosition());
            }
        } else {
            sf::Vector2f startPos = nodeMap[path[currentPathIndex]].getPosition();
            sf::Vector2f endPos = nodeMap[path[currentPathIndex + 1]].getPosition();
            sf::Vector2f delta = endPos - startPos;

            sf::Vector2f interpolatedPos(
                startPos.x + interpolation * delta.x,
                startPos.y + interpolation * delta.y
            );

            playerShape.setPosition(interpolatedPos);
        }

        for (auto& fire : fireAnimations) {
            double fireTravelTime = getWeight(fire.startNode, fire.endNode) * 0.8;
            double fireNormalizedTime = normalizeWeight(fireTravelTime, minWeight, maxWeight);
            fire.interpolation += elapsed.asSeconds() / fireNormalizedTime;

            if (fire.interpolation >= 1.0) {
                fire.interpolation = 1.0;
                nodeMap[fire.endNode].setFillColor(sf::Color::Magenta);
            }

            sf::Vector2f fireStartPos = nodeMap[fire.startNode].getPosition();
            sf::Vector2f fireEndPos = nodeMap[fire.endNode].getPosition();
            sf::Vector2f fireDelta = fireEndPos - fireStartPos;

            sf::Vector2f fireInterpolatedPos(
                fireStartPos.x + fire.interpolation * fireDelta.x,
                fireStartPos.y + fire.interpolation * fireDelta.y
            );

            fire.shape.setPosition(fireInterpolatedPos);
        }

        window.clear();

        for (const auto& edge : edgesShapes) {
            window.draw(edge);
        }

        for (const auto& edge : pathEdgesShapes) {
            window.draw(edge);
        }

        for (const auto& shape : nodeShapes) {
            window.draw(shape);
        }

        window.draw(playerShape);

        for (const auto& fire : fireAnimations) {
            window.draw(fire.shape);
        }

        window.display();
    }

    return 0;
}
