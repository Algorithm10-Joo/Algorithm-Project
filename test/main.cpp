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

// Node Ŭ���� ����
class Node {
public:
    std::string code;  // ��� �ڵ�
    std::string centralNode;  // �߾� ��� ���� (�߾� ����� ��� "O")
    double latitude;  // ����
    double longitude;  // �浵
    std::vector<std::pair<std::string, double>> nearNodes;  // ���� ��� ��� �� ����ġ

    Node(std::string c, std::string cn, double lat, double lon,
        std::vector<std::pair<std::string, double>> nn)
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

// ���ڿ��� ������(delimiter)�� �����ϴ� �Լ�
std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

// CSV ������ �о� Node ��ü���� ���͸� ��ȯ�ϴ� �Լ�
std::vector<Node> readCSV(const std::string& filename) {
    std::ifstream file(filename);
    std::string line;

    std::vector<Node> nodes;

    // ��� ��ŵ
    std::getline(file, line);

    // ������ �б�
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

// ����� ��ǥ�� ����ȭ�ϴ� �Լ�
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

// ���ͽ�Ʈ�� �˰����� ����Ͽ� �ִ� ��θ� ã�� �Լ�
std::vector<std::string> dijkstra(const std::vector<Node>& nodes, const std::string& startCode, const std::string& exitCode, const std::unordered_set<std::string>& fireNodes) {
    std::unordered_map<std::string, double> distances;
    std::unordered_map<std::string, std::string> previous;
    auto cmp = [&distances](const std::string& left, const std::string& right) { return distances[left] > distances[right]; };
    std::priority_queue<std::string, std::vector<std::string>, decltype(cmp)> queue(cmp);

    for (const auto& node : nodes) {
        distances[node.code] = std::numeric_limits<double>::infinity();
        previous[node.code] = "";
    }
    distances[startCode] = 0;
    queue.push(startCode);

    while (!queue.empty()) {
        std::string current = queue.top();
        queue.pop();

        if (current == exitCode) break;

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
                continue; // ȭ�簡 �߻��� ���� �̵����� ����
            }
            double alt = distances[current] + neighbor.second;
            if (alt < distances[neighbor.first]) {
                distances[neighbor.first] = alt;
                previous[neighbor.first] = current;
                queue.push(neighbor.first);
            }
        }
    }

    std::vector<std::string> path;
    if (distances[exitCode] == std::numeric_limits<double>::infinity()) {
        return {}; // ��ΰ� ���� ��� �� ���� ��ȯ
    }
    for (std::string at = exitCode; !at.empty(); at = previous[at]) {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());
    return path;
}


// bellman-ford �˰����� ����Ͽ� �ִ� ��θ� ã�� �Լ�
std::vector<std::string> bellmanFord(const std::vector<Node>& nodes, const std::string& startCode, const std::string& exitCode, const std::unordered_set<std::string>& fireNodes) {
    std::unordered_map<std::string, double> distances;
    std::unordered_map<std::string, std::string> previous;

    for (const auto& node : nodes) {
        distances[node.code] = std::numeric_limits<double>::infinity();
        previous[node.code] = "";
    }
    distances[startCode] = 0.0;

    for (size_t i = 0; i < nodes.size() - 1; ++i) {     // (���� ���� - 1)�� �ݺ�  
        for (const auto& node : nodes) {
            if (distances[node.code] == std::numeric_limits<double>::infinity()) continue;
            for (const auto& neighbor : node.nearNodes) {
                if (fireNodes.find(neighbor.first) != fireNodes.end()) continue; // ȭ�簡 �߻��� ���δ� �̵����� ����
                // ���ݱ����� neighbor������ �Ÿ����� ���� node�� ��ģ neighbor������ �Ÿ��� �� ���� ��� ������Ʈ 
                if (distances[neighbor.first] > distances[node.code] + neighbor.second) {
                    distances[neighbor.first] = distances[node.code] + neighbor.second;
                    previous[neighbor.first] = node.code;
                }
            }
        }
    }

    std::vector<std::string> path;
    for (std::string at = exitCode; !at.empty(); at = previous[at]) {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());

    // �ⱸ�� ���� ��ΰ� ���� ���
    if (path.size() == 1 && path[0] == startCode) {
        path.clear();
    }

    return path;
}

// �÷��̵���� �˰����� ����Ͽ� �ִ� ��θ� ã�� �Լ�
std::vector<std::string> floydWarshall(const std::vector<Node>& nodes, const std::string& startCode, const std::string& exitCode, const std::unordered_set<std::string>& fireNodes) {
    // ��� �ڵ�� �ε����� �����ϴ� �� ����
    std::unordered_map<std::string, int> nodeIndex;
    int n = nodes.size();
    for (int i = 0; i < n; ++i) {
        nodeIndex[nodes[i].code] = i;
    }

    // �Ÿ� ��� �ʱ�ȭ
    std::vector<std::vector<double>> dist(n, std::vector<double>(n, std::numeric_limits<double>::infinity()));
    std::vector<std::vector<int>> next(n, std::vector<int>(n, -1));

    for (int i = 0; i < n; ++i) {
        dist[i][i] = 0;
        next[i][i] = i;
    }

    for (const auto& node : nodes) {
        int u = nodeIndex[node.code];
        for (const auto& neighbor : node.nearNodes) {
            if (fireNodes.find(neighbor.first) == fireNodes.end()) { // ȭ�簡 �߻��� ������ ���� ����
                int v = nodeIndex[neighbor.first];
                dist[u][v] = neighbor.second;
                next[u][v] = v;
            }
        }
    }

    // �÷��̵�-���� �˰��� ����
    for (int k = 0; k < n; ++k) {
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                if (dist[i][k] + dist[k][j] < dist[i][j]) {
                    dist[i][j] = dist[i][k] + dist[k][j];
                    next[i][j] = next[i][k];
                }
            }
        }
    }

    // ��� �籸��
    std::vector<std::string> path;
    int u = nodeIndex[startCode];
    int v = nodeIndex[exitCode];

    if (next[u][v] == -1) {
        // ��ΰ� �������� ����
        return path;
    }

    while (u != v) {
        path.push_back(nodes[u].code);
        u = next[u][v];
    }
    path.push_back(nodes[v].code);

    return path;
}

// ����ġ�� ����ȭ�ϴ� �Լ�
double normalizeWeight(double weight, double minWeight, double maxWeight) {
    return 0.5 + 4.5 * (weight - minWeight) / (maxWeight - minWeight);
}

// ȭ�� �ִϸ��̼��� ���� ����ü
struct Fire {
    sf::CircleShape shape;
    std::string startNode;
    std::string endNode;
    double interpolation;
    Fire(const sf::CircleShape& shape, const std::string& startNode, const std::string& endNode)
        : shape(shape), startNode(startNode), endNode(endNode), interpolation(0.0) {}
};

// ��� ���� ������ �����ϴ� �Լ�
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

// �� ��� ������ ����ġ�� �������� �Լ�
double getWeight(const std::string& from, const std::string& to, const std::vector<Node>& nodes) {
    for (const auto& node : nodes) {
        if (node.code == from) {
            for (const auto& neighbor : node.nearNodes) {
                if (neighbor.first == to) {
                    return neighbor.second;
                }
            }
        }
    }
    return 1.0; // �⺻ ����ġ
}

int main() {
    sf::RenderWindow window(sf::VideoMode(780, 580), "SFML Nodes Visualization");

    // ���� ���� �ð� ���
    auto gameStartTime = std::chrono::high_resolution_clock::now();

    // CSV ���� ��� ����
    std::string csvFilePath = "nodes.csv";

    // CSV ���� �б� �� ����ȭ
    std::vector<Node> nodes = readCSV(csvFilePath);
    if (nodes.empty()) {
        std::cerr << "Error: No nodes were loaded from the CSV file." << std::endl;
        return 1; // ���� �ڵ� ��ȯ
    }

    normalizeNodes(nodes);

    // �÷��̾�� �ⱸ�� ��ġ�� ����
    auto playerNodeIt = std::min_element(nodes.begin(), nodes.end(), [](const Node& a, const Node& b) {
        return a.longitude < b.longitude;
    });
    auto exitNodeIt = std::max_element(nodes.begin(), nodes.end(), [](const Node& a, const Node& b) {
        return a.longitude < b.longitude;
    });

    std::string playerNodeCode = playerNodeIt->code;
    std::string exitNodeCode = exitNodeIt->code;

    // ȭ�� �߻� �ʱ�ȭ
    std::unordered_set<std::string> fireNodes;
    fireNodes.insert(nodes[std::rand() % nodes.size()].code);

    std::vector<std::string> path;
    switch (3) {
    case 1:
        // �����ͽ�Ʈ�� �˰������� ��� ã��
        path = dijkstra(nodes, playerNodeCode, exitNodeCode, fireNodes);
        break;
    case 2:
        // bellman-Ford �˰������� ��� ã��
        path = bellmanFord(nodes, playerNodeCode, exitNodeCode, fireNodes);
        break;
    case 3:
        // �÷��̵� ���� �˰������� ��� ã��
        path = floydWarshall(nodes, playerNodeCode, exitNodeCode, fireNodes);
        break;
    }

    // ��ΰ� ���� ��� ó��
    if (path.empty()) {
        std::cout << "Initial path is blocked by fire. Exiting game." << std::endl;
        return 1;
    }

    // �ּ� �� �ִ� ����ġ ã��
    double minWeight = std::numeric_limits<double>::max();
    double maxWeight = std::numeric_limits<double>::lowest();
    for (const auto& node : nodes) {
        for (const auto& neighbor : node.nearNodes) {
            if (neighbor.second < minWeight) minWeight = neighbor.second;
            if (neighbor.second > maxWeight) maxWeight = neighbor.second;
        }
    }

    // ���� ���� �ð�ȭ �غ�
    std::vector<sf::CircleShape> nodeShapes;
    std::vector<sf::VertexArray> edgesShapes;
    std::unordered_map<std::string, sf::CircleShape> nodeMap;
    std::vector<sf::VertexArray> pathEdgesShapes;

    for (const auto& node : nodes) {
        sf::CircleShape shape(5);
        if (node.centralNode == "O") {
            shape.setFillColor(sf::Color::Green); // centralNode ���� "O"�� �� �ʷϻ����� ����
        }
        else {
            shape.setFillColor(sf::Color::Yellow); // �� �ܿ��� ��������� ����
        }
        shape.setPosition(node.longitude * 760 + 10, (1.0 - node.latitude) * 560 + 10); // Y�� ���� �� ���� �߰�
        if (node.code == playerNodeCode) {
            shape.setFillColor(sf::Color::Red); // �÷��̾� ��ġ�� ���������� ����
        }
        if (node.code == exitNodeCode) {
            shape.setFillColor(sf::Color::Blue); // �ⱸ ��ġ�� �Ķ������� ����
        }
        if (fireNodes.find(node.code) != fireNodes.end()) {
            shape.setFillColor(sf::Color::Magenta); // ȭ�� �߻� ���� ����Ÿ ������ ����
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

    // ��� ���� �ð�ȭ �غ�
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

    // ��θ� ���� �̵��ϴ� �÷��̾��� �ʱ� ��ġ ����
    sf::CircleShape playerShape(5);
    playerShape.setFillColor(sf::Color::Red);
    playerShape.setPosition(nodeMap[path[0]].getPosition());

    sf::Clock clock;
    sf::Clock fireClock;
    size_t currentPathIndex = 0;
    double interpolation = 0.0;
    const double maxTravelTime = 5.0; // ���� �̵��� �ִ� �ð�

    double totalWeight = 0.0; // ������ ����ġ�� �� ���

    std::vector<Fire> fireAnimations;
    std::unordered_set<std::string> passedNodes;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        // ȭ�簡 ������ ó��
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

            // ȭ�簡 ���� ��θ� �����ϴ��� Ȯ��
            bool pathBlocked = false;
            for (const auto& node : path) {
                if (fireNodes.find(node) != fireNodes.end() && passedNodes.find(node) == passedNodes.end()) {
                    pathBlocked = true;
                    break;
                }
            }

            // ��� ����
            if (pathBlocked) {
                std::vector<std::string> newPath = dijkstra(nodes, path[currentPathIndex], exitNodeCode, fireNodes);
                if (newPath.empty()) {
                    std::cout << "Game Over: All paths to the exit are blocked by fire." << std::endl;
                    window.close();
                }
                else {
                    resetPathEdgesColors(path, nodeMap, pathEdgesShapes, sf::Color::White); // ���� ��� ���� �ǵ�����
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
        }

        // �̵� �ð� ���
        sf::Time elapsed = clock.restart();
        double travelTime = getWeight(path[currentPathIndex], path[currentPathIndex + 1], nodes);
        double normalizedTime = normalizeWeight(travelTime, minWeight, maxWeight);
        interpolation += elapsed.asSeconds() / normalizedTime;

        if (interpolation >= 1.0) {
            interpolation = 0.0;
            totalWeight += travelTime; // �̵� �Ϸ�� ����� ����ġ�� �ջ�
            passedNodes.insert(path[currentPathIndex]); // ������ ��带 �߰�
            currentPathIndex++;
            if (currentPathIndex + 1 >= path.size()) {
                playerShape.setPosition(nodeMap[exitNodeCode].getPosition());
                std::cout << "Player reached the exit!" << std::endl;

                // ��� �ð� ���
                auto gameEndTime = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> gameDuration = gameEndTime - gameStartTime;

                std::cout << "Total time taken: " << gameDuration.count() << " seconds" << std::endl;
                std::cout << "Total weight of the path: " << totalWeight << std::endl;

                window.close();
            }
            else {
                // �÷��̾ ������ ���� ������ ���������� ����
                nodeMap[path[currentPathIndex]].setFillColor(sf::Color::Red);
                playerShape.setPosition(nodeMap[path[currentPathIndex]].getPosition());
            }
        }
        else {
            sf::Vector2f startPos = nodeMap[path[currentPathIndex]].getPosition();
            sf::Vector2f endPos = nodeMap[path[currentPathIndex + 1]].getPosition();
            sf::Vector2f delta = endPos - startPos;

            // �� ��Һ� ������ �������� ���
            sf::Vector2f interpolatedPos(
                startPos.x + interpolation * delta.x,
                startPos.y + interpolation * delta.y
            );

            playerShape.setPosition(interpolatedPos);
        }

        // ȭ�� �ִϸ��̼� ������Ʈ
        for (auto& fire : fireAnimations) {
            double fireTravelTime = getWeight(fire.startNode, fire.endNode, nodes) * 0.8;
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

        // ��� ���� �׸���
        for (const auto& edge : edgesShapes) {
            window.draw(edge);
        }

        // ��� ���� �׸���
        for (const auto& edge : pathEdgesShapes) {
            window.draw(edge);
        }

        // ��� ��� �׸���
        for (const auto& shape : nodeShapes) {
            window.draw(shape);
        }

        // �÷��̾� �׸���
        window.draw(playerShape);

        // ȭ�� �ִϸ��̼� �׸���
        for (const auto& fire : fireAnimations) {
            window.draw(fire.shape);
        }

        window.display();
    }

    return 0;
}
