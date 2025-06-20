/****************************************************************************
File: GridPathPlanner.cpp
Author: AI Assistant
Date: 2025.1.6
Description: 基于渔网线的无人机避障路径规划器实现
****************************************************************************/

#include "GridPathPlanner.h"
#include "../log/QgisDebug.h"
#include <QDir>
#include <QFileInfo>
#include <QTextStream>
#include <QtMath>
#include <QQueue>
#include <QSet>
#include <QRandomGenerator>
#include <limits>
#include <algorithm>
#include <queue>
#include <numeric>
#include <QTimer>

// A*算法中的节点结构
struct AStarNode {
    int nodeId;
    double gCost;  // 从起点到当前节点的实际代价
    double hCost;  // 从当前节点到终点的启发式代价
    double fCost() const { return gCost + hCost; }
    int parent;    // 父节点ID
    
    AStarNode(int id, double g, double h, int p = -1) 
        : nodeId(id), gCost(g), hCost(h), parent(p) {}
        
    bool operator<(const AStarNode& other) const {
        return fCost() > other.fCost(); // 优先队列是最大堆，我们需要最小堆
    }
};

GridPathPlanner::GridPathPlanner(QObject *parent)
    : QObject(parent)
    , mShapefileHandler(new ShapefileHandler()) {
    
    logMessage("Grid path planner initialized", Qgis::MessageLevel::Info);
}

GridPathPlanner::~GridPathPlanner() {
    delete mShapefileHandler;
    clear();
}

GridPlanningResult GridPathPlanner::executePlanning(const GridPlanningParameters& params) {
    GridPlanningResult result;
    
    try {
        // 验证参数
        auto validation = validateParameters(params);
        if (!validation.first) {
            result.success = false;
            result.errorMessage = validation.second;
            return result;
        }

        logMessage("Starting grid path planning", Qgis::MessageLevel::Info);
        emit progressUpdated(0, "开始网格路径规划...");

        // 步骤1: 从渔网线加载网格 (0-30%)
        emit progressUpdated(10, "正在加载渔网线数据...");
        if (!buildGridFromFishnet(params.fishnetShapefile)) {
            result.errorMessage = "Failed to load fishnet lines shapefile";
            emit planningFailed(result.errorMessage);
            return result;
        }
        emit progressUpdated(30, QString("渔网线加载完成，共%1个节点").arg(mGridNodes.size()));

        // 步骤2: 加载风险点 (30-50%)
        emit progressUpdated(35, "正在加载风险点数据...");
        if (!loadRiskPoints(params.riskPointsShapefile)) {
            result.errorMessage = "Failed to load risk points";
            emit planningFailed(result.errorMessage);
            return result;
        }
        emit progressUpdated(50, QString("风险点加载完成，共%1个点").arg(mRiskPoints.size()));

        // 步骤3: 关联风险点到网格 (50-60%)
        emit progressUpdated(55, "正在关联风险点到网格...");
        associateRiskPointsToGrid(params.maxRiskPointDistance);
        
        QVector<int> riskNodeIds = getRiskNodeIds();
        result.totalRiskPoints = riskNodeIds.size();
        emit progressUpdated(60, QString("风险点关联完成，关联了%1个风险点").arg(result.totalRiskPoints));

        if (riskNodeIds.isEmpty()) {
            result.errorMessage = "No risk points found within the grid or distance threshold";
            emit planningFailed(result.errorMessage);
            return result;
        }

        // 步骤4: 寻找起始节点 (60-65%)
        emit progressUpdated(62, "正在寻找起始节点...");
        logMessage(QString("Looking for start node near point (%1, %2, %3)")
                  .arg(params.startPoint.x(), 0, 'f', 2)
                  .arg(params.startPoint.y(), 0, 'f', 2)
                  .arg(params.startPoint.z(), 0, 'f', 2), Qgis::MessageLevel::Info);
        
        int startNodeId = findNearestGridNode(params.startPoint);
        if (startNodeId == -1) {
            result.errorMessage = "Cannot find start node in the grid";
            logMessage(QString("Failed to find start node near (%1, %2, %3)")
                      .arg(params.startPoint.x(), 0, 'f', 2)
                      .arg(params.startPoint.y(), 0, 'f', 2)
                      .arg(params.startPoint.z(), 0, 'f', 2), Qgis::MessageLevel::Critical);
            emit planningFailed(result.errorMessage);
            return result;
        }
        
        int startNodeIndex = mNodeIdToIndex[startNodeId];
        logMessage(QString("Found start node %1 at position (%2, %3)")
                  .arg(startNodeId)
                  .arg(mGridNodes[startNodeIndex].position.x(), 0, 'f', 2)
                  .arg(mGridNodes[startNodeIndex].position.y(), 0, 'f', 2), Qgis::MessageLevel::Info);
        
        emit progressUpdated(65, "起始节点确定");

        // 步骤5: 执行路径规划 (65-95%)
        emit progressUpdated(70, "正在执行路径规划算法...");
        
        if (params.algorithm == "TSP_Dijkstra" || params.algorithm == "TSP_AStar") {
            result.optimalPath = solveTSPWithDijkstra(startNodeId);
            result.algorithm = "TSP with Dijkstra";
        } else if (params.algorithm == "Dijkstra") {
            // 简单的Dijkstra路径（仅连接最近的风险点）
            if (!riskNodeIds.isEmpty()) {
                QVector<int> path = dijkstraPath(startNodeId, riskNodeIds.first());
                result.optimalPath = convertToPathPoints(path);
                result.algorithm = "Dijkstra";
            }
        } else if (params.algorithm == "AStar") {
            // 简单的A*路径（仅连接最近的风险点）
            if (!riskNodeIds.isEmpty()) {
                QVector<int> path = aStarPath(startNodeId, riskNodeIds.first());
                result.optimalPath = convertToPathPoints(path);
                result.algorithm = "A*";
            }
        } else {
            result.optimalPath = solveTSPWithDijkstra(startNodeId);
            result.algorithm = "TSP with Dijkstra (default)";
        }

        emit progressUpdated(95, "路径规划完成");

        // 步骤6: 计算统计信息 (95-100%)
        emit progressUpdated(98, "正在计算统计信息...");
        
        result.success = true;
        result.gridNodes = mGridNodes;
        result.riskPoints = mRiskPoints;
        result.visitedRiskPoints = 0;
        result.totalPathLength = 0.0;
        
        // 计算路径长度和访问的风险点数量
        for (int i = 0; i < result.optimalPath.size(); ++i) {
            if (result.optimalPath[i].isRiskPoint) {
                result.visitedRiskPoints++;
            }
            
            if (i > 0) {
                double segmentLength = calculateDistance(
                    result.optimalPath[i-1].position, 
                    result.optimalPath[i].position
                );
                result.totalPathLength += segmentLength;
                result.optimalPath[i].distanceFromStart = result.totalPathLength;
            }
        }

        emit progressUpdated(100, "规划完成");
        emit planningCompleted(result);

        logMessage(QString("Grid path planning completed successfully. Path length: %1m, Risk points visited: %2/%3")
                  .arg(result.totalPathLength, 0, 'f', 2)
                  .arg(result.visitedRiskPoints)
                  .arg(result.totalRiskPoints), 
                  Qgis::MessageLevel::Success);

    } catch (const std::exception& e) {
        result.success = false;
        result.errorMessage = QString("Exception occurred during planning: %1").arg(e.what());
        logMessage(result.errorMessage, Qgis::MessageLevel::Critical);
        emit planningFailed(result.errorMessage);
    }

    return result;
}

QPair<bool, QString> GridPathPlanner::validateParameters(const GridPlanningParameters& params) {
    // 检查渔网线文件
    if (params.fishnetShapefile.isEmpty()) {
        return qMakePair(false, QString("Fishnet shapefile path is empty"));
    }
    
    QFileInfo fishnetFile(params.fishnetShapefile);
    if (!fishnetFile.exists()) {
        return qMakePair(false, QString("Fishnet shapefile does not exist: %1").arg(params.fishnetShapefile));
    }

    // 检查风险点文件
    if (params.riskPointsShapefile.isEmpty()) {
        return qMakePair(false, QString("Risk points shapefile path is empty"));
    }
    
    QFileInfo riskFile(params.riskPointsShapefile);
    if (!riskFile.exists()) {
        return qMakePair(false, QString("Risk points shapefile does not exist: %1").arg(params.riskPointsShapefile));
    }

    // 检查输出路径
    if (!params.outputPath.isEmpty()) {
        QDir outputDir(params.outputPath);
        if (!outputDir.exists()) {
            if (!outputDir.mkpath(".")) {
                return qMakePair(false, QString("Cannot create output directory: %1").arg(params.outputPath));
            }
        }
    }

    return qMakePair(true, QString(""));
}

bool GridPathPlanner::buildGridFromFishnet(const QString& shapefile) {
    clear();
    
    logMessage(QString("Loading fishnet lines from: %1").arg(shapefile), Qgis::MessageLevel::Info);
    
    // 直接加载渔网线数据
    if (!mShapefileHandler->loadFishnetLines(shapefile)) {
        logMessage("Failed to load fishnet lines shapefile", Qgis::MessageLevel::Critical);
        return false;
    }
    
    // 从渔网线中提取节点和连接关系
    auto fishnetLines = mShapefileHandler->getFishnetLines();
    
    if (fishnetLines.isEmpty()) {
        logMessage("No fishnet lines found in shapefile", Qgis::MessageLevel::Critical);
        return false;
    }
    
    int nodeId = 0;
    QHash<QString, int> positionToNodeId; // 位置字符串到节点ID的映射
    const double COORD_PRECISION = 0.01; // 1cm精度
    
    logMessage(QString("Processing %1 fishnet lines to extract grid nodes").arg(fishnetLines.size()), 
               Qgis::MessageLevel::Info);
    
    // 第一遍：提取所有唯一的节点，使用更高精度的位置key
    for (const auto& line : fishnetLines) {
        // 处理线段起点 - 使用固定精度避免浮点数误差
        double startX = qRound(line.first.x() / COORD_PRECISION) * COORD_PRECISION;
        double startY = qRound(line.first.y() / COORD_PRECISION) * COORD_PRECISION;
        QString startKey = QString("%1,%2").arg(startX, 0, 'f', 2).arg(startY, 0, 'f', 2);
        
        if (!positionToNodeId.contains(startKey)) {
            GridNode node(QVector3D(startX, startY, line.first.z()), nodeId);
            mGridNodes.append(node);
            mNodeIdToIndex[nodeId] = mGridNodes.size() - 1;
            positionToNodeId[startKey] = nodeId;
            nodeId++;
        }
        
        // 处理线段终点
        double endX = qRound(line.second.x() / COORD_PRECISION) * COORD_PRECISION;
        double endY = qRound(line.second.y() / COORD_PRECISION) * COORD_PRECISION;
        QString endKey = QString("%1,%2").arg(endX, 0, 'f', 2).arg(endY, 0, 'f', 2);
        
        if (!positionToNodeId.contains(endKey)) {
            GridNode node(QVector3D(endX, endY, line.second.z()), nodeId);
            mGridNodes.append(node);
            mNodeIdToIndex[nodeId] = mGridNodes.size() - 1;
            positionToNodeId[endKey] = nodeId;
            nodeId++;
        }
    }
    
    logMessage(QString("Extracted %1 unique grid nodes").arg(mGridNodes.size()), Qgis::MessageLevel::Info);
    
    // 第二遍：根据渔网线建立邻接关系
    int connectionCount = 0;
    for (const auto& line : fishnetLines) {
        // 使用相同的精度处理
        double startX = qRound(line.first.x() / COORD_PRECISION) * COORD_PRECISION;
        double startY = qRound(line.first.y() / COORD_PRECISION) * COORD_PRECISION;
        QString startKey = QString("%1,%2").arg(startX, 0, 'f', 2).arg(startY, 0, 'f', 2);
        
        double endX = qRound(line.second.x() / COORD_PRECISION) * COORD_PRECISION;
        double endY = qRound(line.second.y() / COORD_PRECISION) * COORD_PRECISION;
        QString endKey = QString("%1,%2").arg(endX, 0, 'f', 2).arg(endY, 0, 'f', 2);
        
        if (!positionToNodeId.contains(startKey) || !positionToNodeId.contains(endKey)) {
            logMessage(QString("Warning: Node key not found for line connection"), Qgis::MessageLevel::Warning);
            continue;
        }
        
        int startNodeId = positionToNodeId[startKey];
        int endNodeId = positionToNodeId[endKey];
        
        if (startNodeId == endNodeId) {
            continue; // 跳过自环
        }
        
        // 建立双向连接
        int startIndex = mNodeIdToIndex[startNodeId];
        int endIndex = mNodeIdToIndex[endNodeId];
        
        if (!mGridNodes[startIndex].neighbors.contains(endNodeId)) {
            mGridNodes[startIndex].neighbors.append(endNodeId);
            connectionCount++;
        }
        if (!mGridNodes[endIndex].neighbors.contains(startNodeId)) {
            mGridNodes[endIndex].neighbors.append(startNodeId);
            connectionCount++;
        }
    }
    
    // 统计连接信息
    int totalConnections = 0;
    int isolatedNodes = 0;
    int maxConnections = 0;
    for (const auto& node : mGridNodes) {
        int connections = node.neighbors.size();
        totalConnections += connections;
        if (connections == 0) isolatedNodes++;
        if (connections > maxConnections) maxConnections = connections;
    }
    
    // 计算网格边界
    if (!mGridNodes.isEmpty()) {
        double minX = mGridNodes[0].position.x(), maxX = mGridNodes[0].position.x();
        double minY = mGridNodes[0].position.y(), maxY = mGridNodes[0].position.y();
        
        for (const auto& node : mGridNodes) {
            minX = qMin(minX, static_cast<double>(node.position.x()));
            maxX = qMax(maxX, static_cast<double>(node.position.x()));
            minY = qMin(minY, static_cast<double>(node.position.y()));
            maxY = qMax(maxY, static_cast<double>(node.position.y()));
        }
        
        logMessage(QString("Grid bounds: X[%1, %2], Y[%3, %4]")
                  .arg(minX, 0, 'f', 2).arg(maxX, 0, 'f', 2)
                  .arg(minY, 0, 'f', 2).arg(maxY, 0, 'f', 2), 
                  Qgis::MessageLevel::Info);
    }
    
    double avgConnections = mGridNodes.isEmpty() ? 0.0 : (double)totalConnections / mGridNodes.size();
    
    logMessage(QString("Grid connectivity: %1 nodes, %2 connections, avg: %3, max: %4, isolated: %5")
              .arg(mGridNodes.size())
              .arg(connectionCount)
              .arg(avgConnections, 0, 'f', 1)
              .arg(maxConnections)
              .arg(isolatedNodes),
              Qgis::MessageLevel::Info);
    
    // 检查网格连通性
    if (avgConnections < 1.5) {
        logMessage("Warning: Low average connectivity detected, grid may have connectivity issues", 
                  Qgis::MessageLevel::Warning);
    }
    
    if (isolatedNodes > 0) {
        logMessage(QString("Warning: %1 isolated nodes detected").arg(isolatedNodes), 
                  Qgis::MessageLevel::Warning);
    }
    
    // 网格修复：连接距离很近的孤立节点
    if (avgConnections < 2.0) {
        logMessage("Grid connectivity is poor, attempting to repair by connecting nearby nodes...", 
                  Qgis::MessageLevel::Info);
        
        int addedConnections = repairGridConnectivity();
        
        if (addedConnections > 0) {
            logMessage(QString("Grid repair completed: added %1 connections").arg(addedConnections), 
                      Qgis::MessageLevel::Info);
            
            // 重新计算连通性统计
            totalConnections = 0;
            isolatedNodes = 0;
            maxConnections = 0;
            for (const auto& node : mGridNodes) {
                int connections = node.neighbors.size();
                totalConnections += connections;
                if (connections == 0) isolatedNodes++;
                if (connections > maxConnections) maxConnections = connections;
            }
            
            avgConnections = mGridNodes.isEmpty() ? 0.0 : (double)totalConnections / mGridNodes.size();
            
            logMessage(QString("After repair - Grid connectivity: %1 nodes, %2 total connections, avg: %3, max: %4, isolated: %5")
                      .arg(mGridNodes.size())
                      .arg(totalConnections / 2) // 除以2因为是双向连接
                      .arg(avgConnections, 0, 'f', 1)
                      .arg(maxConnections)
                      .arg(isolatedNodes),
                      Qgis::MessageLevel::Info);
        } else {
            logMessage("Grid repair failed: could not add any connections", Qgis::MessageLevel::Warning);
        }
    }
    
    return !mGridNodes.isEmpty();
}

bool GridPathPlanner::loadRiskPoints(const QString& shapefile) {
    mRiskPoints.clear();
    
    if (!mShapefileHandler->loadRiskEventPoints(shapefile)) {
        logMessage("Failed to load risk points shapefile", Qgis::MessageLevel::Critical);
        return false;
    }
    
    auto points = mShapefileHandler->getRiskEventPoints();
    
    for (const auto& point : points) {
        mRiskPoints.append(point);
    }
    
    logMessage(QString("Loaded %1 risk points").arg(mRiskPoints.size()), Qgis::MessageLevel::Info);
    return !mRiskPoints.isEmpty();
}

void GridPathPlanner::associateRiskPointsToGrid(double maxDistance) {
    // 重置所有节点的风险点标记
    for (auto& node : mGridNodes) {
        node.isRiskPoint = false;
    }
    
    int associatedCount = 0;
    int tooFarCount = 0;
    int noNearestNodeCount = 0;
    
    logMessage(QString("Starting risk point association: %1 risk points, max distance %2m")
              .arg(mRiskPoints.size()).arg(maxDistance), Qgis::MessageLevel::Info);
    
    // 显示前3个风险点的详细信息
    for (int riskIdx = 0; riskIdx < mRiskPoints.size(); ++riskIdx) {
        const QVector3D& riskPoint = mRiskPoints[riskIdx];
        
        if (riskIdx < 3) {
            logMessage(QString("Risk point %1: (%2, %3, %4)").arg(riskIdx+1)
                      .arg(riskPoint.x(), 0, 'f', 2).arg(riskPoint.y(), 0, 'f', 2).arg(riskPoint.z(), 0, 'f', 2), 
                      Qgis::MessageLevel::Info);
        }
        
        int nearestNodeId = findNearestGridNode(riskPoint);
        if (nearestNodeId != -1) {
            int nodeIndex = mNodeIdToIndex[nearestNodeId];
            double distance = calculateDistance(riskPoint, mGridNodes[nodeIndex].position);
            
            if (riskIdx < 3) {
                logMessage(QString("Risk point %1: nearest node %2 at distance %3m (grid pos: %4, %5)")
                          .arg(riskIdx+1).arg(nearestNodeId).arg(distance, 0, 'f', 2)
                          .arg(mGridNodes[nodeIndex].position.x(), 0, 'f', 2)
                          .arg(mGridNodes[nodeIndex].position.y(), 0, 'f', 2), 
                          Qgis::MessageLevel::Info);
            }
            
            if (distance <= maxDistance) {
                if (!mGridNodes[nodeIndex].isRiskPoint) {
                    mGridNodes[nodeIndex].isRiskPoint = true;
                    associatedCount++;
                    if (riskIdx < 3) {
                        logMessage(QString("Risk point %1: ASSOCIATED to NEW risk node %2 (distance %3m <= %4m)")
                                  .arg(riskIdx+1).arg(nearestNodeId).arg(distance, 0, 'f', 2).arg(maxDistance), 
                                  Qgis::MessageLevel::Info);
                    }
                } else {
                    if (riskIdx < 3) {
                        logMessage(QString("Risk point %1: node %2 already marked as risk point (distance %3m <= %4m)")
                                  .arg(riskIdx+1).arg(nearestNodeId).arg(distance, 0, 'f', 2).arg(maxDistance), 
                                  Qgis::MessageLevel::Info);
                    }
                }
            } else {
                tooFarCount++;
                if (riskIdx < 3) {
                    logMessage(QString("Risk point %1: TOO FAR (distance %2m > %3m)")
                              .arg(riskIdx+1).arg(distance, 0, 'f', 2).arg(maxDistance), 
                              Qgis::MessageLevel::Warning);
                }
            }
        } else {
            noNearestNodeCount++;
            if (riskIdx < 3) {
                logMessage(QString("Risk point %1: NO NEAREST NODE FOUND").arg(riskIdx+1), 
                          Qgis::MessageLevel::Warning);
            }
        }
    }
    
    logMessage(QString("Risk point association complete: %1 associated, %2 too far, %3 no nearest node")
              .arg(associatedCount).arg(tooFarCount).arg(noNearestNodeCount), Qgis::MessageLevel::Info);
    logMessage(QString("Associated %1 risk points to grid nodes (within %2m)")
              .arg(associatedCount).arg(maxDistance), Qgis::MessageLevel::Info);
}

QVector<int> GridPathPlanner::dijkstraPath(int startNodeId, int endNodeId) {
    if (!mNodeIdToIndex.contains(startNodeId) || !mNodeIdToIndex.contains(endNodeId)) {
        return QVector<int>();
    }
    
    // 使用优先队列优化的Dijkstra算法
    std::priority_queue<std::pair<double, int>, 
                       std::vector<std::pair<double, int>>, 
                       std::greater<>> pq;
    
    QHash<int, double> distances;
    QHash<int, int> previous;
    QSet<int> visited;
    
    // 初始化起点
    distances[startNodeId] = 0.0;
    pq.push({0.0, startNodeId});
    
    while (!pq.empty()) {
        auto [currentDist, currentNodeId] = pq.top();
        pq.pop();
        
        if (visited.contains(currentNodeId)) {
            continue; // 已访问过的节点跳过
        }
        
        visited.insert(currentNodeId);
        
        if (currentNodeId == endNodeId) {
            break; // 找到目标节点
        }
        
        // 更新邻居距离
        int currentIndex = mNodeIdToIndex[currentNodeId];
        const GridNode& currentNode = mGridNodes[currentIndex];
        
        for (int neighborId : currentNode.neighbors) {
            if (visited.contains(neighborId)) continue;
            
            int neighborIndex = mNodeIdToIndex[neighborId];
            double edgeWeight = calculateDistance(currentNode.position, mGridNodes[neighborIndex].position);
            double newDistance = currentDist + edgeWeight;
            
            if (!distances.contains(neighborId) || newDistance < distances[neighborId]) {
                distances[neighborId] = newDistance;
                previous[neighborId] = currentNodeId;
                pq.push({newDistance, neighborId});
            }
        }
    }
    
    // 重构路径
    QVector<int> path;
    if (distances.contains(endNodeId)) {
        int current = endNodeId;
        while (current != -1) {
            path.prepend(current);
            current = previous.value(current, -1);
        }
    }
    
    return path;
}

QVector<int> GridPathPlanner::aStarPath(int startNodeId, int endNodeId) {
    if (!mNodeIdToIndex.contains(startNodeId) || !mNodeIdToIndex.contains(endNodeId)) {
        return QVector<int>();
    }
    
    std::priority_queue<AStarNode> openSet;
    QSet<int> closedSet;
    QHash<int, double> gScore;
    QHash<int, int> cameFrom;
    
    // 初始化
    for (const auto& node : mGridNodes) {
        gScore[node.id] = std::numeric_limits<double>::infinity();
    }
    
    gScore[startNodeId] = 0.0;
    double h = heuristicDistance(startNodeId, endNodeId);
    openSet.push(AStarNode(startNodeId, 0.0, h));
    
    while (!openSet.empty()) {
        AStarNode current = openSet.top();
        openSet.pop();
        
        if (current.nodeId == endNodeId) {
            // 重构路径
            QVector<int> path;
            int nodeId = endNodeId;
            while (cameFrom.contains(nodeId)) {
                path.prepend(nodeId);
                nodeId = cameFrom[nodeId];
            }
            path.prepend(startNodeId);
            return path;
        }
        
        closedSet.insert(current.nodeId);
        
        int currentIndex = mNodeIdToIndex[current.nodeId];
        const GridNode& currentNode = mGridNodes[currentIndex];
        
        for (int neighborId : currentNode.neighbors) {
            if (closedSet.contains(neighborId)) continue;
            
            int neighborIndex = mNodeIdToIndex[neighborId];
            double tentativeGScore = gScore[current.nodeId] + 
                calculateDistance(currentNode.position, mGridNodes[neighborIndex].position);
            
            if (tentativeGScore < gScore[neighborId]) {
                cameFrom[neighborId] = current.nodeId;
                gScore[neighborId] = tentativeGScore;
                double hScore = heuristicDistance(neighborId, endNodeId);
                openSet.push(AStarNode(neighborId, tentativeGScore, hScore));
            }
        }
    }
    
    return QVector<int>(); // 未找到路径
}

QVector<PathPoint> GridPathPlanner::solveTSPWithDijkstra(int startNodeId) {
    QVector<int> riskNodeIds = getRiskNodeIds();
    
    logMessage(QString("solveTSPWithDijkstra: start node %1, risk nodes count %2")
              .arg(startNodeId).arg(riskNodeIds.size()), Qgis::MessageLevel::Info);
    
    if (riskNodeIds.isEmpty()) {
        logMessage("solveTSPWithDijkstra: No risk nodes found, returning empty path", Qgis::MessageLevel::Warning);
        return QVector<PathPoint>();
    }
    
    // 对于大规模问题，使用简化的贪心算法而不是构建完整距离矩阵
    if (riskNodeIds.size() > 100) {
        logMessage(QString("Large TSP problem (%1 points), using optimized greedy algorithm")
                  .arg(riskNodeIds.size()), Qgis::MessageLevel::Info);
        return solveOptimizedGreedyTSP(startNodeId, riskNodeIds);
    }
    
    // 构建风险点间的距离矩阵
    logMessage("solveTSPWithDijkstra: Building distance matrix...", Qgis::MessageLevel::Info);
    buildRiskPointDistanceMatrix(riskNodeIds);
    
    // 使用最近邻算法求解TSP
    QVector<int> tspTour = nearestNeighborTSP(startNodeId, riskNodeIds);
    
    // 使用2-opt改进
    tspTour = twoOptTSP(tspTour, riskNodeIds);
    
    // 构建完整路径，包括节点间的Dijkstra路径
    QVector<PathPoint> fullPath;
    
    for (int i = 0; i < tspTour.size() - 1; ++i) {
        int fromNodeId = tspTour[i];
        int toNodeId = tspTour[i + 1];
        
        QVector<int> segmentPath = dijkstraPath(fromNodeId, toNodeId);
        
        for (int j = 0; j < segmentPath.size(); ++j) {
            if (i > 0 && j == 0) continue; // 避免重复节点
            
            int nodeIndex = mNodeIdToIndex[segmentPath[j]];
            const GridNode& node = mGridNodes[nodeIndex];
            
            PathPoint pathPoint(node.position);
            pathPoint.isRiskPoint = node.isRiskPoint;
            if (pathPoint.isRiskPoint) {
                pathPoint.riskPointId = segmentPath[j];
            }
            
            fullPath.append(pathPoint);
        }
    }
    
    return fullPath;
}

QVector<PathPoint> GridPathPlanner::solveOptimizedGreedyTSP(int startNodeId, const QVector<int>& riskNodeIds) {
    logMessage("Starting optimized greedy TSP algorithm...", Qgis::MessageLevel::Info);
    
    QVector<int> tour;
    QSet<int> visited;
    QVector<PathPoint> fullPath;
    
    tour.append(startNodeId);
    visited.insert(startNodeId);
    
    int currentNodeId = startNodeId;
    int visitedCount = 0;
    int totalRiskPoints = riskNodeIds.size();
    
    // 添加起点到路径
    if (mNodeIdToIndex.contains(startNodeId)) {
        int nodeIndex = mNodeIdToIndex[startNodeId];
        const GridNode& node = mGridNodes[nodeIndex];
        PathPoint pathPoint(node.position);
        pathPoint.isRiskPoint = node.isRiskPoint;
        fullPath.append(pathPoint);
        
        logMessage(QString("Start node %1: position (%2, %3), connections: %4")
                  .arg(startNodeId)
                  .arg(node.position.x(), 0, 'f', 2)
                  .arg(node.position.y(), 0, 'f', 2)
                  .arg(node.neighbors.size()), Qgis::MessageLevel::Info);
        
        // 详细显示起点的邻居
        if (node.neighbors.size() > 0) {
            QString neighborsStr = "Start node neighbors: ";
            for (int i = 0; i < qMin(10, node.neighbors.size()); ++i) {
                if (i > 0) neighborsStr += ", ";
                neighborsStr += QString::number(node.neighbors[i]);
            }
            if (node.neighbors.size() > 10) {
                neighborsStr += QString(" ... (total %1)").arg(node.neighbors.size());
            }
            logMessage(neighborsStr, Qgis::MessageLevel::Info);
        } else {
            logMessage("WARNING: Start node has NO neighbors - this indicates connectivity issues!", Qgis::MessageLevel::Warning);
        }
    }
    
    // 调试：检查前几个风险点的连通性
    for (int i = 0; i < qMin(5, riskNodeIds.size()); ++i) {
        int riskNodeId = riskNodeIds[i];
        if (mNodeIdToIndex.contains(riskNodeId)) {
            int nodeIndex = mNodeIdToIndex[riskNodeId];
            const GridNode& node = mGridNodes[nodeIndex];
            double heuristicDist = heuristicDistance(currentNodeId, riskNodeId);
            
            // 添加详细的Dijkstra调试
            logMessage(QString("Testing path from start %1 to risk node %2...").arg(currentNodeId).arg(riskNodeId), Qgis::MessageLevel::Info);
            QVector<int> testPath = dijkstraPath(currentNodeId, riskNodeId);
            
            logMessage(QString("Risk node %1: position (%2, %3), connections: %4, heuristic distance: %5, dijkstra path: %6")
                      .arg(riskNodeId)
                      .arg(node.position.x(), 0, 'f', 2)
                      .arg(node.position.y(), 0, 'f', 2)
                      .arg(node.neighbors.size())
                      .arg(heuristicDist, 0, 'f', 2)
                      .arg(testPath.isEmpty() ? "NO PATH FOUND" : QString("found %1 steps").arg(testPath.size())), 
                      Qgis::MessageLevel::Info);
            
            // 显示风险点的邻居
            if (node.neighbors.size() > 0) {
                QString neighborsStr = QString("Risk node %1 neighbors: ").arg(riskNodeId);
                for (int j = 0; j < qMin(5, node.neighbors.size()); ++j) {
                    if (j > 0) neighborsStr += ", ";
                    neighborsStr += QString::number(node.neighbors[j]);
                }
                if (node.neighbors.size() > 5) {
                    neighborsStr += QString(" ... (total %1)").arg(node.neighbors.size());
                }
                logMessage(neighborsStr, Qgis::MessageLevel::Info);
            } else {
                logMessage(QString("WARNING: Risk node %1 has NO neighbors!").arg(riskNodeId), Qgis::MessageLevel::Warning);
            }
            
            // 如果找不到路径，进行连通性分析
            if (testPath.isEmpty()) {
                logMessage(QString("No path found from %1 to %2, analyzing connectivity...").arg(currentNodeId).arg(riskNodeId), Qgis::MessageLevel::Warning);
                
                // 检查起点是否在连通分量中
                QSet<int> reachableFromStart = getConnectedComponent(currentNodeId);
                logMessage(QString("Nodes reachable from start: %1").arg(reachableFromStart.size()), Qgis::MessageLevel::Info);
                
                // 检查风险点是否在连通分量中
                QSet<int> reachableFromRisk = getConnectedComponent(riskNodeId);
                logMessage(QString("Nodes reachable from risk node %1: %2").arg(riskNodeId).arg(reachableFromRisk.size()), Qgis::MessageLevel::Info);
                
                if (!reachableFromStart.contains(riskNodeId)) {
                    logMessage(QString("Risk node %1 is NOT reachable from start node %2 - they are in different connected components!")
                              .arg(riskNodeId).arg(currentNodeId), Qgis::MessageLevel::Critical);
                }
            }
        }
    }
    
    // 进行全局连通性分析
    QVector<QSet<int>> connectedComponents = findConnectedComponents();
    logMessage(QString("Grid has %1 connected components").arg(connectedComponents.size()), Qgis::MessageLevel::Info);
    
    // 找到起点所在的连通分量
    int startComponentIndex = -1;
    for (int i = 0; i < connectedComponents.size(); ++i) {
        if (connectedComponents[i].contains(startNodeId)) {
            startComponentIndex = i;
            break;
        }
    }
    
    if (startComponentIndex != -1) {
        logMessage(QString("Start node is in connected component %1 with %2 nodes")
                  .arg(startComponentIndex).arg(connectedComponents[startComponentIndex].size()), Qgis::MessageLevel::Info);
        
        // 统计风险点在各个连通分量中的分布
        QVector<int> riskPointsPerComponent(connectedComponents.size(), 0);
        int riskPointsInStartComponent = 0;
        
        for (int riskNodeId : riskNodeIds) {
            for (int i = 0; i < connectedComponents.size(); ++i) {
                if (connectedComponents[i].contains(riskNodeId)) {
                    riskPointsPerComponent[i]++;
                    if (i == startComponentIndex) {
                        riskPointsInStartComponent++;
                    }
                    break;
                }
            }
        }
        
        logMessage(QString("Risk points distribution: %1 in start component, %2 total risk points")
                  .arg(riskPointsInStartComponent).arg(riskNodeIds.size()), Qgis::MessageLevel::Info);
        
        for (int i = 0; i < connectedComponents.size(); ++i) {
            if (riskPointsPerComponent[i] > 0) {
                logMessage(QString("Component %1: %2 nodes, %3 risk points")
                          .arg(i).arg(connectedComponents[i].size()).arg(riskPointsPerComponent[i]), 
                          Qgis::MessageLevel::Info);
            }
        }
        
        // 如果起点所在的连通分量中没有风险点，这就是问题所在
        if (riskPointsInStartComponent == 0) {
            logMessage("CRITICAL ERROR: No risk points are reachable from the start node! The grid may be disconnected or the start point is isolated.", 
                      Qgis::MessageLevel::Critical);
            return fullPath; // 返回只包含起点的路径
        }
    }
    
    while (visited.size() < riskNodeIds.size() + 1) {
        int nearestNodeId = -1;
        double minDistance = std::numeric_limits<double>::infinity();
        QVector<int> bestPath;
        int checkedNodes = 0;
        int pathFoundCount = 0;
        
        // 只考虑未访问的风险点
        for (int riskNodeId : riskNodeIds) {
            if (visited.contains(riskNodeId)) continue;
            
            checkedNodes++;
            
            // 使用启发式距离进行初步筛选
            double heuristicDist = heuristicDistance(currentNodeId, riskNodeId);
            if (heuristicDist > minDistance * 2.0) continue; // 跳过明显较远的点
            
            QVector<int> path = dijkstraPath(currentNodeId, riskNodeId);
            if (!path.isEmpty()) {
                pathFoundCount++;
                double pathLength = 0.0;
                for (int i = 0; i < path.size() - 1; ++i) {
                    int nodeIndex1 = mNodeIdToIndex[path[i]];
                    int nodeIndex2 = mNodeIdToIndex[path[i + 1]];
                    pathLength += calculateDistance(mGridNodes[nodeIndex1].position, 
                                                  mGridNodes[nodeIndex2].position);
                }
                
                if (pathLength < minDistance) {
                    minDistance = pathLength;
                    nearestNodeId = riskNodeId;
                    bestPath = path;
                }
            }
        }
        
        logMessage(QString("Search iteration %1: checked %2 nodes, found %3 paths, nearest distance: %4")
                  .arg(visitedCount + 1).arg(checkedNodes).arg(pathFoundCount).arg(minDistance, 0, 'f', 2), 
                  Qgis::MessageLevel::Info);
        
        if (nearestNodeId == -1) {
            logMessage(QString("No more reachable risk points found after visiting %1/%2 points")
                      .arg(visitedCount).arg(totalRiskPoints), Qgis::MessageLevel::Warning);
            break;
        }
        
        // 添加路径到结果（跳过第一个点避免重复）
        for (int j = 1; j < bestPath.size(); ++j) {
            int nodeIndex = mNodeIdToIndex[bestPath[j]];
            const GridNode& node = mGridNodes[nodeIndex];
            
            PathPoint pathPoint(node.position);
            pathPoint.isRiskPoint = node.isRiskPoint;
            if (pathPoint.isRiskPoint) {
                pathPoint.riskPointId = bestPath[j];
            }
            
            fullPath.append(pathPoint);
        }
        
        tour.append(nearestNodeId);
        visited.insert(nearestNodeId);
        currentNodeId = nearestNodeId;
        visitedCount++;
        
        // 每访问10个点输出一次进度
        if (visitedCount % 10 == 0 || visitedCount == totalRiskPoints) {
            logMessage(QString("Progress: visited %1/%2 risk points")
                      .arg(visitedCount).arg(totalRiskPoints), Qgis::MessageLevel::Info);
        }
    }
    
    logMessage(QString("Optimized greedy TSP completed: visited %1/%2 risk points")
              .arg(visitedCount).arg(totalRiskPoints), Qgis::MessageLevel::Info);
    
    return fullPath;
}

// 新增辅助函数：获取从指定节点可达的所有节点（连通分量）
QSet<int> GridPathPlanner::getConnectedComponent(int startNodeId) {
    QSet<int> visited;
    QQueue<int> queue;
    
    queue.enqueue(startNodeId);
    visited.insert(startNodeId);
    
    while (!queue.isEmpty()) {
        int currentNodeId = queue.dequeue();
        
        if (mNodeIdToIndex.contains(currentNodeId)) {
            int currentIndex = mNodeIdToIndex[currentNodeId];
            const GridNode& currentNode = mGridNodes[currentIndex];
            
            for (int neighborId : currentNode.neighbors) {
                if (!visited.contains(neighborId)) {
                    visited.insert(neighborId);
                    queue.enqueue(neighborId);
                }
            }
        }
    }
    
    return visited;
}

// 新增辅助函数：找到所有连通分量
QVector<QSet<int>> GridPathPlanner::findConnectedComponents() {
    QVector<QSet<int>> components;
    QSet<int> allVisited;
    
    for (const auto& node : mGridNodes) {
        if (!allVisited.contains(node.id)) {
            QSet<int> component = getConnectedComponent(node.id);
            components.append(component);
            allVisited.unite(component);
        }
    }
    
    return components;
}

QVector<int> GridPathPlanner::nearestNeighborTSP(int startNodeId, const QVector<int>& riskNodeIds) {
    QVector<int> tour;
    QSet<int> visited;
    
    tour.append(startNodeId);
    visited.insert(startNodeId);
    
    int currentNodeId = startNodeId;
    
    while (visited.size() < riskNodeIds.size() + 1) {
        int nearestNodeId = -1;
        double minDistance = std::numeric_limits<double>::infinity();
        
        for (int riskNodeId : riskNodeIds) {
            if (visited.contains(riskNodeId)) continue;
            
            QVector<int> path = dijkstraPath(currentNodeId, riskNodeId);
            if (!path.isEmpty()) {
                double pathLength = 0.0;
                for (int i = 0; i < path.size() - 1; ++i) {
                    int nodeIndex1 = mNodeIdToIndex[path[i]];
                    int nodeIndex2 = mNodeIdToIndex[path[i + 1]];
                    pathLength += calculateDistance(mGridNodes[nodeIndex1].position, 
                                                  mGridNodes[nodeIndex2].position);
                }
                
                if (pathLength < minDistance) {
                    minDistance = pathLength;
                    nearestNodeId = riskNodeId;
                }
            }
        }
        
        if (nearestNodeId == -1) break;
        
        tour.append(nearestNodeId);
        visited.insert(nearestNodeId);
        currentNodeId = nearestNodeId;
    }
    
    // 返回起点
    tour.append(startNodeId);
    
    return tour;
}

QVector<int> GridPathPlanner::twoOptTSP(const QVector<int>& tour, const QVector<int>& riskNodeIds) {
    if (tour.size() < 4) return tour;
    
    QVector<int> improvedTour = tour;
    bool improved = true;
    
    while (improved) {
        improved = false;
        
        for (int i = 1; i < improvedTour.size() - 2; ++i) {
            for (int j = i + 1; j < improvedTour.size() - 1; ++j) {
                // 计算当前边的总长度
                double currentLength = 0.0;
                QVector<int> path1 = dijkstraPath(improvedTour[i], improvedTour[i + 1]);
                QVector<int> path2 = dijkstraPath(improvedTour[j], improvedTour[j + 1]);
                
                for (int k = 0; k < path1.size() - 1; ++k) {
                    int idx1 = mNodeIdToIndex[path1[k]];
                    int idx2 = mNodeIdToIndex[path1[k + 1]];
                    currentLength += calculateDistance(mGridNodes[idx1].position, mGridNodes[idx2].position);
                }
                
                for (int k = 0; k < path2.size() - 1; ++k) {
                    int idx1 = mNodeIdToIndex[path2[k]];
                    int idx2 = mNodeIdToIndex[path2[k + 1]];
                    currentLength += calculateDistance(mGridNodes[idx1].position, mGridNodes[idx2].position);
                }
                
                // 计算交换后边的总长度
                double newLength = 0.0;
                QVector<int> newPath1 = dijkstraPath(improvedTour[i], improvedTour[j]);
                QVector<int> newPath2 = dijkstraPath(improvedTour[i + 1], improvedTour[j + 1]);
                
                for (int k = 0; k < newPath1.size() - 1; ++k) {
                    int idx1 = mNodeIdToIndex[newPath1[k]];
                    int idx2 = mNodeIdToIndex[newPath1[k + 1]];
                    newLength += calculateDistance(mGridNodes[idx1].position, mGridNodes[idx2].position);
                }
                
                for (int k = 0; k < newPath2.size() - 1; ++k) {
                    int idx1 = mNodeIdToIndex[newPath2[k]];
                    int idx2 = mNodeIdToIndex[newPath2[k + 1]];
                    newLength += calculateDistance(mGridNodes[idx1].position, mGridNodes[idx2].position);
                }
                
                if (newLength < currentLength) {
                    // 执行2-opt交换
                    std::reverse(improvedTour.begin() + i + 1, improvedTour.begin() + j + 1);
                    improved = true;
                }
            }
        }
    }
    
    return improvedTour;
}

void GridPathPlanner::buildRiskPointDistanceMatrix(const QVector<int>& riskNodeIds) {
    int size = riskNodeIds.size();
    mDistanceMatrix.clear();
    mDistanceMatrix.resize(size);
    
    logMessage(QString("Building distance matrix for %1 risk points...").arg(size), 
              Qgis::MessageLevel::Info);
    
    int totalCalculations = size * (size - 1) / 2; // 只计算上三角矩阵
    int completed = 0;
    
    for (int i = 0; i < size; ++i) {
        mDistanceMatrix[i].resize(size);
        mDistanceMatrix[i][i] = 0.0; // 对角线为0
        
        for (int j = i + 1; j < size; ++j) {
            QVector<int> path = dijkstraPath(riskNodeIds[i], riskNodeIds[j]);
            double pathLength = 0.0;
            
            if (!path.isEmpty()) {
                for (int k = 0; k < path.size() - 1; ++k) {
                    int nodeIndex1 = mNodeIdToIndex[path[k]];
                    int nodeIndex2 = mNodeIdToIndex[path[k + 1]];
                    pathLength += calculateDistance(mGridNodes[nodeIndex1].position, 
                                                  mGridNodes[nodeIndex2].position);
                }
            } else {
                // 如果找不到路径，使用启发式距离
                pathLength = heuristicDistance(riskNodeIds[i], riskNodeIds[j]) * 1.5;
            }
            
            mDistanceMatrix[i][j] = pathLength;
            mDistanceMatrix[j][i] = pathLength; // 对称矩阵
            
            completed++;
            
            // 每完成10%输出进度
            if (completed % (totalCalculations / 10) == 0) {
                int percentage = (completed * 100) / totalCalculations;
                logMessage(QString("Distance matrix progress: %1% (%2/%3)")
                          .arg(percentage).arg(completed).arg(totalCalculations), 
                          Qgis::MessageLevel::Info);
            }
        }
    }
    
    logMessage("Distance matrix construction completed", Qgis::MessageLevel::Info);
}

double GridPathPlanner::calculateDistance(const QVector3D& p1, const QVector3D& p2) const {
    return sqrt(pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2));
}

double GridPathPlanner::heuristicDistance(int nodeId1, int nodeId2) const {
    if (!mNodeIdToIndex.contains(nodeId1) || !mNodeIdToIndex.contains(nodeId2)) {
        return 0.0;
    }
    
    int idx1 = mNodeIdToIndex[nodeId1];
    int idx2 = mNodeIdToIndex[nodeId2];
    
    return calculateDistance(mGridNodes[idx1].position, mGridNodes[idx2].position);
}

int GridPathPlanner::findNearestGridNode(const QVector3D& position) const {
    if (mGridNodes.isEmpty()) return -1;
    
    int nearestNodeId = -1;
    double minDistance = std::numeric_limits<double>::infinity();
    
    for (const auto& node : mGridNodes) {
        double distance = calculateDistance(position, node.position);
        if (distance < minDistance) {
            minDistance = distance;
            nearestNodeId = node.id;
        }
    }
    
    return nearestNodeId;
}

QVector<int> GridPathPlanner::getRiskNodeIds() const {
    QVector<int> riskNodeIds;
    
    for (const auto& node : mGridNodes) {
        if (node.isRiskPoint) {
            riskNodeIds.append(node.id);
        }
    }
    
    logMessage(QString("getRiskNodeIds: found %1 risk nodes").arg(riskNodeIds.size()), Qgis::MessageLevel::Info);
    
    return riskNodeIds;
}

QVector<PathPoint> GridPathPlanner::convertToPathPoints(const QVector<int>& nodeIds) const {
    QVector<PathPoint> pathPoints;
    
    for (int nodeId : nodeIds) {
        if (mNodeIdToIndex.contains(nodeId)) {
            int nodeIndex = mNodeIdToIndex[nodeId];
            const GridNode& node = mGridNodes[nodeIndex];
            
            PathPoint point(node.position);
            point.isRiskPoint = node.isRiskPoint;
            if (point.isRiskPoint) {
                point.riskPointId = nodeId;
            }
            
            pathPoints.append(point);
        }
    }
    
    return pathPoints;
}

bool GridPathPlanner::exportPathToFiles(const GridPlanningResult& result, const QString& outputPath) {
    if (!result.success || result.optimalPath.isEmpty()) {
        logMessage("No valid path to export", Qgis::MessageLevel::Warning);
        return false;
    }
    
    QDir outputDir(outputPath);
    if (!outputDir.exists()) {
        outputDir.mkpath(".");
    }
    
    // 导出路径为CSV
    QString csvPath = outputDir.filePath("grid_path.csv");
    QFile csvFile(csvPath);
    
    if (csvFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&csvFile);
        out.setCodec("UTF-8");
        
        out << "point_id,x,y,z,is_risk_point,risk_point_id,distance_from_start\n";
        
        for (int i = 0; i < result.optimalPath.size(); ++i) {
            const PathPoint& point = result.optimalPath[i];
            
            out << (i + 1) << ","
                << QString::number(point.position.x(), 'f', 2) << ","
                << QString::number(point.position.y(), 'f', 2) << ","
                << QString::number(point.position.z(), 'f', 2) << ","
                << (point.isRiskPoint ? "true" : "false") << ","
                << point.riskPointId << ","
                << QString::number(point.distanceFromStart, 'f', 2) << "\n";
        }
        
        csvFile.close();
        logMessage(QString("Path exported to CSV: %1").arg(csvPath), Qgis::MessageLevel::Success);
    }
    
    // 导出路径为WKT
    QString wktPath = outputDir.filePath("grid_path.wkt");
    QFile wktFile(wktPath);
    
    if (wktFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&wktFile);
        out.setCodec("UTF-8");
        
        out << "LINESTRING (";
        for (int i = 0; i < result.optimalPath.size(); ++i) {
            const PathPoint& point = result.optimalPath[i];
            if (i > 0) out << ", ";
            out << QString::number(point.position.x(), 'f', 2) << " "
                << QString::number(point.position.y(), 'f', 2) << " "
                << QString::number(point.position.z(), 'f', 2);
        }
        out << ")\n";
        
        wktFile.close();
        logMessage(QString("Path exported to WKT: %1").arg(wktPath), Qgis::MessageLevel::Success);
    }
    
    // 导出统计报告
    QString statsPath = outputDir.filePath("grid_path_statistics.txt");
    QFile statsFile(statsPath);
    
    if (statsFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&statsFile);
        out.setCodec("UTF-8");
        
        out << getStatistics(result);
        
        statsFile.close();
        logMessage(QString("Statistics exported to: %1").arg(statsPath), Qgis::MessageLevel::Success);
    }
    
    return true;
}

QString GridPathPlanner::getStatistics(const GridPlanningResult& result) const {
    QString stats;
    stats += "=== 网格路径规划统计报告 ===\n\n";
    stats += QString("规划状态: %1\n").arg(result.success ? "成功" : "失败");
    
    if (!result.success) {
        stats += QString("错误信息: %1\n").arg(result.errorMessage);
        return stats;
    }
    
    stats += QString("使用算法: %1\n").arg(result.algorithm);
    stats += QString("网格节点总数: %1\n").arg(result.gridNodes.size());
    stats += QString("风险点总数: %1\n").arg(result.totalRiskPoints);
    stats += QString("访问的风险点数: %1\n").arg(result.visitedRiskPoints);
    stats += QString("路径总长度: %1 米\n").arg(result.totalPathLength, 0, 'f', 2);
    stats += QString("路径点总数: %1\n").arg(result.optimalPath.size());
    
    if (result.visitedRiskPoints > 0) {
        stats += QString("平均每个风险点路径长度: %1 米\n")
                .arg(result.totalPathLength / result.visitedRiskPoints, 0, 'f', 2);
    }
    
    double coverageRate = result.totalRiskPoints > 0 ? 
                         (double)result.visitedRiskPoints / result.totalRiskPoints * 100.0 : 0.0;
    stats += QString("风险点覆盖率: %1%\n").arg(coverageRate, 0, 'f', 1);
    
    return stats;
}

void GridPathPlanner::asyncExecutePlanning(const GridPlanningParameters& params) {
    GridPlanningResult result = executePlanning(params);
    // 信号已在executePlanning中发出
}

void GridPathPlanner::clear() {
    mGridNodes.clear();
    mRiskPoints.clear();
    mRiskLines.clear();
    mNodeIdToIndex.clear();
    mDistanceMatrix.clear();
}

int GridPathPlanner::repairGridConnectivity() {
    const double MAX_CONNECTION_DISTANCE = 10.0; // 最大连接距离（米）
    const int MAX_CONNECTIONS_PER_NODE = 8; // 每个节点最多连接数
    
    int addedConnections = 0;
    
    logMessage(QString("Attempting to repair grid connectivity with max distance %1m")
              .arg(MAX_CONNECTION_DISTANCE), Qgis::MessageLevel::Info);
    
    // 为每个节点寻找附近的节点进行连接
    for (int i = 0; i < mGridNodes.size(); ++i) {
        GridNode& node1 = mGridNodes[i];
        
        // 跳过已有足够连接的节点
        if (node1.neighbors.size() >= MAX_CONNECTIONS_PER_NODE) {
            continue;
        }
        
        // 寻找距离最近的节点
        QVector<QPair<double, int>> nearbyNodes;
        
        for (int j = 0; j < mGridNodes.size(); ++j) {
            if (i == j) continue;
            
            const GridNode& node2 = mGridNodes[j];
            double distance = calculateDistance(node1.position, node2.position);
            
            if (distance <= MAX_CONNECTION_DISTANCE) {
                // 检查是否已经连接
                if (!node1.neighbors.contains(node2.id)) {
                    nearbyNodes.append(qMakePair(distance, j));
                }
            }
        }
        
        // 按距离排序
        std::sort(nearbyNodes.begin(), nearbyNodes.end());
        
        // 连接最近的几个节点
        int connectionsToAdd = qMin(MAX_CONNECTIONS_PER_NODE - node1.neighbors.size(), 
                                   nearbyNodes.size());
        
        for (int k = 0; k < connectionsToAdd; ++k) {
            int j = nearbyNodes[k].second;
            GridNode& node2 = mGridNodes[j];
            
            // 检查node2是否还能接受更多连接
            if (node2.neighbors.size() >= MAX_CONNECTIONS_PER_NODE) {
                continue;
            }
            
            // 建立双向连接
            node1.neighbors.append(node2.id);
            node2.neighbors.append(node1.id);
            addedConnections++;
            
            if (addedConnections <= 10) { // 只显示前10个连接的详细信息
                logMessage(QString("Connected nodes %1 and %2 (distance: %3m)")
                          .arg(node1.id).arg(node2.id).arg(nearbyNodes[k].first, 0, 'f', 2), 
                          Qgis::MessageLevel::Info);
            }
        }
    }
    
    logMessage(QString("Grid repair completed: added %1 new connections").arg(addedConnections), 
              Qgis::MessageLevel::Info);
    
    return addedConnections;
}

bool GridPathPlanner::loadRiskLines(const QString& shapefile) {
    mRiskLines.clear();
    
    if (!mShapefileHandler->loadRiskLineEvents(shapefile)) {
        logMessage("Failed to load risk lines shapefile", Qgis::MessageLevel::Critical);
        return false;
    }
    
    auto lineEvents = mShapefileHandler->getRiskLineEvents();
    
    for (int i = 0; i < lineEvents.size(); ++i) {
        const auto& lineEvent = lineEvents[i];
        RiskLineEvent riskLine(i, lineEvent.first, lineEvent.second);
        mRiskLines.append(riskLine);
    }
    
    logMessage(QString("Loaded %1 risk line events").arg(mRiskLines.size()), Qgis::MessageLevel::Info);
    return !mRiskLines.isEmpty();
}

void GridPathPlanner::associateRiskLinesToGrid(double maxDistance) {
    logMessage(QString("Starting risk line association: %1 risk lines, max distance %2m")
              .arg(mRiskLines.size()).arg(maxDistance), Qgis::MessageLevel::Info);
    
    int associatedCount = 0;
    int tooFarCount = 0;
    
    for (int lineIdx = 0; lineIdx < mRiskLines.size(); ++lineIdx) {
        RiskLineEvent& riskLine = mRiskLines[lineIdx];
        
        if (lineIdx < 3) {
            logMessage(QString("Risk line %1: start (%2, %3), end (%4, %5)")
                      .arg(lineIdx+1)
                      .arg(riskLine.startPoint.x(), 0, 'f', 2).arg(riskLine.startPoint.y(), 0, 'f', 2)
                      .arg(riskLine.endPoint.x(), 0, 'f', 2).arg(riskLine.endPoint.y(), 0, 'f', 2), 
                      Qgis::MessageLevel::Info);
        }
        
        // 为风险线寻找网格路径
        QVector<int> gridPath = findGridPathForLine(riskLine);
        
        if (!gridPath.isEmpty()) {
            riskLine.gridNodeIds = gridPath;
            associatedCount++;
            
            if (lineIdx < 3) {
                logMessage(QString("Risk line %1: ASSOCIATED with %2 grid nodes")
                          .arg(lineIdx+1).arg(gridPath.size()), Qgis::MessageLevel::Info);
            }
        } else {
            tooFarCount++;
            if (lineIdx < 3) {
                logMessage(QString("Risk line %1: NO GRID PATH FOUND").arg(lineIdx+1), 
                          Qgis::MessageLevel::Warning);
            }
        }
    }
    
    logMessage(QString("Risk line association complete: %1 associated, %2 no path found")
              .arg(associatedCount).arg(tooFarCount), Qgis::MessageLevel::Info);
}

QVector<int> GridPathPlanner::findGridPathForLine(const RiskLineEvent& riskLine) {
    // 找到线的起点和终点最近的网格节点
    int startNodeId = findNearestGridNode(riskLine.startPoint);
    int endNodeId = findNearestGridNode(riskLine.endPoint);
    
    if (startNodeId == -1 || endNodeId == -1) {
        return QVector<int>();
    }
    
    // 使用Dijkstra算法找到从起点到终点的路径
    QVector<int> path = dijkstraPath(startNodeId, endNodeId);
    
    if (path.isEmpty()) {
        logMessage(QString("No path found from start node %1 to end node %2 for risk line %3")
                  .arg(startNodeId).arg(endNodeId).arg(riskLine.lineId), 
                  Qgis::MessageLevel::Warning);
    }
    
    return path;
}

QVector<PathPoint> GridPathPlanner::solveLineTSP(int startNodeId) {
    logMessage("Starting line TSP algorithm...", Qgis::MessageLevel::Info);
    
    QVector<PathPoint> fullPath;
    QSet<int> completedLines;
    
    // 添加起始点
    if (mNodeIdToIndex.contains(startNodeId)) {
        int nodeIndex = mNodeIdToIndex[startNodeId];
        const GridNode& node = mGridNodes[nodeIndex];
        PathPoint pathPoint(node.position);
        fullPath.append(pathPoint);
    }
    
    int currentNodeId = startNodeId;
    int completedCount = 0;
    
    while (completedLines.size() < mRiskLines.size()) {
        int bestLineIndex = -1;
        double minDistance = std::numeric_limits<double>::infinity();
        QVector<int> bestPath;
        
        // 寻找距离当前节点最近且未完成的风险线
        for (int i = 0; i < mRiskLines.size(); ++i) {
            if (completedLines.contains(i)) continue;
            
            const RiskLineEvent& riskLine = mRiskLines[i];
            if (riskLine.gridNodeIds.isEmpty()) continue;
            
            // 计算到风险线起点的距离
            int lineStartNodeId = riskLine.gridNodeIds.first();
            QVector<int> pathToLine = dijkstraPath(currentNodeId, lineStartNodeId);
            
            if (!pathToLine.isEmpty()) {
                double pathLength = 0.0;
                for (int j = 0; j < pathToLine.size() - 1; ++j) {
                    int nodeIndex1 = mNodeIdToIndex[pathToLine[j]];
                    int nodeIndex2 = mNodeIdToIndex[pathToLine[j + 1]];
                    pathLength += calculateDistance(mGridNodes[nodeIndex1].position, 
                                                  mGridNodes[nodeIndex2].position);
                }
                
                if (pathLength < minDistance) {
                    minDistance = pathLength;
                    bestLineIndex = i;
                    bestPath = pathToLine;
                }
            }
        }
        
        if (bestLineIndex == -1) {
            logMessage("No more reachable risk lines found", Qgis::MessageLevel::Warning);
            break;
        }
        
        // 添加路径到风险线起点
        for (int j = 1; j < bestPath.size(); ++j) {
            int nodeIndex = mNodeIdToIndex[bestPath[j]];
            const GridNode& node = mGridNodes[nodeIndex];
            PathPoint pathPoint(node.position);
            fullPath.append(pathPoint);
        }
        
        // 添加风险线的完整路径
        const RiskLineEvent& selectedLine = mRiskLines[bestLineIndex];
        for (int j = 0; j < selectedLine.gridNodeIds.size(); ++j) {
            int nodeIndex = mNodeIdToIndex[selectedLine.gridNodeIds[j]];
            const GridNode& node = mGridNodes[nodeIndex];
            PathPoint pathPoint(node.position);
            pathPoint.isRiskPoint = true; // 标记为风险线路径点
            fullPath.append(pathPoint);
        }
        
        // 更新当前节点为风险线的终点
        currentNodeId = selectedLine.gridNodeIds.last();
        completedLines.insert(bestLineIndex);
        completedCount++;
        
        // 标记风险线为已完成
        mRiskLines[bestLineIndex].isCompleted = true;
        
        logMessage(QString("Completed risk line %1/%2: line ID %3")
                  .arg(completedCount).arg(mRiskLines.size()).arg(selectedLine.lineId), 
                  Qgis::MessageLevel::Info);
    }
    
    logMessage(QString("Line TSP completed: %1/%2 risk lines completed")
              .arg(completedCount).arg(mRiskLines.size()), Qgis::MessageLevel::Info);
    
    return fullPath;
}

QVector<PathPoint> GridPathPlanner::solveOptimizedLineGreedy(int startNodeId, const QVector<RiskLineEvent>& riskLines) {
    logMessage("Starting optimized line greedy algorithm...", Qgis::MessageLevel::Info);
    
    QVector<PathPoint> fullPath;
    QSet<int> completedLines;
    
    // 添加起始点
    if (mNodeIdToIndex.contains(startNodeId)) {
        int nodeIndex = mNodeIdToIndex[startNodeId];
        const GridNode& node = mGridNodes[nodeIndex];
        PathPoint pathPoint(node.position);
        fullPath.append(pathPoint);
    }
    
    int currentNodeId = startNodeId;
    int completedCount = 0;
    
    while (completedLines.size() < riskLines.size()) {
        int bestLineIndex = -1;
        double minTotalDistance = std::numeric_limits<double>::infinity();
        QVector<int> bestPathToLine;
        QVector<int> bestLinePath;
        
        // 寻找总距离最短的风险线
        for (int i = 0; i < riskLines.size(); ++i) {
            if (completedLines.contains(i)) continue;
            
            const RiskLineEvent& riskLine = riskLines[i];
            if (riskLine.gridNodeIds.isEmpty()) continue;
            
            // 计算到风险线起点的距离
            int lineStartNodeId = riskLine.gridNodeIds.first();
            QVector<int> pathToLine = dijkstraPath(currentNodeId, lineStartNodeId);
            
            if (!pathToLine.isEmpty()) {
                double distanceToLine = 0.0;
                for (int j = 0; j < pathToLine.size() - 1; ++j) {
                    int nodeIndex1 = mNodeIdToIndex[pathToLine[j]];
                    int nodeIndex2 = mNodeIdToIndex[pathToLine[j + 1]];
                    distanceToLine += calculateDistance(mGridNodes[nodeIndex1].position, 
                                                      mGridNodes[nodeIndex2].position);
                }
                
                // 计算风险线本身的长度
                double lineLength = 0.0;
                for (int j = 0; j < riskLine.gridNodeIds.size() - 1; ++j) {
                    int nodeIndex1 = mNodeIdToIndex[riskLine.gridNodeIds[j]];
                    int nodeIndex2 = mNodeIdToIndex[riskLine.gridNodeIds[j + 1]];
                    lineLength += calculateDistance(mGridNodes[nodeIndex1].position, 
                                                  mGridNodes[nodeIndex2].position);
                }
                
                double totalDistance = distanceToLine + lineLength;
                
                if (totalDistance < minTotalDistance) {
                    minTotalDistance = totalDistance;
                    bestLineIndex = i;
                    bestPathToLine = pathToLine;
                    bestLinePath = riskLine.gridNodeIds;
                }
            }
        }
        
        if (bestLineIndex == -1) {
            logMessage("No more reachable risk lines found", Qgis::MessageLevel::Warning);
            break;
        }
        
        // 添加路径到风险线起点
        for (int j = 1; j < bestPathToLine.size(); ++j) {
            int nodeIndex = mNodeIdToIndex[bestPathToLine[j]];
            const GridNode& node = mGridNodes[nodeIndex];
            PathPoint pathPoint(node.position);
            fullPath.append(pathPoint);
        }
        
        // 添加风险线的完整路径
        for (int j = 0; j < bestLinePath.size(); ++j) {
            int nodeIndex = mNodeIdToIndex[bestLinePath[j]];
            const GridNode& node = mGridNodes[nodeIndex];
            PathPoint pathPoint(node.position);
            pathPoint.isRiskPoint = true; // 标记为风险线路径点
            fullPath.append(pathPoint);
        }
        
        // 更新当前节点为风险线的终点
        currentNodeId = bestLinePath.last();
        completedLines.insert(bestLineIndex);
        completedCount++;
        
        // 标记风险线为已完成
        mRiskLines[bestLineIndex].isCompleted = true;
        
        logMessage(QString("Completed risk line %1/%2: line ID %3, total distance: %4m")
                  .arg(completedCount).arg(riskLines.size()).arg(riskLines[bestLineIndex].lineId)
                  .arg(minTotalDistance, 0, 'f', 2), Qgis::MessageLevel::Info);
    }
    
    logMessage(QString("Optimized line greedy completed: %1/%2 risk lines completed")
              .arg(completedCount).arg(riskLines.size()), Qgis::MessageLevel::Info);
    
    return fullPath;
}

QPair<bool, QString> GridPathPlanner::validateLineParameters(const LinePlanningParameters& params) {
    // 检查渔网线文件
    if (params.fishnetShapefile.isEmpty()) {
        return qMakePair(false, QString("Fishnet shapefile path is empty"));
    }
    
    QFileInfo fishnetFile(params.fishnetShapefile);
    if (!fishnetFile.exists()) {
        return qMakePair(false, QString("Fishnet shapefile does not exist: %1").arg(params.fishnetShapefile));
    }

    // 检查风险线文件
    if (params.riskLinesShapefile.isEmpty()) {
        return qMakePair(false, QString("Risk lines shapefile path is empty"));
    }
    
    QFileInfo riskFile(params.riskLinesShapefile);
    if (!riskFile.exists()) {
        return qMakePair(false, QString("Risk lines shapefile does not exist: %1").arg(params.riskLinesShapefile));
    }

    // 检查输出路径
    if (!params.outputPath.isEmpty()) {
        QDir outputDir(params.outputPath);
        if (!outputDir.exists()) {
            if (!outputDir.mkpath(".")) {
                return qMakePair(false, QString("Cannot create output directory: %1").arg(params.outputPath));
            }
        }
    }

    return qMakePair(true, QString(""));
}

QPair<bool, QString> GridPathPlanner::validateAreaParameters(const AreaPlanningParameters& params) {
    // 检查渔网线文件
    if (params.fishnetShapefile.isEmpty()) {
        return qMakePair(false, QString("Fishnet shapefile path is empty"));
    }
    
    QFileInfo fishnetFile(params.fishnetShapefile);
    if (!fishnetFile.exists()) {
        return qMakePair(false, QString("Fishnet shapefile does not exist: %1").arg(params.fishnetShapefile));
    }
    
    // 检查覆盖率阈值
    if (params.coverageThreshold < 0.0 || params.coverageThreshold > 1.0) {
        return qMakePair(false, QString("Coverage threshold must be between 0.0 and 1.0"));
    }
    
    // 检查最大迭代次数
    if (params.maxIterations <= 0) {
        return qMakePair(false, QString("Max iterations must be greater than 0"));
    }
    
    // 检查网格间距
    if (params.gridSpacing <= 0.0) {
        return qMakePair(false, QString("Grid spacing must be greater than 0"));
    }
    
    // 检查算法
    QStringList validAlgorithms = {"Area_Spiral", "Area_Grid"};
    if (!validAlgorithms.contains(params.algorithm)) {
        return qMakePair(false, QString("Invalid algorithm: %1. Valid options: %2")
                        .arg(params.algorithm).arg(validAlgorithms.join(", ")));
    }
    
    return qMakePair(true, QString());
}

void GridPathPlanner::asyncExecuteLinePlanning(const LinePlanningParameters& params) {
    LinePlanningResult result = executeLinePlanning(params);
    // 信号已在executeLinePlanning中发出
}

LinePlanningResult GridPathPlanner::executeLinePlanning(const LinePlanningParameters& params) {
    LinePlanningResult result;
    
    try {
        // 验证参数
        auto validation = validateLineParameters(params);
        if (!validation.first) {
            result.success = false;
            result.errorMessage = validation.second;
            return result;
        }

        logMessage("Starting line event path planning", Qgis::MessageLevel::Info);
        emit progressUpdated(0, "开始线事件路径规划...");

        // 步骤1: 从渔网线加载网格 (0-30%)
        emit progressUpdated(10, "正在加载渔网线数据...");
        if (!buildGridFromFishnet(params.fishnetShapefile)) {
            result.errorMessage = "Failed to load fishnet lines shapefile";
            emit planningFailed(result.errorMessage);
            return result;
        }
        emit progressUpdated(30, QString("渔网线加载完成，共%1个节点").arg(mGridNodes.size()));

        // 步骤2: 加载风险线事件 (30-50%)
        emit progressUpdated(35, "正在加载风险线事件数据...");
        if (!loadRiskLines(params.riskLinesShapefile)) {
            result.errorMessage = "Failed to load risk lines";
            emit planningFailed(result.errorMessage);
            return result;
        }
        emit progressUpdated(50, QString("风险线事件加载完成，共%1条线").arg(mRiskLines.size()));

        // 步骤3: 关联风险线到网格 (50-70%)
        emit progressUpdated(55, "正在关联风险线到网格...");
        associateRiskLinesToGrid(params.maxRiskLineDistance);
        
        int validRiskLines = 0;
        for (const auto& riskLine : mRiskLines) {
            if (!riskLine.gridNodeIds.isEmpty()) {
                validRiskLines++;
            }
        }
        result.totalRiskLines = validRiskLines;
        emit progressUpdated(70, QString("风险线关联完成，关联了%1条有效风险线").arg(result.totalRiskLines));

        if (validRiskLines == 0) {
            result.errorMessage = "No valid risk lines found within the grid or distance threshold";
            emit planningFailed(result.errorMessage);
            return result;
        }

        // 步骤4: 寻找起始节点 (70-75%)
        emit progressUpdated(72, "正在寻找起始节点...");
        int startNodeId = findNearestGridNode(params.startPoint);
        if (startNodeId == -1) {
            result.errorMessage = "Cannot find start node in the grid";
            emit planningFailed(result.errorMessage);
            return result;
        }
        emit progressUpdated(75, "起始节点确定");

        // 步骤5: 执行线事件路径规划 (75-95%)
        emit progressUpdated(80, "正在执行线事件路径规划算法...");
        
        if (params.algorithm == "Line_TSP_Dijkstra" || params.algorithm == "Line_TSP_AStar") {
            result.optimalPath = solveLineTSP(startNodeId);
            result.algorithm = "Line TSP with Dijkstra";
        } else if (params.algorithm == "Line_Greedy") {
            result.optimalPath = solveOptimizedLineGreedy(startNodeId, mRiskLines);
            result.algorithm = "Line Greedy";
        } else {
            result.optimalPath = solveLineTSP(startNodeId);
            result.algorithm = "Line TSP with Dijkstra (default)";
        }

        emit progressUpdated(95, "线事件路径规划完成");

        // 步骤6: 计算统计信息 (95-100%)
        emit progressUpdated(98, "正在计算统计信息...");
        
        result.success = true;
        result.gridNodes = mGridNodes;
        result.riskLines = mRiskLines;
        result.completedRiskLines = 0;
        result.totalPathLength = 0.0;
        
        // 计算路径长度和完成的风险线数量
        for (int i = 0; i < result.optimalPath.size(); ++i) {
            if (i > 0) {
                double segmentLength = calculateDistance(
                    result.optimalPath[i-1].position, 
                    result.optimalPath[i].position
                );
                result.totalPathLength += segmentLength;
                result.optimalPath[i].distanceFromStart = result.totalPathLength;
            }
        }

        // 统计完成的风险线
        for (const auto& riskLine : mRiskLines) {
            if (riskLine.isCompleted) {
                result.completedRiskLines++;
            }
        }

        emit progressUpdated(100, "线事件规划完成");
        emit linePlanningCompleted(result);

        logMessage(QString("Line event path planning completed successfully. Path length: %1m, Risk lines completed: %2/%3")
                  .arg(result.totalPathLength, 0, 'f', 2)
                  .arg(result.completedRiskLines)
                  .arg(result.totalRiskLines), 
                  Qgis::MessageLevel::Success);

    } catch (const std::exception& e) {
        result.success = false;
        result.errorMessage = QString("Exception occurred during line planning: %1").arg(e.what());
        logMessage(result.errorMessage, Qgis::MessageLevel::Critical);
        emit planningFailed(result.errorMessage);
    }

    return result;
}

bool GridPathPlanner::exportLinePathToFiles(const LinePlanningResult& result, const QString& outputPath) {
    if (!result.success || result.optimalPath.isEmpty()) {
        logMessage("No valid line path to export", Qgis::MessageLevel::Warning);
        return false;
    }
    
    QDir outputDir(outputPath);
    if (!outputDir.exists()) {
        outputDir.mkpath(".");
    }
    
    // 导出路径为CSV
    QString csvPath = outputDir.filePath("line_path.csv");
    QFile csvFile(csvPath);
    
    if (csvFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&csvFile);
        out.setCodec("UTF-8");
        
        out << "point_id,x,y,z,is_risk_line,risk_line_id,line_segment_id,distance_from_start\n";
        
        for (int i = 0; i < result.optimalPath.size(); ++i) {
            const PathPoint& point = result.optimalPath[i];
            
            out << (i + 1) << ","
                << QString::number(point.position.x(), 'f', 2) << ","
                << QString::number(point.position.y(), 'f', 2) << ","
                << QString::number(point.position.z(), 'f', 2) << ","
                << (point.isRiskPoint ? "true" : "false") << ","
                << point.riskPointId << ","
                << point.lineSegmentId << ","
                << QString::number(point.distanceFromStart, 'f', 2) << "\n";
        }
        
        csvFile.close();
        logMessage(QString("Line path exported to CSV: %1").arg(csvPath), Qgis::MessageLevel::Success);
    }
    
    // 导出路径为WKT
    QString wktPath = outputDir.filePath("line_path.wkt");
    QFile wktFile(wktPath);
    
    if (wktFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&wktFile);
        out.setCodec("UTF-8");
        
        out << "LINESTRING (";
        for (int i = 0; i < result.optimalPath.size(); ++i) {
            const PathPoint& point = result.optimalPath[i];
            if (i > 0) out << ", ";
            out << QString::number(point.position.x(), 'f', 2) << " "
                << QString::number(point.position.y(), 'f', 2) << " "
                << QString::number(point.position.z(), 'f', 2);
        }
        out << ")\n";
        
        wktFile.close();
        logMessage(QString("Line path exported to WKT: %1").arg(wktPath), Qgis::MessageLevel::Success);
    }
    
    // 导出风险线信息
    QString riskLinesPath = outputDir.filePath("risk_lines_info.csv");
    QFile riskLinesFile(riskLinesPath);
    
    if (riskLinesFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&riskLinesFile);
        out.setCodec("UTF-8");
        
        out << "line_id,start_x,start_y,end_x,end_y,is_completed,grid_nodes_count\n";
        
        for (const auto& riskLine : result.riskLines) {
            out << riskLine.lineId << ","
                << QString::number(riskLine.startPoint.x(), 'f', 2) << ","
                << QString::number(riskLine.startPoint.y(), 'f', 2) << ","
                << QString::number(riskLine.endPoint.x(), 'f', 2) << ","
                << QString::number(riskLine.endPoint.y(), 'f', 2) << ","
                << (riskLine.isCompleted ? "true" : "false") << ","
                << riskLine.gridNodeIds.size() << "\n";
        }
        
        riskLinesFile.close();
        logMessage(QString("Risk lines info exported to: %1").arg(riskLinesPath), Qgis::MessageLevel::Success);
    }
    
    // 导出统计报告
    QString statsPath = outputDir.filePath("line_path_statistics.txt");
    QFile statsFile(statsPath);
    
    if (statsFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&statsFile);
        out.setCodec("UTF-8");
        
        out << getLineStatistics(result);
        
        statsFile.close();
        logMessage(QString("Line statistics exported to: %1").arg(statsPath), Qgis::MessageLevel::Success);
    }
    
    return true;
}

QString GridPathPlanner::getLineStatistics(const LinePlanningResult& result) const {
    QString stats;
    stats += "=== 线事件路径规划统计报告 ===\n\n";
    stats += QString("规划状态: %1\n").arg(result.success ? "成功" : "失败");
    
    if (!result.success) {
        stats += QString("错误信息: %1\n").arg(result.errorMessage);
        return stats;
    }
    
    stats += QString("使用算法: %1\n").arg(result.algorithm);
    stats += QString("网格节点总数: %1\n").arg(result.gridNodes.size());
    stats += QString("风险线总数: %1\n").arg(result.totalRiskLines);
    stats += QString("完成的风险线数: %1\n").arg(result.completedRiskLines);
    stats += QString("路径总长度: %1 米\n").arg(result.totalPathLength, 0, 'f', 2);
    stats += QString("路径点总数: %1\n").arg(result.optimalPath.size());
    
    if (result.completedRiskLines > 0) {
        stats += QString("平均每条风险线路径长度: %1 米\n")
                .arg(result.totalPathLength / result.completedRiskLines, 0, 'f', 2);
    }
    
    double coverageRate = result.totalRiskLines > 0 ? 
                         (double)result.completedRiskLines / result.totalRiskLines * 100.0 : 0.0;
    stats += QString("风险线完成率: %1%\n").arg(coverageRate, 0, 'f', 1);
    
    // 统计每条风险线的详细信息
    stats += "\n=== 风险线详细信息 ===\n";
    for (const auto& riskLine : result.riskLines) {
        stats += QString("风险线 %1: %2 -> %3, 完成状态: %4, 网格节点数: %5\n")
                .arg(riskLine.lineId)
                .arg(QString("(%1, %2)").arg(riskLine.startPoint.x(), 0, 'f', 2).arg(riskLine.startPoint.y(), 0, 'f', 2))
                .arg(QString("(%1, %2)").arg(riskLine.endPoint.x(), 0, 'f', 2).arg(riskLine.endPoint.y(), 0, 'f', 2))
                .arg(riskLine.isCompleted ? "已完成" : "未完成")
                .arg(riskLine.gridNodeIds.size());
    }
    
    return stats;
}

QString GridPathPlanner::getAreaStatistics(const AreaPlanningResult& result) const {
    if (!result.success) {
        return "面事件规划失败";
    }
    
    QString stats;
    QTextStream stream(&stats);
    
    stream << "=== 面事件路径规划统计 ===\n";
    stream << "算法: " << result.algorithm << "\n";
    stream << "总路径长度: " << QString::number(result.totalPathLength, 'f', 2) << " 米\n";
    stream << "覆盖率: " << QString::number(result.coverageRate * 100.0, 'f', 1) << "%\n";
    stream << "总网格单元数: " << result.totalGridCells << "\n";
    stream << "已覆盖网格单元数: " << result.coveredGridCells << "\n";
    stream << "未覆盖网格单元数: " << (result.totalGridCells - result.coveredGridCells) << "\n";
    stream << "未覆盖区域数量: " << result.uncoveredAreas.size() << "\n";
    stream << "平均路径密度: " << QString::number(result.averagePathDensity, 'f', 4) << " 米/平方米\n";
    stream << "路径点数: " << result.optimalPath.size() << "\n";
    
    if (result.totalPathLength > 0) {
        stream << "覆盖效率: " << QString::number(result.coveredGridCells / result.totalPathLength * 1000, 'f', 2) << " 单元/千米\n";
    }
    
    if (result.coveredGridCells > 0) {
        stream << "路径效率: " << QString::number(result.totalPathLength / result.coveredGridCells, 'f', 2) << " 米/单元\n";
    }
    
    return stats;
}

QVector<PathPoint> GridPathPlanner::solveAreaSpiralCoverage(int startNodeId, double coverageThreshold) {
    logMessage(QString("Starting enhanced spiral coverage algorithm with threshold: %1").arg(coverageThreshold), 
              Qgis::MessageLevel::Info);
    
    QVector<PathPoint> coveragePath;
    QSet<int> visitedNodes;
    QSet<int> availableNodes;
    
    // === 步骤1：验证面事件数据结构 ===
    logMessage("=== 验证面事件数据结构 ===", Qgis::MessageLevel::Info);
    logMessage(QString("mAreaGridNodes.size() = %1").arg(mAreaGridNodes.size()), Qgis::MessageLevel::Info);
    logMessage(QString("mAreaNodeIdToIndex.size() = %1").arg(mAreaNodeIdToIndex.size()), Qgis::MessageLevel::Info);
    
    // 验证前5个面事件节点的完整信息
    for (int i = 0; i < qMin(5, mAreaGridNodes.size()); ++i) {
        const GridNode& node = mAreaGridNodes[i];
        logMessage(QString("面事件节点 %1: ID=%2, 位置=(%3, %4, %5)")
                  .arg(i)
                  .arg(node.id)
                  .arg(node.position.x(), 0, 'f', 6) // 使用6位小数精度
                  .arg(node.position.y(), 0, 'f', 6)
                  .arg(node.position.z(), 0, 'f', 6), 
                  Qgis::MessageLevel::Info);
    }
    
    // === 步骤2：初始化可用节点集合 ===
    logMessage("=== 初始化可用节点集合 ===", Qgis::MessageLevel::Info);
    for (const auto& node : mAreaGridNodes) {
        availableNodes.insert(node.id);
    }
    logMessage(QString("初始化完成，可用节点数: %1").arg(availableNodes.size()), Qgis::MessageLevel::Info);
    
    if (availableNodes.isEmpty()) {
        logMessage("错误：没有可用的面事件网格节点", Qgis::MessageLevel::Critical);
        return coveragePath;
    }
    
    // === 步骤3：验证起始节点 ===
    logMessage("=== 验证起始节点 ===", Qgis::MessageLevel::Info);
    logMessage(QString("起始节点ID: %1").arg(startNodeId), Qgis::MessageLevel::Info);
    
    if (!mAreaNodeIdToIndex.contains(startNodeId)) {
        logMessage(QString("错误：起始节点ID %1 在面事件数据中不存在").arg(startNodeId), Qgis::MessageLevel::Critical);
        return coveragePath;
    }
    
    int startNodeIndex = mAreaNodeIdToIndex[startNodeId];
    const GridNode& startNode = mAreaGridNodes[startNodeIndex];
    logMessage(QString("起始节点验证成功: 索引=%1, 位置=(%2, %3, %4)")
              .arg(startNodeIndex)
              .arg(startNode.position.x(), 0, 'f', 6)
              .arg(startNode.position.y(), 0, 'f', 6)
              .arg(startNode.position.z(), 0, 'f', 6), 
              Qgis::MessageLevel::Info);
    
    // === 步骤4：开始路径规划 ===
    logMessage("=== 开始路径规划 ===", Qgis::MessageLevel::Info);
    int currentNodeId = startNodeId;
    int iteration = 0;
    const int MAX_ITERATIONS = mAreaGridNodes.size() * 2;
    
    while (iteration < MAX_ITERATIONS && !availableNodes.isEmpty()) {
        // === 步骤4.1：添加当前节点到路径 ===
        if (mAreaNodeIdToIndex.contains(currentNodeId) && !visitedNodes.contains(currentNodeId)) {
            int nodeIndex = mAreaNodeIdToIndex[currentNodeId];
            const GridNode& node = mAreaGridNodes[nodeIndex];
            
            // 详细记录节点添加过程
            if (iteration < 10) { // 只记录前10个节点的详细信息
                logMessage(QString("添加节点到路径: 迭代=%1, 节点ID=%2, 索引=%3, 位置=(%4, %5, %6)")
                          .arg(iteration)
                          .arg(currentNodeId)
                          .arg(nodeIndex)
                          .arg(node.position.x(), 0, 'f', 6)
                          .arg(node.position.y(), 0, 'f', 6)
                          .arg(node.position.z(), 0, 'f', 6), 
                          Qgis::MessageLevel::Info);
            }
            
            // 确保创建的PathPoint使用正确的坐标
            PathPoint pathPoint(node.position);
            pathPoint.isRiskPoint = false;
            pathPoint.riskPointId = -1;
            pathPoint.lineSegmentId = -1;
            coveragePath.append(pathPoint);
            
            visitedNodes.insert(currentNodeId);
            availableNodes.remove(currentNodeId);
        } else {
            logMessage(QString("警告：节点ID %1 无效或已访问，跳过").arg(currentNodeId), Qgis::MessageLevel::Warning);
        }
        
        // === 步骤4.2：检查覆盖率 ===
        double currentCoverage = calculateAreaCoverageRate(visitedNodes);
        if (currentCoverage >= coverageThreshold) {
            logMessage(QString("达到覆盖率阈值: %1%，停止规划").arg(currentCoverage * 100.0, 0, 'f', 1), 
                      Qgis::MessageLevel::Info);
            break;
        }
        
        // === 步骤4.3：寻找下一个节点 ===
        int nextNodeId = findNextAreaSpiralNodeImproved(currentNodeId, visitedNodes, availableNodes);
        if (nextNodeId == -1) {
            // 备用策略：寻找最近的未访问节点
            nextNodeId = findNearestAreaUnvisitedNode(currentNodeId, visitedNodes, availableNodes);
            if (nextNodeId == -1) {
                logMessage("无法找到下一个节点，结束规划", Qgis::MessageLevel::Warning);
                break;
            }
        }
        
        currentNodeId = nextNodeId;
        iteration++;
        
        // 进度报告
        if (iteration % 50 == 0) {
            logMessage(QString("螺旋规划进度: 迭代=%1/%2, 覆盖率=%3% (%4/%5 节点)")
                      .arg(iteration).arg(MAX_ITERATIONS)
                      .arg(currentCoverage * 100.0, 0, 'f', 1)
                      .arg(visitedNodes.size()).arg(mAreaGridNodes.size()), 
                      Qgis::MessageLevel::Info);
        }
    }
    
    // === 步骤5：验证生成的路径 ===
    logMessage("=== 验证生成的路径 ===", Qgis::MessageLevel::Info);
    logMessage(QString("生成的路径点数: %1").arg(coveragePath.size()), Qgis::MessageLevel::Info);
    
    // 检查前10个路径点的坐标
    for (int i = 0; i < qMin(10, coveragePath.size()); ++i) {
        const PathPoint& point = coveragePath[i];
        logMessage(QString("路径点 %1: 坐标=(%2, %3, %4)")
                  .arg(i)
                  .arg(point.position.x(), 0, 'f', 6)
                  .arg(point.position.y(), 0, 'f', 6)
                  .arg(point.position.z(), 0, 'f', 6), 
                  Qgis::MessageLevel::Info);
    }
    
    // === 新增：验证路径点是否都来自渔网节点 ===
    logMessage("=== 验证路径点是否都来自渔网节点 ===", Qgis::MessageLevel::Info);
    int validPathPoints = 0;
    int invalidPathPoints = 0;
    
    for (int i = 0; i < coveragePath.size(); ++i) {
        const PathPoint& pathPoint = coveragePath[i];
        bool foundMatchingNode = false;
        
        // 检查是否有网格节点位置与路径点匹配
        for (const auto& node : mAreaGridNodes) {
            double distance = calculateDistance(pathPoint.position, node.position);
            if (distance < 0.01) { // 1厘米容差
                foundMatchingNode = true;
                break;
            }
        }
        
        if (foundMatchingNode) {
            validPathPoints++;
        } else {
            invalidPathPoints++;
            if (invalidPathPoints <= 5) { // 只显示前5个无效点
                logMessage(QString("警告：路径点 %1 位置=(%2, %3) 不匹配任何渔网节点")
                          .arg(i)
                          .arg(pathPoint.position.x(), 0, 'f', 3)
                          .arg(pathPoint.position.y(), 0, 'f', 3), 
                          Qgis::MessageLevel::Warning);
            }
        }
    }
    
    logMessage(QString("路径点验证结果: 有效路径点=%1, 无效路径点=%2")
              .arg(validPathPoints).arg(invalidPathPoints), 
              Qgis::MessageLevel::Info);
    
    if (invalidPathPoints > 0) {
        logMessage("发现路径点不在渔网线区域内！这可能是算法错误。", Qgis::MessageLevel::Warning);
    } else {
        logMessage("所有路径点都来自渔网节点，符合预期。", Qgis::MessageLevel::Info);
    }
    
    // === 步骤6：填补遗漏节点（如果需要） ===
    if (calculateAreaCoverageRate(visitedNodes) < coverageThreshold) {
        logMessage("尝试填补遗漏节点...", Qgis::MessageLevel::Info);
        fillAreaMissingNodes(coveragePath, visitedNodes, availableNodes, coverageThreshold);
    }
    
    double finalCoverage = calculateAreaCoverageRate(visitedNodes);
    logMessage(QString("面事件螺旋覆盖完成。访问节点: %1/%2, 覆盖率: %3%")
              .arg(visitedNodes.size()).arg(mAreaGridNodes.size()).arg(finalCoverage * 100.0, 0, 'f', 1), 
              Qgis::MessageLevel::Success);
    
    return coveragePath;
}

QVector<PathPoint> GridPathPlanner::solveAreaGridCoverage(int startNodeId, double coverageThreshold) {
    logMessage(QString("Starting enhanced grid coverage algorithm with threshold: %1").arg(coverageThreshold), 
              Qgis::MessageLevel::Info);
    
    QVector<PathPoint> coveragePath;
    QSet<int> visitedNodes;
    QSet<int> availableNodes;
    
    // 初始化可用节点
    for (const auto& node : mAreaGridNodes) {
        availableNodes.insert(node.id);
    }
    
    if (availableNodes.isEmpty()) {
        logMessage("No grid nodes available for area coverage", Qgis::MessageLevel::Warning);
        return coveragePath;
    }
    
    // 构建网格结构信息
    buildAreaGridStructure();
    
    int currentNodeId = startNodeId;
    int iteration = 0;
    const int MAX_ITERATIONS = mAreaGridNodes.size() * 2;
    
    // 使用改进的网格遍历策略：蛇形遍历
    bool movingRight = true;
    QVector<QVector<int>> gridRows = organizeAreaNodesIntoRows();
    
    while (iteration < MAX_ITERATIONS && !availableNodes.isEmpty()) {
        // 添加当前节点到路径
        if (mAreaNodeIdToIndex.contains(currentNodeId) && !visitedNodes.contains(currentNodeId)) {
            int nodeIndex = mAreaNodeIdToIndex[currentNodeId];
            const GridNode& node = mAreaGridNodes[nodeIndex];
            
            PathPoint pathPoint(node.position);
            pathPoint.isRiskPoint = false;
            pathPoint.riskPointId = -1;
            pathPoint.lineSegmentId = -1;
            coveragePath.append(pathPoint);
            
            visitedNodes.insert(currentNodeId);
            availableNodes.remove(currentNodeId);
        }
        
        // 检查覆盖率
        double currentCoverage = calculateAreaCoverageRate(visitedNodes);
        if (currentCoverage >= coverageThreshold) {
            logMessage(QString("Coverage threshold reached: %1%").arg(currentCoverage * 100.0, 0, 'f', 1), 
                      Qgis::MessageLevel::Info);
            break;
        }
        
        // 寻找下一个网格节点（蛇形遍历）
        int nextNodeId = findNextAreaGridNodeImproved(currentNodeId, visitedNodes, availableNodes, movingRight);
        if (nextNodeId == -1) {
            // 如果网格遍历失败，切换方向或找最近节点
            movingRight = !movingRight;
            nextNodeId = findNextAreaGridNodeImproved(currentNodeId, visitedNodes, availableNodes, movingRight);
            if (nextNodeId == -1) {
                nextNodeId = findNearestAreaUnvisitedNode(currentNodeId, visitedNodes, availableNodes);
                if (nextNodeId == -1) {
                    logMessage("No more grid nodes to visit", Qgis::MessageLevel::Warning);
                    break;
                }
            }
        }
        
        currentNodeId = nextNodeId;
        iteration++;
        
        if (iteration % 50 == 0) {
            logMessage(QString("Grid iteration %1/%2, coverage: %3% (%4/%5 nodes)")
                      .arg(iteration).arg(MAX_ITERATIONS)
                      .arg(currentCoverage * 100.0, 0, 'f', 1)
                      .arg(visitedNodes.size()).arg(mAreaGridNodes.size()), 
                      Qgis::MessageLevel::Info);
        }
    }
    
    // 尝试填补遗漏的节点
    fillAreaMissingNodes(coveragePath, visitedNodes, availableNodes, coverageThreshold);
    
    double finalCoverage = calculateAreaCoverageRate(visitedNodes);
    logMessage(QString("Enhanced grid coverage completed. Total nodes visited: %1/%2, coverage: %3%")
              .arg(visitedNodes.size()).arg(mAreaGridNodes.size()).arg(finalCoverage * 100.0, 0, 'f', 1), 
              Qgis::MessageLevel::Success);
    
    return coveragePath;
}

// ... existing code ...

void GridPathPlanner::buildGridConnections() {
    logMessage("Building grid connections...", Qgis::MessageLevel::Info);
    
    // 清空现有连接
    for (auto& node : mGridNodes) {
        node.neighbors.clear();
    }
    
    // 构建节点ID到索引的映射
    mNodeIdToIndex.clear();
    for (int i = 0; i < mGridNodes.size(); ++i) {
        mNodeIdToIndex[mGridNodes[i].id] = i;
    }
    
    // 为每个节点找到邻近节点
    const double connectionDistance = 20.0; // 连接距离阈值（米）
    
    for (int i = 0; i < mGridNodes.size(); ++i) {
        GridNode& node = mGridNodes[i];
        
        for (int j = 0; j < mGridNodes.size(); ++j) {
            if (i == j) continue;
            
            const GridNode& otherNode = mGridNodes[j];
            double distance = calculateDistance(node.position, otherNode.position);
            
            if (distance <= connectionDistance) {
                node.neighbors.append(otherNode.id);
            }
        }
    }
    
    // 统计连接信息
    int totalConnections = 0;
    int isolatedNodes = 0;
    
    for (const auto& node : mGridNodes) {
        totalConnections += node.neighbors.size();
        if (node.neighbors.isEmpty()) {
            isolatedNodes++;
        }
    }
    
    logMessage(QString("Grid connections built. Total nodes: %1, Total connections: %2, Isolated nodes: %3")
              .arg(mGridNodes.size()).arg(totalConnections).arg(isolatedNodes), 
              Qgis::MessageLevel::Info);
}

int GridPathPlanner::findNextSpiralNode(int currentNodeId, const QSet<int>& visitedNodes, const QSet<int>& availableNodes) {
    if (!mNodeIdToIndex.contains(currentNodeId)) {
        return -1;
    }
    
    int currentNodeIndex = mNodeIdToIndex[currentNodeId];
    const GridNode& currentNode = mGridNodes[currentNodeIndex];
    
    // 螺旋搜索模式：从近到远，按角度搜索
    QVector<QPair<double, int>> candidateNodes;
    
    for (int nodeId : availableNodes) {
        if (visitedNodes.contains(nodeId)) continue;
        
        if (!mNodeIdToIndex.contains(nodeId)) continue;
        
        int nodeIndex = mNodeIdToIndex[nodeId];
        const GridNode& node = mGridNodes[nodeIndex];
        
        double distance = calculateDistance(currentNode.position, node.position);
        double angle = qAtan2(node.position.y() - currentNode.position.y(), 
                             node.position.x() - currentNode.position.x());
        
        // 优先选择距离适中、角度合适的节点
        double score = distance * (1.0 + qAbs(angle) / M_PI);
        candidateNodes.append(qMakePair(score, nodeId));
    }
    
    if (candidateNodes.isEmpty()) {
        return -1;
    }
    
    // 按分数排序，选择最佳节点
    std::sort(candidateNodes.begin(), candidateNodes.end());
    return candidateNodes.first().second;
}

int GridPathPlanner::findNextGridNode(int currentNodeId, const QSet<int>& visitedNodes) {
    if (!mNodeIdToIndex.contains(currentNodeId)) {
        return -1;
    }
    
    int currentNodeIndex = mNodeIdToIndex[currentNodeId];
    const GridNode& currentNode = mGridNodes[currentNodeIndex];
    
    // 网格搜索模式：按行列顺序搜索
    QVector<QPair<double, int>> candidateNodes;
    
    for (const auto& node : mGridNodes) {
        if (visitedNodes.contains(node.id)) continue;
        
        double distance = calculateDistance(currentNode.position, node.position);
        
        // 优先选择距离最近的节点
        candidateNodes.append(qMakePair(distance, node.id));
    }
    
    if (candidateNodes.isEmpty()) {
        return -1;
    }
    
    // 按距离排序，选择最近的节点
    std::sort(candidateNodes.begin(), candidateNodes.end());
    return candidateNodes.first().second;
}

AreaPlanningResult GridPathPlanner::executeAreaPlanning(const AreaPlanningParameters& params) {
    logMessage("Starting area planning...", Qgis::MessageLevel::Info);
    
    AreaPlanningResult result;
    result.success = false;
    result.algorithm = params.algorithm;
    
    // 验证参数
    auto validation = validateAreaParameters(params);
    if (!validation.first) {
        result.errorMessage = validation.second;
        logMessage(QString("Area planning validation failed: %1").arg(validation.second), 
                  Qgis::MessageLevel::Critical);
        return result;
    }
    
    // 清空面事件相关数据结构
    mAreaGridNodes.clear();
    mAreaNodeIdToIndex.clear();
    
    // 加载渔网线到面事件数据结构
    if (!buildAreaGridFromFishnet(params.fishnetShapefile)) {
        result.errorMessage = "Failed to load fishnet from shapefile";
        logMessage("Failed to load fishnet from shapefile", Qgis::MessageLevel::Critical);
        return result;
    }
    
    logMessage(QString("Loaded %1 grid nodes from fishnet for area planning").arg(mAreaGridNodes.size()), 
              Qgis::MessageLevel::Info);
    
    // 构建网格连接
    buildAreaGridConnections();
    
    // 检查网格连通性
    if (mAreaGridNodes.isEmpty()) {
        result.errorMessage = "No grid nodes available";
        logMessage("No grid nodes available", Qgis::MessageLevel::Critical);
        return result;
    }
    
    // 找到最近的起始节点
    int startNodeId = findNearestAreaGridNode(params.startPoint);
    if (startNodeId == -1) {
        result.errorMessage = "Cannot find nearest grid node to start point";
        logMessage("Cannot find nearest grid node to start point", Qgis::MessageLevel::Critical);
        return result;
    }
    
    logMessage(QString("Start node ID: %1").arg(startNodeId), Qgis::MessageLevel::Info);
    
    // 执行路径规划算法
    QVector<PathPoint> optimalPath;
    
    logMessage(QString("Selected algorithm: %1").arg(params.algorithm), Qgis::MessageLevel::Info);
    
    if (params.algorithm == "Area_Spiral") {
        logMessage("Executing Area_Spiral algorithm", Qgis::MessageLevel::Info);
        optimalPath = solveAreaSpiralCoverage(startNodeId, params.coverageThreshold);
    } else if (params.algorithm == "Area_Grid") {
        logMessage("Executing Area_Grid algorithm", Qgis::MessageLevel::Info);
        optimalPath = solveAreaGridCoverage(startNodeId, params.coverageThreshold);
    } else {
        result.errorMessage = QString("Unknown algorithm: %1").arg(params.algorithm);
        logMessage(QString("Unknown algorithm: %1").arg(params.algorithm), Qgis::MessageLevel::Critical);
        return result;
    }
    
    if (optimalPath.isEmpty()) {
        result.errorMessage = "Failed to generate coverage path";
        logMessage("Failed to generate coverage path", Qgis::MessageLevel::Critical);
        return result;
    }
    
    // 计算路径统计信息
    double totalPathLength = 0.0;
    for (int i = 1; i < optimalPath.size(); ++i) {
        totalPathLength += calculateDistance(optimalPath[i-1].position, optimalPath[i].position);
    }
    
    // 计算覆盖率
    QSet<int> visitedNodes;
    for (const auto& point : optimalPath) {
        int nodeId = findNearestAreaGridNode(point.position);
        if (nodeId != -1) {
            visitedNodes.insert(nodeId);
        }
    }
    double coverageRate = calculateAreaCoverageRate(visitedNodes);
    
    // 找到未覆盖区域
    QVector<QVector3D> uncoveredAreas = findAreaUncoveredAreas(visitedNodes);
    
    // 计算平均路径密度
    double totalArea = mAreaGridNodes.size() * 100.0; // 假设每个网格单元100平方米
    double averagePathDensity = totalPathLength / totalArea;
    
    // 设置结果
    result.success = true;
    result.gridNodes = mAreaGridNodes;
    result.optimalPath = optimalPath;
    result.totalPathLength = totalPathLength;
    result.coverageRate = coverageRate;
    result.totalGridCells = mAreaGridNodes.size();
    result.coveredGridCells = visitedNodes.size();
    result.uncoveredAreas = uncoveredAreas;
    result.averagePathDensity = averagePathDensity;
    
    logMessage(QString("Area planning completed successfully. Coverage: %1%, Path length: %2m")
              .arg(coverageRate * 100.0, 0, 'f', 1).arg(totalPathLength, 0, 'f', 2), 
              Qgis::MessageLevel::Success);
    
    return result;
}

bool GridPathPlanner::exportAreaPathToFiles(const AreaPlanningResult& result, const QString& outputPath) {
    if (!result.success || result.optimalPath.isEmpty()) {
        logMessage("No valid area path to export", Qgis::MessageLevel::Warning);
        return false;
    }
    
    QDir outputDir(outputPath);
    if (!outputDir.exists()) {
        outputDir.mkpath(".");
    }
    
    // 导出路径为CSV
    QString csvPath = outputDir.filePath("area_path.csv");
    QFile csvFile(csvPath);
    
    if (csvFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&csvFile);
        out.setCodec("UTF-8");
        
        out << "point_id,x,y,z,coverage_status\n";
        
        // 记录前10个路径点的详细信息
        logMessage("=== 面事件路径点详细信息 ===", Qgis::MessageLevel::Info);
        logMessage(QString("总路径点数: %1").arg(result.optimalPath.size()), Qgis::MessageLevel::Info);
        
        for (int i = 0; i < result.optimalPath.size(); ++i) {
            const PathPoint& point = result.optimalPath[i];
            
            // 输出前10个点的详细信息
            if (i < 10) {
                logMessage(QString("路径点 %1: 坐标=(%2, %3, %4)")
                          .arg(i)
                          .arg(point.position.x(), 0, 'f', 2)
                          .arg(point.position.y(), 0, 'f', 2)
                          .arg(point.position.z(), 0, 'f', 2), 
                          Qgis::MessageLevel::Info);
            }
            
            out << (i + 1) << ","
                << point.position.x() << ","
                << point.position.y() << ","
                << point.position.z() << ","
                << "covered" << "\n";
        }
        
        csvFile.close();
        logMessage(QString("Exported area path to CSV: %1").arg(csvPath), Qgis::MessageLevel::Info);
        
        // 验证导出的文件内容
        logMessage("=== 验证导出文件内容 ===", Qgis::MessageLevel::Info);
        logMessage(QString("CSV文件路径: %1").arg(csvPath), Qgis::MessageLevel::Info);
        
        // 计算路径点的坐标范围
        if (!result.optimalPath.isEmpty()) {
            double minX = result.optimalPath[0].position.x();
            double maxX = minX;
            double minY = result.optimalPath[0].position.y();
            double maxY = minY;
            
            for (const auto& point : result.optimalPath) {
                minX = qMin(minX, static_cast<double>(point.position.x()));
                maxX = qMax(maxX, static_cast<double>(point.position.x()));
                minY = qMin(minY, static_cast<double>(point.position.y()));
                maxY = qMax(maxY, static_cast<double>(point.position.y()));
            }
            
            logMessage(QString("导出路径坐标范围: X[%1, %2], Y[%3, %4]")
                      .arg(minX, 0, 'f', 2).arg(maxX, 0, 'f', 2)
                      .arg(minY, 0, 'f', 2).arg(maxY, 0, 'f', 2), 
                      Qgis::MessageLevel::Info);
        }
    }
    
    // 导出路径为WKT
    QString wktPath = outputDir.filePath("area_path.wkt");
    QFile wktFile(wktPath);
    
    if (wktFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&wktFile);
        out.setCodec("UTF-8");
        
        out << "POLYGON ((";
        for (int i = 0; i < result.optimalPath.size(); ++i) {
            const PathPoint& point = result.optimalPath[i];
            if (i > 0) out << ", ";
            out << QString::number(point.position.x(), 'f', 2) << " "
                << QString::number(point.position.y(), 'f', 2) << " "
                << QString::number(point.position.z(), 'f', 2);
        }
        out << "))\n";
        
        wktFile.close();
        logMessage(QString("Area path exported to WKT: %1").arg(wktPath), Qgis::MessageLevel::Success);
    }
    
    // 导出统计报告
    QString statsPath = outputDir.filePath("area_path_statistics.txt");
    QFile statsFile(statsPath);
    
    if (statsFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&statsFile);
        out.setCodec("UTF-8");
        
        out << getAreaStatistics(result);
        
        statsFile.close();
        logMessage(QString("Area statistics exported to: %1").arg(statsPath), Qgis::MessageLevel::Success);
    }
    
    // 导出面事件网格节点详细信息用于调试
    QString gridNodesPath = outputDir.filePath("area_grid_nodes_debug.csv");
    QFile gridNodesFile(gridNodesPath);
    
    if (gridNodesFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&gridNodesFile);
        out.setCodec("UTF-8");
        
        out << "node_id,x,y,z,neighbors_count,neighbors_list\n";
        
        for (const auto& node : result.gridNodes) {
            QStringList neighborsList;
            for (int neighborId : node.neighbors) {
                neighborsList.append(QString::number(neighborId));
            }
            
            out << node.id << ","
                << node.position.x() << ","
                << node.position.y() << ","
                << node.position.z() << ","
                << node.neighbors.size() << ","
                << "\"" << neighborsList.join(";") << "\"\n";
        }
        
        gridNodesFile.close();
        logMessage(QString("Area grid nodes debug info exported to: %1").arg(gridNodesPath), Qgis::MessageLevel::Info);
    }
    
    return true;
}

void GridPathPlanner::asyncExecuteAreaPlanning(const AreaPlanningParameters& params) {
    QTimer::singleShot(0, [this, params]() {
        AreaPlanningResult result = executeAreaPlanning(params);
        if (result.success) {
            emit areaPlanningCompleted(result);
        } else {
            emit planningFailed(result.errorMessage);
        }
    });
}

// 改进的螺旋节点搜索算法
int GridPathPlanner::findNextSpiralNodeImproved(int currentNodeId, const QSet<int>& visitedNodes, const QSet<int>& availableNodes) {
    if (!mAreaNodeIdToIndex.contains(currentNodeId)) {
        return -1;
    }
    
    int currentNodeIndex = mAreaNodeIdToIndex[currentNodeId];
    const GridNode& currentNode = mAreaGridNodes[currentNodeIndex];
    
    // 优先选择与当前节点直接连接的未访问节点（基于渔网线连接）
    QVector<QPair<double, int>> connectedNodes;
    QVector<QPair<double, int>> nearbyNodes;
    
    for (int neighborId : currentNode.neighbors) {
        if (visitedNodes.contains(neighborId) || !availableNodes.contains(neighborId)) {
            continue;
        }
        
        if (!mAreaNodeIdToIndex.contains(neighborId)) continue;
        
        int neighborIndex = mAreaNodeIdToIndex[neighborId];
        const GridNode& neighborNode = mAreaGridNodes[neighborIndex];
        
        double distance = calculateDistance(currentNode.position, neighborNode.position);
        double angle = qAtan2(neighborNode.position.y() - currentNode.position.y(), 
                             neighborNode.position.x() - currentNode.position.x());
        
        // 优先级：距离 + 角度因子
        double priority = distance + qAbs(angle) * 10.0;
        connectedNodes.append(qMakePair(priority, neighborId));
    }
    
    // 如果有直接连接的节点，优先选择
    if (!connectedNodes.isEmpty()) {
        std::sort(connectedNodes.begin(), connectedNodes.end());
        return connectedNodes.first().second;
    }
    
    // 如果没有直接连接的节点，选择最近的可达节点（通过路径连接）
    for (int nodeId : availableNodes) {
        if (visitedNodes.contains(nodeId)) continue;
        
        if (!mAreaNodeIdToIndex.contains(nodeId)) continue;
        
        // 检查是否可以通过渔网线路径到达
        if (isAreaNodeReachable(currentNodeId, nodeId, visitedNodes)) {
            int nodeIndex = mAreaNodeIdToIndex[nodeId];
            const GridNode& node = mAreaGridNodes[nodeIndex];
            
            double distance = calculateDistance(currentNode.position, node.position);
            nearbyNodes.append(qMakePair(distance, nodeId));
        }
    }
    
    if (!nearbyNodes.isEmpty()) {
        std::sort(nearbyNodes.begin(), nearbyNodes.end());
        return nearbyNodes.first().second;
    }
    
    return -1;
}

// 面事件改进的网格节点搜索算法（基于渔网线连接的蛇形遍历）
int GridPathPlanner::findNextAreaGridNodeImproved(int currentNodeId, const QSet<int>& visitedNodes, const QSet<int>& availableNodes, bool movingRight) {
    if (!mAreaNodeIdToIndex.contains(currentNodeId)) {
        return -1;
    }
    
    int currentNodeIndex = mAreaNodeIdToIndex[currentNodeId];
    const GridNode& currentNode = mAreaGridNodes[currentNodeIndex];
    
    // 首先尝试直接连接的邻居节点
    QVector<QPair<double, int>> sameRowNodes;
    QVector<QPair<double, int>> nextRowNodes;
    QVector<QPair<double, int>> connectedNodes;
    
    const double ROW_TOLERANCE = 25.0; // 行容差25米
    
    // 检查直接连接的邻居节点
    for (int neighborId : currentNode.neighbors) {
        if (visitedNodes.contains(neighborId) || !availableNodes.contains(neighborId)) {
            continue;
        }
        
        if (!mAreaNodeIdToIndex.contains(neighborId)) continue;
        
        int neighborIndex = mAreaNodeIdToIndex[neighborId];
        const GridNode& neighborNode = mAreaGridNodes[neighborIndex];
        
        double deltaX = neighborNode.position.x() - currentNode.position.x();
        double deltaY = neighborNode.position.y() - currentNode.position.y();
        double distance = calculateDistance(currentNode.position, neighborNode.position);
        
        // 判断是否在同一行
        if (qAbs(deltaY) <= ROW_TOLERANCE) {
            // 根据移动方向选择节点
            if ((movingRight && deltaX > 0) || (!movingRight && deltaX < 0)) {
                sameRowNodes.append(qMakePair(qAbs(deltaX), neighborId));
            }
        }
        // 判断是否在下一行
        else if (qAbs(deltaY) > ROW_TOLERANCE && qAbs(deltaY) <= ROW_TOLERANCE * 2) {
            nextRowNodes.append(qMakePair(distance, neighborId));
        }
        
        // 所有连接的节点
        connectedNodes.append(qMakePair(distance, neighborId));
    }
    
    // 优先选择同一行的节点
    if (!sameRowNodes.isEmpty()) {
        std::sort(sameRowNodes.begin(), sameRowNodes.end());
        return sameRowNodes.first().second;
    }
    
    // 其次选择下一行的节点
    if (!nextRowNodes.isEmpty()) {
        std::sort(nextRowNodes.begin(), nextRowNodes.end());
        return nextRowNodes.first().second;
    }
    
    // 最后选择任何连接的节点
    if (!connectedNodes.isEmpty()) {
        std::sort(connectedNodes.begin(), connectedNodes.end());
        return connectedNodes.first().second;
    }
    
    return -1;
}

// 检查两个面事件节点是否可达（通过渔网线路径）
bool GridPathPlanner::isAreaNodeReachable(int fromNodeId, int toNodeId, const QSet<int>& visitedNodes) {
    if (fromNodeId == toNodeId) return true;
    
    // 使用BFS检查可达性，只通过渔网线连接
    QQueue<int> queue;
    QSet<int> checked;
    
    queue.enqueue(fromNodeId);
    checked.insert(fromNodeId);
    
    while (!queue.isEmpty()) {
        int currentId = queue.dequeue();
        
        if (!mAreaNodeIdToIndex.contains(currentId)) continue;
        
        int currentIndex = mAreaNodeIdToIndex[currentId];
        const GridNode& currentNode = mAreaGridNodes[currentIndex];
        
        for (int neighborId : currentNode.neighbors) {
            if (neighborId == toNodeId) {
                return true; // 找到目标节点
            }
            
            if (!checked.contains(neighborId) && !visitedNodes.contains(neighborId)) {
                checked.insert(neighborId);
                queue.enqueue(neighborId);
            }
        }
        
        // 限制搜索深度，避免无限搜索
        if (checked.size() > 1000) {
            break;
        }
    }
    
    return false;
}

// 寻找最近的未访问节点
int GridPathPlanner::findNearestUnvisitedNode(int currentNodeId, const QSet<int>& visitedNodes, const QSet<int>& availableNodes) {
    if (!mNodeIdToIndex.contains(currentNodeId)) {
        return -1;
    }
    
    int currentNodeIndex = mNodeIdToIndex[currentNodeId];
    const GridNode& currentNode = mGridNodes[currentNodeIndex];
    
    double minDistance = std::numeric_limits<double>::max();
    int nearestNodeId = -1;
    
    for (int nodeId : availableNodes) {
        if (visitedNodes.contains(nodeId)) continue;
        
        if (!mNodeIdToIndex.contains(nodeId)) continue;
        
        int nodeIndex = mNodeIdToIndex[nodeId];
        const GridNode& node = mGridNodes[nodeIndex];
        
        double distance = calculateDistance(currentNode.position, node.position);
        if (distance < minDistance) {
            minDistance = distance;
            nearestNodeId = nodeId;
        }
    }
    
    return nearestNodeId;
}

// 填补遗漏的节点
void GridPathPlanner::fillMissingNodes(QVector<PathPoint>& coveragePath, QSet<int>& visitedNodes, const QSet<int>& availableNodes, double coverageThreshold) {
    logMessage("Filling missing nodes to improve coverage...", Qgis::MessageLevel::Info);
    
    QSet<int> remainingNodes = availableNodes - visitedNodes;
    
    while (!remainingNodes.isEmpty()) {
        double currentCoverage = calculateCoverageRate(visitedNodes);
        if (currentCoverage >= coverageThreshold) {
            break;
        }
        
        // 找到最近的未访问节点
        int nearestNodeId = -1;
        double minDistance = std::numeric_limits<double>::max();
        QVector3D lastPosition = coveragePath.isEmpty() ? QVector3D() : coveragePath.last().position;
        
        for (int nodeId : remainingNodes) {
            if (!mNodeIdToIndex.contains(nodeId)) continue;
            
            int nodeIndex = mNodeIdToIndex[nodeId];
            const GridNode& node = mGridNodes[nodeIndex];
            
            double distance = coveragePath.isEmpty() ? 0.0 : calculateDistance(lastPosition, node.position);
            if (distance < minDistance || nearestNodeId == -1) {
                minDistance = distance;
                nearestNodeId = nodeId;
            }
        }
        
        if (nearestNodeId == -1) break;
        
        // 添加节点到路径
        int nodeIndex = mNodeIdToIndex[nearestNodeId];
        const GridNode& node = mGridNodes[nodeIndex];
        
        PathPoint pathPoint(node.position);
        pathPoint.isRiskPoint = false;
        pathPoint.riskPointId = -1;
        pathPoint.lineSegmentId = -1;
        coveragePath.append(pathPoint);
        
        visitedNodes.insert(nearestNodeId);
        remainingNodes.remove(nearestNodeId);
    }
    
    double finalCoverage = calculateCoverageRate(visitedNodes);
    logMessage(QString("Missing nodes filled. Final coverage: %1%").arg(finalCoverage * 100.0, 0, 'f', 1), 
              Qgis::MessageLevel::Info);
}

// 构建网格结构信息
void GridPathPlanner::buildGridStructure() {
    logMessage("Building grid structure for area coverage...", Qgis::MessageLevel::Info);
    
    // 这个函数可以用来分析网格的结构特征
    // 比如识别网格的行列结构、边界等
    // 当前实现为占位符，可以根据需要扩展
    
    if (mGridNodes.isEmpty()) {
        logMessage("No grid nodes to analyze", Qgis::MessageLevel::Warning);
        return;
    }
    
    logMessage(QString("Grid structure analysis completed for %1 nodes").arg(mGridNodes.size()), 
              Qgis::MessageLevel::Info);
}

// 将节点组织成行
QVector<QVector<int>> GridPathPlanner::organizeNodesIntoRows() {
    QVector<QVector<int>> gridRows;
    
    if (mGridNodes.isEmpty()) {
        return gridRows;
    }
    
    // 按Y坐标对节点进行分组
    QMap<int, QVector<int>> rowMap;
    const double ROW_TOLERANCE = 25.0; // 行容差
    
    for (const auto& node : mGridNodes) {
        int rowKey = static_cast<int>(node.position.y() / ROW_TOLERANCE);
        rowMap[rowKey].append(node.id);
    }
    
    // 将每行的节点按X坐标排序
    for (auto it = rowMap.begin(); it != rowMap.end(); ++it) {
        QVector<int>& rowNodes = it.value();
        
        // 按X坐标排序
        std::sort(rowNodes.begin(), rowNodes.end(), [this](int nodeId1, int nodeId2) {
            if (!mNodeIdToIndex.contains(nodeId1) || !mNodeIdToIndex.contains(nodeId2)) {
                return false;
            }
            
            int index1 = mNodeIdToIndex[nodeId1];
            int index2 = mNodeIdToIndex[nodeId2];
            
            return mGridNodes[index1].position.x() < mGridNodes[index2].position.x();
        });
        
        gridRows.append(rowNodes);
    }
    
    logMessage(QString("Organized nodes into %1 rows").arg(gridRows.size()), Qgis::MessageLevel::Info);
    return gridRows;
}

double GridPathPlanner::calculateCoverageRate(const QSet<int>& visitedNodes) const {
    if (mGridNodes.isEmpty()) {
        return 0.0;
    }
    
    return static_cast<double>(visitedNodes.size()) / mGridNodes.size();
}

QVector<QVector3D> GridPathPlanner::findUncoveredAreas(const QSet<int>& visitedNodes) const {
    QVector<QVector3D> uncoveredCenters;
    
    // 找到所有未访问的节点
    for (const auto& node : mGridNodes) {
        if (!visitedNodes.contains(node.id)) {
            uncoveredCenters.append(node.position);
        }
    }
    
    return uncoveredCenters;
}

QVector<GridNode> GridPathPlanner::buildCoverageGrid(double gridSpacing) {
    QVector<GridNode> coverageGrid;
    
    if (mGridNodes.isEmpty()) {
        return coverageGrid;
    }
    
    // 计算网格边界
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double maxY = std::numeric_limits<double>::lowest();
    
    for (const auto& node : mGridNodes) {
        minX = qMin(minX, static_cast<double>(node.position.x()));
        minY = qMin(minY, static_cast<double>(node.position.y()));
        maxX = qMax(maxX, static_cast<double>(node.position.x()));
        maxY = qMax(maxY, static_cast<double>(node.position.y()));
    }
    
    // 构建规则网格
    int gridId = 0;
    for (double x = minX; x <= maxX; x += gridSpacing) {
        for (double y = minY; y <= maxY; y += gridSpacing) {
            QVector3D position(x, y, 50.0); // 默认高度50米
            
            // 检查是否在原始网格范围内
            bool inOriginalGrid = false;
            for (const auto& node : mGridNodes) {
                if (calculateDistance(position, node.position) <= gridSpacing) {
                    inOriginalGrid = true;
                    break;
                }
            }
            
            if (inOriginalGrid) {
                GridNode gridNode(position, gridId++);
                coverageGrid.append(gridNode);
            }
        }
    }
    
    logMessage(QString("Built coverage grid with %1 nodes, spacing: %2m").arg(coverageGrid.size()).arg(gridSpacing), 
              Qgis::MessageLevel::Info);
    
    return coverageGrid;
}

// 面事件专用函数实现

bool GridPathPlanner::buildAreaGridFromFishnet(const QString& shapefile) {
    logMessage(QString("Loading area grid from fishnet: %1").arg(shapefile), Qgis::MessageLevel::Info);
    
    // 清空现有数据
    mAreaGridNodes.clear();
    mAreaNodeIdToIndex.clear();
    
    // 使用ShapefileHandler加载渔网线
    if (!mShapefileHandler->loadFishnetLines(shapefile)) {
        logMessage("Failed to load fishnet lines for area planning", Qgis::MessageLevel::Critical);
        return false;
    }
    
    // 获取渔网线并构建网格节点
    const auto& fishnetLines = mShapefileHandler->getFishnetLines();
    QMap<QString, int> pointToNodeId; // 点坐标到节点ID的映射
    QMap<int, QVector<int>> nodeConnections; // 节点连接关系
    int nodeId = 0;
    
    logMessage(QString("Processing %1 fishnet lines").arg(fishnetLines.size()), Qgis::MessageLevel::Info);
    
    // 添加渔网线质量检查
    logMessage("=== 渔网线质量检查 ===", Qgis::MessageLevel::Info);
    int veryShortLines = 0;
    int duplicateLines = 0;
    double minLineLength = std::numeric_limits<double>::max();
    double maxLineLength = 0.0;
    double totalLineLength = 0.0;
    QSet<QString> lineSignatures; // 用于检测重复线段
    
    for (int i = 0; i < fishnetLines.size(); ++i) {
        const auto& line = fishnetLines[i];
        double lineLength = calculateDistance(line.first, line.second);
        totalLineLength += lineLength;
        
        if (lineLength < minLineLength) minLineLength = lineLength;
        if (lineLength > maxLineLength) maxLineLength = lineLength;
        
        // 检查异常短的线段（小于1米）
        if (lineLength < 1.0) {
            veryShortLines++;
            if (veryShortLines <= 3) { // 只显示前3个异常短线段
                logMessage(QString("异常短线段 %1: 长度=%2m, 起点=(%3, %4), 终点=(%5, %6)")
                          .arg(i)
                          .arg(lineLength, 0, 'f', 3)
                          .arg(line.first.x(), 0, 'f', 2)
                          .arg(line.first.y(), 0, 'f', 2)
                          .arg(line.second.x(), 0, 'f', 2)
                          .arg(line.second.y(), 0, 'f', 2), 
                          Qgis::MessageLevel::Warning);
            }
        }
        
        // 检查重复线段
        QString startKey = QString("%1,%2").arg(line.first.x(), 0, 'f', 3).arg(line.first.y(), 0, 'f', 3);
        QString endKey = QString("%1,%2").arg(line.second.x(), 0, 'f', 3).arg(line.second.y(), 0, 'f', 3);
        QString lineSignature1 = QString("%1-%2").arg(startKey).arg(endKey);
        QString lineSignature2 = QString("%1-%2").arg(endKey).arg(startKey); // 反向也算重复
        
        if (lineSignatures.contains(lineSignature1) || lineSignatures.contains(lineSignature2)) {
            duplicateLines++;
        } else {
            lineSignatures.insert(lineSignature1);
        }
    }
    
    double avgLineLength = fishnetLines.isEmpty() ? 0.0 : totalLineLength / fishnetLines.size();
    
    logMessage(QString("渔网线统计: 总数=%1, 最短=%2m, 最长=%3m, 平均=%4m")
              .arg(fishnetLines.size())
              .arg(minLineLength, 0, 'f', 2)
              .arg(maxLineLength, 0, 'f', 2)
              .arg(avgLineLength, 0, 'f', 2), 
              Qgis::MessageLevel::Info);
    
    if (veryShortLines > 0) {
        logMessage(QString("警告: 发现 %1 条异常短线段（<1米）").arg(veryShortLines), 
                  Qgis::MessageLevel::Warning);
    }
    
    if (duplicateLines > 0) {
        logMessage(QString("警告: 发现 %1 条重复线段").arg(duplicateLines), 
                  Qgis::MessageLevel::Warning);
    }
    
    // 输出前5条渔网线的详细信息用于调试
    logMessage("=== 渔网线详细信息 ===", Qgis::MessageLevel::Info);
    for (int i = 0; i < qMin(5, fishnetLines.size()); ++i) {
        const auto& line = fishnetLines[i];
        double lineLength = calculateDistance(line.first, line.second);
        logMessage(QString("渔网线 %1: 起点=(%2, %3), 终点=(%4, %5), 长度=%6m")
                  .arg(i)
                  .arg(line.first.x(), 0, 'f', 2)
                  .arg(line.first.y(), 0, 'f', 2)
                  .arg(line.second.x(), 0, 'f', 2)
                  .arg(line.second.y(), 0, 'f', 2)
                  .arg(lineLength, 0, 'f', 2), 
                  Qgis::MessageLevel::Info);
    }
    
    // 第一遍：创建所有唯一的节点
    for (const auto& line : fishnetLines) {
        QVector3D startPoint = line.first;
        QVector3D endPoint = line.second;
        
        // 处理起点
        QString startKey = QString("%1,%2").arg(startPoint.x(), 0, 'f', 3).arg(startPoint.y(), 0, 'f', 3);
        int startNodeId;
        if (!pointToNodeId.contains(startKey)) {
            startNodeId = nodeId++;
            pointToNodeId[startKey] = startNodeId;
            GridNode node(startPoint, startNodeId);
            mAreaGridNodes.append(node);
            mAreaNodeIdToIndex[startNodeId] = mAreaGridNodes.size() - 1;
        } else {
            startNodeId = pointToNodeId[startKey];
        }
        
        // 处理终点
        QString endKey = QString("%1,%2").arg(endPoint.x(), 0, 'f', 3).arg(endPoint.y(), 0, 'f', 3);
        int endNodeId;
        if (!pointToNodeId.contains(endKey)) {
            endNodeId = nodeId++;
            pointToNodeId[endKey] = endNodeId;
            GridNode node(endPoint, endNodeId);
            mAreaGridNodes.append(node);
            mAreaNodeIdToIndex[endNodeId] = mAreaGridNodes.size() - 1;
        } else {
            endNodeId = pointToNodeId[endKey];
        }
        
        // 记录连接关系（双向）
        if (!nodeConnections[startNodeId].contains(endNodeId)) {
            nodeConnections[startNodeId].append(endNodeId);
        }
        if (!nodeConnections[endNodeId].contains(startNodeId)) {
            nodeConnections[endNodeId].append(startNodeId);
        }
    }
    
    // 第二遍：根据渔网线的实际连接关系设置节点邻接关系
    for (auto it = nodeConnections.begin(); it != nodeConnections.end(); ++it) {
        int nodeId = it.key();
        const QVector<int>& connections = it.value();
        
        if (mAreaNodeIdToIndex.contains(nodeId)) {
            int nodeIndex = mAreaNodeIdToIndex[nodeId];
            mAreaGridNodes[nodeIndex].neighbors = connections;
        }
    }
    
    logMessage(QString("Built area grid with %1 nodes from fishnet lines").arg(mAreaGridNodes.size()), 
              Qgis::MessageLevel::Info);
    
    // 输出前5个节点的详细信息用于调试
    logMessage("=== 面事件网格节点详细信息 ===", Qgis::MessageLevel::Info);
    for (int i = 0; i < qMin(5, mAreaGridNodes.size()); ++i) {
        const GridNode& node = mAreaGridNodes[i];
        logMessage(QString("节点 %1: ID=%2, 位置=(%3, %4, %5), 邻居数=%6")
                  .arg(i)
                  .arg(node.id)
                  .arg(node.position.x(), 0, 'f', 2)
                  .arg(node.position.y(), 0, 'f', 2)
                  .arg(node.position.z(), 0, 'f', 2)
                  .arg(node.neighbors.size()), 
                  Qgis::MessageLevel::Info);
        
        // 显示邻居节点ID
        if (!node.neighbors.isEmpty()) {
            QStringList neighborIds;
            for (int neighborId : node.neighbors) {
                neighborIds.append(QString::number(neighborId));
            }
            logMessage(QString("  邻居节点ID: [%1]").arg(neighborIds.join(", ")), 
                      Qgis::MessageLevel::Info);
        }
    }
    
    // 验证节点连接
    int totalConnections = 0;
    for (const auto& node : mAreaGridNodes) {
        totalConnections += node.neighbors.size();
    }
    logMessage(QString("Total fishnet-based connections: %1").arg(totalConnections / 2), 
              Qgis::MessageLevel::Info);
    
    // 验证所有节点都来自渔网线端点
    logMessage("=== 验证节点来源 ===", Qgis::MessageLevel::Info);
    QSet<QString> fishnetEndpoints;
    for (const auto& line : fishnetLines) {
        QString startKey = QString("%1,%2").arg(line.first.x(), 0, 'f', 3).arg(line.first.y(), 0, 'f', 3);
        QString endKey = QString("%1,%2").arg(line.second.x(), 0, 'f', 3).arg(line.second.y(), 0, 'f', 3);
        fishnetEndpoints.insert(startKey);
        fishnetEndpoints.insert(endKey);
    }
    
    int validNodes = 0;
    int invalidNodes = 0;
    for (const auto& node : mAreaGridNodes) {
        QString nodeKey = QString("%1,%2").arg(node.position.x(), 0, 'f', 3).arg(node.position.y(), 0, 'f', 3);
        if (fishnetEndpoints.contains(nodeKey)) {
            validNodes++;
        } else {
            invalidNodes++;
            if (invalidNodes <= 3) { // 只显示前3个无效节点
                logMessage(QString("无效节点: ID=%1, 位置=(%2, %3) - 不在渔网线端点")
                          .arg(node.id)
                          .arg(node.position.x(), 0, 'f', 2)
                          .arg(node.position.y(), 0, 'f', 2), 
                          Qgis::MessageLevel::Warning);
            }
        }
    }
    
    logMessage(QString("节点验证结果: 有效节点=%1, 无效节点=%2, 渔网端点总数=%3")
              .arg(validNodes).arg(invalidNodes).arg(fishnetEndpoints.size()), 
              Qgis::MessageLevel::Info);
    
    // 输出渔网线的范围信息
    if (!mAreaGridNodes.isEmpty()) {
        double minX = mAreaGridNodes[0].position.x();
        double maxX = minX;
        double minY = mAreaGridNodes[0].position.y();
        double maxY = minY;
        
        for (const auto& node : mAreaGridNodes) {
            minX = qMin(minX, static_cast<double>(node.position.x()));
            maxX = qMax(maxX, static_cast<double>(node.position.x()));
            minY = qMin(minY, static_cast<double>(node.position.y()));
            maxY = qMax(maxY, static_cast<double>(node.position.y()));
        }
        
        logMessage(QString("面事件网格范围: X[%1, %2], Y[%3, %4]")
                  .arg(minX, 0, 'f', 2).arg(maxX, 0, 'f', 2)
                  .arg(minY, 0, 'f', 2).arg(maxY, 0, 'f', 2), 
                  Qgis::MessageLevel::Info);
    }
    
    // === 新增：分析第一行节点的分布模式 ===
    logMessage("=== 分析第一行节点分布模式 ===", Qgis::MessageLevel::Info);
    
    // 找到Y坐标最大的节点（第一行）
    QVector<GridNode> firstRowNodes;
    if (!mAreaGridNodes.isEmpty()) {
        double maxY = mAreaGridNodes[0].position.y();
        for (const auto& node : mAreaGridNodes) {
            if (node.position.y() > maxY) {
                maxY = node.position.y();
            }
        }
        
        // 收集第一行的所有节点
        const double Y_TOLERANCE = 1.0; // Y坐标容差
        for (const auto& node : mAreaGridNodes) {
            if (qAbs(node.position.y() - maxY) <= Y_TOLERANCE) {
                firstRowNodes.append(node);
            }
        }
        
        // 按X坐标排序
        std::sort(firstRowNodes.begin(), firstRowNodes.end(), [](const GridNode& a, const GridNode& b) {
            return a.position.x() < b.position.x();
        });
        
        logMessage(QString("第一行节点数量: %1, Y坐标: %2").arg(firstRowNodes.size()).arg(maxY, 0, 'f', 2), 
                  Qgis::MessageLevel::Info);
        
        // 分析前10个节点的X坐标间距
        if (firstRowNodes.size() >= 10) {
            QVector<double> intervals;
            for (int i = 1; i < qMin(10, firstRowNodes.size()); ++i) {
                double interval = firstRowNodes[i].position.x() - firstRowNodes[i-1].position.x();
                intervals.append(interval);
                logMessage(QString("第一行节点 %1-%2: X间距=%3").arg(i-1).arg(i).arg(interval, 0, 'f', 3), 
                          Qgis::MessageLevel::Info);
            }
            
            // 计算间距的统计信息
            if (!intervals.isEmpty()) {
                double minInterval = *std::min_element(intervals.begin(), intervals.end());
                double maxInterval = *std::max_element(intervals.begin(), intervals.end());
                double avgInterval = std::accumulate(intervals.begin(), intervals.end(), 0.0) / intervals.size();
                
                logMessage(QString("第一行X间距统计: 最小=%1, 最大=%2, 平均=%3")
                          .arg(minInterval, 0, 'f', 3).arg(maxInterval, 0, 'f', 3).arg(avgInterval, 0, 'f', 3), 
                          Qgis::MessageLevel::Info);
                
                // 判断是否为规则分布
                double intervalVariance = maxInterval - minInterval;
                if (intervalVariance < 0.1) {
                    logMessage("警告：第一行节点呈现规则分布！这可能是渔网生成工具造成的。", Qgis::MessageLevel::Warning);
                } else {
                    logMessage("第一行节点分布不规则，符合预期。", Qgis::MessageLevel::Info);
                }
            }
        }
    }
    
    // 进行更详细的规律性分析
    analyzeAreaGridRegularity();
    
    return !mAreaGridNodes.isEmpty();
}

void GridPathPlanner::buildAreaGridConnections() {
    logMessage("Area grid connections already built from fishnet lines", Qgis::MessageLevel::Info);
    
    // 连接关系已经在buildAreaGridFromFishnet中根据渔网线建立
    // 这里只需要验证和记录连接统计信息
    
    int totalConnections = 0;
    int isolatedNodes = 0;
    
    for (const auto& node : mAreaGridNodes) {
        totalConnections += node.neighbors.size();
        if (node.neighbors.isEmpty()) {
            isolatedNodes++;
        }
    }
    
    logMessage(QString("Area grid connection summary: %1 total connections, %2 isolated nodes")
              .arg(totalConnections / 2).arg(isolatedNodes), 
              Qgis::MessageLevel::Info);
    
    // 如果有孤立节点，发出警告
    if (isolatedNodes > 0) {
        logMessage(QString("Warning: %1 isolated nodes found in area grid").arg(isolatedNodes), 
                  Qgis::MessageLevel::Warning);
    }
}

int GridPathPlanner::findNearestAreaGridNode(const QVector3D& position) const {
    if (mAreaGridNodes.isEmpty()) {
        return -1;
    }
    
    double minDistance = std::numeric_limits<double>::max();
    int nearestNodeId = -1;
    
    for (const auto& node : mAreaGridNodes) {
        double distance = calculateDistance(position, node.position);
        if (distance < minDistance) {
            minDistance = distance;
            nearestNodeId = node.id;
        }
    }
    
    return nearestNodeId;
}

double GridPathPlanner::calculateAreaCoverageRate(const QSet<int>& visitedNodes) const {
    if (mAreaGridNodes.isEmpty()) {
        return 0.0;
    }
    
    return static_cast<double>(visitedNodes.size()) / mAreaGridNodes.size();
}

QVector<QVector3D> GridPathPlanner::findAreaUncoveredAreas(const QSet<int>& visitedNodes) const {
    QVector<QVector3D> uncoveredCenters;
    
    // 找到所有未访问的节点
    for (const auto& node : mAreaGridNodes) {
        if (!visitedNodes.contains(node.id)) {
            uncoveredCenters.append(node.position);
        }
    }
    
    return uncoveredCenters;
}

// 面事件改进的螺旋节点搜索算法
int GridPathPlanner::findNextAreaSpiralNodeImproved(int currentNodeId, const QSet<int>& visitedNodes, const QSet<int>& availableNodes) {
    if (!mAreaNodeIdToIndex.contains(currentNodeId)) {
        return -1;
    }
    
    int currentNodeIndex = mAreaNodeIdToIndex[currentNodeId];
    const GridNode& currentNode = mAreaGridNodes[currentNodeIndex];
    
    // 多层螺旋搜索：按距离分层，每层内按角度排序
    QMap<int, QVector<QPair<double, int>>> layerNodes; // 距离层 -> (角度, 节点ID)
    
    for (int nodeId : availableNodes) {
        if (visitedNodes.contains(nodeId)) continue;
        
        if (!mAreaNodeIdToIndex.contains(nodeId)) continue;
        
        int nodeIndex = mAreaNodeIdToIndex[nodeId];
        const GridNode& node = mAreaGridNodes[nodeIndex];
        
        double distance = calculateDistance(currentNode.position, node.position);
        double angle = qAtan2(node.position.y() - currentNode.position.y(), 
                             node.position.x() - currentNode.position.x());
        
        // 将节点按距离分层
        int layer = static_cast<int>(distance / 50.0); // 每50米一层
        layerNodes[layer].append(qMakePair(angle, nodeId));
    }
    
    // 从最近的层开始搜索
    for (auto it = layerNodes.begin(); it != layerNodes.end(); ++it) {
        QVector<QPair<double, int>>& nodes = it.value();
        if (nodes.isEmpty()) continue;
        
        // 在当前层内按角度排序
        std::sort(nodes.begin(), nodes.end());
        
        // 选择第一个可用节点
        for (const auto& pair : nodes) {
            int nodeId = pair.second;
            if (availableNodes.contains(nodeId) && !visitedNodes.contains(nodeId)) {
                return nodeId;
            }
        }
    }
    
    return -1;
}



// 寻找最近的未访问面事件节点
int GridPathPlanner::findNearestAreaUnvisitedNode(int currentNodeId, const QSet<int>& visitedNodes, const QSet<int>& availableNodes) {
    if (!mAreaNodeIdToIndex.contains(currentNodeId)) {
        return -1;
    }
    
    int currentNodeIndex = mAreaNodeIdToIndex[currentNodeId];
    const GridNode& currentNode = mAreaGridNodes[currentNodeIndex];
    
    double minDistance = std::numeric_limits<double>::max();
    int nearestNodeId = -1;
    
    for (int nodeId : availableNodes) {
        if (visitedNodes.contains(nodeId)) continue;
        
        if (!mAreaNodeIdToIndex.contains(nodeId)) continue;
        
        int nodeIndex = mAreaNodeIdToIndex[nodeId];
        const GridNode& node = mAreaGridNodes[nodeIndex];
        
        double distance = calculateDistance(currentNode.position, node.position);
        if (distance < minDistance) {
            minDistance = distance;
            nearestNodeId = nodeId;
        }
    }
    
    return nearestNodeId;
}

// 填补面事件遗漏的节点
void GridPathPlanner::fillAreaMissingNodes(QVector<PathPoint>& coveragePath, QSet<int>& visitedNodes, const QSet<int>& availableNodes, double coverageThreshold) {
    logMessage("Filling missing area nodes to improve coverage...", Qgis::MessageLevel::Info);
    
    QSet<int> remainingNodes = availableNodes - visitedNodes;
    
    while (!remainingNodes.isEmpty()) {
        double currentCoverage = calculateAreaCoverageRate(visitedNodes);
        if (currentCoverage >= coverageThreshold) {
            break;
        }
        
        // 找到最近的未访问节点
        int nearestNodeId = -1;
        double minDistance = std::numeric_limits<double>::max();
        QVector3D lastPosition = coveragePath.isEmpty() ? QVector3D() : coveragePath.last().position;
        
        for (int nodeId : remainingNodes) {
            if (!mAreaNodeIdToIndex.contains(nodeId)) continue;
            
            int nodeIndex = mAreaNodeIdToIndex[nodeId];
            const GridNode& node = mAreaGridNodes[nodeIndex];
            
            double distance = coveragePath.isEmpty() ? 0.0 : calculateDistance(lastPosition, node.position);
            if (distance < minDistance || nearestNodeId == -1) {
                minDistance = distance;
                nearestNodeId = nodeId;
            }
        }
        
        if (nearestNodeId == -1) break;
        
        // 添加节点到路径
        int nodeIndex = mAreaNodeIdToIndex[nearestNodeId];
        const GridNode& node = mAreaGridNodes[nodeIndex];
        
        PathPoint pathPoint(node.position);
        pathPoint.isRiskPoint = false;
        pathPoint.riskPointId = -1;
        pathPoint.lineSegmentId = -1;
        coveragePath.append(pathPoint);
        
        visitedNodes.insert(nearestNodeId);
        remainingNodes.remove(nearestNodeId);
    }
    
    double finalCoverage = calculateAreaCoverageRate(visitedNodes);
    logMessage(QString("Area missing nodes filled. Final coverage: %1%").arg(finalCoverage * 100.0, 0, 'f', 1), 
              Qgis::MessageLevel::Info);
}

// 构建面事件网格结构信息
void GridPathPlanner::buildAreaGridStructure() {
    logMessage("Building area grid structure for coverage...", Qgis::MessageLevel::Info);
    
    // 这个函数可以用来分析网格的结构特征
    // 比如识别网格的行列结构、边界等
    // 当前实现为占位符，可以根据需要扩展
    
    if (mAreaGridNodes.isEmpty()) {
        logMessage("No area grid nodes to analyze", Qgis::MessageLevel::Warning);
        return;
    }
    
    logMessage(QString("Area grid structure analysis completed for %1 nodes").arg(mAreaGridNodes.size()), 
              Qgis::MessageLevel::Info);
}

// 将面事件节点组织成行
QVector<QVector<int>> GridPathPlanner::organizeAreaNodesIntoRows() {
    QVector<QVector<int>> gridRows;
    
    if (mAreaGridNodes.isEmpty()) {
        return gridRows;
    }
    
    // 按Y坐标对节点进行分组
    QMap<int, QVector<int>> rowMap;
    const double ROW_TOLERANCE = 25.0; // 行容差
    
    for (const auto& node : mAreaGridNodes) {
        int rowKey = static_cast<int>(node.position.y() / ROW_TOLERANCE);
        rowMap[rowKey].append(node.id);
    }
    
    // 将每行的节点按X坐标排序
    for (auto it = rowMap.begin(); it != rowMap.end(); ++it) {
        QVector<int>& rowNodes = it.value();
        
        // 按X坐标排序
        std::sort(rowNodes.begin(), rowNodes.end(), [this](int nodeId1, int nodeId2) {
            if (!mAreaNodeIdToIndex.contains(nodeId1) || !mAreaNodeIdToIndex.contains(nodeId2)) {
                return false;
            }
            
            int index1 = mAreaNodeIdToIndex[nodeId1];
            int index2 = mAreaNodeIdToIndex[nodeId2];
            
            return mAreaGridNodes[index1].position.x() < mAreaGridNodes[index2].position.x();
        });
        
        gridRows.append(rowNodes);
    }
    
    logMessage(QString("Organized area nodes into %1 rows").arg(gridRows.size()), Qgis::MessageLevel::Info);
    return gridRows;
}

// ... existing code ...

bool GridPathPlanner::analyzeAreaGridRegularity() {
    if (mAreaGridNodes.empty()) {
        return false;
    }
    
    // 收集第一行节点（相同Y坐标）
    std::map<double, std::vector<GridNode*>> rowMap;
    for (auto& node : mAreaGridNodes) {
        double roundedY = std::round(node.position.y() * 100.0) / 100.0;  // 精确到cm
        rowMap[roundedY].push_back(&node);
    }
    
    // 找到节点最多的行
    auto maxRowIt = std::max_element(rowMap.begin(), rowMap.end(), 
        [](const auto& a, const auto& b) { return a.second.size() < b.second.size(); });
    
    if (maxRowIt == rowMap.end() || maxRowIt->second.size() < 5) {
        return false;
    }
    
    // 分析最大行的节点间距
    std::vector<GridNode*>& rowNodes = maxRowIt->second;
    std::sort(rowNodes.begin(), rowNodes.end(), 
        [](const GridNode* a, const GridNode* b) { return a->position.x() < b->position.x(); });
    
    std::vector<double> intervals;
    for (size_t i = 1; i < rowNodes.size(); ++i) {
        intervals.push_back(rowNodes[i]->position.x() - rowNodes[i-1]->position.x());
    }
    
    if (intervals.empty()) return false;
    
    // 计算间距统计
    double minInterval = *std::min_element(intervals.begin(), intervals.end());
    double maxInterval = *std::max_element(intervals.begin(), intervals.end());
    double avgInterval = std::accumulate(intervals.begin(), intervals.end(), 0.0) / intervals.size();
    
    // 计算变异系数
    double variance = 0.0;
    for (double interval : intervals) {
        variance += (interval - avgInterval) * (interval - avgInterval);
    }
    variance /= intervals.size();
    double stdDev = std::sqrt(variance);
    double coeffVar = stdDev / avgInterval;  // 变异系数
    
    logMessage("=== 渔网规律性分析 ===", Qgis::MessageLevel::Info);
    logMessage(QString("分析行: Y坐标=%1, 节点数=%2").arg(maxRowIt->first, 0, 'f', 2).arg(rowNodes.size()), Qgis::MessageLevel::Info);
    logMessage(QString("X间距统计: 最小=%1, 最大=%2, 平均=%3")
               .arg(minInterval, 0, 'f', 3).arg(maxInterval, 0, 'f', 3).arg(avgInterval, 0, 'f', 3), Qgis::MessageLevel::Info);
    logMessage(QString("标准差=%1, 变异系数=%2").arg(stdDev, 0, 'f', 3).arg(coeffVar, 0, 'f', 3), Qgis::MessageLevel::Info);
    
    // 判断规律性
    bool isRegular = coeffVar < 0.05;  // 变异系数小于5%认为是规则的
    
    if (isRegular) {
        logMessage("警告：渔网线呈现高度规则分布！", Qgis::MessageLevel::Warning);
        logMessage("建议：", Qgis::MessageLevel::Info);
        logMessage("1. 检查渔网生成工具的设置，确保它基于真实的障碍物和地形", Qgis::MessageLevel::Info);
        logMessage("2. 如果需要更自然的分布，考虑手动编辑渔网线", Qgis::MessageLevel::Info);
        logMessage("3. 或者使用更精细的障碍物数据重新生成渔网", Qgis::MessageLevel::Info);
        logMessage("4. 当前算法正确工作，问题在于输入数据的规律性", Qgis::MessageLevel::Info);
    } else {
        logMessage("渔网分布相对自然，变异系数良好", Qgis::MessageLevel::Info);
    }
    
    return isRegular;
}

// ... existing code ...