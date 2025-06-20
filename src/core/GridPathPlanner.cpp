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
    logMessage(QString("Starting spiral coverage algorithm with threshold: %1").arg(coverageThreshold), 
              Qgis::MessageLevel::Info);
    
    QVector<PathPoint> coveragePath;
    QSet<int> visitedNodes;
    QSet<int> availableNodes;
    
    // 初始化可用节点
    for (const auto& node : mGridNodes) {
        availableNodes.insert(node.id);
    }
    
    int currentNodeId = startNodeId;
    int iteration = 0;
    const int MAX_ITERATIONS = 1000;
    
    while (iteration < MAX_ITERATIONS) {
        // 添加当前节点到路径
        if (mNodeIdToIndex.contains(currentNodeId)) {
            int nodeIndex = mNodeIdToIndex[currentNodeId];
            const GridNode& node = mGridNodes[nodeIndex];
            
            PathPoint pathPoint(node.position);
            pathPoint.isRiskPoint = false;
            pathPoint.riskPointId = -1;
            pathPoint.lineSegmentId = -1;
            coveragePath.append(pathPoint);
            
            visitedNodes.insert(currentNodeId);
            availableNodes.remove(currentNodeId);
        }
        
        // 检查覆盖率
        double currentCoverage = calculateCoverageRate(visitedNodes);
        if (currentCoverage >= coverageThreshold) {
            logMessage(QString("Coverage threshold reached: %1%").arg(currentCoverage * 100.0, 0, 'f', 1), 
                      Qgis::MessageLevel::Info);
            break;
        }
        
        // 寻找下一个最佳节点（螺旋模式）
        int nextNodeId = findNextSpiralNode(currentNodeId, visitedNodes, availableNodes);
        if (nextNodeId == -1) {
            logMessage("No more nodes to visit in spiral pattern", Qgis::MessageLevel::Warning);
            break;
        }
        
        currentNodeId = nextNodeId;
        iteration++;
        
        if (iteration % 100 == 0) {
            logMessage(QString("Spiral iteration %1, coverage: %2%")
                      .arg(iteration).arg(currentCoverage * 100.0, 0, 'f', 1), 
                      Qgis::MessageLevel::Info);
        }
    }
    
    logMessage(QString("Spiral coverage completed. Total nodes visited: %1, coverage: %2%")
              .arg(visitedNodes.size()).arg(calculateCoverageRate(visitedNodes) * 100.0, 0, 'f', 1), 
              Qgis::MessageLevel::Success);
    
    return coveragePath;
}

QVector<PathPoint> GridPathPlanner::solveAreaGridCoverage(int startNodeId, double coverageThreshold) {
    logMessage(QString("Starting grid coverage algorithm with threshold: %1").arg(coverageThreshold), 
              Qgis::MessageLevel::Info);
    
    QVector<PathPoint> coveragePath;
    QSet<int> visitedNodes;
    
    // 构建规则网格
    QVector<GridNode> coverageGrid = buildCoverageGrid(10.0); // 10米网格间距
    
    // 从起始点开始，按网格顺序访问
    int currentNodeId = startNodeId;
    int iteration = 0;
    const int MAX_ITERATIONS = 1000;
    
    while (iteration < MAX_ITERATIONS) {
        // 添加当前节点到路径
        if (mNodeIdToIndex.contains(currentNodeId)) {
            int nodeIndex = mNodeIdToIndex[currentNodeId];
            const GridNode& node = mGridNodes[nodeIndex];
            
            PathPoint pathPoint(node.position);
            pathPoint.isRiskPoint = false;
            pathPoint.riskPointId = -1;
            pathPoint.lineSegmentId = -1;
            coveragePath.append(pathPoint);
            
            visitedNodes.insert(currentNodeId);
        }
        
        // 检查覆盖率
        double currentCoverage = calculateCoverageRate(visitedNodes);
        if (currentCoverage >= coverageThreshold) {
            logMessage(QString("Coverage threshold reached: %1%").arg(currentCoverage * 100.0, 0, 'f', 1), 
                      Qgis::MessageLevel::Info);
            break;
        }
        
        // 寻找下一个网格节点
        int nextNodeId = findNextGridNode(currentNodeId, visitedNodes);
        if (nextNodeId == -1) {
            logMessage("No more grid nodes to visit", Qgis::MessageLevel::Warning);
            break;
        }
        
        currentNodeId = nextNodeId;
        iteration++;
        
        if (iteration % 100 == 0) {
            logMessage(QString("Grid iteration %1, coverage: %2%")
                      .arg(iteration).arg(currentCoverage * 100.0, 0, 'f', 1), 
                      Qgis::MessageLevel::Info);
        }
    }
    
    logMessage(QString("Grid coverage completed. Total nodes visited: %1, coverage: %2%")
              .arg(visitedNodes.size()).arg(calculateCoverageRate(visitedNodes) * 100.0, 0, 'f', 1), 
              Qgis::MessageLevel::Success);
    
    return coveragePath;
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
    
    // 加载渔网线
    if (!buildGridFromFishnet(params.fishnetShapefile)) {
        result.errorMessage = "Failed to load fishnet from shapefile";
        logMessage("Failed to load fishnet from shapefile", Qgis::MessageLevel::Critical);
        return result;
    }
    
    logMessage(QString("Loaded %1 grid nodes from fishnet").arg(mGridNodes.size()), 
              Qgis::MessageLevel::Info);
    
    // 构建网格连接
    buildGridConnections();
    
    // 检查网格连通性
    if (mGridNodes.isEmpty()) {
        result.errorMessage = "No grid nodes available";
        logMessage("No grid nodes available", Qgis::MessageLevel::Critical);
        return result;
    }
    
    // 找到最近的起始节点
    int startNodeId = findNearestGridNode(params.startPoint);
    if (startNodeId == -1) {
        result.errorMessage = "Cannot find nearest grid node to start point";
        logMessage("Cannot find nearest grid node to start point", Qgis::MessageLevel::Critical);
        return result;
    }
    
    logMessage(QString("Start node ID: %1").arg(startNodeId), Qgis::MessageLevel::Info);
    
    // 执行路径规划算法
    QVector<PathPoint> optimalPath;
    
    if (params.algorithm == "Area_Spiral") {
        optimalPath = solveAreaSpiralCoverage(startNodeId, params.coverageThreshold);
    } else if (params.algorithm == "Area_Grid") {
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
        int nodeId = findNearestGridNode(point.position);
        if (nodeId != -1) {
            visitedNodes.insert(nodeId);
        }
    }
    double coverageRate = calculateCoverageRate(visitedNodes);
    
    // 找到未覆盖区域
    QVector<QVector3D> uncoveredAreas = findUncoveredAreas(visitedNodes);
    
    // 计算平均路径密度
    double totalArea = mGridNodes.size() * 100.0; // 假设每个网格单元100平方米
    double averagePathDensity = totalPathLength / totalArea;
    
    // 设置结果
    result.success = true;
    result.gridNodes = mGridNodes;
    result.optimalPath = optimalPath;
    result.totalPathLength = totalPathLength;
    result.coverageRate = coverageRate;
    result.totalGridCells = mGridNodes.size();
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
        
        for (int i = 0; i < result.optimalPath.size(); ++i) {
            const PathPoint& point = result.optimalPath[i];
            
            out << (i + 1) << ","
                << point.position.x() << ","
                << point.position.y() << ","
                << point.position.z() << ","
                << "covered" << "\n";
        }
        
        csvFile.close();
        logMessage(QString("Exported area path to CSV: %1").arg(csvPath), Qgis::MessageLevel::Info);
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