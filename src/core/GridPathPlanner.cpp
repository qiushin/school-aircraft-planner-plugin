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
            minX = qMin(minX, (double)node.position.x());
            maxX = qMax(maxX, (double)node.position.x());
            minY = qMin(minY, (double)node.position.y());
            maxY = qMax(maxY, (double)node.position.y());
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