/****************************************************************************
File: GridPathPlanner.h
Author: AI Assistant
Date: 2025.1.6
Description: 基于渔网线的无人机避障路径规划器
****************************************************************************/

#ifndef GRID_PATH_PLANNER_H
#define GRID_PATH_PLANNER_H

#include <QObject>
#include <QVector>
#include <QVector3D>
#include <QString>
#include <QPolygonF>
#include <QLineF>
#include <QSet>
#include <QHash>
#include <QPair>
#include "ShapefileHandler.h"

/**
 * @brief 网格节点结构
 */
struct GridNode {
    QVector3D position;          // 节点位置
    int id;                      // 节点ID
    bool isRiskPoint;           // 是否为风险点
    bool isObstacle;            // 是否为障碍物
    QVector<int> neighbors;     // 邻接节点ID列表
    
    GridNode() : id(-1), isRiskPoint(false), isObstacle(false) {}
    GridNode(const QVector3D& pos, int nodeId) 
        : position(pos), id(nodeId), isRiskPoint(false), isObstacle(false) {}
};

/**
 * @brief 路径节点结构（用于最终路径）
 */
struct PathPoint {
    QVector3D position;         // 位置
    int riskPointId;           // 关联的风险点ID（-1表示非风险点）
    bool isRiskPoint;          // 是否为风险点
    double distanceFromStart;   // 从起点的累计距离
    int lineSegmentId;         // 关联的线段ID（用于线事件规划，-1表示非线段点）
    
    PathPoint() : riskPointId(-1), isRiskPoint(false), distanceFromStart(0.0), lineSegmentId(-1) {}
    PathPoint(const QVector3D& pos) 
        : position(pos), riskPointId(-1), isRiskPoint(false), distanceFromStart(0.0), lineSegmentId(-1) {}
};

/**
 * @brief 风险线事件结构
 */
struct RiskLineEvent {
    int lineId;                    // 线事件ID
    QVector3D startPoint;          // 线起点
    QVector3D endPoint;            // 线终点
    QVector<int> gridNodeIds;      // 匹配的网格节点ID序列
    bool isCompleted;              // 是否已完成飞行
    
    RiskLineEvent() : lineId(-1), isCompleted(false) {}
    RiskLineEvent(int id, const QVector3D& start, const QVector3D& end) 
        : lineId(id), startPoint(start), endPoint(end), isCompleted(false) {}
};

/**
 * @brief 网格路径规划参数
 */
struct GridPlanningParameters {
    QString fishnetShapefile;        // 渔网线SHP文件路径
    QString riskPointsShapefile;     // 风险点SHP文件路径
    QString outputPath;              // 输出路径
    QVector3D startPoint;            // 起始点
    QString algorithm;               // 算法类型 ("Dijkstra", "AStar", "TSP_Dijkstra")
    double maxRiskPointDistance;     // 风险点最大距离阈值
    
    GridPlanningParameters() : 
        startPoint(558856.516f, 3371566.848f, 50.0f),
        algorithm("TSP_Dijkstra"),
        maxRiskPointDistance(50.0) {}
};

/**
 * @brief 线事件路径规划参数
 */
struct LinePlanningParameters {
    QString fishnetShapefile;        // 渔网线SHP文件路径
    QString riskLinesShapefile;      // 风险线SHP文件路径
    QString outputPath;              // 输出路径
    QVector3D startPoint;            // 起始点
    QString algorithm;               // 算法选择
    double maxRiskLineDistance;      // 风险线匹配最大距离
};

/**
 * @brief 面事件路径规划参数
 */
struct AreaPlanningParameters {
    QString fishnetShapefile;        // 渔网线SHP文件路径（已根据风险面生成）
    QString outputPath;              // 输出路径
    QVector3D startPoint;            // 起始点
    QString algorithm;               // 算法选择
    double coverageThreshold;        // 覆盖率阈值（0.0-1.0）
    int maxIterations;               // 最大迭代次数
    double gridSpacing;              // 网格间距（米）
    
    AreaPlanningParameters() : 
        startPoint(558856.516f, 3371566.848f, 50.0f),
        algorithm("Area_Spiral"),
        coverageThreshold(0.95),
        maxIterations(1000),
        gridSpacing(10.0) {}
};

/**
 * @brief 网格路径规划结果
 */
struct GridPlanningResult {
    bool success;                    // 是否成功
    QString errorMessage;            // 错误信息
    QString algorithm;               // 使用的算法
    QVector<GridNode> gridNodes;     // 网格节点
    QVector<QVector3D> riskPoints;   // 风险点
    QVector<PathPoint> optimalPath;  // 最优路径
    int totalRiskPoints;             // 总风险点数
    int visitedRiskPoints;           // 访问的风险点数
    double totalPathLength;          // 总路径长度
};

/**
 * @brief 线事件路径规划结果
 */
struct LinePlanningResult {
    bool success;                    // 是否成功
    QString errorMessage;            // 错误信息
    QString algorithm;               // 使用的算法
    QVector<GridNode> gridNodes;     // 网格节点
    QVector<RiskLineEvent> riskLines; // 风险线事件
    QVector<PathPoint> optimalPath;  // 最优路径
    int totalRiskLines;              // 总风险线数
    int completedRiskLines;          // 完成的风险线数
    double totalPathLength;          // 总路径长度
};

/**
 * @brief 面事件路径规划结果
 */
struct AreaPlanningResult {
    bool success;                    // 是否成功
    QString errorMessage;            // 错误信息
    QString algorithm;               // 使用的算法
    QVector<GridNode> gridNodes;     // 网格节点
    QVector<PathPoint> optimalPath;  // 最优路径
    double totalPathLength;          // 总路径长度
    double coverageRate;             // 覆盖率（0.0-1.0）
    int totalGridCells;              // 总网格单元数
    int coveredGridCells;            // 已覆盖网格单元数
    QVector<QVector3D> uncoveredAreas; // 未覆盖区域中心点
    double averagePathDensity;       // 平均路径密度（米/平方米）
};

/**
 * @brief 基于渔网线的网格路径规划器
 */
class GridPathPlanner : public QObject {
    Q_OBJECT

public:
    explicit GridPathPlanner(QObject *parent = nullptr);
    ~GridPathPlanner();

    /**
     * @brief 执行网格路径规划
     * @param params 规划参数
     * @return 规划结果
     */
    GridPlanningResult executePlanning(const GridPlanningParameters& params);

    /**
     * @brief 执行线事件路径规划
     * @param params 线事件规划参数
     * @return 线事件规划结果
     */
    LinePlanningResult executeLinePlanning(const LinePlanningParameters& params);

    /**
     * @brief 执行面事件路径规划
     * @param params 面事件规划参数
     * @return 面事件规划结果
     */
    AreaPlanningResult executeAreaPlanning(const AreaPlanningParameters& params);

    /**
     * @brief 验证规划参数
     * @param params 待验证的参数
     * @return 验证结果和错误信息
     */
    QPair<bool, QString> validateParameters(const GridPlanningParameters& params);

    /**
     * @brief 验证线事件规划参数
     * @param params 线事件规划参数
     * @return 验证结果和错误信息
     */
    QPair<bool, QString> validateLineParameters(const LinePlanningParameters& params);

    /**
     * @brief 验证面事件规划参数
     * @param params 面事件规划参数
     * @return 验证结果和错误信息
     */
    QPair<bool, QString> validateAreaParameters(const AreaPlanningParameters& params);

    /**
     * @brief 导出路径到文件
     * @param result 规划结果
     * @param outputPath 输出路径
     * @return 成功返回true
     */
    bool exportPathToFiles(const GridPlanningResult& result, const QString& outputPath);

    /**
     * @brief 导出线事件路径到文件
     * @param result 线事件规划结果
     * @param outputPath 输出路径
     * @return 成功返回true
     */
    bool exportLinePathToFiles(const LinePlanningResult& result, const QString& outputPath);

    /**
     * @brief 导出面事件路径到文件
     * @param result 面事件规划结果
     * @param outputPath 输出路径
     * @return 成功返回true
     */
    bool exportAreaPathToFiles(const AreaPlanningResult& result, const QString& outputPath);

    /**
     * @brief 获取统计信息
     * @param result 规划结果
     * @return 统计信息字符串
     */
    QString getStatistics(const GridPlanningResult& result) const;

    /**
     * @brief 获取线事件统计信息
     * @param result 线事件规划结果
     * @return 统计信息字符串
     */
    QString getLineStatistics(const LinePlanningResult& result) const;

    /**
     * @brief 获取面事件统计信息
     * @param result 面事件规划结果
     * @return 统计信息字符串
     */
    QString getAreaStatistics(const AreaPlanningResult& result) const;

public slots:
    /**
     * @brief 异步执行网格路径规划
     * @param params 规划参数
     */
    void asyncExecutePlanning(const GridPlanningParameters& params);

    /**
     * @brief 异步执行线事件路径规划
     * @param params 线事件规划参数
     */
    void asyncExecuteLinePlanning(const LinePlanningParameters& params);

    /**
     * @brief 异步执行面事件路径规划
     * @param params 面事件规划参数
     */
    void asyncExecuteAreaPlanning(const AreaPlanningParameters& params);

signals:
    /**
     * @brief 规划进度更新信号
     * @param percentage 完成百分比 (0-100)
     * @param message 当前步骤描述
     */
    void progressUpdated(int percentage, const QString& message);

    /**
     * @brief 规划完成信号
     * @param result 规划结果
     */
    void planningCompleted(const GridPlanningResult& result);

    /**
     * @brief 线事件规划完成信号
     * @param result 线事件规划结果
     */
    void linePlanningCompleted(const LinePlanningResult& result);

    /**
     * @brief 面事件规划完成信号
     * @param result 面事件规划结果
     */
    void areaPlanningCompleted(const AreaPlanningResult& result);

    /**
     * @brief 规划失败信号
     * @param errorMessage 错误信息
     */
    void planningFailed(const QString& errorMessage);

private:
    ShapefileHandler* mShapefileHandler;  // SHP文件处理器
    QVector<GridNode> mGridNodes;         // 网格节点
    QVector<QVector3D> mRiskPoints;       // 风险点
    QVector<RiskLineEvent> mRiskLines;    // 风险线事件
    QHash<int, int> mNodeIdToIndex;       // 节点ID到索引的映射
    QVector<QVector<double>> mDistanceMatrix; // 距离矩阵（用于TSP）

    /**
     * @brief 从渔网线SHP文件构建网格
     * @param shapefile SHP文件路径
     * @return 成功返回true
     */
    bool buildGridFromFishnet(const QString& shapefile);

    /**
     * @brief 加载风险点数据
     * @param shapefile SHP文件路径
     * @return 成功返回true
     */
    bool loadRiskPoints(const QString& shapefile);

    /**
     * @brief 将风险点关联到最近的网格节点
     * @param maxDistance 最大关联距离
     */
    void associateRiskPointsToGrid(double maxDistance);

    /**
     * @brief 使用Dijkstra算法计算两点间最短路径
     * @param startNodeId 起始节点ID
     * @param endNodeId 目标节点ID
     * @return 路径节点ID序列
     */
    QVector<int> dijkstraPath(int startNodeId, int endNodeId);

    /**
     * @brief 使用A*算法计算两点间最短路径
     * @param startNodeId 起始节点ID
     * @param endNodeId 目标节点ID
     * @return 路径节点ID序列
     */
    QVector<int> aStarPath(int startNodeId, int endNodeId);

    /**
     * @brief 解决TSP问题（访问所有风险点的最短路径）
     * @param startNodeId 起始节点ID
     * @return 访问所有风险点的路径
     */
    QVector<PathPoint> solveTSPWithDijkstra(int startNodeId);

    /**
     * @brief 优化的贪心TSP算法（适用于大规模问题）
     * @param startNodeId 起始节点ID
     * @param riskNodeIds 风险节点ID列表
     * @return 访问所有风险点的路径
     */
    QVector<PathPoint> solveOptimizedGreedyTSP(int startNodeId, const QVector<int>& riskNodeIds);

    /**
     * @brief 使用最近邻算法求解TSP
     * @param startNodeId 起始节点ID
     * @param riskNodeIds 风险节点ID列表
     * @return 访问顺序（节点ID序列）
     */
    QVector<int> nearestNeighborTSP(int startNodeId, const QVector<int>& riskNodeIds);

    /**
     * @brief 2-opt算法优化TSP路径
     * @param tour TSP路径
     * @param riskNodeIds 风险节点ID列表
     * @return 优化后的路径
     */
    QVector<int> twoOptTSP(const QVector<int>& tour, const QVector<int>& riskNodeIds);

    /**
     * @brief 构建风险点间的距离矩阵
     * @param riskNodeIds 风险节点ID列表
     */
    void buildRiskPointDistanceMatrix(const QVector<int>& riskNodeIds);

    /**
     * @brief 计算两点间的欧氏距离
     * @param p1 点1
     * @param p2 点2
     * @return 距离
     */
    double calculateDistance(const QVector3D& p1, const QVector3D& p2) const;

    /**
     * @brief 计算A*算法的启发式距离
     * @param nodeId1 节点1 ID
     * @param nodeId2 节点2 ID
     * @return 启发式距离
     */
    double heuristicDistance(int nodeId1, int nodeId2) const;

    /**
     * @brief 找到距离给定位置最近的网格节点
     * @param position 位置
     * @return 最近节点ID，找不到返回-1
     */
    int findNearestGridNode(const QVector3D& position) const;

    /**
     * @brief 获取包含风险点的网格节点ID列表
     * @return 风险节点ID列表
     */
    QVector<int> getRiskNodeIds() const;

    /**
     * @brief 将节点ID路径转换为路径点
     * @param nodeIds 节点ID序列
     * @return 路径点序列
     */
    QVector<PathPoint> convertToPathPoints(const QVector<int>& nodeIds) const;

    /**
     * @brief 清除所有数据
     */
    void clear();

    /**
     * @brief 获取从指定节点可达的所有节点（连通分量）
     * @param startNodeId 起始节点ID
     * @return 连通分量中的所有节点ID
     */
    QSet<int> getConnectedComponent(int startNodeId);

    /**
     * @brief 找到网格中的所有连通分量
     * @return 所有连通分量的列表
     */
    QVector<QSet<int>> findConnectedComponents();

    /**
     * @brief 修复网格连通性，通过连接距离很近的节点
     * @return 添加的连接数量
     */
    int repairGridConnectivity();

    /**
     * @brief 加载风险线事件数据
     * @param shapefile SHP文件路径
     * @return 成功返回true
     */
    bool loadRiskLines(const QString& shapefile);

    /**
     * @brief 将风险线关联到网格节点
     * @param maxDistance 最大关联距离
     */
    void associateRiskLinesToGrid(double maxDistance);

    /**
     * @brief 为风险线寻找网格路径
     * @param riskLine 风险线事件
     * @return 网格节点ID序列
     */
    QVector<int> findGridPathForLine(const RiskLineEvent& riskLine);

    /**
     * @brief 解决线事件TSP问题（按顺序访问所有风险线）
     * @param startNodeId 起始节点ID
     * @return 访问所有风险线的路径
     */
    QVector<PathPoint> solveLineTSP(int startNodeId);

    /**
     * @brief 优化的线事件贪心算法
     * @param startNodeId 起始节点ID
     * @param riskLines 风险线列表
     * @return 访问所有风险线的路径
     */
    QVector<PathPoint> solveOptimizedLineGreedy(int startNodeId, const QVector<RiskLineEvent>& riskLines);

    /**
     * @brief 解决面事件螺旋覆盖算法
     * @param startNodeId 起始节点ID
     * @param coverageThreshold 覆盖率阈值
     * @return 覆盖路径
     */
    QVector<PathPoint> solveAreaSpiralCoverage(int startNodeId, double coverageThreshold);

    /**
     * @brief 解决面事件网格覆盖算法
     * @param startNodeId 起始节点ID
     * @param coverageThreshold 覆盖率阈值
     * @return 覆盖路径
     */
    QVector<PathPoint> solveAreaGridCoverage(int startNodeId, double coverageThreshold);

    /**
     * @brief 计算网格覆盖率
     * @param visitedNodes 已访问节点集合
     * @return 覆盖率（0.0-1.0）
     */
    double calculateCoverageRate(const QSet<int>& visitedNodes) const;

    /**
     * @brief 找到未覆盖区域的中心点
     * @param visitedNodes 已访问节点集合
     * @return 未覆盖区域中心点列表
     */
    QVector<QVector3D> findUncoveredAreas(const QSet<int>& visitedNodes) const;

    /**
     * @brief 构建覆盖网格
     * @param gridSpacing 网格间距
     * @return 覆盖网格节点
     */
    QVector<GridNode> buildCoverageGrid(double gridSpacing);

    /**
     * @brief 构建网格连接
     */
    void buildGridConnections();

    /**
     * @brief 寻找下一个螺旋节点
     * @param currentNodeId 当前节点ID
     * @param visitedNodes 已访问节点集合
     * @param availableNodes 可用节点集合
     * @return 下一个节点ID
     */
    int findNextSpiralNode(int currentNodeId, const QSet<int>& visitedNodes, const QSet<int>& availableNodes);

    /**
     * @brief 寻找下一个网格节点
     * @param currentNodeId 当前节点ID
     * @param visitedNodes 已访问节点集合
     * @return 下一个节点ID
     */
    int findNextGridNode(int currentNodeId, const QSet<int>& visitedNodes);
};

#endif // GRID_PATH_PLANNER_H 