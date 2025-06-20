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
    
    PathPoint() : riskPointId(-1), isRiskPoint(false), distanceFromStart(0.0) {}
    PathPoint(const QVector3D& pos) 
        : position(pos), riskPointId(-1), isRiskPoint(false), distanceFromStart(0.0) {}
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
 * @brief 网格路径规划结果
 */
struct GridPlanningResult {
    bool success;                        // 是否成功
    QString errorMessage;                // 错误信息
    QVector<PathPoint> optimalPath;      // 最优路径
    QVector<GridNode> gridNodes;         // 网格节点
    QVector<QVector3D> riskPoints;       // 风险点
    double totalPathLength;              // 总路径长度
    int visitedRiskPoints;               // 访问的风险点数量
    int totalRiskPoints;                 // 总风险点数量
    QString algorithm;                   // 使用的算法
    
    GridPlanningResult() : success(false), totalPathLength(0.0), 
                          visitedRiskPoints(0), totalRiskPoints(0) {}
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
     * @brief 验证规划参数
     * @param params 待验证的参数
     * @return 验证结果和错误信息
     */
    QPair<bool, QString> validateParameters(const GridPlanningParameters& params);

    /**
     * @brief 导出路径到文件
     * @param result 规划结果
     * @param outputPath 输出路径
     * @return 成功返回true
     */
    bool exportPathToFiles(const GridPlanningResult& result, const QString& outputPath);

    /**
     * @brief 获取统计信息
     * @param result 规划结果
     * @return 统计信息字符串
     */
    QString getStatistics(const GridPlanningResult& result) const;

public slots:
    /**
     * @brief 异步执行路径规划
     * @param params 规划参数
     */
    void asyncExecutePlanning(const GridPlanningParameters& params);

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
     * @brief 规划失败信号
     * @param errorMessage 错误信息
     */
    void planningFailed(const QString& errorMessage);

private:
    ShapefileHandler* mShapefileHandler;  // SHP文件处理器
    QVector<GridNode> mGridNodes;         // 网格节点
    QVector<QVector3D> mRiskPoints;       // 风险点
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
};

#endif // GRID_PATH_PLANNER_H 