
#ifndef PATH_OPTIMIZER_H
#define PATH_OPTIMIZER_H

#include <QVector>
#include <QVector3D>
#include <QPolygonF>
#include <QPair>
#include "TriangulationHandler.h"

/**
 * @brief 路径节点结构
 */
struct PathNode {
    QVector3D position;     // 节点位置
    int riskEventId;        // 关联的风险事件ID（-1表示非风险事件点）
    bool isRiskEvent;       // 是否为风险事件点
    
    PathNode() : riskEventId(-1), isRiskEvent(false) {}
    PathNode(const QVector3D& pos, int id = -1, bool isRisk = false) 
        : position(pos), riskEventId(id), isRiskEvent(isRisk) {}
};

/**
 * @brief 路径优化器类
 * 负责生成经过所有风险事件点的最短路径
 */
class PathOptimizer {
public:
    PathOptimizer();
    ~PathOptimizer();

    /**
     * @brief 设置风险事件点
     * @param riskPoints 风险事件点集合
     */
    void setRiskEventPoints(const QVector<QVector3D>& riskPoints);

    /**
     * @brief 设置三角网数据（用于路径约束）
     * @param triangles 裁剪后的三角形集合
     */
    void setTriangulationData(const QVector<Triangle>& triangles);

    /**
     * @brief 设置可飞行区域约束
     * @param flightZones 可飞行区域多边形
     */
    void setFlightZoneConstraints(const QVector<QPolygonF>& flightZones);

    /**
     * @brief 生成最优巡查路径
     * @param startPoint 起始点
     * @param algorithm 优化算法类型
     * @return 优化后的路径
     */
    QVector<PathNode> generateOptimalPath(const QVector3D& startPoint, 
                                         const QString& algorithm = "NearestNeighbor");

    /**
     * @brief 使用最近邻算法求解TSP
     * @param startPoint 起始点
     * @return 路径节点序列
     */
    QVector<PathNode> nearestNeighborTSP(const QVector3D& startPoint);

    /**
     * @brief 使用2-opt改进算法优化路径
     * @param path 待优化的路径
     * @return 优化后的路径
     */
    QVector<PathNode> twoOptImprovement(const QVector<PathNode>& path);

    /**
     * @brief 使用遗传算法求解TSP（可选的高级算法）
     * @param startPoint 起始点
     * @param generations 迭代代数
     * @param populationSize 种群大小
     * @return 路径节点序列
     */
    QVector<PathNode> geneticAlgorithmTSP(const QVector3D& startPoint, 
                                         int generations = 100, 
                                         int populationSize = 50);

    /**
     * @brief 检查两点间是否有直线路径（无障碍）
     * @param from 起点
     * @param to 终点
     * @return 可直达返回true，否则返回false
     */
    bool hasDirectPath(const QVector3D& from, const QVector3D& to);

    /**
     * @brief 在约束条件下寻找两点间的路径
     * @param from 起点
     * @param to 终点
     * @return 路径上的中间点（不包括起点和终点）
     */
    QVector<QVector3D> findConstrainedPath(const QVector3D& from, const QVector3D& to);

    /**
     * @brief 计算路径总长度
     * @param path 路径节点序列
     * @return 总长度
     */
    double calculatePathLength(const QVector<PathNode>& path);

    /**
     * @brief 获取距离矩阵
     * @return 点对点距离矩阵
     */
    const QVector<QVector<double>>& getDistanceMatrix() const { return mDistanceMatrix; }

    /**
     * @brief 清除所有数据
     */
    void clear();

private:
    QVector<QVector3D> mRiskEventPoints;      // 风险事件点
    QVector<Triangle> mTriangles;             // 三角网约束
    QVector<QPolygonF> mFlightZones;          // 可飞行区域
    QVector<QVector<double>> mDistanceMatrix; // 距离矩阵
    QVector<PathNode> mAllNodes;              // 所有路径节点

    /**
     * @brief 计算两点间的欧氏距离
     * @param p1 点1
     * @param p2 点2
     * @return 距离
     */
    double calculateDistance(const QVector3D& p1, const QVector3D& p2);

    /**
     * @brief 构建距离矩阵
     */
    void buildDistanceMatrix();

    /**
     * @brief 检查线段是否与可飞行区域相交
     * @param from 起点
     * @param to 终点
     * @return 相交返回true，否则返回false
     */
    bool lineIntersectsFlightZone(const QVector3D& from, const QVector3D& to);

    /**
     * @brief 使用A*算法寻路（在三角网约束下）
     * @param start 起点
     * @param goal 终点
     * @return 路径点序列
     */
    QVector<QVector3D> aStarPathfinding(const QVector3D& start, const QVector3D& goal);

    /**
     * @brief 获取三角形的邻接三角形
     * @param triangle 当前三角形
     * @return 邻接三角形列表
     */
    QVector<Triangle> getAdjacentTriangles(const Triangle& triangle);

    /**
     * @brief 生成随机路径（用于遗传算法）
     * @param startIndex 起始点索引
     * @return 随机路径
     */
    QVector<int> generateRandomPath(int startIndex);

    /**
     * @brief 路径交叉操作（遗传算法）
     * @param parent1 父路径1
     * @param parent2 父路径2
     * @return 子路径
     */
    QVector<int> crossoverPaths(const QVector<int>& parent1, const QVector<int>& parent2);

    /**
     * @brief 路径变异操作（遗传算法）
     * @param path 待变异路径
     * @param mutationRate 变异率
     * @return 变异后路径
     */
    QVector<int> mutatePath(const QVector<int>& path, double mutationRate = 0.1);
};

#endif // PATH_OPTIMIZER_H 