/****************************************************************************
File: RiskEventPlanner.h
Author: AI Assistant
Date: 2025.1.6
Description: 校园风险点事件无人机巡查路径规划主类
****************************************************************************/

#ifndef RISK_EVENT_PLANNER_H
#define RISK_EVENT_PLANNER_H

#include <QObject>
#include <QVector>
#include <QVector3D>
#include <QString>
#include <QPolygonF>
#include "ShapefileHandler.h"
#include "TriangulationHandler.h"
#include "PathOptimizer.h"

/**
 * @brief 规划参数结构
 */
struct PlanningParameters {
    QString flightZoneShapefile;        // 可飞行区域SHP文件路径
    QString riskEventShapefile;         // 风险事件点SHP文件路径
    QString outputPath;                 // 输出路径
    QVector3D startPoint;               // 起始点
    double triangulationSpacing;        // 三角网点间距
    QString optimizationAlgorithm;      // 优化算法类型
    
    PlanningParameters() : 
        startPoint(558856.516f, 3371566.848f, 50.0f),
        triangulationSpacing(10.0), 
        optimizationAlgorithm("NearestNeighbor") {}
};

/**
 * @brief 规划结果结构
 */
struct PlanningResult {
    bool success;                                    // 是否成功
    QString errorMessage;                            // 错误信息
    QVector<PathNode> optimalPath;                   // 最优路径
    QVector<Triangle> triangulationTriangles;       // 三角网
    QVector<QPolygonF> flightZones;                  // 可飞行区域
    QVector<QVector3D> riskEventPoints;              // 风险事件点
    double totalPathLength;                          // 总路径长度
    int triangleCount;                               // 三角形数量
    int riskEventCount;                              // 风险事件点数量
    
    PlanningResult() : success(false), totalPathLength(0.0), triangleCount(0), riskEventCount(0) {}
};

/**
 * @brief 校园风险点事件无人机巡查路径规划主类
 */
class RiskEventPlanner : public QObject {
    Q_OBJECT

public:
    explicit RiskEventPlanner(QObject *parent = nullptr);
    ~RiskEventPlanner();

    /**
     * @brief 执行完整的路径规划流程
     * @param params 规划参数
     * @return 规划结果
     */
    PlanningResult executePlanning(const PlanningParameters& params);

    /**
     * @brief 设置规划参数
     * @param params 规划参数
     */
    void setPlanningParameters(const PlanningParameters& params);

    /**
     * @brief 获取当前规划结果
     * @return 规划结果
     */
    const PlanningResult& getPlanningResult() const { return mResult; }

    /**
     * @brief 导出路径到SHP文件
     * @param outputPath 输出文件路径
     * @return 成功返回true，失败返回false
     */
    bool exportPathToShapefile(const QString& outputPath);

    /**
     * @brief 导出三角网到SHP文件
     * @param outputPath 输出文件路径
     * @return 成功返回true，失败返回false
     */
    bool exportTriangulationToShapefile(const QString& outputPath);

    /**
     * @brief 获取统计信息
     * @return 统计信息字符串
     */
    QString getStatistics() const;

    /**
     * @brief 清除所有数据
     */
    void clear();

    /**
     * @brief 验证输入参数
     * @param params 待验证的参数
     * @return 验证结果和错误信息
     */
    QPair<bool, QString> validateParameters(const PlanningParameters& params);

public slots:
    /**
     * @brief 异步执行路径规划
     * @param params 规划参数
     */
    void asyncExecutePlanning(const PlanningParameters& params);

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
    void planningCompleted(const PlanningResult& result);

    /**
     * @brief 规划失败信号
     * @param errorMessage 错误信息
     */
    void planningFailed(const QString& errorMessage);

private:
    PlanningParameters mParams;          // 规划参数
    PlanningResult mResult;              // 规划结果
    
    ShapefileHandler* mShapefileHandler;      // SHP文件处理器
    TriangulationHandler* mTriangulationHandler;  // 三角网处理器
    PathOptimizer* mPathOptimizer;            // 路径优化器

    /**
     * @brief 步骤1：加载数据文件
     * @return 成功返回true，失败返回false
     */
    bool loadDataFiles();

    /**
     * @brief 步骤2：生成三角网
     * @return 成功返回true，失败返回false
     */
    bool generateTriangulation();

    /**
     * @brief 步骤3：优化路径
     * @return 成功返回true，失败返回false
     */
    bool optimizePath();

    /**
     * @brief 步骤4：后处理和验证
     * @return 成功返回true，失败返回false
     */
    bool postProcessing();

    /**
     * @brief 创建输出目录
     * @param path 目录路径
     * @return 成功返回true，失败返回false
     */
    bool createOutputDirectory(const QString& path);

    /**
     * @brief 生成详细的结果报告
     * @return 报告内容
     */
    QString generateDetailedReport() const;
};

#endif // RISK_EVENT_PLANNER_H 