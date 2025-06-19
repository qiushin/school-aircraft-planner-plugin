/****************************************************************************
File: RouteVisualization.h
Author: AI Assistant
Date: 2025.1.6
Description: 路径可视化模块，用于在2D视图中显示规划结果
****************************************************************************/

#ifndef ROUTE_VISUALIZATION_H
#define ROUTE_VISUALIZATION_H

#include <QObject>
#include <QVector>
#include <QVector3D>
#include <QPolygonF>
#include <QPainter>
#include <QPen>
#include <QBrush>
#include <QColor>
#include <QRectF>
#include "../core/RiskEventPlanner.h"
#include "../core/TriangulationHandler.h"

/**
 * @brief 可视化样式配置
 */
struct VisualizationStyle {
    // 路径样式
    QColor pathColor;           // 路径颜色
    int pathWidth;              // 路径宽度
    Qt::PenStyle pathStyle;     // 路径样式
    
    // 风险点样式
    QColor riskPointColor;      // 风险点颜色
    int riskPointSize;          // 风险点大小
    QColor riskPointBorderColor; // 风险点边框颜色
    int riskPointBorderWidth;   // 风险点边框宽度
    
    // 起始点样式
    QColor startPointColor;     // 起始点颜色
    int startPointSize;         // 起始点大小
    
    // 可飞行区域样式
    QColor flightZoneColor;     // 可飞行区域颜色
    QColor flightZoneBorderColor; // 可飞行区域边框颜色
    int flightZoneBorderWidth;  // 可飞行区域边框宽度
    int flightZoneAlpha;        // 可飞行区域透明度
    
    // 三角网样式
    QColor triangulationColor;  // 三角网颜色
    int triangulationWidth;     // 三角网线宽
    int triangulationAlpha;     // 三角网透明度
    bool showTriangulation;     // 是否显示三角网
    
    // 默认构造函数，设置默认样式
    VisualizationStyle() {
        pathColor = QColor(255, 0, 0);          // 红色路径
        pathWidth = 3;
        pathStyle = Qt::SolidLine;
        
        riskPointColor = QColor(255, 255, 0);   // 黄色风险点
        riskPointSize = 8;
        riskPointBorderColor = QColor(0, 0, 0); // 黑色边框
        riskPointBorderWidth = 2;
        
        startPointColor = QColor(0, 255, 0);    // 绿色起始点
        startPointSize = 12;
        
        flightZoneColor = QColor(0, 255, 0);    // 绿色可飞行区域
        flightZoneBorderColor = QColor(0, 150, 0); // 深绿色边框
        flightZoneBorderWidth = 2;
        flightZoneAlpha = 50;                   // 半透明
        
        triangulationColor = QColor(100, 100, 100); // 灰色三角网
        triangulationWidth = 1;
        triangulationAlpha = 100;
        showTriangulation = true;
    }
};

/**
 * @brief 视图变换参数
 */
struct ViewTransform {
    QRectF worldBounds;     // 世界坐标边界
    QRectF viewBounds;      // 视图坐标边界
    double scale;           // 缩放比例
    QPointF offset;         // 偏移量
    
    ViewTransform() : scale(1.0), offset(0, 0) {}
};

/**
 * @brief 路径可视化类
 */
class RouteVisualization : public QObject {
    Q_OBJECT

public:
    explicit RouteVisualization(QObject *parent = nullptr);
    ~RouteVisualization();

    /**
     * @brief 设置规划结果数据
     * @param result 规划结果
     */
    void setPlanningResult(const PlanningResult& result);

    /**
     * @brief 设置可视化样式
     * @param style 样式配置
     */
    void setVisualizationStyle(const VisualizationStyle& style);

    /**
     * @brief 获取当前样式
     * @return 样式配置
     */
    const VisualizationStyle& getVisualizationStyle() const { return mStyle; }

    /**
     * @brief 渲染到指定的绘制设备
     * @param painter 绘制器
     * @param viewRect 视图矩形
     */
    void render(QPainter& painter, const QRectF& viewRect);

    /**
     * @brief 计算适合的视图边界
     * @return 视图边界
     */
    QRectF calculateOptimalViewBounds() const;

    /**
     * @brief 设置视图变换
     * @param transform 变换参数
     */
    void setViewTransform(const ViewTransform& transform);

    /**
     * @brief 世界坐标转视图坐标
     * @param worldPoint 世界坐标点
     * @return 视图坐标点
     */
    QPointF worldToView(const QVector3D& worldPoint) const;

    /**
     * @brief 视图坐标转世界坐标
     * @param viewPoint 视图坐标点
     * @return 世界坐标点
     */
    QVector3D viewToWorld(const QPointF& viewPoint) const;

    /**
     * @brief 获取数据边界
     * @return 数据边界矩形
     */
    QRectF getDataBounds() const { return mDataBounds; }

    /**
     * @brief 检查是否有数据可显示
     * @return 有数据返回true，否则返回false
     */
    bool hasData() const;

    /**
     * @brief 导出为图像文件
     * @param filePath 文件路径
     * @param size 图像尺寸
     * @return 成功返回true，失败返回false
     */
    bool exportToImage(const QString& filePath, const QSize& size = QSize(1920, 1080));

    /**
     * @brief 清除所有数据
     */
    void clear();

signals:
    /**
     * @brief 渲染完成信号
     */
    void renderCompleted();

    /**
     * @brief 数据更新信号
     */
    void dataUpdated();

private:
    PlanningResult mResult;         // 规划结果
    VisualizationStyle mStyle;      // 可视化样式
    ViewTransform mViewTransform;   // 视图变换
    QRectF mDataBounds;            // 数据边界

    /**
     * @brief 绘制可飞行区域
     * @param painter 绘制器
     */
    void drawFlightZones(QPainter& painter);

    /**
     * @brief 绘制三角网
     * @param painter 绘制器
     */
    void drawTriangulation(QPainter& painter);

    /**
     * @brief 绘制风险事件点
     * @param painter 绘制器
     */
    void drawRiskEventPoints(QPainter& painter);

    /**
     * @brief 绘制路径
     * @param painter 绘制器
     */
    void drawOptimalPath(QPainter& painter);

    /**
     * @brief 绘制起始点
     * @param painter 绘制器
     */
    void drawStartPoint(QPainter& painter);

    /**
     * @brief 绘制图例
     * @param painter 绘制器
     * @param legendRect 图例位置
     */
    void drawLegend(QPainter& painter, const QRectF& legendRect);

    /**
     * @brief 计算数据边界
     */
    void calculateDataBounds();

    /**
     * @brief 设置绘制器样式
     * @param painter 绘制器
     * @param color 颜色
     * @param width 线宽
     * @param style 线型
     * @param alpha 透明度
     */
    void setupPainter(QPainter& painter, const QColor& color, int width = 1, 
                     Qt::PenStyle style = Qt::SolidLine, int alpha = 255);

    /**
     * @brief 创建画刷
     * @param color 颜色
     * @param alpha 透明度
     * @return 画刷
     */
    QBrush createBrush(const QColor& color, int alpha = 255);

    /**
     * @brief 多边形转换为视图坐标
     * @param worldPolygon 世界坐标多边形
     * @return 视图坐标多边形
     */
    QPolygonF worldPolygonToView(const QPolygonF& worldPolygon) const;
};

#endif // ROUTE_VISUALIZATION_H 