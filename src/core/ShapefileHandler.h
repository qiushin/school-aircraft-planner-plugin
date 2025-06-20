#ifndef SHAPEFILE_HANDLER_H
#define SHAPEFILE_HANDLER_H

#include <QVector>
#include <QVector3D>
#include <QPolygonF>
#include <QString>
#include <QPointF>

// QGIS相关头文件
#include <qgsgeometry.h>
#include <qgspoint.h>
#include <qgspolygon.h>
#include <qgsvectorlayer.h>
#include <qgsfeature.h>
#include <qgsfeatureiterator.h>
#include <qgswkbtypes.h>
#include <qgsabstractgeometry.h>
#include <qgslinestring.h>

/**
 * @brief SHP文件处理器类
 * 负责读取和处理SHP格式的地理数据文件
 */
class ShapefileHandler {
public:
    ShapefileHandler();
    ~ShapefileHandler();

    /**
     * @brief 加载可飞行区域面数据
     * @param filePath SHP文件路径
     * @return 成功返回true，失败返回false
     */
    bool loadFlightZonePolygon(const QString& filePath);

    /**
     * @brief 加载风险事件点数据
     * @param filePath SHP文件路径
     * @return 成功返回true，失败返回false
     */
    bool loadRiskEventPoints(const QString& filePath);

    /**
     * @brief 加载风险线事件数据
     * @param filePath 文件路径
     * @return 成功返回true
     */
    bool loadRiskLineEvents(const QString& filePath);

    /**
     * @brief 加载渔网线数据（线状几何）
     * @param filePath SHP文件路径
     * @return 成功返回true，失败返回false
     */
    bool loadFishnetLines(const QString& filePath);

    /**
     * @brief 获取可飞行区域多边形
     * @return 可飞行区域多边形点集
     */
    const QVector<QPolygonF>& getFlightZonePolygons() const { return mFlightZonePolygons; }

    /**
     * @brief 获取风险事件点列表
     * @return 风险事件点列表
     */
    QVector<QVector3D> getRiskEventPoints() const;

    /**
     * @brief 获取风险线事件列表
     * @return 风险线事件列表
     */
    QVector<QPair<QVector3D, QVector3D>> getRiskLineEvents() const;

    /**
     * @brief 获取渔网线集合
     * @return 渔网线集合，每条线由起点和终点组成
     */
    const QVector<QPair<QVector3D, QVector3D>>& getFishnetLines() const { return mFishnetLines; }

    /**
     * @brief 检查点是否在可飞行区域内
     * @param point 待检查的点
     * @return 在区域内返回true，否则返回false
     */
    bool isPointInFlightZone(const QVector3D& point) const;

    /**
     * @brief 获取可飞行区域边界框
     * @return 边界框
     */
    QRectF getFlightZoneBounds() const;

    /**
     * @brief 清除所有数据
     */
    void clear();

private:
    QVector<QPolygonF> mFlightZonePolygons;  // 可飞行区域多边形集合
    QVector<QVector3D> mRiskEventPoints;     // 风险事件点集合
    QVector<QPair<QVector3D, QVector3D>> mRiskLineEvents; // 风险线事件（起点-终点对）
    QVector<QPair<QVector3D, QVector3D>> mFishnetLines;   // 渔网线集合
    QRectF mFlightZoneBounds;                // 可飞行区域边界框

    /**
     * @brief 从QGIS几何对象转换为Qt多边形
     * @param geometry QGIS几何对象
     * @return Qt多边形
     */
    QPolygonF geometryToPolygon(const QgsGeometry& geometry);

    /**
     * @brief 从QGIS点转换为QVector3D
     * @param point QGIS点
     * @return QVector3D点
     */
    QVector3D qgsPointToVector3D(const QgsPoint& point);
};

#endif // SHAPEFILE_HANDLER_H 