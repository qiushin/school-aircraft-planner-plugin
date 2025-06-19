#ifndef TRIANGULATION_HANDLER_H
#define TRIANGULATION_HANDLER_H

#include <QVector>
#include <QVector3D>
#include <QPolygonF>
#include <QRectF>
#include <qgsgeometry.h>
#include <qgspoint.h>
#include <qgswkbtypes.h>
#include <qgsfeature.h>

class QgsTriangulation;

/**
 * @brief 三角形结构
 */
struct Triangle {
    QVector3D p1, p2, p3;  // 三角形的三个顶点
    QVector3D center;      // 三角形中心点
    
    Triangle() = default;
    Triangle(const QVector3D& point1, const QVector3D& point2, const QVector3D& point3)
        : p1(point1), p2(point2), p3(point3) {
        center = (p1 + p2 + p3) / 3.0f;
    }
};

/**
 * @brief 三角网生成和处理器类
 */
class TriangulationHandler {
public:
    TriangulationHandler();
    ~TriangulationHandler();

    /**
     * @brief 在指定区域内生成均匀分布的点
     * @param bounds 区域边界框
     * @param spacing 点间距
     * @param flightZonePolygons 可飞行区域多边形（用于过滤）
     * @return 生成的点集
     */
    QVector<QVector3D> generateUniformPoints(const QRectF& bounds, 
                                             double spacing,
                                             const QVector<QPolygonF>& flightZonePolygons);

    /**
     * @brief 对点集进行Delaunay三角剖分
     * @param points 输入点集
     * @return 成功返回true，失败返回false
     */
    bool performDelaunayTriangulation(const QVector<QVector3D>& points);

    /**
     * @brief 将三角网与可飞行区域进行裁剪
     * @param flightZonePolygons 可飞行区域多边形
     * @return 裁剪后的三角形集合
     */
    QVector<Triangle> clipTriangulationWithFlightZone(const QVector<QPolygonF>& flightZonePolygons);

    /**
     * @brief 获取原始三角形集合
     * @return 三角形集合
     */
    const QVector<Triangle>& getTriangles() const { return mTriangles; }

    /**
     * @brief 获取裁剪后的三角形集合
     * @return 裁剪后的三角形集合
     */
    const QVector<Triangle>& getClippedTriangles() const { return mClippedTriangles; }

    /**
     * @brief 获取三角网的所有边
     * @return 边的集合（每条边用两个点表示）
     */
    QVector<QPair<QVector3D, QVector3D>> getTriangulationEdges() const;

    /**
     * @brief 检查三角形是否与多边形相交
     * @param triangle 三角形
     * @param polygon 多边形
     * @return 相交返回true，否则返回false
     */
    static bool triangleIntersectsPolygon(const Triangle& triangle, const QPolygonF& polygon);

    /**
     * @brief 检查三角形是否完全在多边形内
     * @param triangle 三角形
     * @param polygon 多边形
     * @return 完全在内返回true，否则返回false
     */
    static bool triangleInPolygon(const Triangle& triangle, const QPolygonF& polygon);

    /**
     * @brief 清除所有数据
     */
    void clear();

private:
    QVector<QVector3D> mPoints;           // 输入点集
    QVector<Triangle> mTriangles;         // 原始三角形集合
    QVector<Triangle> mClippedTriangles;  // 裁剪后的三角形集合

    /**
     * @brief 检查点是否在任意一个可飞行区域内
     * @param point 待检查的点
     * @param polygons 可飞行区域多边形集合
     * @return 在区域内返回true，否则返回false
     */
    bool isPointInAnyPolygon(const QVector3D& point, const QVector<QPolygonF>& polygons);

    /**
     * @brief 简单的三角剖分实现（当QGIS mesh功能不可用时）
     * @param points 输入点集
     */
    void performSimpleTriangulation(const QVector<QVector3D>& points);
};

#endif // TRIANGULATION_HANDLER_H 