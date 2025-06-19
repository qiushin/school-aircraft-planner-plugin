#include "TriangulationHandler.h"
#include "../log/QgisDebug.h"
#include <QLineF>
#include <QtMath>
#include <algorithm>

TriangulationHandler::TriangulationHandler() {
    // 构造函数
}

TriangulationHandler::~TriangulationHandler() {
    clear();
}

QVector<QVector3D> TriangulationHandler::generateUniformPoints(const QRectF& bounds, 
                                                               double spacing,
                                                               const QVector<QPolygonF>& flightZonePolygons) {
    QVector<QVector3D> points;
    
    if (spacing <= 0) {
        logMessage("point spacing must be greater than 0", Qgis::MessageLevel::Warning);
        return points;
    }

    // 在边界框内生成网格点
    for (double x = bounds.left(); x <= bounds.right(); x += spacing) {
        for (double y = bounds.bottom(); y <= bounds.top(); y += spacing) {
            QVector3D point(static_cast<float>(x), static_cast<float>(y), 0.0f);
            
            // 检查点是否在可飞行区域内
            if (isPointInAnyPolygon(point, flightZonePolygons)) {
                points.append(point);
            }
        }
    }

    logMessage(QString("generated %1 uniform distributed points in flight zone").arg(points.size()), 
               Qgis::MessageLevel::Info);
    return points;
}

bool TriangulationHandler::performDelaunayTriangulation(const QVector<QVector3D>& points) {
    mPoints = points;
    mTriangles.clear();

    if (points.size() < 3) {
        logMessage("at least 3 points are required for triangulation", Qgis::MessageLevel::Warning);
        return false;
    }

    try {
        // 使用简化的三角剖分实现
        performSimpleTriangulation(points);

        logMessage(QString("successfully generated %1 triangles").arg(mTriangles.size()), 
                   Qgis::MessageLevel::Success);
        return true;

    } catch (const std::exception& e) {
        logMessage(QString("triangulation failed: %1").arg(e.what()), Qgis::MessageLevel::Critical);
        return false;
    }
}

QVector<Triangle> TriangulationHandler::clipTriangulationWithFlightZone(const QVector<QPolygonF>& flightZonePolygons) {
    mClippedTriangles.clear();

    for (const Triangle& triangle : mTriangles) {
        // 检查三角形是否与任何可飞行区域相交或在其内部
        bool keepTriangle = false;
        for (const QPolygonF& polygon : flightZonePolygons) {
            if (triangleInPolygon(triangle, polygon) || triangleIntersectsPolygon(triangle, polygon)) {
                keepTriangle = true;
                break;
            }
        }

        if (keepTriangle) {
            mClippedTriangles.append(triangle);
        }
    }

    logMessage(QString("after clipping, %1 triangles are retained").arg(mClippedTriangles.size()), 
               Qgis::MessageLevel::Info);
    return mClippedTriangles;
}

QVector<QPair<QVector3D, QVector3D>> TriangulationHandler::getTriangulationEdges() const {
    QVector<QPair<QVector3D, QVector3D>> edges;
    
    for (const Triangle& triangle : mClippedTriangles) {
        // 添加三角形的三条边
        edges.append(qMakePair(triangle.p1, triangle.p2));
        edges.append(qMakePair(triangle.p2, triangle.p3));
        edges.append(qMakePair(triangle.p3, triangle.p1));
    }

    return edges;
}

bool TriangulationHandler::triangleIntersectsPolygon(const Triangle& triangle, const QPolygonF& polygon) {
    // 创建三角形的Qt多边形
    QPolygonF trianglePolygon;
    trianglePolygon << QPointF(triangle.p1.x(), triangle.p1.y())
                   << QPointF(triangle.p2.x(), triangle.p2.y())
                   << QPointF(triangle.p3.x(), triangle.p3.y());

    // 检查多边形是否相交
    return trianglePolygon.intersected(polygon).size() > 0;
}

bool TriangulationHandler::triangleInPolygon(const Triangle& triangle, const QPolygonF& polygon) {
    // 检查三角形的所有顶点是否都在多边形内
    QPointF p1(triangle.p1.x(), triangle.p1.y());
    QPointF p2(triangle.p2.x(), triangle.p2.y());
    QPointF p3(triangle.p3.x(), triangle.p3.y());

    return polygon.containsPoint(p1, Qt::OddEvenFill) &&
           polygon.containsPoint(p2, Qt::OddEvenFill) &&
           polygon.containsPoint(p3, Qt::OddEvenFill);
}

void TriangulationHandler::clear() {
    mPoints.clear();
    mTriangles.clear();
    mClippedTriangles.clear();
}

bool TriangulationHandler::isPointInAnyPolygon(const QVector3D& point, const QVector<QPolygonF>& polygons) {
    QPointF qpoint(point.x(), point.y());
    
    for (const QPolygonF& polygon : polygons) {
        if (polygon.containsPoint(qpoint, Qt::OddEvenFill)) {
            return true;
        }
    }
    return false;
}

void TriangulationHandler::performSimpleTriangulation(const QVector<QVector3D>& points) {
    // 简化的三角剖分实现
    // 这里使用一个基本的扇形三角剖分方法
    if (points.size() < 3) return;
    
    // 找到中心点
    QVector3D center(0, 0, 0);
    for (const QVector3D& point : points) {
        center += point;
    }
    center /= static_cast<float>(points.size());
    
    // 按角度排序点
    QVector<QPair<QVector3D, double>> sortedPoints;
    for (const QVector3D& point : points) {
        double angle = qAtan2(point.y() - center.y(), point.x() - center.x());
        sortedPoints.append(qMakePair(point, angle));
    }
    
    std::sort(sortedPoints.begin(), sortedPoints.end(), 
              [](const QPair<QVector3D, double>& a, const QPair<QVector3D, double>& b) {
                  return a.second < b.second;
              });
    
    // 创建三角形
    for (int i = 0; i < sortedPoints.size(); ++i) {
        int next = (i + 1) % sortedPoints.size();
        Triangle triangle(center, sortedPoints[i].first, sortedPoints[next].first);
        mTriangles.append(triangle);
    }
} 