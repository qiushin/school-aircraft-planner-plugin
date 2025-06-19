#include "ShapefileHandler.h"
#include "../log/QgisDebug.h"
#include <QFileInfo>
#include <QApplication>
#include <qgspolygon.h>
#include <qgslinestring.h>
#include <qgspoint.h>
#include <qgsproviderregistry.h>
#include <qgsapplication.h>
#include <qgsrectangle.h>
#include <qgswkbtypes.h>
#include <qgspointxy.h>

ShapefileHandler::ShapefileHandler() {
    // 检查QGIS应用程序是否初始化
    if (!QgsApplication::instance()) {
        logMessage("QGIS application not initialized", Qgis::MessageLevel::Critical);
    }
    
    // 检查OGR提供程序是否可用
    QgsProviderRegistry* registry = QgsProviderRegistry::instance();
    if (!registry) {
        logMessage("Cannot access QGIS provider registry", Qgis::MessageLevel::Critical);
    } else {
        QStringList providers = registry->providerList();
        logMessage(QString("Available QGIS providers: %1").arg(providers.join(", ")), Qgis::MessageLevel::Info);
        
        if (!providers.contains("ogr")) {
            logMessage("OGR provider not available - this may cause shapefile loading to fail", Qgis::MessageLevel::Warning);
        } else {
            logMessage("OGR provider is available", Qgis::MessageLevel::Info);
        }
    }
}

ShapefileHandler::~ShapefileHandler() {
    clear();
}

bool ShapefileHandler::loadFlightZonePolygon(const QString& filePath) {
    logMessage(QString("attempting to load flight zone file: %1").arg(filePath), Qgis::MessageLevel::Info);
    
    // 检查文件是否存在
    QFileInfo fileInfo(filePath);
    if (!fileInfo.exists()) {
        logMessage(QString("file not found: %1").arg(filePath), Qgis::MessageLevel::Critical);
        return false;
    }

    // 创建矢量图层
    QgsVectorLayer* layer = new QgsVectorLayer(filePath, "flight_zone", "ogr");
    if (!layer || !layer->isValid()) {
        QString errorMsg = layer ? layer->error().message() : "failed to create layer";
        logMessage(QString("cannot load flight zone file: %1, error: %2").arg(filePath).arg(errorMsg), Qgis::MessageLevel::Critical);
        if (layer) delete layer;
        return false;
    }

    logMessage(QString("successfully created layer, feature count: %1").arg(layer->featureCount()), Qgis::MessageLevel::Info);

    // 清除旧数据
    mFlightZonePolygons.clear();
    mFlightZoneBounds = QRectF();

    // 读取所有要素
    QgsFeatureIterator features = layer->getFeatures();
    QgsFeature feature;
    QRectF bounds;
    bool firstFeature = true;

    while (features.nextFeature(feature)) {
        QgsGeometry geometry = feature.geometry();
        if (geometry.isNull()) continue;

        // 转换为多边形
        QPolygonF polygon = geometryToPolygon(geometry);
        if (!polygon.isEmpty()) {
            mFlightZonePolygons.append(polygon);
            
            // 更新边界框
            QRectF polyBounds = polygon.boundingRect();
            if (firstFeature) {
                bounds = polyBounds;
                firstFeature = false;
            } else {
                bounds = bounds.united(polyBounds);
            }
        }
    }

    mFlightZoneBounds = bounds;
    delete layer;

    logMessage(QString("successfully loaded %1 flight zone polygons").arg(mFlightZonePolygons.size()), 
               Qgis::MessageLevel::Success);
    return !mFlightZonePolygons.isEmpty();
}

bool ShapefileHandler::loadRiskEventPoints(const QString& filePath) {
    logMessage(QString("attempting to load risk event points file: %1").arg(filePath), Qgis::MessageLevel::Info);
    
    // 检查文件是否存在
    QFileInfo fileInfo(filePath);
    if (!fileInfo.exists()) {
        logMessage(QString("file not found: %1").arg(filePath), Qgis::MessageLevel::Critical);
        return false;
    }

    // 创建矢量图层
    QgsVectorLayer* layer = new QgsVectorLayer(filePath, "risk_points", "ogr");
    if (!layer || !layer->isValid()) {
        QString errorMsg = layer ? layer->error().message() : "failed to create layer";
        logMessage(QString("cannot load risk event points file: %1, error: %2").arg(filePath).arg(errorMsg), Qgis::MessageLevel::Critical);
        if (layer) delete layer;
        return false;
    }

    logMessage(QString("successfully created risk points layer, feature count: %1").arg(layer->featureCount()), Qgis::MessageLevel::Info);

    // 清除旧数据
    mRiskEventPoints.clear();

    // 读取所有要素
    QgsFeatureIterator features = layer->getFeatures();
    QgsFeature feature;

    while (features.nextFeature(feature)) {
        QgsGeometry geometry = feature.geometry();
        if (geometry.isNull()) continue;

        // 简单检查是否为点几何 - 尝试转换为点
        try {
            QgsPointXY point = geometry.asPoint();
            QVector3D vec3d = QVector3D(static_cast<float>(point.x()), 
                                      static_cast<float>(point.y()), 0.0f);
            mRiskEventPoints.append(vec3d);
        } catch (...) {
            // 如果转换失败，跳过这个几何体
            continue;
        }
    }

    delete layer;

    logMessage(QString("successfully loaded %1 risk event points").arg(mRiskEventPoints.size()), 
               Qgis::MessageLevel::Success);
    return !mRiskEventPoints.isEmpty();
}

bool ShapefileHandler::isPointInFlightZone(const QVector3D& point) const {
    QPointF qpoint(point.x(), point.y());
    
    for (const QPolygonF& polygon : mFlightZonePolygons) {
        if (polygon.containsPoint(qpoint, Qt::OddEvenFill)) {
            return true;
        }
    }
    return false;
}

QRectF ShapefileHandler::getFlightZoneBounds() const {
    return mFlightZoneBounds;
}

void ShapefileHandler::clear() {
    mFlightZonePolygons.clear();
    mRiskEventPoints.clear();
    mFlightZoneBounds = QRectF();
}

QPolygonF ShapefileHandler::geometryToPolygon(const QgsGeometry& geometry) {
    QPolygonF polygon;
    
    if (geometry.isNull()) {
        logMessage("geometry is null", Qgis::MessageLevel::Warning);
        return polygon;
    }
    
    logMessage("attempting to convert geometry to polygon", Qgis::MessageLevel::Info);
    
    try {
        // 尝试直接获取边界坐标作为备用方案
        QgsRectangle bbox = geometry.boundingBox();
        if (bbox.isNull()) {
            logMessage("geometry has null bounding box", Qgis::MessageLevel::Warning);
            return polygon;
        }
        
        // 方法1: 尝试转换为单个多边形
        try {
            QgsPolygonXY singlePolygon = geometry.asPolygon();
            if (!singlePolygon.isEmpty() && !singlePolygon.first().isEmpty()) {
                logMessage("successfully converted as single polygon", Qgis::MessageLevel::Info);
                QgsPolylineXY outerRing = singlePolygon.first(); // 外环
                for (const QgsPointXY& point : outerRing) {
                    polygon.append(QPointF(point.x(), point.y()));
                }
                logMessage(QString("extracted %1 points from single polygon").arg(polygon.size()), Qgis::MessageLevel::Info);
                return polygon;
            }
        } catch (...) {
            // 单个多边形转换失败，继续尝试其他方法
        }
        
        // 方法2: 尝试转换为多多边形
        try {
            QgsMultiPolygonXY multiPolygon = geometry.asMultiPolygon();
            if (!multiPolygon.isEmpty() && !multiPolygon.first().isEmpty() && !multiPolygon.first().first().isEmpty()) {
                logMessage("successfully converted as multi polygon", Qgis::MessageLevel::Info);
                QgsPolygonXY firstPolygon = multiPolygon.first();
                QgsPolylineXY outerRing = firstPolygon.first(); // 第一个多边形的外环
                for (const QgsPointXY& point : outerRing) {
                    polygon.append(QPointF(point.x(), point.y()));
                }
                logMessage(QString("extracted %1 points from multi polygon").arg(polygon.size()), Qgis::MessageLevel::Info);
                return polygon;
            }
        } catch (...) {
            // 多多边形转换失败，使用备用方案
        }
        
        // 方法3: 备用方案 - 使用边界框创建矩形多边形
        logMessage("failed to extract polygon points, using bounding box as fallback", Qgis::MessageLevel::Warning);
        polygon.append(QPointF(bbox.xMinimum(), bbox.yMinimum()));
        polygon.append(QPointF(bbox.xMaximum(), bbox.yMinimum()));
        polygon.append(QPointF(bbox.xMaximum(), bbox.yMaximum()));
        polygon.append(QPointF(bbox.xMinimum(), bbox.yMaximum()));
        polygon.append(QPointF(bbox.xMinimum(), bbox.yMinimum())); // 闭合
        
        logMessage(QString("created bounding box polygon with %1 points").arg(polygon.size()), Qgis::MessageLevel::Info);
        
    } catch (const std::exception& e) {
        logMessage(QString("exception in geometry conversion: %1").arg(e.what()), Qgis::MessageLevel::Critical);
        polygon.clear();
    } catch (...) {
        logMessage("unknown exception in geometry conversion", Qgis::MessageLevel::Critical);
        polygon.clear();
    }

    return polygon;
}

QVector3D ShapefileHandler::qgsPointToVector3D(const QgsPoint& point) {
    return QVector3D(static_cast<float>(point.x()), 
                     static_cast<float>(point.y()), 
                     static_cast<float>(point.z()));
} 