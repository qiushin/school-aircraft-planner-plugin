/****************************************************************************
File: PathOptimizer.cpp
Author: AI Assistant
Date: 2025.1.6
Description: 路径优化算法实现
****************************************************************************/

#include "PathOptimizer.h"
#include "../log/QgisDebug.h"
#include <QtMath>
#include <QLineF>
#include <QRandomGenerator>
#include <algorithm>
#include <limits>
#include <random>

PathOptimizer::PathOptimizer() {
    // 构造函数
}

PathOptimizer::~PathOptimizer() {
    clear();
}

void PathOptimizer::setRiskEventPoints(const QVector<QVector3D>& riskPoints) {
    mRiskEventPoints = riskPoints;
    
    // 重新构建节点列表
    mAllNodes.clear();
    for (int i = 0; i < riskPoints.size(); ++i) {
        mAllNodes.append(PathNode(riskPoints[i], i, true));
    }
    
    buildDistanceMatrix();
    logMessage(QString("set %1 risk event points").arg(riskPoints.size()), 
               Qgis::MessageLevel::Info);
}

void PathOptimizer::setTriangulationData(const QVector<Triangle>& triangles) {
    mTriangles = triangles;
    logMessage(QString("set %1 triangles constraints").arg(triangles.size()), 
               Qgis::MessageLevel::Info);
}

void PathOptimizer::setFlightZoneConstraints(const QVector<QPolygonF>& flightZones) {
    mFlightZones = flightZones;
    logMessage(QString("set %1 flight zone constraints").arg(flightZones.size()), 
               Qgis::MessageLevel::Info);
}

QVector<PathNode> PathOptimizer::generateOptimalPath(const QVector3D& startPoint, const QString& algorithm) {
    if (mRiskEventPoints.isEmpty()) {
        logMessage("no risk event points data", Qgis::MessageLevel::Warning);
        return QVector<PathNode>();
    }

    QVector<PathNode> path;

    if (algorithm == "NearestNeighbor") {
        path = nearestNeighborTSP(startPoint);
        // 使用2-opt改进
        path = twoOptImprovement(path);
    } else if (algorithm == "GeneticAlgorithm") {
        path = geneticAlgorithmTSP(startPoint);
    } else {
        // 默认使用最近邻算法
        path = nearestNeighborTSP(startPoint);
    }

    double totalLength = calculatePathLength(path);
    logMessage(QString("generate path completed, total length: %1").arg(totalLength, 0, 'f', 2), 
               Qgis::MessageLevel::Success);

    return path;
}

QVector<PathNode> PathOptimizer::nearestNeighborTSP(const QVector3D& startPoint) {
    QVector<PathNode> path;
    QVector<bool> visited(mRiskEventPoints.size(), false);

    // 添加起始点
    path.append(PathNode(startPoint, -1, false));

    // 找到最近的风险事件点作为第一个访问点
    int currentIndex = -1;
    double minDistance = std::numeric_limits<double>::max();
    
    for (int i = 0; i < mRiskEventPoints.size(); ++i) {
        double dist = calculateDistance(startPoint, mRiskEventPoints[i]);
        if (dist < minDistance) {
            minDistance = dist;
            currentIndex = i;
        }
    }

    if (currentIndex == -1) return path;

    // 访问所有风险事件点
    while (true) {
        visited[currentIndex] = true;
        path.append(PathNode(mRiskEventPoints[currentIndex], currentIndex, true));

        // 寻找下一个最近的未访问点
        int nextIndex = -1;
        minDistance = std::numeric_limits<double>::max();

        for (int i = 0; i < mRiskEventPoints.size(); ++i) {
            if (!visited[i]) {
                double dist = calculateDistance(mRiskEventPoints[currentIndex], mRiskEventPoints[i]);
                if (dist < minDistance) {
                    minDistance = dist;
                    nextIndex = i;
                }
            }
        }

        if (nextIndex == -1) break; // 所有点都已访问
        currentIndex = nextIndex;
    }

    // 返回起始点
    path.append(PathNode(startPoint, -1, false));

    return path;
}

QVector<PathNode> PathOptimizer::twoOptImprovement(const QVector<PathNode>& path) {
    if (path.size() < 4) return path; // 不足以进行2-opt改进

    QVector<PathNode> improvedPath = path;
    bool improved = true;

    while (improved) {
        improved = false;
        
        for (int i = 1; i < improvedPath.size() - 2; ++i) {
            for (int j = i + 1; j < improvedPath.size() - 1; ++j) {
                // 计算当前边的长度
                double currentLength = calculateDistance(improvedPath[i].position, improvedPath[i+1].position) +
                                     calculateDistance(improvedPath[j].position, improvedPath[j+1].position);
                
                // 计算交换后边的长度
                double newLength = calculateDistance(improvedPath[i].position, improvedPath[j].position) +
                                 calculateDistance(improvedPath[i+1].position, improvedPath[j+1].position);

                if (newLength < currentLength) {
                    // 执行2-opt交换
                    std::reverse(improvedPath.begin() + i + 1, improvedPath.begin() + j + 1);
                    improved = true;
                }
            }
        }
    }

    return improvedPath;
}

QVector<PathNode> PathOptimizer::geneticAlgorithmTSP(const QVector3D& startPoint, int generations, int populationSize) {
    if (mRiskEventPoints.size() < 2) {
        return nearestNeighborTSP(startPoint);
    }

    // 初始化种群
    QVector<QVector<int>> population;
    for (int i = 0; i < populationSize; ++i) {
        population.append(generateRandomPath(0)); // 假设起始点索引为0
    }

    QVector<int> bestPath;
    double bestFitness = std::numeric_limits<double>::max();

    for (int generation = 0; generation < generations; ++generation) {
        // 评估适应度
        QVector<double> fitness(populationSize);
        for (int i = 0; i < populationSize; ++i) {
            fitness[i] = 0.0;
            for (int j = 0; j < population[i].size() - 1; ++j) {
                int from = population[i][j];
                int to = population[i][j + 1];
                if (from < mDistanceMatrix.size() && to < mDistanceMatrix[from].size()) {
                    fitness[i] += mDistanceMatrix[from][to];
                }
            }
            
            if (fitness[i] < bestFitness) {
                bestFitness = fitness[i];
                bestPath = population[i];
            }
        }

        // 选择、交叉和变异
        QVector<QVector<int>> newPopulation;
        for (int i = 0; i < populationSize; ++i) {
            // 锦标赛选择
            int parent1 = QRandomGenerator::global()->bounded(populationSize);
            int parent2 = QRandomGenerator::global()->bounded(populationSize);
            if (fitness[parent2] < fitness[parent1]) {
                std::swap(parent1, parent2);
            }

            // 交叉
            QVector<int> offspring = crossoverPaths(population[parent1], population[parent2]);
            
            // 变异
            offspring = mutatePath(offspring);
            
            newPopulation.append(offspring);
        }

        population = newPopulation;
    }

    // 转换为路径节点
    QVector<PathNode> result;
    result.append(PathNode(startPoint, -1, false));
    
    for (int index : bestPath) {
        if (index < mRiskEventPoints.size()) {
            result.append(PathNode(mRiskEventPoints[index], index, true));
        }
    }
    
    result.append(PathNode(startPoint, -1, false));
    
    return result;
}

bool PathOptimizer::hasDirectPath(const QVector3D& from, const QVector3D& to) {
    // 检查线段是否完全在可飞行区域内
    return lineIntersectsFlightZone(from, to);
}

QVector<QVector3D> PathOptimizer::findConstrainedPath(const QVector3D& from, const QVector3D& to) {
    // 如果有直线路径，直接返回
    if (hasDirectPath(from, to)) {
        return QVector<QVector3D>();
    }

    // 否则使用A*算法在三角网上寻路
    return aStarPathfinding(from, to);
}

double PathOptimizer::calculatePathLength(const QVector<PathNode>& path) {
    if (path.size() < 2) return 0.0;

    double totalLength = 0.0;
    for (int i = 0; i < path.size() - 1; ++i) {
        totalLength += calculateDistance(path[i].position, path[i+1].position);
    }
    return totalLength;
}

void PathOptimizer::clear() {
    mRiskEventPoints.clear();
    mTriangles.clear();
    mFlightZones.clear();
    mDistanceMatrix.clear();
    mAllNodes.clear();
}

double PathOptimizer::calculateDistance(const QVector3D& p1, const QVector3D& p2) {
    return qSqrt(qPow(p1.x() - p2.x(), 2) + qPow(p1.y() - p2.y(), 2) + qPow(p1.z() - p2.z(), 2));
}

void PathOptimizer::buildDistanceMatrix() {
    int size = mRiskEventPoints.size();
    mDistanceMatrix.clear();
    mDistanceMatrix.resize(size);

    for (int i = 0; i < size; ++i) {
        mDistanceMatrix[i].resize(size);
        for (int j = 0; j < size; ++j) {
            if (i == j) {
                mDistanceMatrix[i][j] = 0.0;
            } else {
                mDistanceMatrix[i][j] = calculateDistance(mRiskEventPoints[i], mRiskEventPoints[j]);
            }
        }
    }
}

bool PathOptimizer::lineIntersectsFlightZone(const QVector3D& from, const QVector3D& to) {
    QLineF line(from.x(), from.y(), to.x(), to.y());
    
    for (const QPolygonF& zone : mFlightZones) {
        // 检查线段是否完全在多边形内
        if (zone.containsPoint(QPointF(from.x(), from.y()), Qt::OddEvenFill) &&
            zone.containsPoint(QPointF(to.x(), to.y()), Qt::OddEvenFill)) {
            
            // 进一步检查线段是否与多边形边界相交
            bool intersects = false;
            for (int i = 0; i < zone.size() - 1; ++i) {
                QLineF edge(zone[i], zone[i + 1]);
                QPointF intersection;
                if (line.intersects(edge, &intersection) == QLineF::BoundedIntersection) {
                    intersects = true;
                    break;
                }
            }
            
            if (!intersects) {
                return true; // 线段在多边形内且不与边界相交
            }
        }
    }
    return false;
}

QVector<QVector3D> PathOptimizer::aStarPathfinding(const QVector3D& start, const QVector3D& goal) {
    // 简化的A*实现，在实际应用中应该更完善
    QVector<QVector3D> path;
    
    // 这里实现一个简化版本，实际应该在三角网上进行寻路
    // 目前直接返回起点和终点之间的直线路径的中点
    QVector3D midPoint = (start + goal) / 2.0f;
    path.append(midPoint);
    
    return path;
}

QVector<Triangle> PathOptimizer::getAdjacentTriangles(const Triangle& triangle) {
    QVector<Triangle> adjacent;
    
    for (const Triangle& other : mTriangles) {
        if (&other == &triangle) continue;
        
        // 检查是否有共享边
        int sharedVertices = 0;
        if (triangle.p1 == other.p1 || triangle.p1 == other.p2 || triangle.p1 == other.p3) sharedVertices++;
        if (triangle.p2 == other.p1 || triangle.p2 == other.p2 || triangle.p2 == other.p3) sharedVertices++;
        if (triangle.p3 == other.p1 || triangle.p3 == other.p2 || triangle.p3 == other.p3) sharedVertices++;
        
        if (sharedVertices >= 2) {
            adjacent.append(other);
        }
    }
    
    return adjacent;
}

QVector<int> PathOptimizer::generateRandomPath(int startIndex) {
    QVector<int> path;
    QVector<int> indices;
    
    // 生成除起始点外的所有索引
    for (int i = 0; i < mRiskEventPoints.size(); ++i) {
        if (i != startIndex) {
            indices.append(i);
        }
    }
    
    // 使用std::shuffle代替std::random_shuffle
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(indices.begin(), indices.end(), g);
    
    path.append(startIndex);
    path.append(indices);
    path.append(startIndex); // 回到起点
    
    return path;
}

QVector<int> PathOptimizer::crossoverPaths(const QVector<int>& parent1, const QVector<int>& parent2) {
    if (parent1.size() != parent2.size() || parent1.size() < 4) {
        return parent1;
    }
    
    // 部分映射交叉(PMX)
    int size = parent1.size() - 2; // 除去起点和终点
    int crossPoint1 = QRandomGenerator::global()->bounded(1, size);
    int crossPoint2 = QRandomGenerator::global()->bounded(crossPoint1 + 1, size + 1);
    
    QVector<int> offspring = parent1;
    
    // 从parent2复制中间段
    for (int i = crossPoint1; i < crossPoint2; ++i) {
        offspring[i] = parent2[i];
    }
    
    return offspring;
}

QVector<int> PathOptimizer::mutatePath(const QVector<int>& path, double mutationRate) {
    QVector<int> mutated = path;
    
    if (QRandomGenerator::global()->generateDouble() < mutationRate && path.size() > 3) {
        // 交换两个随机位置（不包括起点和终点）
        int pos1 = QRandomGenerator::global()->bounded(1, path.size() - 2);
        int pos2 = QRandomGenerator::global()->bounded(1, path.size() - 2);
        if (pos1 != pos2) {
            std::swap(mutated[pos1], mutated[pos2]);
        }
    }
    
    return mutated;
} 