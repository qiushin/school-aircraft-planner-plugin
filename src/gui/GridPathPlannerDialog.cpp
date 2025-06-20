/****************************************************************************
File: GridPathPlannerDialog.cpp
Author: AI Assistant
Date: 2025.1.6
Description: 基于渔网线的网格路径规划对话框实现
****************************************************************************/

#include "GridPathPlannerDialog.h"
#include "../log/QgisDebug.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QStandardPaths>
#include <QDir>
#include <QCoreApplication>

GridPathPlannerDialog::GridPathPlannerDialog(QWidget *parent)
    : QDialog(parent)
    , mPlanner(new GridPathPlanner(this))
    , mPlanningInProgress(false) {
    
    setWindowTitle("基于渔网线的无人机路径规划");
    setMinimumSize(900, 700);
    resize(1100, 800);
    
    initializeUI();
    connectSignals();
    setInitialValues();
    updateUIState(true);
    
    logMessage("Grid path planner dialog initialized", Qgis::MessageLevel::Info);
}

GridPathPlannerDialog::~GridPathPlannerDialog() {
    
}

GridPlanningParameters GridPathPlannerDialog::getPlanningParameters() const {
    GridPlanningParameters params;
    
    params.fishnetShapefile = mFishnetEdit->text();
    params.riskPointsShapefile = mRiskPointsEdit->text();
    params.outputPath = mOutputEdit->text();
    params.startPoint = QVector3D(
        static_cast<float>(mStartXSpin->value()),
        static_cast<float>(mStartYSpin->value()),
        static_cast<float>(mStartZSpin->value())
    );
    params.algorithm = mAlgorithmCombo->currentText();
    params.maxRiskPointDistance = mMaxDistanceSpin->value();
    
    return params;
}

void GridPathPlannerDialog::setPlanningParameters(const GridPlanningParameters& params) {
    mFishnetEdit->setText(params.fishnetShapefile);
    mRiskPointsEdit->setText(params.riskPointsShapefile);
    mOutputEdit->setText(params.outputPath);
    mStartXSpin->setValue(params.startPoint.x());
    mStartYSpin->setValue(params.startPoint.y());
    mStartZSpin->setValue(params.startPoint.z());
    mMaxDistanceSpin->setValue(params.maxRiskPointDistance);
    
    int algorithmIndex = mAlgorithmCombo->findText(params.algorithm);
    if (algorithmIndex >= 0) {
        mAlgorithmCombo->setCurrentIndex(algorithmIndex);
    }
}

void GridPathPlannerDialog::resetToDefaults() {
    GridPlanningParameters defaultParams;
    setPlanningParameters(defaultParams);
    mResultsText->clear();
    mProgressBar->setValue(0);
    mProgressLabel->setText("准备就绪");
    mCurrentResult = GridPlanningResult();
}

void GridPathPlannerDialog::onProgressUpdated(int percentage, const QString& message) {
    mProgressBar->setValue(percentage);
    mProgressLabel->setText(message);
    
    // 在结果文本中添加进度信息
    mResultsText->append(QString("[%1%] %2").arg(percentage).arg(message));
    QCoreApplication::processEvents(); // 强制更新UI
}

void GridPathPlannerDialog::onPlanningCompleted(const GridPlanningResult& result) {
    mCurrentResult = result;
    mPlanningInProgress = false;
    updateUIState(true);
    
    if (result.success) {
        QString successMessage = QString("网格路径规划完成!\n总路径长度: %1 米\n访问风险点: %2/%3")
                                .arg(result.totalPathLength, 0, 'f', 2)
                                .arg(result.visitedRiskPoints)
                                .arg(result.totalRiskPoints);
        
        mResultsText->append("\n=== 规划完成 ===");
        mResultsText->append(successMessage);
        mResultsText->append(mPlanner->getStatistics(result));
        
        showSuccess("网格路径规划完成!");
        
        // 启用导出和预览按钮
        mExportBtn->setEnabled(true);
        mPreviewBtn->setEnabled(true);
        
    } else {
        showError(QString("规划失败: %1").arg(result.errorMessage));
    }
}

void GridPathPlannerDialog::onPlanningFailed(const QString& errorMessage) {
    mPlanningInProgress = false;
    updateUIState(true);
    showError(QString("规划失败: %1").arg(errorMessage));
    mResultsText->append(QString("\n=== 规划失败 ===\n%1").arg(errorMessage));
}

void GridPathPlannerDialog::browseFishnetFile() {
    QString fileName = QFileDialog::getOpenFileName(
        this,
        "选择渔网线Shapefile",
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation),
        "Shapefile (*.shp);;所有文件 (*.*)"
    );
    
    if (!fileName.isEmpty()) {
        mFishnetEdit->setText(fileName);
        validateInput();
    }
}

void GridPathPlannerDialog::browseRiskPointsFile() {
    QString fileName = QFileDialog::getOpenFileName(
        this,
        "选择风险点Shapefile",
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation),
        "Shapefile (*.shp);;所有文件 (*.*)"
    );
    
    if (!fileName.isEmpty()) {
        mRiskPointsEdit->setText(fileName);
        validateInput();
    }
}

void GridPathPlannerDialog::browseOutputDirectory() {
    QString dirName = QFileDialog::getExistingDirectory(
        this,
        "选择输出目录",
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)
    );
    
    if (!dirName.isEmpty()) {
        mOutputEdit->setText(dirName);
        validateInput();
    }
}

void GridPathPlannerDialog::executePlanning() {
    if (mPlanningInProgress) {
        showError("规划正在进行中，请等待完成");
        return;
    }
    
    // 验证输入
    GridPlanningParameters params = getPlanningParameters();
    
    auto validation = mPlanner->validateParameters(params);
    if (!validation.first) {
        showError(validation.second);
        return;
    }
    
    // 开始规划
    mPlanningInProgress = true;
    updateUIState(false);
    mResultsText->clear();
    mResultsText->append("开始执行网格路径规划...");
    
    // 重置进度
    mProgressBar->setValue(0);
    mProgressLabel->setText("准备中...");
    
    // 执行规划
    mPlanner->asyncExecutePlanning(params);
}

void GridPathPlannerDialog::exportResults() {
    if (!mCurrentResult.success) {
        showError("没有可导出的结果");
        return;
    }
    
    QString outputPath = mOutputEdit->text();
    if (outputPath.isEmpty()) {
        outputPath = QFileDialog::getExistingDirectory(
            this,
            "选择导出目录",
            QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)
        );
        
        if (outputPath.isEmpty()) return;
    }
    
    if (mPlanner->exportPathToFiles(mCurrentResult, outputPath)) {
        showSuccess(QString("结果已导出至: %1").arg(outputPath));
        mResultsText->append(QString("\n结果导出完成: %1").arg(outputPath));
    } else {
        showError("导出失败");
    }
}

void GridPathPlannerDialog::previewResults() {
    if (!mCurrentResult.success) {
        showError("没有可预览的结果");
        return;
    }
    
    emit showResults(mCurrentResult);
}

void GridPathPlannerDialog::resetForm() {
    resetToDefaults();
    mExportBtn->setEnabled(false);
    mPreviewBtn->setEnabled(false);
}

void GridPathPlannerDialog::validateInput() {
    bool valid = true;
    
    // 检查渔网线文件
    if (!validateFilePath(mFishnetEdit->text())) {
        valid = false;
    }
    
    // 检查风险点文件
    if (!validateFilePath(mRiskPointsEdit->text())) {
        valid = false;
    }
    
    mExecuteBtn->setEnabled(valid && !mPlanningInProgress);
}

void GridPathPlannerDialog::onAlgorithmChanged() {
    QString algorithm = mAlgorithmCombo->currentText();
    updateAlgorithmDescription(algorithm);
}

void GridPathPlannerDialog::initializeUI() {
    mMainLayout = new QVBoxLayout(this);
    
    // 创建各个组
    mInputGroup = createInputGroup();
    mParametersGroup = createParametersGroup();
    mAlgorithmGroup = createAlgorithmGroup();
    mOutputGroup = createOutputGroup();
    mProgressGroup = createProgressGroup();
    mResultsGroup = createResultsGroup();
    
    // 添加到主布局
    mMainLayout->addWidget(mInputGroup);
    mMainLayout->addWidget(mParametersGroup);
    mMainLayout->addWidget(mAlgorithmGroup);
    mMainLayout->addWidget(mOutputGroup);
    mMainLayout->addWidget(mProgressGroup);
    mMainLayout->addWidget(mResultsGroup);
    
    // 创建按钮
    createButtons();
}

QGroupBox* GridPathPlannerDialog::createInputGroup() {
    QGroupBox* group = new QGroupBox("输入文件");
    QGridLayout* layout = new QGridLayout(group);
    
    // 渔网线文件
    layout->addWidget(new QLabel("渔网线文件:"), 0, 0);
    mFishnetEdit = new QLineEdit();
    layout->addWidget(mFishnetEdit, 0, 1);
    mFishnetBrowseBtn = new QPushButton("浏览...");
    layout->addWidget(mFishnetBrowseBtn, 0, 2);
    
    // 风险点文件
    layout->addWidget(new QLabel("风险点文件:"), 1, 0);
    mRiskPointsEdit = new QLineEdit();
    layout->addWidget(mRiskPointsEdit, 1, 1);
    mRiskPointsBrowseBtn = new QPushButton("浏览...");
    layout->addWidget(mRiskPointsBrowseBtn, 1, 2);
    
    return group;
}

QGroupBox* GridPathPlannerDialog::createParametersGroup() {
    QGroupBox* group = new QGroupBox("规划参数");
    QGridLayout* layout = new QGridLayout(group);
    
    // 起始点坐标
    layout->addWidget(new QLabel("起始点 X:"), 0, 0);
    mStartXSpin = new QDoubleSpinBox();
    mStartXSpin->setRange(-999999, 999999999);
    mStartXSpin->setDecimals(3);
    layout->addWidget(mStartXSpin, 0, 1);
    
    layout->addWidget(new QLabel("起始点 Y:"), 0, 2);
    mStartYSpin = new QDoubleSpinBox();
    mStartYSpin->setRange(-999999, 999999999);
    mStartYSpin->setDecimals(3);
    layout->addWidget(mStartYSpin, 0, 3);
    
    layout->addWidget(new QLabel("起始点 Z:"), 1, 0);
    mStartZSpin = new QDoubleSpinBox();
    mStartZSpin->setRange(0, 1000);
    mStartZSpin->setDecimals(1);
    layout->addWidget(mStartZSpin, 1, 1);
    
    // 最大关联距离
    layout->addWidget(new QLabel("风险点最大关联距离(米):"), 1, 2);
    mMaxDistanceSpin = new QDoubleSpinBox();
    mMaxDistanceSpin->setRange(1.0, 500.0);
    mMaxDistanceSpin->setDecimals(1);
    layout->addWidget(mMaxDistanceSpin, 1, 3);
    
    return group;
}

QGroupBox* GridPathPlannerDialog::createAlgorithmGroup() {
    QGroupBox* group = new QGroupBox("算法选择");
    QVBoxLayout* layout = new QVBoxLayout(group);
    
    // 功能说明
    QLabel* noteLabel = new QLabel("说明：本功能直接使用您提供的渔网线作为可飞行路径网络，无需重新构建网格。");
    noteLabel->setWordWrap(true);
    noteLabel->setStyleSheet("QLabel { color: #0066cc; font-weight: bold; padding: 4px; background-color: #f0f8ff; border: 1px solid #cce0ff; border-radius: 3px; }");
    layout->addWidget(noteLabel);
    
    // 算法选择
    QHBoxLayout* algoLayout = new QHBoxLayout();
    algoLayout->addWidget(new QLabel("算法:"));
    mAlgorithmCombo = new QComboBox();
    mAlgorithmCombo->addItems({"TSP_Dijkstra", "Dijkstra", "AStar"});
    algoLayout->addWidget(mAlgorithmCombo);
    algoLayout->addStretch();
    layout->addLayout(algoLayout);
    
    // 算法描述
    mAlgorithmDescLabel = new QLabel();
    mAlgorithmDescLabel->setWordWrap(true);
    mAlgorithmDescLabel->setStyleSheet("QLabel { color: #666; font-style: italic; }");
    layout->addWidget(mAlgorithmDescLabel);
    
    return group;
}

QGroupBox* GridPathPlannerDialog::createOutputGroup() {
    QGroupBox* group = new QGroupBox("输出设置");
    QHBoxLayout* layout = new QHBoxLayout(group);
    
    layout->addWidget(new QLabel("输出目录:"));
    mOutputEdit = new QLineEdit();
    layout->addWidget(mOutputEdit);
    mOutputBrowseBtn = new QPushButton("浏览...");
    layout->addWidget(mOutputBrowseBtn);
    
    return group;
}

QGroupBox* GridPathPlannerDialog::createProgressGroup() {
    QGroupBox* group = new QGroupBox("规划进度");
    QVBoxLayout* layout = new QVBoxLayout(group);
    
    mProgressBar = new QProgressBar();
    layout->addWidget(mProgressBar);
    
    mProgressLabel = new QLabel("准备就绪");
    layout->addWidget(mProgressLabel);
    
    return group;
}

QGroupBox* GridPathPlannerDialog::createResultsGroup() {
    QGroupBox* group = new QGroupBox("规划结果");
    QVBoxLayout* layout = new QVBoxLayout(group);
    
    mResultsText = new QTextEdit();
    mResultsText->setMaximumHeight(200);
    mResultsText->setReadOnly(true);
    layout->addWidget(mResultsText);
    
    return group;
}

void GridPathPlannerDialog::createButtons() {
    mButtonLayout = new QHBoxLayout();
    
    mExecuteBtn = new QPushButton("执行规划");
    mExportBtn = new QPushButton("导出结果");
    mPreviewBtn = new QPushButton("预览结果");
    mResetBtn = new QPushButton("重置");
    mCloseBtn = new QPushButton("关闭");
    
    mExportBtn->setEnabled(false);
    mPreviewBtn->setEnabled(false);
    
    mButtonLayout->addWidget(mExecuteBtn);
    mButtonLayout->addWidget(mExportBtn);
    mButtonLayout->addWidget(mPreviewBtn);
    mButtonLayout->addWidget(mResetBtn);
    mButtonLayout->addStretch();
    mButtonLayout->addWidget(mCloseBtn);
    
    mMainLayout->addLayout(mButtonLayout);
}

void GridPathPlannerDialog::connectSignals() {
    // 文件浏览按钮
    connect(mFishnetBrowseBtn, &QPushButton::clicked, this, &GridPathPlannerDialog::browseFishnetFile);
    connect(mRiskPointsBrowseBtn, &QPushButton::clicked, this, &GridPathPlannerDialog::browseRiskPointsFile);
    connect(mOutputBrowseBtn, &QPushButton::clicked, this, &GridPathPlannerDialog::browseOutputDirectory);
    
    // 操作按钮
    connect(mExecuteBtn, &QPushButton::clicked, this, &GridPathPlannerDialog::executePlanning);
    connect(mExportBtn, &QPushButton::clicked, this, &GridPathPlannerDialog::exportResults);
    connect(mPreviewBtn, &QPushButton::clicked, this, &GridPathPlannerDialog::previewResults);
    connect(mResetBtn, &QPushButton::clicked, this, &GridPathPlannerDialog::resetForm);
    connect(mCloseBtn, &QPushButton::clicked, this, &QDialog::accept);
    
    // 输入验证
    connect(mFishnetEdit, &QLineEdit::textChanged, this, &GridPathPlannerDialog::validateInput);
    connect(mRiskPointsEdit, &QLineEdit::textChanged, this, &GridPathPlannerDialog::validateInput);
    
    // 算法改变
    connect(mAlgorithmCombo, QOverload<const QString&>::of(&QComboBox::currentTextChanged),
            this, &GridPathPlannerDialog::onAlgorithmChanged);
    
    // 规划器信号
    connect(mPlanner, &GridPathPlanner::progressUpdated, this, &GridPathPlannerDialog::onProgressUpdated);
    connect(mPlanner, &GridPathPlanner::planningCompleted, this, &GridPathPlannerDialog::onPlanningCompleted);
    connect(mPlanner, &GridPathPlanner::planningFailed, this, &GridPathPlannerDialog::onPlanningFailed);
}

void GridPathPlannerDialog::setInitialValues() {
    GridPlanningParameters defaultParams;
    setPlanningParameters(defaultParams);
    updateAlgorithmDescription(mAlgorithmCombo->currentText());
}

void GridPathPlannerDialog::updateUIState(bool enabled) {
    mFishnetEdit->setEnabled(enabled);
    mFishnetBrowseBtn->setEnabled(enabled);
    mRiskPointsEdit->setEnabled(enabled);
    mRiskPointsBrowseBtn->setEnabled(enabled);
    mStartXSpin->setEnabled(enabled);
    mStartYSpin->setEnabled(enabled);
    mStartZSpin->setEnabled(enabled);
    mMaxDistanceSpin->setEnabled(enabled);
    mAlgorithmCombo->setEnabled(enabled);
    mOutputEdit->setEnabled(enabled);
    mOutputBrowseBtn->setEnabled(enabled);
    mResetBtn->setEnabled(enabled);
    
    if (enabled) {
        validateInput();
    } else {
        mExecuteBtn->setEnabled(false);
    }
}

bool GridPathPlannerDialog::validateFilePath(const QString& filePath) {
    return !filePath.isEmpty() && QFileInfo::exists(filePath);
}

void GridPathPlannerDialog::showError(const QString& message) {
    QMessageBox::critical(this, "错误", message);
}

void GridPathPlannerDialog::showSuccess(const QString& message) {
    QMessageBox::information(this, "成功", message);
}

void GridPathPlannerDialog::updateAlgorithmDescription(const QString& algorithm) {
    QString description;
    
    if (algorithm == "TSP_Dijkstra") {
        description = "TSP+Dijkstra算法: 使用最近邻算法求解旅行商问题，然后用Dijkstra算法在网格中寻找最短路径。适合处理大量风险点的避障路径规划。";
    } else if (algorithm == "Dijkstra") {
        description = "Dijkstra算法: 经典的最短路径算法，从起点到最近的风险点。适合简单的点对点路径规划。";
    } else if (algorithm == "AStar") {
        description = "A*算法: 带启发式的最短路径算法，比Dijkstra更高效。适合有明确目标的路径规划。";
    }
    
    mAlgorithmDescLabel->setText(description);
} 