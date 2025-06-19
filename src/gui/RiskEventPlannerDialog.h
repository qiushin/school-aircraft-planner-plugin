/****************************************************************************
File: RiskEventPlannerDialog.h
Author: AI Assistant
Date: 2025.1.6
Description: 风险事件路径规划对话框
****************************************************************************/

#ifndef RISK_EVENT_PLANNER_DIALOG_H
#define RISK_EVENT_PLANNER_DIALOG_H

#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QTextEdit>
#include <QProgressBar>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QVector3D>
#include "../core/RiskEventPlanner.h"

/**
 * @brief 风险事件路径规划对话框类
 * 提供用户友好的界面来配置规划参数并执行规划
 */
class RiskEventPlannerDialog : public QDialog {
    Q_OBJECT

public:
    explicit RiskEventPlannerDialog(QWidget *parent = nullptr);
    ~RiskEventPlannerDialog();

    /**
     * @brief 获取规划参数
     * @return 用户设置的规划参数
     */
    PlanningParameters getPlanningParameters() const;

    /**
     * @brief 设置规划参数
     * @param params 规划参数
     */
    void setPlanningParameters(const PlanningParameters& params);

    /**
     * @brief 重置对话框到默认状态
     */
    void resetToDefaults();

public slots:
    /**
     * @brief 处理规划进度更新
     * @param percentage 完成百分比
     * @param message 当前步骤描述
     */
    void onProgressUpdated(int percentage, const QString& message);

    /**
     * @brief 处理规划完成
     * @param result 规划结果
     */
    void onPlanningCompleted(const PlanningResult& result);

    /**
     * @brief 处理规划失败
     * @param errorMessage 错误信息
     */
    void onPlanningFailed(const QString& errorMessage);

signals:
    /**
     * @brief 开始规划信号
     * @param params 规划参数
     */
    void startPlanning(const PlanningParameters& params);

    /**
     * @brief 导出路径信号
     * @param outputPath 输出路径
     */
    void exportPath(const QString& outputPath);

    /**
     * @brief 显示结果信号
     * @param result 规划结果
     */
    void showResults(const PlanningResult& result);

private slots:
    /**
     * @brief 浏览可飞行区域文件
     */
    void browseFlightZoneFile();

    /**
     * @brief 浏览风险事件点文件
     */
    void browseRiskEventFile();

    /**
     * @brief 浏览输出目录
     */
    void browseOutputDirectory();

    /**
     * @brief 执行规划
     */
    void executePlanning();

    /**
     * @brief 导出结果
     */
    void exportResults();

    /**
     * @brief 预览结果
     */
    void previewResults();

    /**
     * @brief 重置表单
     */
    void resetForm();

    /**
     * @brief 验证输入
     */
    void validateInput();

private:
    // UI组件
    QGroupBox* mInputGroup;              // 输入文件组
    QGroupBox* mParametersGroup;         // 参数设置组
    QGroupBox* mOutputGroup;             // 输出设置组
    QGroupBox* mProgressGroup;           // 进度显示组
    QGroupBox* mResultsGroup;            // 结果显示组

    // 输入文件控件
    QLineEdit* mFlightZoneEdit;          // 可飞行区域文件路径
    QPushButton* mFlightZoneBrowseBtn;   // 浏览可飞行区域文件按钮
    QLineEdit* mRiskEventEdit;           // 风险事件点文件路径
    QPushButton* mRiskEventBrowseBtn;    // 浏览风险事件点文件按钮

    // 参数设置控件
    QDoubleSpinBox* mStartXSpin;         // 起始点X坐标
    QDoubleSpinBox* mStartYSpin;         // 起始点Y坐标
    QDoubleSpinBox* mStartZSpin;         // 起始点Z坐标
    QDoubleSpinBox* mSpacingSpin;        // 三角网点间距
    QComboBox* mAlgorithmCombo;          // 优化算法选择

    // 输出设置控件
    QLineEdit* mOutputEdit;              // 输出目录路径
    QPushButton* mOutputBrowseBtn;       // 浏览输出目录按钮

    // 进度控件
    QProgressBar* mProgressBar;          // 进度条
    QLabel* mProgressLabel;              // 进度标签

    // 结果显示控件
    QTextEdit* mResultsText;             // 结果文本显示

    // 操作按钮
    QPushButton* mExecuteBtn;            // 执行规划按钮
    QPushButton* mExportBtn;             // 导出结果按钮
    QPushButton* mPreviewBtn;            // 预览结果按钮
    QPushButton* mResetBtn;              // 重置按钮
    QPushButton* mCloseBtn;              // 关闭按钮

    // 布局
    QVBoxLayout* mMainLayout;            // 主布局
    QHBoxLayout* mButtonLayout;          // 按钮布局

    // 业务逻辑
    RiskEventPlanner* mPlanner;          // 路径规划器
    PlanningResult mCurrentResult;       // 当前规划结果
    bool mPlanningInProgress;            // 是否正在规划中

    /**
     * @brief 初始化UI组件
     */
    void initializeUI();

    /**
     * @brief 创建输入文件组
     * @return 输入文件组控件
     */
    QGroupBox* createInputGroup();

    /**
     * @brief 创建参数设置组
     * @return 参数设置组控件
     */
    QGroupBox* createParametersGroup();

    /**
     * @brief 创建输出设置组
     * @return 输出设置组控件
     */
    QGroupBox* createOutputGroup();

    /**
     * @brief 创建进度显示组
     * @return 进度显示组控件
     */
    QGroupBox* createProgressGroup();

    /**
     * @brief 创建结果显示组
     * @return 结果显示组控件
     */
    QGroupBox* createResultsGroup();

    /**
     * @brief 创建操作按钮
     */
    void createButtons();

    /**
     * @brief 连接信号和槽
     */
    void connectSignals();

    /**
     * @brief 设置初始值
     */
    void setInitialValues();

    /**
     * @brief 更新UI状态
     * @param enabled 是否启用
     */
    void updateUIState(bool enabled);

    /**
     * @brief 验证文件路径
     * @param filePath 文件路径
     * @return 验证结果
     */
    bool validateFilePath(const QString& filePath);

    /**
     * @brief 显示错误信息
     * @param message 错误信息
     */
    void showError(const QString& message);

    /**
     * @brief 显示成功信息
     * @param message 成功信息
     */
    void showSuccess(const QString& message);
};

#endif // RISK_EVENT_PLANNER_DIALOG_H 