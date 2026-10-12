#ifndef UWSCANMAINWINDOW_H
#define UWSCANMAINWINDOW_H

#include "SARibbon.h"
#include "imagewidget.h"
#include "infofullstatusbar.h"
#include "mainwindow/advancedstatusbar.h"
#include "pointcloud/cloudtreewidget.h"
#include "pointcloud/cloudwindow.h"
#include "scanner/imagethread.h"
#include "scanner/reconstructor.h"
#include "scanner/scanner.h"
#include "scanner/scannerinterface.h"
#include "server/commanddialog.h"
#include "server/commandserver.h"
#include "utils.h"

#include <QDockWidget>
#include <QHash>
#include <QSplitter>
#include <QTabWidget>
#include <QThread>
#include <vector>

NAMESPACE_BEGIN { namespace mainwindow {
    struct CloudLegendInfo {
        QString title;
        double maxAbsError = 0.0;
    };

    enum class FitFeatureKind {
        Unknown = 0,
        Cylinder,
        Sphere,
        Plane
    };

    struct FitCloudContext {
        FitFeatureKind kind = FitFeatureKind::Unknown;
        bool standardDiameterEditable = false;
        double standardDiameter = 0.0;
    };

    class UWScanMainWindow : public SARibbonMainWindow {
        Q_OBJECT

    private:
        SARibbonBar* ribbon = nullptr;

        SARibbonCategory* fileCategory = nullptr;
        SARibbonPanel* filePanel = nullptr;
        QAction* newAction = nullptr;
        QAction* openAction = nullptr;
        QMenu* saveMenu = nullptr;
        QAction* savePointcloudAction = nullptr;
        QAction* saveImageAction = nullptr;
        QAction* saveDepthAction = nullptr;
        QAction* saveCenterLineAction = nullptr;

        SARibbonCategory* scanCategory = nullptr;
        SARibbonPanel* connPanel = nullptr;
        QAction* connAction = nullptr;
        SARibbonPanel* scanPanel = nullptr;
        QAction* previewAction = nullptr;
        QAction* stopPreviewAction = nullptr;
        QAction* scanAction = nullptr;
        QAction* stopScanAction = nullptr;
        QAction* staticReconstructionAction = nullptr;

        SARibbonCategory* pointcloudCategory = nullptr;
        SARibbonPanel* selectPanel = nullptr;
        QAction* guideInAction = nullptr;
        QAction* selectAllAction = nullptr;
        QAction* selectNoneAction = nullptr;
        QAction* deleteAction = nullptr;
        QAction* reverseAction = nullptr;
        QAction* rotateCenterAction = nullptr;

        QMenu* selectToolMenu = nullptr;
        QAction* rectToolAction = nullptr;
        QAction* lassoToolAction = nullptr;
        QAction* polyToolAction = nullptr;
        QAction* ellipseToolAction = nullptr;

        SARibbonPanel* featurePanel = nullptr;
        SARibbonPanel* defectPanel = nullptr;
        QAction* ballAction = nullptr;
        QAction* planeAction = nullptr;
        QAction* planeDefectAction = nullptr;
        QAction* cylinderDefectAction = nullptr;
        QAction* cylinderAction = nullptr;
        QAction* cylinderErrorAction = nullptr;
        QAction* sphereAction = nullptr;
        QAction* extractAction = nullptr;

        SARibbonPanel* filterPanel = nullptr;
        QAction* VoxelFilterAction = nullptr;
        QAction* sorFilterAction = nullptr;
        QAction* passFilterAction = nullptr;

        SARibbonCategory* settingCategory = nullptr;
        SARibbonPanel* settingPanel = nullptr;
        QAction* settingAction = nullptr;

        SARibbonCategory* helpCategory = nullptr;
        SARibbonPanel* helpPanel = nullptr;
        QAction* helpAction = nullptr;

        QTabWidget* centerTabWidget = nullptr;
        QTabWidget* sideTabWidget = nullptr;
        pointcloud::CloudWindow* cloudWidget = nullptr;
        ImageWidget* imageWidget = nullptr;
        ImageWidget* centerLineWidget = nullptr;
        ImageWidget* depthWidget = nullptr;
        ImageWidget* compositeWidget = nullptr;

        AdvancedStatusBar* statusBar = nullptr;

        pointcloud::CloudTreeWidget* treeWidget = nullptr;

        scanner::ScannerInterface* scannerWidget = nullptr;
        scanner::Scanner* scanner = nullptr;

        scanner::Reconstructor* reconstructor = nullptr;
        scanner::ImageThread* imageThread = nullptr;

        QThread* serverThread = nullptr;
        server::RemoteCommandServer* commandServer = nullptr;
        server::RemoteCommandDialog* serverDialog = nullptr;
        SARibbonPanel* serverPanel = nullptr;
        QAction* startServerAction = nullptr;

        cv::Mat centerLineImage;
        cv::Mat centerIntensityAccumImage;
        cv::Mat depthAccumImage;
        cv::Mat depthImage;
        cv::Mat latestReconstructionImage;
        std::vector<Eigen::Vector3f> depthPreviewPoints;
        std::vector<scanner::AssociatedPoints> offlineDepthAssociatedPoints;
        cv::Size offlineDepthImageSize;
        int depthPreviewAxis = 0;
        std::vector<cv::Point2d> linePoints;
        bool scannerDisconnectWarningShown = false;
        bool scannerConnectionEstablished = false;
        bool suppressDepthPreviewFromCloudSelection = false;
        QHash<QString, QString> cloudInfoMessages;
        QHash<QString, CloudLegendInfo> cloudLegendInfos;
        QHash<QString, FitCloudContext> fitCloudContexts;
        QString offlineReconstructionSessionId;

        void updateDepthPreview();
        void hideCloudsByNames(const QStringList& names);
        void updateCenterIntensityPreview();
        void refreshCenterlineOverlayPreview();
        void refreshDisplayedImage();
        double promptStandardDiameter(const QString& title,
                                      const QString& label,
                                      double minimumValue,
                                      bool allowZero,
                                      bool* ok,
                                      double fallbackValue = -1.0) const;
        void updateFitCloudStandardDiameter(const QString& cloudId);

    private slots:
        void onNewActionTriggered();
        void onOpenActionTriggered();
        void onSavePointcloudActionTriggered();
        void onSaveImageActionTriggered();
        void onSaveDepthActionTriggered();
        void onSaveCenterLineActionTriggered();
        void onConnActionTriggered();
        void onPreviewActionTriggered();
        void onStopPreviewActionTriggered();
        void onScanActionTriggered();
        void onStopScanActionTriggered();
        void onStaticReconstructionActionTriggered();
        void onGuideInActionTriggered();
        void onSelectAllActionTriggered();
        void onSelectNoneActionTriggered();
        void onDeleteActionTriggered();
        void onReverseActionTriggered();
        void onRotateCenterActionTriggered();
        void onSelectToolActionTriggered();
        void onVoxelFilterActionTriggered();
        void onSorFilterActionTriggered();
        void onPassFilterActionTriggered();
        void onCylinderFeatureActionTriggered();
        void onCylinderErrorAnalysisActionTriggered();
        void onSphereFeatureActionTriggered();
        void onDoubleSphereFeatureActionTriggered();
        void onPlaneFeatureActionTriggered();
        void onPlaneDefectMeasurementActionTriggered();
        void onCylinderDefectMeasurementActionTriggered();
        void onHelpActionTriggered();
        void onSettingActionTriggered();
        void onStartServerActionTriggered();
        void refreshAllClouds();
        void onSelectionFinished(const std::vector<QPoint>& polygon);

    public:
        explicit UWScanMainWindow(QWidget* parent = nullptr);
        ~UWScanMainWindow() noexcept;
    };
}}

#endif
