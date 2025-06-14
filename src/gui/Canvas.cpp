#include "Canvas.h"
#include "../core/WorkspaceState.h"
#include "../log/QgisDebug.h"
#include "OpenGLCanvas.h"

Canvas::Canvas(QWidget *parent) : QStackedWidget(parent) {
  init2DWidget();
  init3DWidget();
  setCurrentWidget(mpOpenGLWidget);
  connect(mpOpenGLWidget, &OpenGLCanvas::refreash3DParms, this, &Canvas::refreashParms);
  logMessage("create canvas", Qgis::MessageLevel::Success);
}

// switch to 3D
void Canvas::switchTo3D() {
  wsp::WindowManager::getInstance().setCurrentCanvas(wsp::CanvasType::ThreeD);
  setCurrentWidget(mpOpenGLWidget);
  logMessage("switch to 3D model view", Qgis::MessageLevel::Success);
}
// switch to 2D
void Canvas::switchTo2D() {
  wsp::WindowManager::getInstance().setCurrentCanvas(wsp::CanvasType::TwoD);
  setCurrentWidget(mpImageLabel); // switch to 2D map view
  logMessage("switch to 2D map view", Qgis::MessageLevel::Success);
}

void Canvas::viewReset() {
  Camera::getInstance().resetView();
  logMessage("reset view", Qgis::MessageLevel::Success);
}

void Canvas::init3DWidget() {
  mpOpenGLWidget = new OpenGLCanvas(this);
  addWidget(mpOpenGLWidget);
  logMessage("create 3D view widget", Qgis::MessageLevel::Success);
}
void Canvas::init2DWidget() {
  // create QLabel to display local image
  mpImageLabel = new QLabel(this);
  QPixmap mapImage(
      ":/schoolcore/map/capture.png"); // use resource path to load image
  if (mapImage.isNull()) {
    logMessage("failed to load local map image", Qgis::MessageLevel::Critical);
    return;
  }
  mpImageLabel->setPixmap(mapImage);
  mpImageLabel->setScaledContents(
      true); // let image adapt to label size, keep ratio
  mpImageLabel->setSizePolicy(QSizePolicy::Ignored,
                              QSizePolicy::Ignored); // set size policy
  addWidget(mpImageLabel);
  logMessage("create QLabel to display local map image",Qgis::MessageLevel::Success);
}

Canvas::~Canvas() {
    delete mpImageLabel;
    delete mpOpenGLWidget;
    logMessage("Canvas destroyed", Qgis::MessageLevel::Success);
}