#include "main_window.h"

#include <QAction>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QMenuBar>
#include <QMessageBox>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QStatusBar>
#include <QToolBar>
#include <QVBoxLayout>
#include <thread>

#include "layerslicer/stl_io.h"
#include "slice_view.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setWindowTitle(tr("Layer Slicer"));

    // All widgets below are owned by Qt's parent-child hierarchy.
    auto* central = new QWidget(this);
    auto* root = new QVBoxLayout(central);
    root->setContentsMargins(6, 6, 6, 6);

    auto* controls = new QHBoxLayout;
    layerHeight_ = new QDoubleSpinBox(central);
    layerHeight_->setRange(0.01, 10.0);
    layerHeight_->setDecimals(3);
    layerHeight_->setSingleStep(0.05);
    layerHeight_->setValue(0.2);
    spacing_ = new QDoubleSpinBox(central);
    spacing_->setRange(0.05, 20.0);
    spacing_->setDecimals(2);
    spacing_->setSingleStep(0.1);
    spacing_->setValue(0.5);
    angles_ = new QComboBox(central);
    angles_->addItem(tr("0 / 90"), QVariant::fromValue(QList<double>{0, 90}));
    angles_->addItem(tr("0 / 45 / 90 / 135"), QVariant::fromValue(QList<double>{0, 45, 90, 135}));
    angles_->addItem(tr("45 / 135"), QVariant::fromValue(QList<double>{45, 135}));
    angles_->addItem(tr("0 (fixed)"), QVariant::fromValue(QList<double>{0}));
    angles_->addItem(tr("67 (rotating)"), QVariant::fromValue(QList<double>{0, 67, 134, 201, 268, 335}));
    controls->addWidget(new QLabel(tr("Layer height"), central));
    controls->addWidget(layerHeight_);
    controls->addSpacing(12);
    controls->addWidget(new QLabel(tr("Hatch spacing"), central));
    controls->addWidget(spacing_);
    controls->addSpacing(12);
    controls->addWidget(new QLabel(tr("Hatch angles"), central));
    controls->addWidget(angles_);
    controls->addSpacing(12);
    auto* showContours = new QCheckBox(tr("Contours"), central);
    auto* showPerims = new QCheckBox(tr("Perimeter"), central);
    auto* showInfill = new QCheckBox(tr("Infill"), central);
    auto* showTravel = new QCheckBox(tr("Travel"), central);
    showContours->setChecked(true);
    showPerims->setChecked(true);
    showInfill->setChecked(true);
    for (QCheckBox* c : {showContours, showPerims, showInfill, showTravel}) controls->addWidget(c);
    controls->addStretch(1);
    root->addLayout(controls);

    view_ = new SliceView(central);
    root->addWidget(view_, 1);

    auto* layerRow = new QHBoxLayout;
    slider_ = new QSlider(Qt::Horizontal, central);
    layerSpin_ = new QSpinBox(central);
    slider_->setRange(0, 0);
    layerSpin_->setRange(0, 0);
    layerRow->addWidget(new QLabel(tr("Layer"), central));
    layerRow->addWidget(slider_, 1);
    layerRow->addWidget(layerSpin_);
    root->addLayout(layerRow);

    info_ = new QLabel(central);
    info_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    QFont mono = info_->font();
    mono.setFamily(QStringLiteral("Consolas"));
    mono.setStyleHint(QFont::Monospace);
    info_->setFont(mono);
    root->addWidget(info_);
    setCentralWidget(central);

    QMenu* fileMenu = menuBar()->addMenu(tr("&File"));
    QAction* openAct = fileMenu->addAction(tr("&Open STL..."), this, &MainWindow::openFile);
    openAct->setShortcut(QKeySequence::Open);
    fileMenu->addSeparator();
    QAction* quitAct = fileMenu->addAction(tr("&Quit"), this, &QWidget::close);
    quitAct->setShortcut(QKeySequence::Quit);
    QMenu* viewMenu = menuBar()->addMenu(tr("&View"));
    QAction* fitAct = viewMenu->addAction(tr("&Fit to view"), view_, &SliceView::fitToView);
    fitAct->setShortcut(Qt::Key_F);

    QToolBar* tb = addToolBar(tr("Main"));
    tb->setMovable(false);
    tb->addAction(openAct);
    tb->addAction(fitAct);

    connect(slider_, &QSlider::valueChanged, this, &MainWindow::setLayer);
    connect(layerSpin_, qOverload<int>(&QSpinBox::valueChanged), this, &MainWindow::setLayer);
    connect(layerHeight_, &QDoubleSpinBox::editingFinished, this, &MainWindow::reslice);
    connect(spacing_, &QDoubleSpinBox::editingFinished, this, &MainWindow::reslice);
    connect(angles_, qOverload<int>(&QComboBox::currentIndexChanged), this, &MainWindow::reslice);
    connect(showContours, &QCheckBox::toggled, view_, &SliceView::setShowContours);
    connect(showPerims, &QCheckBox::toggled, view_, &SliceView::setShowPerimeters);
    connect(showInfill, &QCheckBox::toggled, view_, &SliceView::setShowInfill);
    connect(showTravel, &QCheckBox::toggled, view_, &SliceView::setShowTravel);

    resize(1200, 820);
    updateInfo();
}

MainWindow::~MainWindow() {
    // Child spin boxes emit editingFinished when they lose focus during
    // teardown; make sure that cannot call back into a half-destroyed window.
    for (QObject* sender : {static_cast<QObject*>(layerHeight_), static_cast<QObject*>(spacing_),
                            static_cast<QObject*>(angles_), static_cast<QObject*>(slider_),
                            static_cast<QObject*>(layerSpin_)}) {
        QObject::disconnect(sender, nullptr, this, nullptr);
    }
}

bool MainWindow::loadFile(const QString& path, QString* error) {
    try {
        mesh_ = std::make_shared<const ls::Mesh>(ls::loadStl(std::filesystem::path(path.toStdWString())));
    } catch (const std::exception& e) {
        const QString msg = tr("Could not load %1:\n%2").arg(path, QString::fromUtf8(e.what()));
        if (error) *error = msg;
        return false;
    }
    fileName_ = QFileInfo(path).fileName();
    setWindowTitle(tr("Layer Slicer - %1").arg(fileName_));
    reslice();
    setLayer(layerCount() / 2);
    view_->fitToView();
    return true;
}

void MainWindow::openFile() {
    const QString path =
        QFileDialog::getOpenFileName(this, tr("Open STL"), QString(), tr("STL files (*.stl *.STL);;All files (*)"));
    if (path.isEmpty()) return;
    QString error;
    if (!loadFile(path, &error)) QMessageBox::warning(this, tr("Layer Slicer"), error);
}

void MainWindow::reslice() {
    if (!mesh_) return;
    ls::SliceOptions opts;
    opts.paths.hatchSpacing = spacing_->value();
    opts.paths.anglesDeg.clear();
    for (double a : angles_->currentData().value<QList<double>>()) opts.paths.anglesDeg.push_back(a);

    QElapsedTimer timer;
    timer.start();
    const ls::Slicer slicer(mesh_, opts);
    const auto zs = ls::Slicer::uniformHeights(mesh_->bounds(), layerHeight_->value());
    layers_ = std::make_shared<const std::vector<ls::Layer>>(slicer.sliceAll(zs, 0));
    sliceMs_ = static_cast<double>(timer.nsecsElapsed()) / 1e6;

    const int keep = slider_->value();
    const int maxLayer = std::max(0, static_cast<int>(layers_->size()) - 1);
    {
        const QSignalBlocker b1(slider_);
        const QSignalBlocker b2(layerSpin_);
        slider_->setRange(0, maxLayer);
        layerSpin_->setRange(0, maxLayer);
    }
    view_->setLayers(layers_, mesh_->bounds());
    setLayer(std::min(keep, maxLayer));
}

int MainWindow::layerCount() const { return layers_ ? static_cast<int>(layers_->size()) : 0; }

void MainWindow::setLayerHeight(double h) {
    layerHeight_->setValue(h);
    reslice();
}

void MainWindow::setHatchSpacing(double s) {
    spacing_->setValue(s);
    reslice();
}

void MainWindow::setLayer(int index) {
    const QSignalBlocker b1(slider_);
    const QSignalBlocker b2(layerSpin_);
    slider_->setValue(index);
    layerSpin_->setValue(index);
    view_->setLayer(index);
    updateInfo();
}

void MainWindow::updateInfo() {
    if (!layers_ || layers_->empty()) {
        info_->setText(tr("No model loaded."));
        return;
    }
    const int i = std::clamp(slider_->value(), 0, static_cast<int>(layers_->size()) - 1);
    const ls::Layer& L = (*layers_)[static_cast<std::size_t>(i)];
    int outer = 0, holes = 0;
    for (const ls::Contour& c : L.contours) {
        outer += c.isOuter() ? 1 : 0;
        holes += c.isHole() ? 1 : 0;
    }
    info_->setText(tr("%1  |  %2 triangles  |  layer %3/%4  z = %5  |  contours %6 (outer %7, holes %8, open %9)\n"
                      "scan path %10 (perimeter %11 + infill %12, %13 lines @ %14 deg)  travel %15  |  "
                      "sliced %16 layers in %17 ms on %18 threads")
                       .arg(fileName_)
                       .arg(mesh_->triangleCount())
                       .arg(i)
                       .arg(layers_->size() - 1)
                       .arg(L.z, 0, 'f', 3)
                       .arg(L.contours.size())
                       .arg(outer)
                       .arg(holes)
                       .arg(L.openContours)
                       .arg(L.paths.scanLength(), 0, 'f', 1)
                       .arg(L.paths.perimeterLength, 0, 'f', 1)
                       .arg(L.paths.infillLength, 0, 'f', 1)
                       .arg(L.paths.infill.size())
                       .arg(L.paths.angleDeg, 0, 'f', 0)
                       .arg(L.paths.travelLength, 0, 'f', 1)
                       .arg(layers_->size())
                       .arg(sliceMs_, 0, 'f', 1)
                       .arg(std::max(1u, std::thread::hardware_concurrency())));
}
