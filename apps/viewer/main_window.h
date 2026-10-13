#pragma once

#include <QMainWindow>
#include <memory>
#include <vector>

#include "layerslicer/mesh.h"
#include "layerslicer/slicer.h"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QSlider;
class QSpinBox;
class SliceView;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    bool loadFile(const QString& path, QString* error = nullptr);
    void setLayer(int index);
    int layerCount() const;
    void setLayerHeight(double h);
    void setHatchSpacing(double s);
    SliceView* view() const { return view_; }

private slots:
    void openFile();
    void reslice();
    void updateInfo();

private:
    SliceView* view_ = nullptr;
    QSlider* slider_ = nullptr;
    QSpinBox* layerSpin_ = nullptr;
    QDoubleSpinBox* layerHeight_ = nullptr;
    QDoubleSpinBox* spacing_ = nullptr;
    QComboBox* angles_ = nullptr;
    QLabel* info_ = nullptr;

    QString fileName_;
    std::shared_ptr<const ls::Mesh> mesh_;
    std::shared_ptr<const std::vector<ls::Layer>> layers_;
    double sliceMs_ = 0.0;
};
