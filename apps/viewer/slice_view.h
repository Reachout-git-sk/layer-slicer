#pragma once

#include <QPointF>
#include <QWidget>
#include <memory>
#include <vector>

#include "layerslicer/slicer.h"

// 2D view of one slice: contours, perimeter pass and infill lines.
// Mouse wheel zooms around the cursor, left-drag pans, double-click fits.
class SliceView : public QWidget {
    Q_OBJECT
public:
    explicit SliceView(QWidget* parent = nullptr);

    void setLayers(std::shared_ptr<const std::vector<ls::Layer>> layers, const ls::BBox3& bounds);
    void setLayer(int index);
    int layer() const { return layer_; }

    void setShowContours(bool on);
    void setShowPerimeters(bool on);
    void setShowInfill(bool on);
    void setShowTravel(bool on);

public slots:
    void fitToView();

protected:
    void paintEvent(QPaintEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void showEvent(QShowEvent* event) override;

private:
    QPointF toScreen(double x, double y) const;
    QPointF toWorld(QPointF screen) const;
    void drawLegend(class QPainter& p) const;

    std::shared_ptr<const std::vector<ls::Layer>> layers_;
    ls::BBox3 bounds_;
    int layer_ = 0;
    bool showContours_ = true;
    bool showPerimeters_ = true;
    bool showInfill_ = true;
    bool showTravel_ = false;
    bool fitPending_ = true;

    double scale_ = 10.0;    // pixels per model unit
    QPointF center_{0, 0};   // model point shown at the widget centre
    bool dragging_ = false;
    QPointF lastMouse_;
};
