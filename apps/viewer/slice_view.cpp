#include "slice_view.h"

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>

namespace {

const QColor kBackground(250, 250, 247);
const QColor kGrid(232, 232, 226);
const QColor kBounds(170, 170, 160);
const QColor kContour(90, 140, 220, 110);   // wide translucent band under the paths
const QColor kHoleContour(160, 70, 210, 130);
const QColor kOpenContour(220, 30, 30);
const QColor kPerimeter(225, 105, 15);
const QColor kInfill(30, 150, 75);
const QColor kTravel(120, 120, 120);

}  // namespace

SliceView::SliceView(QWidget* parent) : QWidget(parent) {
    setMinimumSize(320, 240);
    setMouseTracking(false);
    setFocusPolicy(Qt::StrongFocus);
    setAttribute(Qt::WA_OpaquePaintEvent);
}

void SliceView::setLayers(std::shared_ptr<const std::vector<ls::Layer>> layers, const ls::BBox3& bounds) {
    layers_ = std::move(layers);
    const bool sameModel = bounds_.valid && bounds.valid && bounds.min == bounds_.min && bounds.max == bounds_.max;
    bounds_ = bounds;
    if (!sameModel) fitToView();  // re-slicing the same model keeps the current zoom/pan
    update();
}

void SliceView::setLayer(int index) {
    layer_ = index;
    update();
}

void SliceView::setShowContours(bool on) {
    showContours_ = on;
    update();
}
void SliceView::setShowPerimeters(bool on) {
    showPerimeters_ = on;
    update();
}
void SliceView::setShowInfill(bool on) {
    showInfill_ = on;
    update();
}
void SliceView::setShowTravel(bool on) {
    showTravel_ = on;
    update();
}

void SliceView::fitToView() {
    if (!bounds_.valid || !isVisible()) {
        fitPending_ = true;
        update();
        return;
    }
    const ls::Vec3 size = bounds_.size();
    const double w = std::max(size.x, 1e-9), h = std::max(size.y, 1e-9);
    scale_ = 0.9 * std::min((width() - 40) / w, (height() - 40) / h);
    center_ = QPointF(0.5 * (bounds_.min.x + bounds_.max.x), 0.5 * (bounds_.min.y + bounds_.max.y));
    fitPending_ = false;
    update();
}

QPointF SliceView::toScreen(double x, double y) const {
    return {width() * 0.5 + (x - center_.x()) * scale_, height() * 0.5 - (y - center_.y()) * scale_};
}

QPointF SliceView::toWorld(QPointF s) const {
    return {center_.x() + (s.x() - width() * 0.5) / scale_, center_.y() - (s.y() - height() * 0.5) / scale_};
}

void SliceView::resizeEvent(QResizeEvent*) {
    if (fitPending_) fitToView();
}

void SliceView::showEvent(QShowEvent*) {
    if (fitPending_) fitToView();
}

void SliceView::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.fillRect(rect(), kBackground);
    p.setRenderHint(QPainter::Antialiasing, true);

    // Grid at a "nice" spacing of roughly 50 px.
    const double raw = 50.0 / scale_;
    const double mag = std::pow(10.0, std::floor(std::log10(raw)));
    const double step = raw / mag < 2 ? 2 * mag : raw / mag < 5 ? 5 * mag : 10 * mag;
    const QPointF tl = toWorld({0, 0}), br = toWorld({double(width()), double(height())});
    p.setPen(QPen(kGrid, 1));
    for (double x = std::floor(tl.x() / step) * step; x <= br.x(); x += step) {
        p.drawLine(toScreen(x, tl.y()), toScreen(x, br.y()));
    }
    for (double y = std::floor(br.y() / step) * step; y <= tl.y(); y += step) {
        p.drawLine(toScreen(tl.x(), y), toScreen(br.x(), y));
    }

    if (bounds_.valid) {
        p.setPen(QPen(kBounds, 1, Qt::DashLine));
        p.setBrush(Qt::NoBrush);
        p.drawRect(QRectF(toScreen(bounds_.min.x, bounds_.max.y), toScreen(bounds_.max.x, bounds_.min.y)));
    }

    if (layers_ && layer_ >= 0 && layer_ < static_cast<int>(layers_->size())) {
        const ls::Layer& L = (*layers_)[static_cast<std::size_t>(layer_)];

        if (showContours_) {
            for (const ls::Contour& c : L.contours) {
                if (c.points.size() < 2) continue;
                QPainterPath path(toScreen(c.points[0].x, c.points[0].y));
                for (std::size_t i = 1; i < c.points.size(); ++i) path.lineTo(toScreen(c.points[i].x, c.points[i].y));
                if (c.closed) path.closeSubpath();
                const QColor col = !c.closed ? kOpenContour : c.isHole() ? kHoleContour : kContour;
                p.setPen(QPen(col, c.closed ? 9 : 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
                p.drawPath(path);
            }
        }
        if (showInfill_) {
            QVector<QLineF> lines;
            lines.reserve(static_cast<int>(L.paths.infill.size()));
            for (const ls::ScanLine& s : L.paths.infill) lines.append(QLineF(toScreen(s.a.x, s.a.y), toScreen(s.b.x, s.b.y)));
            p.setPen(QPen(kInfill, 1.1));
            p.drawLines(lines);
        }
        if (showPerimeters_) {
            p.setPen(QPen(kPerimeter, 1.8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            for (const auto& loop : L.paths.perimeters) {
                QPolygonF poly;
                poly.reserve(static_cast<int>(loop.size()));
                for (const ls::Vec2& v : loop) poly.append(toScreen(v.x, v.y));
                p.drawPolyline(poly);
            }
        }
        if (showTravel_) {
            // Laser-off jumps between consecutive passes.
            p.setPen(QPen(kTravel, 0.8, Qt::DotLine));
            bool have = false;
            ls::Vec2 pos{};
            auto jump = [&](ls::Vec2 to) {
                if (have) p.drawLine(toScreen(pos.x, pos.y), toScreen(to.x, to.y));
                have = true;
            };
            for (const auto& loop : L.paths.perimeters) {
                jump(loop.front());
                pos = loop.back();
            }
            for (const ls::ScanLine& s : L.paths.infill) {
                jump(s.a);
                pos = s.b;
            }
        }
    } else {
        p.setPen(QColor(120, 120, 120));
        p.drawText(rect(), Qt::AlignCenter, tr("Open an STL file (File > Open, or pass a path on the command line)"));
    }
    drawLegend(p);
}

void SliceView::drawLegend(QPainter& p) const {
    struct Item {
        QColor color;
        QString label;
        double width;
    };
    const Item items[] = {{kContour, tr("outer contour"), 9},  {kHoleContour, tr("hole contour"), 9},
                          {kPerimeter, tr("perimeter pass"), 2}, {kInfill, tr("infill hatch"), 2},
                          {kOpenContour, tr("open contour"), 3}};
    p.setRenderHint(QPainter::Antialiasing, true);
    const int x = 14, lineH = 20;
    int y = 14;
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(255, 255, 255, 220));
    p.drawRoundedRect(QRectF(x - 6, y - 6, 150, lineH * 5 + 8), 6, 6);
    QFont f = p.font();
    f.setPointSizeF(9);
    p.setFont(f);
    for (const Item& it : items) {
        p.setPen(QPen(it.color, it.width, Qt::SolidLine, Qt::RoundCap));
        p.drawLine(QPointF(x + 2, y + lineH / 2.0), QPointF(x + 28, y + lineH / 2.0));
        p.setPen(QColor(50, 50, 50));
        p.drawText(QRectF(x + 36, y, 110, lineH), Qt::AlignVCenter | Qt::AlignLeft, it.label);
        y += lineH;
    }
}

void SliceView::wheelEvent(QWheelEvent* event) {
    const QPointF anchor = event->position();
    const QPointF before = toWorld(anchor);
    const double factor = std::pow(1.0015, event->angleDelta().y());
    scale_ = std::clamp(scale_ * factor, 1e-3, 1e6);
    const QPointF after = toWorld(anchor);
    center_ += before - after;  // keep the point under the cursor fixed
    update();
    event->accept();
}

void SliceView::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton || event->button() == Qt::MiddleButton) {
        dragging_ = true;
        lastMouse_ = event->position();
        setCursor(Qt::ClosedHandCursor);
    }
}

void SliceView::mouseMoveEvent(QMouseEvent* event) {
    if (!dragging_) return;
    const QPointF d = event->position() - lastMouse_;
    lastMouse_ = event->position();
    center_ -= QPointF(d.x() / scale_, -d.y() / scale_);
    update();
}

void SliceView::mouseReleaseEvent(QMouseEvent*) {
    dragging_ = false;
    unsetCursor();
}

void SliceView::mouseDoubleClickEvent(QMouseEvent*) { fitToView(); }
