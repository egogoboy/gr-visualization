#include "map_widget.h"
#include <qboxlayout.h>
#include <qnamespace.h>
#include <QGraphicsPathItem>
#include <QPixmap>
#include <QPushButton>
#include <QWheelEvent>
#include <QtMath>
#include <iostream>

map_widget::map_widget(QWidget* parent) : QWidget(parent)
{
    view_ = new QGraphicsView(this);
    scene_ = new QGraphicsScene(this);
    view_->setScene(scene_);

    view_->setRenderHint(QPainter::Antialiasing);
    view_->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);

    constexpr qreal initialScale = 1000.0;
    view_->scale(initialScale, initialScale);
    scaleFactor_ = initialScale;

    scene_->setSceneRect(-180, -90, 360, 180);

    constexpr double shiftLon = -1;
    constexpr double shiftLat = 5.44;
    constexpr double minLon = 22.377378812499998 + shiftLon;
    constexpr double maxLon = 55.336363187500005 + shiftLon;
    constexpr double minLat = 48.2132640860518 + shiftLat;
    constexpr double maxLat = 65.75969768682313 + shiftLat;

    QPixmap mapPixmap("/usr/local/share/gr-visualization_assets/mok_map.jpeg");
    if (!mapPixmap.isNull()) {
        std::cout << "Found map" << std::endl;
        QGraphicsPixmapItem* mapItem = scene_->addPixmap(mapPixmap);

        double lonPerPixel = (maxLon - minLon) / mapPixmap.width();
        double latPerPixel = (maxLat - minLat) / mapPixmap.height() * 1.5;

        mapItem->setTransform(QTransform::fromScale(lonPerPixel, latPerPixel));

        mapItem->setPos(minLon, -maxLat);

        mapItem->setZValue(-1);
    } else {
        std::cout << "Cannot find map" << std::endl;
    }

    qreal radius = 5.0;
    airplaneItem_ = scene_->addEllipse(
        -radius, -radius, 2 * radius, 2 * radius, QPen(Qt::red), QBrush(Qt::red));
    airplaneItem_->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);

    view_->centerOn(airplaneItem_);

    QPushButton* centerButton = new QPushButton("Выключить центрирование", this);
    QPushButton* zoomInButton = new QPushButton("+", this);
    QPushButton* zoomOutButton = new QPushButton("-", this);

    connect(centerButton, &QPushButton::clicked, [this, centerButton]() {
        followAirplane_ = !followAirplane_;
        if (followAirplane_) {
            centerButton->setText("Выключить центрирование");
        } else {
            centerButton->setText("Включить центирование");
        }
    });

    constexpr qreal zoomStep = 1.15;
    connect(zoomInButton, &QPushButton::clicked, [this]() {
        view_->scale(zoomStep, zoomStep);
    });

    connect(zoomOutButton, &QPushButton::clicked, [this]() {
        view_->scale(1 / zoomStep, 1 / zoomStep);
    });

    routePen_ = QPen(Qt::blue);
    routePen_.setWidth(0.1);
    routeItem_ = scene_->addPath(routePath_, routePen_);

    QVBoxLayout* layout = new QVBoxLayout(this);
    QHBoxLayout* buttons_layout = new QHBoxLayout(this);

    layout->addWidget(view_);

    buttons_layout->addWidget(centerButton);
    buttons_layout->addWidget(zoomInButton);
    buttons_layout->addWidget(zoomOutButton);

    layout->addLayout(buttons_layout);
    setLayout(layout);
}

QPointF map_widget::convertLatLonToScene(double lat, double lon)
{
    return QPointF(lon, -lat);
}

void map_widget::updatePosition(double lat, double lon)
{
    QPointF pos = convertLatLonToScene(lat, lon);
    airplaneItem_->setPos(pos);

    if (hasLastPos_) {
        auto* line = scene_->addLine(QLineF(lastPos_, pos), routePen_);
        line->setZValue(-1);
    }

    lastPos_ = pos;
    hasLastPos_ = true;

    if (followAirplane_) {
        view_->centerOn(airplaneItem_);
    }
}

void map_widget::setHeading(double degrees) { airplaneItem_->setRotation(degrees); }
