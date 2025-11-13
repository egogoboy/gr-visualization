#pragma once

#include <qpoint.h>
#include <QGraphicsPathItem>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QPen>
#include <QPointF>
#include <QWidget>

class map_widget : public QWidget
{
    Q_OBJECT
public:
    explicit map_widget(QWidget* parent = nullptr);

    void updatePosition(double lat, double lon);

    void setHeading(double degrees);

private:
    QPointF convertLatLonToScene(double lat, double lon);

    QGraphicsView* view_;
    QGraphicsScene* scene_;
    QGraphicsEllipseItem* airplaneItem_;
    QGraphicsPathItem* routeItem_;
    QPainterPath routePath_;

    QPen routePen_;
    QPointF lastPos_;
    bool hasLastPos_ = false;

    qreal scaleFactor_;
    bool followAirplane_ = true;
};
