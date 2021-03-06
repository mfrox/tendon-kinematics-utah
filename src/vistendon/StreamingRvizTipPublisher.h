#ifndef STREAMING_RVIZ_TIP_PUBLISHER_H
#define STREAMING_RVIZ_TIP_PUBLISHER_H

#include <vistendon/marker_array_conversions.h>
#include <vistendon/shapes.h>

#include <QColor>
#include <QObject>
#include <QTimer>
#include <QVector3D>

#include <Eigen/Core>

#include <rclcpp/node.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <string>

namespace vistendon {

/** Publishes the tip position at most 60 Hz and at least .5 Hz
 *
 * Every time the tip is updated, it will mark the next 60 Hz frame to publish
 * it.  So feel free to update the tip as often as you wish.
 */
class StreamingRvizTipPublisher : public QObject {
  Q_OBJECT

public:
  using Marker= visualization_msgs::msg::Marker;

  StreamingRvizTipPublisher(
      rclcpp::Node::SharedPtr node,
      const std::string &frame = "/map",
      const std::string &rviz_namespace = "tip-publisher")
    : _tip()
    , _color(0, 255, 0, 255) // solid green
    , _is_new(false)
    , _radius(0.002)
    , _force_timer(this)
    , _fast_timer(this)
  {
    _publisher = node->create_publisher<Marker>(
        "visualization_marker", 10);
    populate_marker_attributes(_marker, rviz_namespace, frame);

    // connect the timers
    connect(&_force_timer, &QTimer::timeout,
            this, &StreamingRvizTipPublisher::force_publish);
    connect(&_fast_timer, &QTimer::timeout,
            this, &StreamingRvizTipPublisher::publish_if_new);

  }

  double radius() const { return _radius; }

public slots:
  /// start the timers
  void start() {
    if (!_force_timer.isActive()) {
      _force_timer.start(2000); // every 2 seconds (0.5 Hz)
    }
    if (!_fast_timer.isActive()) {
      _fast_timer .start(  17); // at 60 Hz
    }
  }

  void stop() {
    _force_timer.stop();
    _fast_timer.stop();
  }

  void set_radius(double r) {
    _radius = r;
  }

  void update_tip(const QVector3D &tip) {
    this->start();
    _tip = tip;
    _is_new = true;
  }

  void force_publish() {
    Eigen::Vector3d tip(_tip.x(), _tip.y(), _tip.z());
    populate_marker(_marker, Sphere{tip, _radius});
    auto color = _color;
    _marker.color.r = color.redF();
    _marker.color.g = color.greenF();
    _marker.color.b = color.blueF();
    _marker.color.a = color.alphaF();
    qDebug() << "publishing tip marker: ["
             << _tip.x() << _tip.y() << _tip.z() << "]";
    this->_publisher->publish(_marker);
  }

  void publish_if_new() {
    if (_is_new) {
      force_publish();
      _is_new = false;
    }
  }

  const QColor& color() const { return _color; }
  void set_color(const QColor &color) {
    _color = color;
    _is_new = true;
  }

  void set_yellow() { set_color(QColor{255, 255,   0, 255}); }
  void set_green()  { set_color(QColor{  0, 255,   0, 255}); }

private:
  QVector3D _tip;
  QColor _color;
  bool _is_new;
  rclcpp::Publisher<Marker>::SharedPtr _publisher;
  Marker _marker;
  double _radius;
  QTimer _force_timer;
  QTimer _fast_timer;
};

} // end of namespace vistendon

#endif // STREAMING_RVIZ_TIP_PUBLISHER_H
