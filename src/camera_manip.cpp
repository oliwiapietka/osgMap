#include <osgGA/CameraManipulator>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIActionAdapter>
#include <osg/Matrix>
#include <osg/Vec3d>
#include <osg/BoundingSphere>
#include <osg/Node>
#include <algorithm>
#include <cmath>

// A Google-Maps-like camera manipulator for large (ECEF) coordinates.
// Panning follows local tangent (East/North) frame, using a fixed local frame.
class GoogleMapsManipulator : public osgGA::CameraManipulator {
public:
    GoogleMapsManipulator()
        : _center(0.0, 0.0, 0.0), _distance(100.0), _lastX(0.0f), _lastY(0.0f),
          _fixedUp(0.0, 0.0, 1.0), _fixedNorth(0.0, 1.0, 0.0), _fixedEast(1.0, 0.0, 0.0)
    {}

    const char* className() const override { return "GoogleMapsManipulator"; }

private:
    // --- Constants ---
    static constexpr double kMinDistance = 10.0; // Minimum camera-ground distance
    static constexpr double kScrollFactor = 0.8; // Zoom multiplier

    // Reset center/distance to scene bounds when available, and compute fixed frame once.
    void resetFromBounds()
    {
        if (!_node.valid()) return;
        osg::BoundingSphere bs = _node->getBound();
        if (!bs.valid()) return;
        _center = bs.center();
        _distance = std::max(100.0, bs.radius() * 2.5);

        // One-time tangent frame derived from center (ECEF radial as Up).
        _fixedUp = _center;
        _fixedUp.normalize();
        osg::Vec3d globalNorth(0.0, 0.0, 1.0);
        _fixedEast = globalNorth ^ _fixedUp;
        _fixedEast.normalize();
        _fixedNorth = _fixedUp ^ _fixedEast;
        _fixedNorth.normalize();
    }

public:
    // --- CameraManipulator overrides ---

    void setNode(osg::Node* node) override
    {
        _node = node;
        osgGA::CameraManipulator::setNode(node);
        if (_node.valid())
        {
            resetFromBounds();
        }
    }

    osg::Node* getNode() override { return _node.get(); }
    const osg::Node* getNode() const override { return _node.get(); }

    void setByMatrix(const osg::Matrixd& /*matrix*/) override
    { 
        // not needed for this manipulator
    }

    void setByInverseMatrix(const osg::Matrixd& /*inv*/) override
    {
        // not needed for this manipulator
    }

    osg::Matrixd getMatrix() const override
    {
        return osg::Matrixd::inverse(getInverseMatrix());
    }

    osg::Matrixd getInverseMatrix() const override
    {
        // Eye is located along fixed Up at the current distance.
        const osg::Vec3d eye = _center + (_fixedUp * _distance);
        return osg::Matrixd::lookAt(eye, _center, _fixedNorth);
    }

    void home(double /*currentTime*/) override
    {
        resetFromBounds();
        _distance = std::max(_distance, kMinDistance);
    }

    void home(const osgGA::GUIEventAdapter&, osgGA::GUIActionAdapter&) override
    {
        home(0.0);
    }

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) override
    {
        switch (ea.getEventType())
        {
            case osgGA::GUIEventAdapter::PUSH:
                _lastX = ea.getXnormalized();
                _lastY = ea.getYnormalized();
                return true;

            case osgGA::GUIEventAdapter::DRAG: {
                if (!(ea.getButtonMask() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON))
                    return false;

                const float x = ea.getXnormalized();
                const float y = ea.getYnormalized();
                const float dx = x - _lastX;
                const float dy = y - _lastY;

                _center -= (_fixedEast * static_cast<double>(dx) * _distance);
                _center -= (_fixedNorth * static_cast<double>(dy) * _distance);

                _lastX = x;
                _lastY = y;
                aa.requestRedraw();
                return true;
            }

            case osgGA::GUIEventAdapter::KEYDOWN:
                if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Home)
                {
                    home(ea, aa);
                    aa.requestRedraw();
                    return true;
                }
                return false;

            case osgGA::GUIEventAdapter::SCROLL: {
                switch (ea.getScrollingMotion())
                {
                    case osgGA::GUIEventAdapter::SCROLL_UP:
                        _distance *= kScrollFactor;
                        break;
                    case osgGA::GUIEventAdapter::SCROLL_DOWN:
                        _distance /= kScrollFactor;
                        break;
                    default: break;
                }
                _distance = std::max(_distance, kMinDistance);
                aa.requestRedraw();
                return true;
            }

            default: return false;
        }
    }

private:
    osg::observer_ptr<osg::Node> _node;
    osg::Vec3d _center;
    double _distance;
    float _lastX, _lastY;

    // Cached local frame
    osg::Vec3d _fixedUp;
    osg::Vec3d _fixedNorth;
    osg::Vec3d _fixedEast;
};