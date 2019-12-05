#include "hexapod_parameter_view.hpp"

#include <Qt3DExtras/QPhongMaterial>

//#include <QtCore/QDebug>

// Refactor into a separate "get transforms for parameters" function/class?
#include "robot/hexapod.hpp"
#include "robot_model.hpp"

void makeSphere(Qt3DCore::QEntity*& sphere, Qt3DCore::QTransform*& sphere_transform, QRgb rgb,
                Qt3DCore::QEntity* parent, float radius)
{
  // Sphere shape data
  Qt3DExtras::QSphereMesh *sphereMesh = new Qt3DExtras::QSphereMesh();
  sphereMesh->setRings(20);
  sphereMesh->setSlices(20);
  sphereMesh->setRadius(radius);

  // Material
  Qt3DExtras::QPhongMaterial *sphereMaterial = new Qt3DExtras::QPhongMaterial();
  sphereMaterial->setDiffuse(QColor(rgb));

  // Sphere mesh transform
  sphere_transform = new Qt3DCore::QTransform();
  sphere_transform->setScale(1.0f);
  sphere_transform->setTranslation(QVector3D(0,0,0));

  // Entity
  sphere = new Qt3DCore::QEntity(parent);
  sphere->addComponent(sphereMesh);
  sphere->addComponent(sphereMaterial);
  sphere->addComponent(sphere_transform);

  sphere->setParent(parent);
}

void makeCylinder(Qt3DCore::QEntity*& cylinder, Qt3DCore::QTransform*& cylinder_transform, Qt3DExtras::QCylinderMesh*& cylinder_mesh, QRgb rgb,
                  Qt3DCore::QEntity* parent, float radius)
{
  // Cylinder shape data
  cylinder_mesh = new Qt3DExtras::QCylinderMesh();
  cylinder_mesh->setRadius(radius);
  cylinder_mesh->setRings(100);
  cylinder_mesh->setSlices(20);

  // CylinderMesh Transform
  cylinder_transform = new Qt3DCore::QTransform();
  cylinder_transform->setScale(1.0f);

  // Material
  Qt3DExtras::QPhongMaterial *cylinderMaterial = new Qt3DExtras::QPhongMaterial();
  cylinderMaterial->setDiffuse(QColor(rgb));

  // Cylinder
  cylinder = new Qt3DCore::QEntity(parent);
  cylinder->addComponent(cylinder_mesh);
  cylinder->addComponent(cylinderMaterial);
  cylinder->addComponent(cylinder_transform);
}

float dist(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  float dx = b.x() - a.x();
  float dy = b.y() - a.y();
  float dz = b.z() - a.z();
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

QQuaternion cylinderRotation(const Eigen::MatrixXd& transform)
{
  // NOTE: here, we postmultiply by [0 -1 0; 1 0 0; 0 0 1] to account for
  // cylinder coordinate frame.
  float rot[9] = {
    static_cast<float>(transform(0,1)), -static_cast<float>(transform(0,0)), static_cast<float>(transform(0,2)),
    static_cast<float>(transform(1,1)), -static_cast<float>(transform(1,0)), static_cast<float>(transform(1,2)),
    static_cast<float>(transform(2,1)), -static_cast<float>(transform(2,0)), static_cast<float>(transform(2,2)),
  };
  return QQuaternion::fromRotationMatrix(QMatrix3x3(rot));
}

QVector3D cylinderTranslation(const Eigen::Vector3d a, const Eigen::Vector3d b)
{
  return QVector3D(
    a.x() * 0.5 + b.x() * 0.5,
    a.y() * 0.5 + b.y() * 0.5,
    a.z() * 0.5 + b.z() * 0.5);
}

// Compute new kinematics and transform the visualized robot elements
// accordingly.
void HexapodParameterView::resetTransforms()
{
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > leg_bases(6);
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > leg_knees(6);
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > leg_ends(6);

  auto hexapod = hebi::Hexapod::createDummy(_params);
  Eigen::VectorXd angles;
  Eigen::VectorXd leg_vels;
  Eigen::MatrixXd jacobian_ee;
  hebi::robot_model::MatrixXdVector jacobian_com;
  Eigen::Vector3d body;
  body << 0, 0, 0;

  for (int i = 0; i < 6; ++i)
  {
    // Get positions from the hexapod class
    hebi::robot_model::Matrix4dVector frames;
    hexapod->getLeg(i)->setCmdStanceToHomeStance();
    hexapod->getLeg(i)->computeState(0, angles, leg_vels, jacobian_ee, jacobian_com);
    hexapod->getLeg(i)->getKinematics().getFK(robot_model::FrameType::Output, angles, frames);
    leg_bases[i] = frames[0].topRightCorner<3,1>();
    leg_knees[i] = frames[3].topRightCorner<3,1>();
    leg_ends[i]  = frames[5].topRightCorner<3,1>();

    // To shoulders
    _to_shoulder_mesh[i]->setLength(dist(body, leg_bases[i]));
    _to_shoulder_tf[i]->setRotation(cylinderRotation(frames[0]));
    _to_shoulder_tf[i]->setTranslation(cylinderTranslation(body, leg_bases[i]));

    // Shoulders
    _shoulder_tf[i]->setTranslation(
      QVector3D(leg_bases[i].x(), leg_bases[i].y(), leg_bases[i].z()));

    // Thighs
    _shoulder_to_elbow_mesh[i]->setLength(dist(leg_bases[i], leg_knees[i]));
    _shoulder_to_elbow_tf[i]->setRotation(cylinderRotation(frames[3]));
    _shoulder_to_elbow_tf[i]->setTranslation(cylinderTranslation(leg_bases[i], leg_knees[i]));

    // Knees
    _elbow_tf[i]->setTranslation(
      QVector3D(leg_knees[i].x(), leg_knees[i].y(), leg_knees[i].z()));

    // Lower legs
    _elbow_to_foot_mesh[i]->setLength(dist(leg_knees[i], leg_ends[i]));
    _elbow_to_foot_tf[i]->setRotation(cylinderRotation(frames[5]));
    _elbow_to_foot_tf[i]->setTranslation(cylinderTranslation(leg_knees[i], leg_ends[i]));  
  }  
}

HexapodParameterView::HexapodParameterView(Qt3DCore::QEntity *rootEntity)
    : _root(rootEntity)
{
  makeSphere(_body, _body_tf,
             QRgb(0x404040),
             _root, 0.1f);

  for (int i = 0; i < 6; ++i) 
  {
    // Links to shoulders
    makeCylinder(_to_shoulder[i], _to_shoulder_tf[i], _to_shoulder_mesh[i],
                 QRgb(0x404040),
                 _root, 0.03f);

    // Make joints (shoulders)
    makeSphere(_shoulder[i], _shoulder_tf[i],
               QRgb(0xb03030),
               _root, 0.05f);

    // Upper legs
    makeCylinder(_shoulder_to_elbow[i], _shoulder_to_elbow_tf[i], _shoulder_to_elbow_mesh[i],
                 QRgb(0x404040),
                 _root, 0.03f);

    // Make joints (elbows)
    makeSphere(_elbow[i], _elbow_tf[i],
               QRgb(0xb03030),
               _root, 0.05f);

    // Lower legs
    makeCylinder(_elbow_to_foot[i], _elbow_to_foot_tf[i], _elbow_to_foot_mesh[i],
                 QRgb(0x404040),
                 _root, 0.03f);
  }

}

HexapodParameterView::~HexapodParameterView()
{
}
