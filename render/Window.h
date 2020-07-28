#ifndef __MASS_WINDOW_H__
#define __MASS_WINDOW_H__
#include "dart/dart.hpp"
#include <FL/gl.h>
#include <FL/glu.h>
#include <FL/Fl_Gl_Window.H>
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>

namespace p = boost::python;
namespace np = boost::python::numpy;

class Camera {
public:
    Camera(){
		this->center << 0., 0., 0.;
		this->rotateX = -30.0 / 180. * M_PI;
		this->rotateY = 0.;
		this->distance = 20.;
		this->center_offset << 0., 0., 0.;
	}

    Eigen::Affine3d getSE3() {
		Eigen::Affine3d SE3;
		SE3.translation() = this->center;
		SE3.rotate(Eigen::AngleAxisd(this->rotateY, Eigen::Vector3d::UnitY()));
		SE3.rotate(Eigen::AngleAxisd(this->rotateX, Eigen::Vector3d::UnitX()));
		SE3.translate(Eigen::Vector3d(0., 0., this->distance));
		return SE3;
	}

    void transform() {
	//    glMultMatrixd(this->getSE3().inverse().matrix().transpose().data());
		glTranslated(0., 0., -this->distance);
		glRotated(-this->rotateX, 1., 0., 0.);
		glRotated(-this->rotateY, 0., 1., 0.);
		glTranslated(this->center[0], this->center[1], this->center[2]);

	}

    Eigen::Vector3d center;
    Eigen::Vector3d center_offset;
    double rotateY;
    double rotateX;
    double distance;
};



namespace MASS
{
class Environment;
class Muscle;
class Window : public Fl_Gl_Window
{
public:
	explicit Window(Environment* env);
	Window(Environment* env,const std::string& nn_path);
	Window(Environment* env,const std::string& nn_path,const std::string& muscle_nn_path);
    Window(Environment* env,const std::string& nn_path,const std::string& muscle_nn_path,const std::string& device_nn_path);

	void draw() override;
	int handle(int e) override;
	
    void Step();

	void SetFocusing();

	void DrawEntity(const dart::dynamics::Entity* entity);
	void DrawBodyNode(const dart::dynamics::BodyNode* bn);
	void DrawSkeleton(const dart::dynamics::SkeletonPtr& skel);
	void DrawShapeFrame(const dart::dynamics::ShapeFrame* shapeFrame);
	void DrawShape(const dart::dynamics::Shape* shape,const Eigen::Vector4d& color);

	void DrawMuscles(const std::vector<Muscle*>& muscles);
	void DrawShadow(const Eigen::Vector3d& scale, const aiScene* mesh,double y);
	void DrawAiMesh(const struct aiScene *sc, const struct aiNode* nd,const Eigen::Affine3d& M,double y);
	void DrawGround(double y);
	void Reset();

	Eigen::VectorXd GetActionFromNN();
    Eigen::VectorXd GetDeviceActionFromNN();
	Eigen::VectorXd GetActivationFromNN(const Eigen::VectorXd& mt);

	p::object mm,mns,sys_module,nn_module,muscle_nn_module;
	p::object device_nn_module;


	Environment* mEnv;
	Camera mCamera;
	int last_x, last_y;
	bool mouse_downed;
	int mouse_button;

	bool initGL;
    GLUquadricObj *quadric;
	bool mFocus;
	bool mSimulating;
	bool mDrawOBJ;
	bool mDrawShadow;
	bool mNNLoaded;
	bool mMuscleNNLoaded;
    bool mDeviceNNLoaded;
	Eigen::Affine3d mViewMatrix;

	int index_for_rendering;
	int reset_count;
};
};


#endif