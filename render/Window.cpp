#include "Window.h"
#include "Environment.h"
#include "Character.h"
#include "BVH.h"
#include "Muscle.h"
#include <iostream>
#include <FL/glut.H>
#include "lodepng.h"

using namespace dart;
using namespace dart::dynamics;
using namespace dart::simulation;

static void dump_png(const char* filename, int w, int h)
{

    int image_size = 3*w*h;
    unsigned char image_tmp[image_size];
    unsigned char image[image_size];
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadBuffer(GL_BACK_LEFT);
    glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, image_tmp);

    for(int i=0; i<h; i++)
        memcpy(&image[3*w*(h-1-i)], &image_tmp[3*w*i], 3*w*sizeof(unsigned char));

    unsigned int error = lodepng_encode24_file(filename, image, w, h);
    if(error) printf("error %u: %s\n", error, lodepng_error_text(error));
}

static void
onTimer(MASS::Window *window)
{
    if (window->mSimulating)
    {
        window->Step();
        window->redraw();
	// dump png
        // std::string file_name = std::string("dump/");
        // if(window->index_for_rendering < 100)
        //     file_name += std::string("0");
        // if(window->index_for_rendering < 10)
        //     file_name += std::string("0");
        // file_name += std::to_string(window->index_for_rendering) + std::string(".png");
        //  dump_png(file_name.c_str(), 1280, 720);
        // window->index_for_rendering++;
        // if(window->index_for_rendering == 300)
        // {
        //     window->mSimulating = false;
        //     std::string ffmpeg_cmd = "ffmpeg -loglevel 0 -y -framerate 30 -i dump/%03d.png -vcodec libx264 -crf 20 -pix_fmt yuv420p dump";
        //     ffmpeg_cmd += std::to_string(window->reset_count) + std::string(".mp4");
        //     int system_return = system(ffmpeg_cmd.c_str());
        //     // system("ffplay -loglevel 0 dump.mp4 &");
        // }
    }
    Fl::repeat_timeout(0.0333, reinterpret_cast<Fl_Timeout_Handler>(onTimer), (void *)window);
}

namespace MASS {
    Window::
    Window(Environment *env)
            : Fl_Gl_Window(0, 0, 1280, 720, "renderer"), mEnv(env), mFocus(true), mSimulating(false), mDrawOBJ(false),
              mDrawShadow(true) {

        index_for_rendering = 0;
        reset_count = 0;
        mouse_downed = false;
        mouse_button = 0;

        SetFocusing();
        mFocus = false;
        mNNLoaded = false;
        mMuscleNNLoaded = false;
        initGL = false;
        quadric = gluNewQuadric();

        mm = p::import("__main__");
        mns = mm.attr("__dict__");
        sys_module = p::import("sys");
        Fl::add_timeout(0.0333, reinterpret_cast<Fl_Timeout_Handler>(onTimer), (void *) this);

	std::string module_dir = std::string(MASS_ROOT_DIR) + std::string("/python");
        sys_module.attr("path").attr("insert")(1, module_dir);
        p::exec("import torch", mns);
        p::exec("import torch.nn as nn", mns);
        p::exec("import torch.optim as optim", mns);
        p::exec("import torch.nn.functional as F", mns);
        p::exec("import torchvision.transforms as T", mns);
        p::exec("import numpy as np", mns);
        p::exec("from Model import SimulationNN, MuscleNN",mns);
    }

    Window::
    Window(Environment *env, const std::string &nn_path)
            : Window(env) {
        mNNLoaded = true;

        std::string str = std::string("num_state = ") + std::to_string(mEnv->GetNumState());
        p::exec(str.c_str(), mns);
        str = std::string("num_action = ") + std::to_string(mEnv->GetNumAction());
        p::exec(str.c_str(), mns);

        nn_module = p::eval("SimulationNN(num_state,num_action)", mns);

        p::object load = nn_module.attr("load");
        load(nn_path);
        nn_module.attr("eval")();
    }

    Window::
    Window(Environment *env, const std::string &nn_path, const std::string &muscle_or_device_nn_path)
            : Window(env, nn_path) {
        if (env->GetUseMuscle()) {
            mMuscleNNLoaded = true;

            std::string str = std::string("num_total_muscle_related_dofs = ") + std::to_string(mEnv->GetNumTotalRelatedDofs());
            p::exec(str.c_str(), mns);
            str = "num_actions = " + std::to_string(mEnv->GetNumAction());
            p::exec(str.c_str(), mns);
            str = "num_muscles = " + std::to_string(mEnv->GetCharacter()->GetMuscles().size());
            p::exec(str.c_str(), mns);

            muscle_nn_module = p::eval("MuscleNN(num_total_muscle_related_dofs,num_actions,num_muscles)", mns);

            p::object load = muscle_nn_module.attr("load");
            load(muscle_or_device_nn_path);
        }
    }

    void
    Window::
    draw() {
        if (!initGL) {
            initGL = true;
            glViewport(0, 0, this->w(), this->h());
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glFrustum(-.08, .08, -.045, .045, 0.1, 1000.);
            // gluPerspective(45., float(this->w()) / float(this->h()), 0.1, 1000.);
            glMatrixMode(GL_MODELVIEW);
            float lightPos[] = {-10000, 20000, 5000, 0};
            float lightPos2[] = {.5, 1., .5, 0};
            float lightDiffuse[] = {.2, .2, .2, 1.};

            glEnable(GL_LIGHTING);
            glEnable(GL_LIGHT0);
            glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
            // glLightfv(GL_LIGHT0, GL_POSITION, lightPos2);
            glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
        }
        mCamera.center = -mEnv->GetWorld()->getSkeleton("Human")->getRootBodyNode()->getCOM();
        mCamera.center[1] -= 0.3;
        mCamera.center += mCamera.center_offset;
        glLoadIdentity();
        glClearColor(1., 1., 1., 1.);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        mCamera.transform();
        GLfloat matrix[16];
        glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
        Eigen::Matrix3d A;
        Eigen::Vector3d b;
        A << matrix[0], matrix[4], matrix[8],
                matrix[1], matrix[5], matrix[9],
                matrix[2], matrix[6], matrix[10];
        b << matrix[12], matrix[13], matrix[14];
        mViewMatrix.linear() = A;
        mViewMatrix.translation() = b;

        auto ground = mEnv->GetGround();
        float y = ground->getBodyNode(0)->getTransform().translation()[1] +
                  dynamic_cast<const BoxShape *>(ground->getBodyNode(
                          0)->getShapeNodesWith<dart::dynamics::VisualAspect>()[0]->getShape().get())->getSize()[1] *
                  0.5 + 0.001;

        DrawGround(y);
        DrawMuscles(mEnv->GetCharacter()->GetMuscles());
        DrawSkeleton(mEnv->GetCharacter()->GetSkeleton());

        // DrawSkeleton(mEnv->GetGround());

        // Eigen::Quaterniond q = mTrackBall.getCurrQuat();
        // q.x() = 0.0;
        // q.z() = 0.0;
        // q.normalize();
        // mTrackBall.setQuaternion(q);
        SetFocusing();
    }

    void
    Window::
    SetFocusing() {
        if (mFocus) {
            mCamera.center = -mEnv->GetWorld()->getSkeleton("Human")->getRootBodyNode()->getCOM();
            mCamera.center[1] -= 0.3;
            // mTrans *=1000.0;
        }
    }

    int
    Window::
    handle(int e) {
        if ( e == FL_MOUSEWHEEL ){
            int dy = Fl::event_dy();
            if (dy < 0) {
                mCamera.distance *= 0.8;
            } else {
                mCamera.distance *= 1.25;
            }
            redraw();
        } else if (e == FL_PUSH) {
            last_x = Fl::event_x();
            last_y = Fl::event_y();
            mouse_downed = false;
            mouse_button = Fl::event_button();
        } else if (e == FL_RELEASE) {
            last_x = Fl::event_x();
            last_y = Fl::event_y();
            mouse_button = 0;
        } else if (e == FL_DRAG) {
            int x = Fl::event_x();
            int y = Fl::event_y();
            int dx = x - last_x;
            int dy = y - last_y;

            int button = Fl::event_button();
            if (button == FL_LEFT_MOUSE) {
                mCamera.rotateY -= dx / 45. * M_PI;
                mCamera.rotateX -= dy / 45. * M_PI;
                redraw();
            } else if (button == FL_RIGHT_MOUSE) {
                mCamera.center_offset[1] -= dy / 100.;
                redraw();
            }
            last_x = x;
            last_y = y;
        } else if (e == FL_KEYUP) {
            int _key = Fl::event_key();
            switch (_key) {
                case 's':
                    this->Step();
                    this->redraw();
                    return 1;
                case 'f':
                    mFocus = !mFocus;
                    return 1;
                case 'r':
                    mSimulating = false;
                    this->Reset();
                    this->redraw();
                    return 1;
                case ' ':
                    mSimulating = !mSimulating;
                    return 1;
                case 'o':
                    mDrawOBJ = !mDrawOBJ;
                    return 1;
                case 27 :
                    exit(0);
                    return 1;
                default:
                    break;
            }
        }

        return Fl_Gl_Window::handle(e);
    }

    void
    Window::
    Step() {
//        int num = mEnv->GetSimulationHz() / mEnv->GetControlHz();
        int num = mEnv->GetNumSteps();

        Eigen::VectorXd action;
        if (mNNLoaded)
            action = GetActionFromNN();
        else
            action = Eigen::VectorXd::Zero(mEnv->GetNumAction());
        mEnv->SetAction(action);

        if (mEnv->GetUseMuscle()) {
            int inference_per_sim = 2;
            for (int i = 0; i < num; i += inference_per_sim) {
                Eigen::VectorXd mt = mEnv->GetMuscleTorques();
                mEnv->SetActivationLevels(GetActivationFromNN(mt));
                for (int j = 0; j < inference_per_sim; j++)
                    mEnv->Step();
            }
        } 
    }

    void
    Window::
    Reset() {
        mEnv->Reset(true);
        this->index_for_rendering = 0;
    }


    np::ndarray toNumPyArray(const Eigen::VectorXd &vec) {
        int n = vec.rows();
        p::tuple shape = p::make_tuple(n);
        np::dtype dtype = np::dtype::get_builtin<float>();
        np::ndarray array = np::empty(shape, dtype);

        float *dest = reinterpret_cast<float *>(array.get_data());
        for (int i = 0; i < n; i++) {
            dest[i] = vec[i];
        }

        return array;
    }


    Eigen::VectorXd
    Window::
    GetActionFromNN() {
        p::object get_action;
        get_action = nn_module.attr("get_action");
        Eigen::VectorXd state = mEnv->GetState();
        p::tuple shape = p::make_tuple(state.rows());
        np::dtype dtype = np::dtype::get_builtin<float>();
        np::ndarray state_np = np::empty(shape, dtype);

        float *dest = reinterpret_cast<float *>(state_np.get_data());
        for (int i = 0; i < state.rows(); i++)
            dest[i] = state[i];

        p::object temp = get_action(state_np);
        np::ndarray action_np = np::from_object(temp);

        float *srcs = reinterpret_cast<float *>(action_np.get_data());

        Eigen::VectorXd action(mEnv->GetNumAction());
        for (int i = 0; i < action.rows(); i++)
            action[i] = srcs[i];

        return action;
    }

    Eigen::VectorXd
    Window::
    GetActivationFromNN(const Eigen::VectorXd &mt) {
        if (!mMuscleNNLoaded) {
            mEnv->GetDesiredTorques();
            return Eigen::VectorXd::Zero(mEnv->GetCharacter()->GetMuscles().size());
        }
        p::object get_activation = muscle_nn_module.attr("get_activation");
        Eigen::VectorXd dt = mEnv->GetDesiredTorques();
        np::ndarray mt_np = toNumPyArray(mt);
        np::ndarray dt_np = toNumPyArray(dt);

        p::object temp = get_activation(mt_np, dt_np);
        np::ndarray activation_np = np::from_object(temp);

        Eigen::VectorXd activation(mEnv->GetCharacter()->GetMuscles().size());
        float *srcs = reinterpret_cast<float *>(activation_np.get_data());
        for (int i = 0; i < activation.rows(); i++)
            activation[i] = srcs[i];

        return activation;
    }

    void
    Window::
    DrawEntity(const Entity *entity) {
        if (!entity)
            return;
        const auto &bn = dynamic_cast<const BodyNode *>(entity);
        if (bn) {
            DrawBodyNode(bn);
            return;
        }

        const auto &sf = dynamic_cast<const ShapeFrame *>(entity);
        if (sf) {
            DrawShapeFrame(sf);
            return;
        }
    }

    void
    Window::
    DrawBodyNode(const BodyNode *bn) {
        if (!bn)
            return;

        glPushMatrix();
        glMultMatrixd(bn->getRelativeTransform().data());

        auto sns = bn->getShapeNodesWith<VisualAspect>();
        for (const auto &sn : sns)
            DrawShapeFrame(sn);

        for (const auto &et : bn->getChildEntities())
            DrawEntity(et);

        glPopMatrix();
    }

    void
    Window::
    DrawSkeleton(const SkeletonPtr &skel) {
        DrawBodyNode(skel->getRootBodyNode());
    }

    void
    Window::
    DrawShapeFrame(const ShapeFrame *sf) {
        if (!sf)
            return;

        const auto &va = sf->getVisualAspect();

        if (!va || va->isHidden())
            return;

        glPushMatrix();
        glMultMatrixd(sf->getRelativeTransform().data());

        DrawShape(sf->getShape().get(), va->getRGBA());
        glPopMatrix();
    }

    void
    Window::
    DrawShape(const Shape *shape, const Eigen::Vector4d &color) {
        if (!shape)
            return;

        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
        glEnable(GL_COLOR_MATERIAL);
        glEnable(GL_DEPTH_TEST);
        glColor4dv(color.data());
        if (!mDrawOBJ) {
            if (shape->is<SphereShape>()) {
                const auto *sphere = static_cast<const SphereShape *>(shape);
                double sphere_radius = sphere->getRadius();
                glScalef(sphere_radius, sphere_radius, sphere_radius);
                glutSolidSphere(1., 16, 16);
                // mRI->drawSphere(sphere->getRadius());
            } else if (shape->is<BoxShape>()) {
                const auto *box = static_cast<const BoxShape *>(shape);
                auto box_size = box->getSize();
                glScalef(box_size[0], box_size[1], box_size[2]);
                glutSolidCube(1.);
                // mRI->drawCube(box->getSize());
            } else if (shape->is<CapsuleShape>()) {
                const auto *capsule = static_cast<const CapsuleShape *>(shape);
                // mRI->drawCapsule(capsule->getRadius(), capsule->getHeight());
            }
        } else {
            if (shape->is<MeshShape>()) {
                const auto &mesh = static_cast<const MeshShape *>(shape);
                glDisable(GL_COLOR_MATERIAL);
                // mRI->drawMesh(mesh->getScale(), mesh->getMesh());
                float y = mEnv->GetGround()->getBodyNode(0)->getTransform().translation()[1] +
                          dynamic_cast<const BoxShape *>(mEnv->GetGround()->getBodyNode(
                                  0)->getShapeNodesWith<dart::dynamics::VisualAspect>()[0]->getShape().get())->getSize()[1] *
                          0.5;
                this->DrawShadow(mesh->getScale(), mesh->getMesh(), y);
            }

        }

        glDisable(GL_COLOR_MATERIAL);
    }

    void
    Window::
    DrawMuscles(const std::vector<Muscle *> &muscles) {
        int count = 0;
        glEnable(GL_LIGHTING);
        glEnable(GL_DEPTH_TEST);
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
        glEnable(GL_COLOR_MATERIAL);

        for (auto muscle : muscles) {
            auto aps = muscle->GetAnchors();
            bool lower_body = true;
            double a = muscle->activation;
            // Eigen::Vector3d color(0.7*(3.0*a),0.2,0.7*(1.0-3.0*a));
            Eigen::Vector4d color(0.4 + (2.0 * a), 0.4, 0.4, 1.0);//0.7*(1.0-3.0*a));
            // glColor3f(1.0,0.0,0.362);
            // glColor3f(0.0,0.0,0.0);
            glColor4dv(color.data());
            // mRI->setPenColor(color);
            for (int i = 0; i < aps.size(); i++) {
                Eigen::Vector3d p = aps[i]->GetPoint();
                // mRI->pushMatrix();
                // mRI->translate(p);
                // mRI->drawSphere(0.005*sqrt(muscle->f0/1000.0));
                // mRI->popMatrix();
                glPushMatrix();
                glTranslated(p[0], p[1], p[2]);
                glutSolidSphere(0.005 * sqrt(muscle->f0 / 1000.), 8, 8);
                glPopMatrix();
            }

            for (int i = 0; i < aps.size() - 1; i++) {
                Eigen::Vector3d p = aps[i]->GetPoint();
                Eigen::Vector3d p1 = aps[i + 1]->GetPoint();

                Eigen::Vector3d u(0, 0, 1);
                Eigen::Vector3d v = p - p1;
                Eigen::Vector3d mid = 0.5 * (p + p1);
                double len = v.norm();
                v /= len;
                Eigen::Isometry3d T;
                T.setIdentity();
                Eigen::Vector3d axis = u.cross(v);
                axis.normalize();
                double angle = acos(u.dot(v));
                Eigen::Matrix3d w_bracket = Eigen::Matrix3d::Zero();
                w_bracket(0, 1) = -axis(2);
                w_bracket(1, 0) = axis(2);
                w_bracket(0, 2) = axis(1);
                w_bracket(2, 0) = -axis(1);
                w_bracket(1, 2) = -axis(0);
                w_bracket(2, 1) = axis(0);


                Eigen::Matrix3d R = Eigen::Matrix3d::Identity() + (sin(angle)) * w_bracket +
                                    (1.0 - cos(angle)) * w_bracket * w_bracket;
                T.linear() = R;
                T.translation() = mid;
                glPushMatrix();
                glMultMatrixd(T.data());
                double rad = 0.005 * sqrt(muscle->f0 / 1000.0);
                glTranslated(0., 0., -len/2.);
                gluCylinder(quadric, rad, rad, len, 8, 8);
                glPopMatrix();

                //mRI->pushMatrix();
                //mRI->transform(T);
                //mRI->drawCylinder(0.005*sqrt(muscle->f0/1000.0),len);
                //mRI->popMatrix();
            }

        }
        glEnable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
    }

    void
    Window::
    DrawShadow(const Eigen::Vector3d &scale, const aiScene *mesh, double y) {
        glDisable(GL_LIGHTING);
        glPushMatrix();
        glScalef(scale[0], scale[1], scale[2]);
        GLfloat matrix[16];
        glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
        Eigen::Matrix3d A;
        Eigen::Vector3d b;
        A << matrix[0], matrix[4], matrix[8],
                matrix[1], matrix[5], matrix[9],
                matrix[2], matrix[6], matrix[10];
        b << matrix[12], matrix[13], matrix[14];

        Eigen::Affine3d M;
        M.linear() = A;
        M.translation() = b;
        M = (mViewMatrix.inverse()) * M;

        glPushMatrix();
        glLoadIdentity();
        glMultMatrixd(mViewMatrix.data());
        DrawAiMesh(mesh, mesh->mRootNode, M, y);
        glPopMatrix();
        glPopMatrix();
        glEnable(GL_LIGHTING);
    }

    void
    Window::
    DrawAiMesh(const struct aiScene *sc, const struct aiNode *nd, const Eigen::Affine3d &M, double y) {
        unsigned int i;
        unsigned int n = 0, t;
        Eigen::Vector3d v;
        Eigen::Vector3d dir(0.4, 0, -0.4);
        glColor3f(0.3, 0.3, 0.3);

        // update transform

        // draw all meshes assigned to this node
        for (; n < nd->mNumMeshes; ++n) {
            const struct aiMesh *mesh = sc->mMeshes[nd->mMeshes[n]];

            for (t = 0; t < mesh->mNumFaces; ++t) {
                const struct aiFace *face = &mesh->mFaces[t];
                GLenum face_mode;

                switch (face->mNumIndices) {
                    case 1:
                        face_mode = GL_POINTS;
                        break;
                    case 2:
                        face_mode = GL_LINES;
                        break;
                    case 3:
                        face_mode = GL_TRIANGLES;
                        break;
                    default:
                        face_mode = GL_POLYGON;
                        break;
                }
                glBegin(face_mode);
                for (i = 0; i < face->mNumIndices; i++) {
                    int index = face->mIndices[i];

                    v[0] = (&mesh->mVertices[index].x)[0];
                    v[1] = (&mesh->mVertices[index].x)[1];
                    v[2] = (&mesh->mVertices[index].x)[2];
                    v = M * v;
                    double h = v[1] - y;

                    v += h * dir;

                    v[1] = y + 0.001;
                    glVertex3f(v[0], v[1], v[2]);
                }
                glEnd();
            }

        }

        // draw all children
        for (n = 0; n < nd->mNumChildren; ++n) {
            DrawAiMesh(sc, nd->mChildren[n], M, y);
        }

    }

    void
    Window::
    DrawGround(double y) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDisable(GL_LIGHTING);
        double width = 0.005;
        int count = 0;
        glBegin(GL_QUADS);
        for (double x = -20.0; x < 20.01; x += 1.0) {
            for (double z = -20.0; z < 20.01; z += 1.0) {
                if (count % 2 == 0)
                    glColor3f(216.0 / 255.0, 211.0 / 255.0, 204.0 / 255.0);
                else
                    glColor3f(216.0 / 255.0 - 0.1, 211.0 / 255.0 - 0.1, 204.0 / 255.0 - 0.1);
                count++;
                glVertex3f(x, y, z);
                glVertex3f(x + 1.0, y, z);
                glVertex3f(x + 1.0, y, z + 1.0);
                glVertex3f(x, y, z + 1.0);
            }
        }
        glColor3f(180.0 / 255.0 - 0.1, 180.0 / 255.0 - 0.1, 255.0 / 255.0 - 0.1);
//        glVertex3f(-0.8, y +0.001, mEnv->walk_speed * 3.);
//        glVertex3f(-0.8 + 1.6, y+0.001, mEnv->walk_speed * 3.);
//        glVertex3f(-0.8 + 1.6, y+0.001, mEnv->walk_speed * 3. + mEnv->stride_length * 1.2);
//        glVertex3f(-0.8, y+0.001, mEnv->walk_speed * 3. + mEnv->stride_length * 1.2);
        glEnd();
        glEnable(GL_LIGHTING);
    }
}
