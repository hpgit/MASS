#include "Character.h"
#include "BVH.h"
#include "DARTHelper.h"
#include "Muscle.h"

#include <tinyxml.h>
#include <cstdio>
#include <sstream>

using namespace dart;
using namespace dart::dynamics;
using namespace MASS;

Character::Character()
	:mSkeleton(nullptr), mBVH(nullptr), mTc(Eigen::Isometry3d::Identity())
{

}

Character::
~Character() {
	if (nullptr != mBVH)
	{
		mBVH = nullptr;
		delete mBVH;
	}

	for (size_t i = 0; i < mMuscles.size(); i++)
	{
		if (nullptr != mMuscles[i])
		{
			delete mMuscles[i];
		}
	}
	mMuscles.clear();

	mSkeleton.reset();
}
void Character::LoadSkeleton(const std::string& path,bool create_obj)
{
	mSkeleton = BuildFromFile(path, create_obj);
	std::map<std::string, std::string> bvh_map;
	TiXmlDocument doc;
	doc.LoadFile(path);
	TiXmlElement *skel_elem = doc.FirstChildElement("Skeleton");

	for (TiXmlElement* node = skel_elem->FirstChildElement("Node"); node != nullptr; node = node->NextSiblingElement("Node"))
	{
		if (node->Attribute("endeffector") != nullptr)
		{
			std::string ee = node->Attribute("endeffector");
			if(ee == "True")
			{
				mEndEffectors.push_back(mSkeleton->getBodyNode(std::string(node->Attribute("name"))));
			}
		}
		TiXmlElement* joint_elem = node->FirstChildElement("Joint");
		if (joint_elem->Attribute("bvh") != nullptr)
		{
			bvh_map.insert(std::make_pair(node->Attribute("name"),joint_elem->Attribute("bvh")));
		}
	}
	
	mBVH = new BVH(mSkeleton,bvh_map);
}

void Character::LoadMuscles(const std::string& path)
{
	TiXmlDocument doc;
	if (!doc.LoadFile(path))
	{
		std::cout << "Can't open file : " << path << std::endl;
		return;
	}

	TiXmlElement *muscledoc = doc.FirstChildElement("Muscle");
	for (TiXmlElement* unit = muscledoc->FirstChildElement("Unit"); unit != nullptr; unit = unit->NextSiblingElement("Unit"))
	{
		std::string name = unit->Attribute("name");
		double f0 = std::stod(unit->Attribute("f0"));
		double lm = std::stod(unit->Attribute("lm"));
		double lt = std::stod(unit->Attribute("lt"));
		double pa = std::stod(unit->Attribute("pen_angle"));
		double lmax = std::stod(unit->Attribute("lmax"));
		mMuscles.push_back(new Muscle(name, f0, lm, lt, pa, lmax));
		int num_waypoints = 0;
		for(TiXmlElement* waypoint = unit->FirstChildElement("Waypoint");waypoint!=nullptr;waypoint = waypoint->NextSiblingElement("Waypoint"))	
			num_waypoints++;
		int i = 0;
		for(TiXmlElement* waypoint = unit->FirstChildElement("Waypoint");waypoint!=nullptr;waypoint = waypoint->NextSiblingElement("Waypoint"))	
		{
			std::string body = waypoint->Attribute("body");
			Eigen::Vector3d glob_pos = string_to_vector3d(waypoint->Attribute("p"));
			if(i==0||i==num_waypoints-1)
			// if(true)
				mMuscles.back()->AddAnchor(mSkeleton->getBodyNode(body),glob_pos);
			else
				mMuscles.back()->AddAnchor(mSkeleton,mSkeleton->getBodyNode(body),glob_pos,2);

			i++;
		}
	}
	

}

void
Character::LoadBVH(const std::string& path,bool cyclic)
{
	if (mBVH == nullptr)
	{
		std::cout << "Initialize BVH class first" << std::endl;
		return;
	}
	mBVH->Parse(path,cyclic);
}

void
Character::Reset()
{
	// mTc = mBVH->GetT0();
	mTc.translation()[1] = 0.0;
}

void
Character::SetPDParameters(double kp, double kv)
{
	int dof = mSkeleton->getNumDofs();
	mKp = Eigen::VectorXd::Constant(dof, kp);
	mKv = Eigen::VectorXd::Constant(dof, kv);
}
Eigen::VectorXd
Character::GetSPDForces(const Eigen::VectorXd& p_desired, double dt)
{
	Eigen::VectorXd q = mSkeleton->getPositions();
	Eigen::VectorXd dq = mSkeleton->getVelocities();
	// double dt = mSkeleton->getTimeStep();
	// Eigen::MatrixXd M_inv = mSkeleton->getInvMassMatrix();
	Eigen::MatrixXd M_inv = (mSkeleton->getMassMatrix() + Eigen::MatrixXd((dt * mKv).asDiagonal())).inverse();

	Eigen::VectorXd qdqdt = q + dq*dt;

	Eigen::VectorXd p_diff = mKp.cwiseProduct(mSkeleton->getPositionDifferences(p_desired, qdqdt));
	Eigen::VectorXd v_diff = -mKv.cwiseProduct(dq);
	Eigen::VectorXd ddq = M_inv * (-mSkeleton->getCoriolisAndGravityForces() + p_diff + v_diff + mSkeleton->getConstraintForces());

	Eigen::VectorXd tau = p_diff + v_diff - dt * mKv.cwiseProduct(ddq);

	tau.head<6>().setZero();

	return tau;
}

Eigen::VectorXd Character::GetTargetPositions(double t, double dt)
{
	Eigen::VectorXd p = mBVH->GetMotion(t);
	if (mBVH->IsCyclic())
	{
		int k = (int)(t / mBVH->GetMaxTime());
		Eigen::Vector3d bvh_vec = mBVH->GetT1().translation() - mBVH->GetT0().translation();
		bvh_vec[1] = 0.;
		p.segment<3>(3) += k * bvh_vec;
	}

	p.segment<3>(3) -= mSkeleton->getRootJoint()->getTransformFromParentBodyNode().translation();

	return p;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd>
Character::GetTargetPosAndVel(double t, double dt)
{
	Eigen::VectorXd p = this->GetTargetPositions(t, dt);
	// Eigen::Isometry3d Tc = mTc;
	Eigen::VectorXd p1 = this->GetTargetPositions(t+dt, dt);
	// mTc = Tc;

	return std::make_pair(p, mSkeleton->getPositionDifferences(p1, p)/dt);
}
