#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/aabb.h"

class SandBox : public igl::opengl::glfw::Viewer
{
public:
	SandBox();
	~SandBox();
	void Init(const std::string& config);
	double doubleVariable;
private:
	// Prepare array-based edge data structures and priority queue
	bool boxesCreated;
	
	
	
	void Animate();
	void create_bounding_box(Eigen::AlignedBox <double, 3>& box, int index);
	bool Check_Collision();
	bool Check_Collision(igl::AABB<Eigen::MatrixXd, 3>& A, int indexA, igl::AABB<Eigen::MatrixXd, 3>& B, int indexB);
	bool Is_Collide(Eigen::AlignedBox <double, 3>& A_box, int indexA, Eigen::AlignedBox <double, 3>& B_box, int indexB);
};

