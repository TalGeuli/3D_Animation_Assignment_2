#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include "Eigen/dense"
#include <functional>



SandBox::SandBox()
{
	

}

void SandBox::Init(const std::string &config)
{
	std::string item_name;
	std::ifstream nameFileout;
	doubleVariable = 0;
	treeSize = 0;
	up = false;
	down = false;
	left = false;
	right = false;
	trees.resize(50);
	nameFileout.open(config);
	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file "<<config << std::endl;
	}
	else
	{
		
		while (nameFileout >> item_name)
		{
			std::cout << "openning " << item_name << std::endl;
			load_mesh_from_file(item_name);
			
			parents.push_back(-1);
			data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
			data().show_overlay_depth = false;
			data().point_size = 10;
			data().line_width = 2;
			data().set_visible(false, 1);
			
			
			
			
		}
		nameFileout.close();
	}
	MyTranslate(Eigen::Vector3d(0, 0, -1), true);
	
	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));
	
}

SandBox::~SandBox()
{
	
}

void SandBox::Animate()
{
	if (isActive)
	{
		if(left)
			data().MyTranslate(Eigen::Vector3d(-0.01, 0, 0), true);
		if(right)
			data().MyTranslate(Eigen::Vector3d(0.01, 0, 0), true);
		if(up)
			data().MyTranslate(Eigen::Vector3d(0, 0.01, 0), true);
		if(down)
			data().MyTranslate(Eigen::Vector3d(0, -0.01, 0), true);
		if (Check_Collision()) {
			left = false;
			right = false;
			up = false;
			down = false;
			isActive = !isActive;
		}
		
	}
}


//for the collision bounding box.
void SandBox::create_bounding_box(Eigen::AlignedBox <double, 3>& box, int index)
{
	// Corners of the bounding box
	Eigen::MatrixXd V_box(8, 3);
	V_box <<
		box.corner(box.BottomLeftFloor)[0],
		box.corner(box.BottomLeftFloor)[1],
		box.corner(box.BottomLeftFloor)[2],
		box.corner(box.BottomRightFloor)[0],
		box.corner(box.BottomRightFloor)[1],
		box.corner(box.BottomRightFloor)[2],
		box.corner(box.TopLeftFloor)[0],
		box.corner(box.TopLeftFloor)[1],
		box.corner(box.TopLeftFloor)[2],
		box.corner(box.TopRightFloor)[0],
		box.corner(box.TopRightFloor)[1],
		box.corner(box.TopRightFloor)[2],
		box.corner(box.BottomLeftCeil)[0],
		box.corner(box.BottomLeftCeil)[1],
		box.corner(box.BottomLeftCeil)[2],
		box.corner(box.BottomRightCeil)[0],
		box.corner(box.BottomRightCeil)[1],
		box.corner(box.BottomRightCeil)[2],
		box.corner(box.TopLeftCeil)[0],
		box.corner(box.TopLeftCeil)[1],
		box.corner(box.TopLeftCeil)[2],
		box.corner(box.TopRightCeil)[0],
		box.corner(box.TopRightCeil)[1],
		box.corner(box.TopRightCeil)[2];

	// Edges of the bounding box
	Eigen::MatrixXi E_box(12, 2);
	E_box <<
		0, 1,
		1, 3,
		2, 3,
		2, 0,
		4, 5,
		5, 7,
		6, 7,
		6, 4,
		0, 4,
		1, 5,
		2, 6,
		7, 3;


	// Plot the edges of the bounding box
	for (unsigned i = 0; i < E_box.rows(); ++i)
		data_list.at(index).add_edges
		(
			V_box.row(E_box(i, 0)),
			V_box.row(E_box(i, 1)),
			Eigen::RowVector3d(1, 0, 0)
		);
}


bool SandBox::Check_Collision()
{
	for (int i = 0; i < treeSize; i++) 
	{
		if (i != selected_data_index)
		{
			boxesCreated = false;
			if (Check_Collision (*trees.at(selected_data_index), selected_data_index, *trees.at(i), i))
				return true;
			
				
				
		}		
	}
	return false;
}

bool SandBox::Check_Collision(igl::AABB<Eigen::MatrixXd, 3>& A, int indexA, igl::AABB<Eigen::MatrixXd, 3>& B, int indexB)
{
	Eigen::AlignedBox <double, 3> A_box = A.m_box;
	Eigen::AlignedBox <double, 3> B_box = B.m_box;



	if (Is_Collide(A_box, indexA, B_box, indexB))
	{
		if (A.is_leaf() & B.is_leaf())
		{
			create_bounding_box(B_box, indexB);
			create_bounding_box(A_box, indexA);
			return true;
		}
		if (A.is_leaf())
		{
			return Check_Collision(A, indexA, *B.m_left, indexB) || Check_Collision(A, indexA, *B.m_right, indexB);
		}
		if (B.is_leaf())
		{
			return Check_Collision(*A.m_left, indexA, B, indexB) || Check_Collision(*A.m_right, indexA, B, indexB);
		}
		return Check_Collision(*A.m_left, indexA, *B.m_left, indexB) || Check_Collision(*A.m_left, indexA, *B.m_right, indexB) || Check_Collision(*A.m_right, indexA, *B.m_left, indexB) || Check_Collision(*A.m_right, indexA, *B.m_right, indexB);
	}
	return false;


}

bool SandBox::Is_Collide(Eigen::AlignedBox <double, 3>& A_box, int indexA, Eigen::AlignedBox <double, 3>& B_box, int indexB)
{
	
	Eigen::Vector3d ARightCol(data_list.at(indexA).MakeTransd()(0, 3), data_list.at(indexA).MakeTransd()(1, 3), data_list.at(indexA).MakeTransd()(2, 3));
	Eigen::Vector3d BRightCol(data_list.at(indexB).MakeTransd()(0, 3), data_list.at(indexB).MakeTransd()(1, 3), data_list.at(indexB).MakeTransd()(2, 3));
	Eigen::Vector3d C0 = A_box.center();
	Eigen::Vector3d C1 = B_box.center();
	//Eigen::Vector3d D =  (B_box.center() + BRightCol) - (A_box.center() + ARightCol);
	
	Eigen::Matrix3d A = data_list.at(indexA).GetRotation();
	Eigen::Matrix3d B = data_list.at(indexB).GetRotation();

	Eigen::Vector3d D = (B*C1 + BRightCol) - (A*C0 + ARightCol);

	Eigen::Vector3d A0 (A(0, 0), A(1, 0), A(2, 0));
	Eigen::Vector3d A1 (A(0, 1), A(1, 1), A(2, 1));
	Eigen::Vector3d A2 (A(0, 2), A(1, 2), A(2, 2));

	Eigen::Vector3d B0 (B(0, 0), B(1, 0), B(2, 0));
	Eigen::Vector3d B1 (B(0, 1), B(1, 1), B(2, 1));
	Eigen::Vector3d B2 (B(0, 2), B(1, 2), B(2, 2));

	double a0 = A_box.sizes()(0) / 2;
	double a1 = A_box.sizes()(1) / 2;
	double a2 = A_box.sizes()(2) / 2;
	double b0 = B_box.sizes()(0) / 2;
	double b1 = B_box.sizes()(1) / 2;
	double b2 = B_box.sizes()(2) / 2;

	Eigen::Matrix3d C = data_list.at(indexA).GetRotation().transpose() * data_list.at(indexB).GetRotation();
	
	double R0, R1, R;
	//CASE 1
	R0 = a0;
	R1 = b0 * abs(C(0,0)) + b1 * abs(C(0,1)) + b2 * abs(C(0,2));
	R = (A0.transpose() * D).norm();
	if (R > R0 + R1)
		return false;

	//CASE 2
	R0 = a1;
	R1 = b0 * abs(C(1, 0)) + b1 * abs(C(1, 1)) + b2 * abs(C(1, 2));
	R = (A1.transpose() * D).norm();
	if (R > R0 + R1)
		return false;
	
	//CASE 3
	R0 = a2;
	R1 = b0 * abs(C(2, 0)) + b1 * abs(C(2, 1)) + b2 * abs(C(2, 2));
	R = (A2.transpose() * D).norm();
	if (R > R0 + R1)
		return false;
	
	//CASE 4
	R0 = a0 * abs(C(0, 0)) + a1 * abs(C(1, 0)) + a2 * abs(C(2, 0));
	R1 = b0;
	R = (B0.transpose() * D).norm();
	if (R > R0 + R1)
		return false;
	
	//CASE 5
	R0 = a0 * abs(C(0, 1)) + a1 * abs(C(1, 1)) + a2 * abs(C(2, 1));
	R1 = b1;
	R = (B1.transpose() * D).norm();
	if (R > R0 + R1)
		return false;

	//CASE 6
	R0 = a0 * abs(C(0, 2)) + a1 * abs(C(1, 2)) + a2 * abs(C(2, 2));
	R1 = b2;
	R = (B2.transpose() * D).norm();
	if (R > R0 + R1)
		return false;

	//CASE 7
	R0 = a1 * abs(C(2, 0)) + a2 * abs(C(1, 0));
	R1 = b1 * abs(C(0,2)) + b2 * abs(C(0, 1));
	R = (C(1,0) * A2.transpose() * D - C(2,0) * A1.transpose() * D).norm();
	if (R > R0 + R1)
		return false;

	//CASE 8
	R0 = a1 * abs(C(2, 1)) + (a2) * abs(C(1, 1));
	R1 = b0 * abs(C(0, 2)) + b2 * abs(C(0, 0));
	R = (C(1, 1) * A2.transpose() * D - C(2, 1) * A1.transpose() * D).norm();
	if (R > R0 + R1)
		return false;
	
	//CASE 9
	R0 = a1 * abs(C(2, 2)) + a2 * abs(C(1, 2));
	R1 = b0 * abs(C(0, 1)) + b1 * abs(C(0, 0));
	R = (C(1, 2) * A2.transpose() * D - C(2, 2) * A1.transpose() * D).norm();
	if (R > R0 + R1)
		return false;

	//CASE 10
	R0 = a0 * abs(C(2, 0)) + a2 * abs(C(0, 0));
	R1 = b1 * abs(C(1, 2)) + b2 * abs(C(1, 1));
	R = (C(2, 0) * A0.transpose() * D - C(0, 0) * A2.transpose() * D).norm();
	if (R > R0 + R1)
		return false;

	//CASE 11
	R0 = a0 * abs(C(2, 1)) + a2 * abs(C(0, 1));
	R1 = b0 * abs(C(1, 2)) + b2 * abs(C(1, 0));
	R = (C(2, 1) * A0.transpose() * D - C(0, 1) * A2.transpose() * D).norm();
	if (R > R0 + R1)
		return false;

	//CASE 12
	R0 = a0 * abs(C(2, 2)) + a2 * abs(C(0, 2));
	R1 = b0 * abs(C(1, 1)) + b1 * abs(C(1, 0));
	R = (C(2, 2) * A0.transpose() * D - C(0, 2) * A2.transpose() * D).norm();
	if (R > R0 + R1)
		return false;

	//CASE 13
	R0 = a0 * abs(C(1, 0)) + a1 * abs(C(0, 0));
	R1 =b1 * abs(C(2, 2)) + b2 * abs(C(2, 1));
	R = (C(0, 0) * A1.transpose() * D - C(1, 0) * A0.transpose() * D).norm();
	if (R > R0 + R1)
		return false;

	//CASE 14
	R0 = a0 * abs(C(1, 1)) + a1 * abs(C(0, 1));
	R1 = b0 * abs(C(2, 2)) + b2 * abs(C(2, 0));
	R = (C(0, 1) * A1.transpose() * D - C(1, 1) * A0.transpose() * D).norm();
	if (R > R0 + R1)
		return false;

	//CASE 15
	R0 = a0 * abs(C(1, 2)) + a1 * abs(C(0, 2));
	R1 = b0 * abs(C(2, 1)) + b1 * abs(C(2, 0));
	R = (C(0, 2) * A1.transpose() * D - C(1, 2) * A0.transpose() * D).norm();
	if (R > R0 + R1)
		return false;
	return true;
}






