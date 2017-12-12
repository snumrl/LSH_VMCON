#include "BoxQP.h"

BoxQP::
BoxQP(const Eigen::MatrixXd _H,const Eigen::VectorXd& _g,
		const Eigen::VectorXd& _lower,const Eigen::VectorXd& _upper)
	:H(_H),g(_g),lower(_lower),upper(_upper),mSolution(Eigen::VectorXd::Zero(_g.rows()))
{

}
BoxQP::
~BoxQP()
{
	H.resize(0,0);
	g.resize(0);
	lower.resize(0);
	upper.resize(0);
	mSolution.resize(0);
}

bool					
BoxQP::
get_nlp_info(	Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
				Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style)
{
	n = g.rows();
	m = 0;
	nnz_jac_g = 0;
	nnz_h_lag = n*n;
	index_style = TNLP::C_STYLE;
	return true;
}

bool					
BoxQP::
get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
				Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u) 	
{
	for(int i =0;i<n;i++)
	{
		x_l[i] = lower[i];
		x_u[i] = upper[i];
	}
	return true;
}

bool					
BoxQP::
get_starting_point(	Ipopt::Index n, bool init_x, Ipopt::Number* x,
					bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
					Ipopt::Index m, bool init_lambda,
					Ipopt::Number* lambda) 
{
	for(int i =0;i<n;i++)
		x[i] = mSolution[i];

	return true;
}

bool					
BoxQP::
eval_f(	Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value) 
{
	Eigen::VectorXd ex(n);
	for(int i =0;i<n;i++)
		ex[i] = x[i];

	obj_value = 0.5*ex.dot(H*ex) + ex.dot(g);
	return true;
}
bool					
BoxQP::
eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f) 
{
	Eigen::VectorXd ex(n);
	for(int i =0;i<n;i++)
		ex[i] = x[i];

	Eigen::VectorXd grad = H*ex + g;
	for(int i =0;i<n;i++)
		grad_f[i] = grad[i];
	return true;
}
bool					
BoxQP::
eval_g(	Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g) 
{
	return true;
}
bool					
BoxQP::
eval_jac_g( Ipopt::Index n, const Ipopt::Number* x, bool new_x,
			Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index *jCol,
			Ipopt::Number* values) 
{
	return true;
}
bool					
BoxQP::
eval_h( Ipopt::Index n, const Ipopt::Number* x, bool new_x,
		Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
		bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
		Ipopt::Index* jCol, Ipopt::Number* values) 
{
	int nnz = 0;

	if(values == NULL)
	{
		for(int i=0;i<n;i++)
		{
			for(int j=0;j<n;j++)
			{
				iRow[nnz] = i;
				jCol[nnz++] = j;
			}
			
		}
	}
	else
	{
		for(int i=0;i<n;i++)
		{
			for(int j=0;j<n;j++)
			{
				values[nnz++] = H(i,j);
			}
			
		}
	}

	return true;
}
void 					
BoxQP::
finalize_solution(	Ipopt::SolverReturn status,
					Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U,
					Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda,
					Ipopt::Number obj_value,
					const Ipopt::IpoptData* ip_data,
					Ipopt::IpoptCalculatedQuantities* ip_cq)
{
	for(int i=0;i<n;i++)
		mSolution[i] = x[i];
}
