#ifndef __BOX_QP_H__
#define __BOX_QP_H__
#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include <Eigen/Core>
class BoxQP : public Ipopt::TNLP
{
public:
	
	BoxQP(const Eigen::MatrixXd _H,const Eigen::VectorXd& _g,
		const Eigen::VectorXd& _lower,const Eigen::VectorXd& _upper);
	~BoxQP();

	bool					get_nlp_info(	Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
														Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style) override;

	 bool					get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
														Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u) override;	

	 bool					get_starting_point(	Ipopt::Index n, bool init_x, Ipopt::Number* x,
															bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
															Ipopt::Index m, bool init_lambda,
															Ipopt::Number* lambda) override;

	 bool					eval_f(	Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value) override;

	 bool					eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f) override;

	 bool					eval_g(	Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g) override;

	 bool					eval_jac_g( Ipopt::Index n, const Ipopt::Number* x, bool new_x,
													Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index *jCol,
													Ipopt::Number* values) override;

	 bool					eval_h( Ipopt::Index n, const Ipopt::Number* x, bool new_x,
												Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
												bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
												Ipopt::Index* jCol, Ipopt::Number* values) override;

	 void 					finalize_solution(	Ipopt::SolverReturn status,
															Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U,
															Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda,
															Ipopt::Number obj_value,
															const Ipopt::IpoptData* ip_data,
															Ipopt::IpoptCalculatedQuantities* ip_cq) override;
		const Eigen::VectorXd& GetSolution(){return mSolution;};
	protected:
		BoxQP();
		BoxQP(const BoxQP&);
		BoxQP& operator=(const BoxQP&);

	Eigen::MatrixXd	H;
	Eigen::VectorXd	g;
	Eigen::VectorXd	lower,upper;
	Eigen::VectorXd											mSolution;

};
#endif