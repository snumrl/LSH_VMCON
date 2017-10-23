#ifndef __FEM_INTERFACE_H__
#define __FEM_INTERFACE_H__
#include "fem/fem.h"
#include "gui/gui.h"

namespace FEM
{
void DrawWorld(const std::shared_ptr<World>& world);
void DrawConstraint(const std::shared_ptr<Cst>& c,const Eigen::VectorXd& x);
};

#endif
