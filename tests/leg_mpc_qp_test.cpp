#include "wbr/leg_mpc.hpp"
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

using Mpc = wbr::control::LegMpc;
static void Check(const Mpc::Result& result, const Mpc::Parameters& p,
                  Mpc::Input previous) {
  assert(result.converged && !result.used_fallback);
  for (const auto& force : result.predicted_force_n) {
    assert(force.allFinite());
    assert(force.minCoeff() >= p.minimum_force_n-p.constraint_tolerance_n);
    assert(force.maxCoeff() <= p.maximum_force_n+p.constraint_tolerance_n);
    assert((force-previous).cwiseAbs().maxCoeff() <= p.maximum_force_step_n+p.constraint_tolerance_n);
    previous=force;
  }
}
int main(int argc, char**) {
  Mpc controller;
  const auto& p=controller.parameters();
  const double hover=p.mass_kg*p.gravity_m_per_s2/2;
  Mpc::Input previous=Mpc::Input::Constant(hover);
  Mpc::State state;
  Mpc::Reference reference;
  reference.height_m=.25;
  state << .3, 2., .18, -1.;
  const auto begin=std::chrono::steady_clock::now();
  auto result=controller.Solve(state,reference,previous);
  Check(result,p,previous);
  if (argc>1) {
    std::cout.precision(17);
    std::cout<<"A"; for(int i=0;i<4;++i) for(int j=0;j<4;++j) std::cout<<' '<<controller.discrete_a()(i,j); std::cout<<'\n';
    std::cout<<"B"; for(int i=0;i<4;++i) for(int j=0;j<2;++j) std::cout<<' '<<controller.discrete_b()(i,j); std::cout<<'\n';
    std::cout<<"Q "<<p.state_weights.transpose()<<"\nR "<<p.input_weights.transpose()<<'\n';
    std::cout<<"X "<<state.transpose()<<"\nREF 0 0 "<<reference.height_m<<" 0\n";
    std::cout<<"PARAM "<<hover<<' '<<p.nominal_height_m<<' '<<p.minimum_force_n<<' '<<p.maximum_force_n<<' '<<p.maximum_force_step_n<<' '<<p.terminal_weight_scale<<'\n';
    std::cout<<"U";for(const auto& force:result.predicted_force_n) std::cout<<' '<<force(0)<<' '<<force(1); std::cout<<'\n';
  }
  std::cout<<"constrained iter="<<result.iterations<<" primal="<<result.primal_residual
           <<" dual="<<result.dual_residual<<" solve_us="
           <<std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count()<<'\n';
  bool active=false;
  auto prior=previous;
  for (const auto& force:result.predicted_force_n) {
    active |= (force-prior).cwiseAbs().maxCoeff() > p.maximum_force_step_n-.01;
    prior=force;
  }
  assert(active); // Exercise the coupled horizon slew constraints, not just a free solution.
  Mpc other;
  state << 0., 0., .25, 0.;
  Check(other.Solve(state,reference,previous),p,previous);
  state(0)=std::numeric_limits<double>::quiet_NaN();
  auto invalid=controller.Solve(state,reference,previous);
  assert(!invalid.converged && invalid.used_fallback);
  assert(invalid.left_force_n==hover && invalid.right_force_n==hover);
  Mpc::Parameters limited=p;
  limited.maximum_iterations=1;
  Mpc truncated(limited);
  state << .3, 2., .18, -1.;
  auto failed=truncated.Solve(state,reference,previous);
  assert(!failed.converged && failed.used_fallback);
  controller.Reset();
  Check(controller.Solve(state,reference,previous),p,previous);
  std::cout<<"full-horizon force/slew bounds, independent instances, invalid input, iteration limit and reset passed\n";
}
