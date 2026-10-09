#include "wbr/leg_mpc.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>

int main() {
  using Mpc=wbr::control::LegMpc;
  Mpc controller;
  const auto& p=controller.parameters();
  Mpc::Input previous=Mpc::Input::Constant(p.mass_kg*p.gravity_m_per_s2/2.0);
  Mpc::Reference reference;
  Mpc::State state;
  constexpr int count=5000;
  std::array<double,count> timing{};
  int failed=0,slow=0,max_iterations=0;
  double sum=0,max_violation=0;
  for(int k=0;k<count;++k) {
    const double t=k*.01;
    state << .18*std::sin(t*1.7), .4*std::cos(t*1.7), .23+.04*std::sin(t), .1*std::cos(t);
    reference.height_m=.24+.06*std::sin(t*.7);
    const auto start=std::chrono::steady_clock::now();
    auto result=controller.Solve(state,reference,previous);
    timing[k]=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
    sum+=timing[k];slow+=timing[k]>10000;
    failed+=!result.converged;
    max_iterations=std::max(max_iterations,result.iterations);
    max_violation=std::max(max_violation,result.constraint_violation_n);
    previous << result.left_force_n,result.right_force_n;
  }
  std::sort(timing.begin(),timing.end());
  std::cout<<"calls="<<count<<" mean_us="<<sum/count<<" p99_us="<<timing[4949]
           <<" max_us="<<timing.back()<<" over_10ms="<<slow<<" not_solved="<<failed
           <<" max_iterations="<<max_iterations<<" max_constraint_violation_n="<<max_violation<<'\n';
  return failed ? 1:0;
}
