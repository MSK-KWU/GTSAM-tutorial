#include <iostream>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Values.h>


using namespace std;
using namespace gtsam;

//inherit
class UnaryFactor: public NoiseModelFactor1<Pose2>
{
double x_measure, y_measure; ///<X andY measurements

public:
  UnaryFactor(Key j, double x, double y, const SharedNoiseModel&model):
  NoiseModelFactor1<Pose2>(model, j), x_measure(x), y_measure(y){}

  Vector evaluateError(const Pose2& q, boost::optional<Matrix&>H = boost::none) const
  {
    if (H) 
    {
      const Rot2& R = q.rotation();
      if (H) (*H) = (gtsam::Matrix(2, 3) <<
            R.c(), -R.s(), 0.0,
            R.s(), R.c(), 0.0).finished();
    }
    
    return (Vector(2) << q.x() - x_measure, q.y() - y_measure).finished();
  }
};

int main(int argc, char** argv)
{
  // 1. 비선형 factor graph를 생성
  NonlinearFactorGraph graph;
  //x0
  Pose2 priorMean(0.0, 0.0, 0.0);
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise));

  //move +2, x-axis
  Pose2 odometry(2.0, 0.0, 0.0);
  noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

  //x1
  graph.add(BetweenFactor<Pose2>(1, 2, odometry, odometryNoise));

  //x2
  graph.add(BetweenFactor<Pose2>(2, 3, odometry, odometryNoise));

  //x3

  // graph.print("\nFactor Graph:\n");

  // // 5. 변수들의 초기값을 설정합니다.
  // // 최적화를 시작하기 위한 초기 추정치입니다.
  Values initial;
  initial.insert(1, Pose2(0.5, 0.0, 0.2));
  initial.insert(2, Pose2(2.3, 0.1, -0.2));
  initial.insert(3, Pose2(4.1, 0.1, 0.1));
  // initial.print("\nInitial Estimate:\n");

  //LM optimizer
  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  // result.print("Final Result:\n");

  //유효숫자 2개
  cout.precision(2);
  Marginals marginals(graph, result);
  cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;

  // return 0;
}

