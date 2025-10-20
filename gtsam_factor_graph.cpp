#include <iostream>
#include <fstream>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Values.h>


using namespace std;
using namespace gtsam;

// Helper function to save poses to file for visualization
void savePosesToFile(const Values& values, const string& filename) {
  ofstream file(filename);
  for(const auto& key_value : values) {
    Key key = key_value.key;
    Pose2 pose = values.at<Pose2>(key);
    file << key << " " << pose.x() << " " << pose.y() << " " << pose.theta() << endl;
  }
  file.close();
  cout << "Saved poses to " << filename << endl;
}

//inherit
class UnaryFactor: public NoiseModelFactor1<Pose2>
{
double x_measure, y_measure; ///<X andY measurements

public:
  UnaryFactor(Key j, double x, double y, const SharedNoiseModel&model):
  NoiseModelFactor1<Pose2>(model, j), x_measure(x), y_measure(y){}

  // Override the correct evaluateError signature for newer GTSAM
  Vector evaluateError(const Pose2& q, OptionalMatrixType H) const override
  {
    if (H) 
    {
      const Rot2& R = q.rotation();
      *H = (gtsam::Matrix(2, 3) <<
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

  // 2. Odometry 노이즈 모델 생성
  Pose2 odometry(2.0, 0.0, 0.0);
  noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

  // 3. Odometry factors 추가 (x1-x2, x2-x3)
  graph.add(BetweenFactor<Pose2>(1, 2, odometry, odometryNoise));
  graph.add(BetweenFactor<Pose2>(2, 3, odometry, odometryNoise));

  // ========================================
  // Section 3.3: Using Custom Factors
  // ========================================
  
  // 4. GPS 같은 Unary measurement factors 추가
  noiseModel::Diagonal::shared_ptr unaryNoise =
    noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1)); // 10cm std on x,y
  
    
  graph.add(std::make_shared<UnaryFactor>(1, 0.0, 0.0, unaryNoise));
  graph.add(std::make_shared<UnaryFactor>(2, 2.0, 0.0, unaryNoise));
  graph.add(std::make_shared<UnaryFactor>(3, 4.0, 0.0, unaryNoise));

  // Factor graph 출력
  graph.print("\nFactor Graph:\n");

  // 5. 초기 추정값 설정
  Values initial;
  initial.insert(1, Pose2(0.5, 0.0, 0.2));
  initial.insert(2, Pose2(2.3, 0.1, -0.2));
  initial.insert(3, Pose2(4.1, 0.1, 0.1));
  initial.print("\nInitial Estimate:\n");

  // 6. Levenberg-Marquardt 최적화
  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  result.print("\nFinal Result:\n");

  // Save poses to file
  savePosesToFile(result, "../result/result_2d.txt");

  // ========================================
  // Section 3.4: Full Posterior Inference
  // ========================================
  
  // 7. Marginal covariances 계산 및 출력
  cout << "\n========================================" << endl;
  cout << "Section 3.4: Marginal Covariances" << endl;
  cout << "========================================" << endl;
  
  // 소수점 4자리까지 출력 설정
  cout.precision(4);
  Marginals marginals(graph, result);
  
  cout << "\nx1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "\nx2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "\nx3 covariance:\n" << marginals.marginalCovariance(3) << endl;
  
  // ========================================
  // Section 4: PoseSLAM with Loop Closure
  // ========================================
  
  cout << "\n\n========================================" << endl;
  cout << "Section 4.1: PoseSLAM with Loop Closure" << endl;
  cout << "========================================" << endl;
  
  // 새로운 factor graph 생성
  NonlinearFactorGraph graph2;
  
  // Prior factor on first pose
  noiseModel::Diagonal::shared_ptr priorNoise2 = 
    noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  graph2.add(PriorFactor<Pose2>(1, Pose2(0, 0, 0), priorNoise2));
  
  // Odometry factors (로봇이 사각형으로 이동)
  noiseModel::Diagonal::shared_ptr model2 = 
    noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
  
  graph2.add(BetweenFactor<Pose2>(1, 2, Pose2(2, 0, 0), model2));
  graph2.add(BetweenFactor<Pose2>(2, 3, Pose2(2, 0, M_PI_2), model2));
  graph2.add(BetweenFactor<Pose2>(3, 4, Pose2(2, 0, M_PI_2), model2));
  graph2.add(BetweenFactor<Pose2>(4, 5, Pose2(2, 0, M_PI_2), model2));
  
  // Loop closure constraint (x5와 x2 사이)
  graph2.add(BetweenFactor<Pose2>(5, 2, Pose2(2, 0, M_PI_2), model2));
  
  graph2.print("\nFactor Graph (PoseSLAM):\n");
  
  // Initial estimate
  Values initial2;
  initial2.insert(1, Pose2(0.5, 0.0, 0.2));
  initial2.insert(2, Pose2(2.3, 0.1, -0.2));
  initial2.insert(3, Pose2(4.1, 0.1, M_PI_2));
  initial2.insert(4, Pose2(4.0, 2.0, M_PI));
  initial2.insert(5, Pose2(2.1, 2.1, -M_PI_2));
  initial2.print("\nInitial Estimate (PoseSLAM):\n");
  
  // Optimize
  Values result2 = LevenbergMarquardtOptimizer(graph2, initial2).optimize();
  result2.print("\nFinal Result (PoseSLAM):\n");
  
  // Save poses to file
  savePosesToFile(result2, "../result/result_poseslam.txt");
  
  // Marginal covariances
  Marginals marginals2(graph2, result2);
  cout << "\nx1 covariance:\n" << marginals2.marginalCovariance(1) << endl;
  cout << "\nx2 covariance:\n" << marginals2.marginalCovariance(2) << endl;
  cout << "\nx3 covariance:\n" << marginals2.marginalCovariance(3) << endl;
  cout << "\nx4 covariance:\n" << marginals2.marginalCovariance(4) << endl;
  cout << "\nx5 covariance:\n" << marginals2.marginalCovariance(5) << endl;
  
  // Save results for visualization
  savePosesToFile(initial2, "../result/poses_initial.txt");
  savePosesToFile(result2, "../result/poses_optimized.txt");
  
  cout << "\nRun 'python3 visualize_poses.py' to visualize the results!" << endl;
  
  return 0;
}

